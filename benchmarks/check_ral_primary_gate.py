#!/usr/bin/env python3
"""Enforce the frozen correctness gates on paired RA-L primary results."""

import argparse
import csv
import math
from pathlib import Path


def as_float(row, field):
    try:
        return float(row[field])
    except (KeyError, TypeError, ValueError):
        return math.nan


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pairs", type=Path)
    parser.add_argument("--expected-pairs", type=int, required=True)
    parser.add_argument(
        "--maximum-absolute-objective-gap-percent", type=float, default=5.0)
    arguments = parser.parse_args()
    if arguments.expected_pairs <= 0:
        parser.error("--expected-pairs must be positive")
    if (not math.isfinite(arguments.maximum_absolute_objective_gap_percent)
            or arguments.maximum_absolute_objective_gap_percent < 0.0):
        parser.error("--maximum-absolute-objective-gap-percent must be nonnegative")

    with arguments.pairs.open(newline="", encoding="utf-8-sig") as stream:
        rows = list(csv.DictReader(stream))
    failures = []
    if len(rows) != arguments.expected_pairs:
        failures.append(
            f"expected {arguments.expected_pairs} paired rows, found {len(rows)}")

    selectors = [row.get("selector", "") for row in rows]
    missing_selectors = [index for index, value in enumerate(selectors) if not value]
    if missing_selectors:
        failures.append(f"{len(missing_selectors)} rows have no selector")
    if len(set(selectors)) != len(selectors):
        failures.append("paired rows contain duplicate selectors")

    unsuccessful = [
        row.get("selector", "<missing>")
        for row in rows if row.get("candidate_success") != "1"
    ]
    nonconverged = [
        row.get("selector", "<missing>")
        for row in rows if row.get("candidate_converged") != "1"
    ]
    if unsuccessful:
        failures.append(
            f"TurboADMM-NL failed validation on {len(unsuccessful)} rows: "
            + ", ".join(unsuccessful[:10]))
    if nonconverged:
        failures.append(
            f"TurboADMM-NL failed strict convergence on {len(nonconverged)} rows: "
            + ", ".join(nonconverged[:10]))

    comparisons = [row for row in rows if row.get("paired_outcome") == "both_success"]
    excessive_gaps = []
    for row in comparisons:
        gap = as_float(row, "candidate_objective_gap_percent")
        if (not math.isfinite(gap)
                or abs(gap) > arguments.maximum_absolute_objective_gap_percent):
            excessive_gaps.append((row.get("selector", "<missing>"), gap))
    if not comparisons:
        failures.append("no successful paired objective comparisons")
    if excessive_gaps:
        detail = ", ".join(
            f"{selector}={gap:.6g}%" for selector, gap in excessive_gaps[:10])
        failures.append(
            f"{len(excessive_gaps)} objective gaps exceed the frozen threshold: {detail}")

    finite_gaps = [
        abs(as_float(row, "candidate_objective_gap_percent"))
        for row in comparisons
        if math.isfinite(as_float(row, "candidate_objective_gap_percent"))
    ]
    print(
        f"paired={len(rows)}/{arguments.expected_pairs} "
        f"turbo_success={len(rows) - len(unsuccessful)} "
        f"turbo_strict={len(rows) - len(nonconverged)} "
        f"objective_comparisons={len(comparisons)} "
        f"maximum_absolute_objective_gap_percent="
        f"{max(finite_gaps) if finite_gaps else math.nan:.9g}")
    if failures:
        raise SystemExit("RA-L primary gate failed:\n- " + "\n- ".join(failures))


if __name__ == "__main__":
    main()
