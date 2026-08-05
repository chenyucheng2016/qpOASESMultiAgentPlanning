#!/usr/bin/env python3
"""Enforce frozen correctness gates on repeated deterministic benchmarks."""

import argparse
import csv
import math
from pathlib import Path


def parse_cases(value):
    cases = tuple(item.strip() for item in value.split(",") if item.strip())
    if not cases or len(set(cases)) != len(cases):
        raise argparse.ArgumentTypeError("cases must be a nonempty unique list")
    return cases


def as_float(row, field):
    try:
        return float(row[field])
    except (KeyError, TypeError, ValueError):
        return math.nan


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("results", type=Path)
    parser.add_argument("--cases", type=parse_cases, required=True)
    parser.add_argument("--repetitions", type=int, required=True)
    parser.add_argument("--candidate", default="full")
    parser.add_argument("--baseline", default="centralized_osqp")
    parser.add_argument(
        "--maximum-absolute-objective-gap-percent", type=float, default=5.0)
    arguments = parser.parse_args()
    if arguments.repetitions <= 0:
        parser.error("--repetitions must be positive")
    if (not math.isfinite(arguments.maximum_absolute_objective_gap_percent)
            or arguments.maximum_absolute_objective_gap_percent < 0.0):
        parser.error("--maximum-absolute-objective-gap-percent must be nonnegative")

    with arguments.results.open(newline="", encoding="utf-8-sig") as stream:
        rows = list(csv.DictReader(stream))
    expected_keys = {
        (case_id, repetition)
        for case_id in arguments.cases
        for repetition in range(1, arguments.repetitions + 1)
    }
    candidate_rows = [row for row in rows if row.get("method") == arguments.candidate]
    candidate_by_key = {}
    duplicates = []
    for row in candidate_rows:
        try:
            key = (row["case_id"], int(row.get("repetition") or "1"))
        except (KeyError, ValueError):
            duplicates.append(("<malformed>", 0))
            continue
        if key in candidate_by_key:
            duplicates.append(key)
        candidate_by_key[key] = row

    failures = []
    missing = sorted(expected_keys - set(candidate_by_key))
    unexpected = sorted(set(candidate_by_key) - expected_keys)
    if missing:
        failures.append(
            f"missing {len(missing)} Turbo rows: "
            + ", ".join(f"{case}/r{rep}" for case, rep in missing[:10]))
    if unexpected:
        failures.append(
            f"found {len(unexpected)} unexpected Turbo rows: "
            + ", ".join(f"{case}/r{rep}" for case, rep in unexpected[:10]))
    if duplicates:
        failures.append(f"found {len(duplicates)} duplicate or malformed Turbo rows")

    selected = [candidate_by_key[key] for key in sorted(expected_keys & set(candidate_by_key))]
    unsuccessful = [row for row in selected if row.get("protocol_success") != "1"]
    nonconverged = [row for row in selected if row.get("converged") != "1"]
    if unsuccessful:
        failures.append(f"TurboADMM-NL failed validation on {len(unsuccessful)} repetitions")
    if nonconverged:
        failures.append(f"TurboADMM-NL failed strict convergence on {len(nonconverged)} repetitions")

    baseline_by_key = {}
    for row in rows:
        if row.get("method") != arguments.baseline:
            continue
        try:
            key = (row["case_id"], int(row.get("repetition") or "1"))
        except (KeyError, ValueError):
            continue
        if key in expected_keys and row.get("protocol_success") == "1":
            baseline_by_key[key] = row
    gaps = []
    excessive = []
    for key, baseline in baseline_by_key.items():
        candidate = candidate_by_key.get(key)
        if candidate is None or candidate.get("protocol_success") != "1":
            continue
        baseline_objective = as_float(baseline, "objective")
        candidate_objective = as_float(candidate, "objective")
        gap = (
            100.0 * (candidate_objective - baseline_objective) / abs(baseline_objective)
            if math.isfinite(candidate_objective) and math.isfinite(baseline_objective)
            and baseline_objective != 0.0 else math.nan)
        gaps.append(gap)
        if (not math.isfinite(gap)
                or abs(gap) > arguments.maximum_absolute_objective_gap_percent):
            excessive.append((key, gap))
    if not gaps:
        failures.append("no successful Turbo/OSQP objective comparisons")
    if excessive:
        detail = ", ".join(
            f"{case}/r{rep}={gap:.6g}%" for (case, rep), gap in excessive[:10])
        failures.append(f"{len(excessive)} objective gaps exceed the frozen threshold: {detail}")

    finite_gaps = [abs(value) for value in gaps if math.isfinite(value)]
    print(
        f"turbo_rows={len(selected)}/{len(expected_keys)} "
        f"turbo_success={len(selected) - len(unsuccessful)} "
        f"turbo_strict={len(selected) - len(nonconverged)} "
        f"objective_comparisons={len(gaps)} "
        f"maximum_absolute_objective_gap_percent="
        f"{max(finite_gaps) if finite_gaps else math.nan:.9g}")
    if failures:
        raise SystemExit("RA-L deterministic gate failed:\n- " + "\n- ".join(failures))


if __name__ == "__main__":
    main()
