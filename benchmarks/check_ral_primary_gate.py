#!/usr/bin/env python3
"""Enforce the frozen correctness gates on paired RA-L primary results."""

import argparse
import csv
import math
import statistics
from pathlib import Path


def as_float(row, field):
    try:
        return float(row[field])
    except (KeyError, TypeError, ValueError):
        return math.nan


def as_int(row, field):
    try:
        return int(row[field])
    except (KeyError, TypeError, ValueError):
        return None


def exact_sign_p_value(above, below):
    trials = above + below
    if trials == 0:
        return 1.0
    tail = min(above, below)
    probability = sum(
        math.comb(trials, value) for value in range(tail + 1)
    ) / (2.0 ** trials)
    return min(1.0, 2.0 * probability)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pairs", type=Path)
    parser.add_argument("--expected-pairs", type=int, required=True)
    parser.add_argument(
        "--maximum-absolute-objective-gap-percent", type=float, default=5.0)
    parser.add_argument("--minimum-speedup-agent-count", type=int, default=0)
    parser.add_argument("--minimum-median-wall-speedup", type=float, default=1.0)
    parser.add_argument(
        "--minimum-largest-scale-wall-speedup", type=float, default=1.0)
    parser.add_argument(
        "--maximum-wall-speedup-sign-p-value", type=float, default=0.05)
    arguments = parser.parse_args()
    if arguments.expected_pairs <= 0:
        parser.error("--expected-pairs must be positive")
    if (not math.isfinite(arguments.maximum_absolute_objective_gap_percent)
            or arguments.maximum_absolute_objective_gap_percent < 0.0):
        parser.error("--maximum-absolute-objective-gap-percent must be nonnegative")
    if arguments.minimum_speedup_agent_count < 0:
        parser.error("--minimum-speedup-agent-count must be nonnegative")
    if (not math.isfinite(arguments.minimum_median_wall_speedup)
            or arguments.minimum_median_wall_speedup <= 0.0
            or not math.isfinite(arguments.minimum_largest_scale_wall_speedup)
            or arguments.minimum_largest_scale_wall_speedup <= 0.0):
        parser.error("wall-speedup thresholds must be positive")
    if (not math.isfinite(arguments.maximum_wall_speedup_sign_p_value)
            or not 0.0 <= arguments.maximum_wall_speedup_sign_p_value <= 1.0):
        parser.error("--maximum-wall-speedup-sign-p-value must be in [0, 1]")

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

    runtime_report = ""
    if arguments.minimum_speedup_agent_count > 0:
        eligible = [
            row for row in comparisons
            if (as_int(row, "n") is not None
                and as_int(row, "n") >= arguments.minimum_speedup_agent_count)
        ]
        speedups = [
            as_float(row, "baseline_over_candidate_wall") for row in eligible
        ]
        finite_speedups = [
            value for value in speedups if math.isfinite(value) and value > 0.0
        ]
        if not eligible:
            failures.append("no successful medium/large-scale wall-time comparisons")
        elif len(finite_speedups) != len(eligible):
            failures.append("medium/large-scale wall-time comparisons are incomplete")
        else:
            median_speedup = statistics.median(finite_speedups)
            above = sum(value > 1.0 for value in finite_speedups)
            below = sum(value < 1.0 for value in finite_speedups)
            sign_p = exact_sign_p_value(above, below)
            largest_n = max(as_int(row, "n") for row in eligible)
            largest_speedups = [
                as_float(row, "baseline_over_candidate_wall") for row in eligible
                if as_int(row, "n") == largest_n
            ]
            largest_median = statistics.median(largest_speedups)
            runtime_report = (
                f" speedup_n_ge_{arguments.minimum_speedup_agent_count}="
                f"{median_speedup:.9g} largest_n={largest_n} "
                f"largest_speedup={largest_median:.9g} sign_p={sign_p:.9g}")
            if median_speedup < arguments.minimum_median_wall_speedup:
                failures.append(
                    "medium/large median wall speedup is below the frozen threshold: "
                    f"{median_speedup:.6g} < "
                    f"{arguments.minimum_median_wall_speedup:.6g}")
            if largest_median < arguments.minimum_largest_scale_wall_speedup:
                failures.append(
                    "largest-scale median wall speedup is below the frozen threshold: "
                    f"{largest_median:.6g} < "
                    f"{arguments.minimum_largest_scale_wall_speedup:.6g}")
            if above <= below or sign_p > arguments.maximum_wall_speedup_sign_p_value:
                failures.append(
                    "medium/large wall speedup is not significant in the favorable "
                    f"direction: above={above} below={below} p={sign_p:.6g}")

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
        f"{max(finite_gaps) if finite_gaps else math.nan:.9g}"
        f"{runtime_report}")
    if failures:
        raise SystemExit("RA-L primary gate failed:\n- " + "\n- ".join(failures))


if __name__ == "__main__":
    main()
