#!/usr/bin/env python3
"""Audit the frozen TurboADMM-NL continuation ablation protocol."""

import argparse
import csv
import json
import math
import statistics
from pathlib import Path


def read_csv(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def as_float(row, field):
    try:
        value = float(row[field])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"missing or invalid {field}") from error
    if not math.isfinite(value):
        raise ValueError(f"non-finite {field}")
    return value


def exact_sign_p(wins, losses):
    count = wins + losses
    if count == 0:
        return 1.0
    tail = min(wins, losses)
    probability = sum(math.comb(count, value) for value in range(tail + 1))
    return min(1.0, 2.0 * probability / (2.0 ** count))


def audit_pairs(path, baseline, arguments):
    rows = read_csv(path)
    issues = []
    if len(rows) != arguments.expected_pairs:
        issues.append(
            f"{baseline}: expected {arguments.expected_pairs} pairs, found {len(rows)}"
        )

    selectors = [row.get("selector", "") for row in rows]
    if len(set(selectors)) != len(selectors) or "" in selectors:
        issues.append(f"{baseline}: selectors are missing or not unique")

    runtime_reductions = []
    qp_reductions = []
    objective_gaps = []
    wins = losses = ties = 0
    candidate_strict = baseline_strict = 0

    for row in rows:
        selector = row.get("selector", "<unknown>")
        if row.get("candidate") != "full" or row.get("baseline") != baseline:
            issues.append(f"{selector}: expected full versus {baseline}")
        if row.get("candidate_execution_status") != "completed":
            issues.append(f"{selector}: full execution did not complete")
        if row.get("baseline_execution_status") != "completed":
            issues.append(f"{selector}: {baseline} execution did not complete")
        if row.get("candidate_success") != "1":
            issues.append(f"{selector}: full trajectory is not independently valid")
        if row.get("baseline_success") != "1":
            issues.append(f"{selector}: {baseline} trajectory is not independently valid")
        if row.get("candidate_converged") == "1":
            candidate_strict += 1
        else:
            issues.append(f"{selector}: full method did not strictly converge")
        if row.get("baseline_converged") == "1":
            baseline_strict += 1

        try:
            candidate_wall = as_float(row, "candidate_wall_time_s")
            baseline_wall = as_float(row, "baseline_wall_time_s")
            gap = abs(as_float(row, "candidate_objective_gap_percent"))
            candidate_qp = as_float(row, "candidate_qp_solves")
            baseline_qp = as_float(row, "baseline_qp_solves")
        except ValueError as error:
            issues.append(f"{selector}: {error}")
            continue
        if candidate_wall <= 0.0 or baseline_wall <= 0.0:
            issues.append(f"{selector}: wall times must be positive")
            continue
        if candidate_qp < 0.0 or baseline_qp <= 0.0:
            issues.append(f"{selector}: local QP counts are invalid")
            continue

        runtime_reductions.append(100.0 * (baseline_wall - candidate_wall) / baseline_wall)
        qp_reductions.append(100.0 * (baseline_qp - candidate_qp) / baseline_qp)
        objective_gaps.append(gap)
        if baseline_wall > candidate_wall:
            wins += 1
        elif baseline_wall < candidate_wall:
            losses += 1
        else:
            ties += 1

    median_runtime_reduction = (
        statistics.median(runtime_reductions) if runtime_reductions else math.nan
    )
    median_qp_reduction = statistics.median(qp_reductions) if qp_reductions else math.nan
    maximum_objective_gap = max(objective_gaps, default=math.nan)
    sign_p = exact_sign_p(wins, losses)

    if maximum_objective_gap > arguments.maximum_absolute_objective_gap_percent:
        issues.append(
            f"{baseline}: maximum objective gap {maximum_objective_gap:.6g}% exceeds "
            f"{arguments.maximum_absolute_objective_gap_percent:.6g}%"
        )
    if median_runtime_reduction < arguments.minimum_median_runtime_reduction_percent:
        issues.append(
            f"{baseline}: median wall reduction {median_runtime_reduction:.6g}% is below "
            f"{arguments.minimum_median_runtime_reduction_percent:.6g}%"
        )
    if sign_p > arguments.maximum_runtime_sign_p_value:
        issues.append(
            f"{baseline}: paired runtime sign p={sign_p:.6g} exceeds "
            f"{arguments.maximum_runtime_sign_p_value:.6g}"
        )
    if baseline == "inner" and median_qp_reduction < arguments.minimum_median_qp_reduction_percent:
        issues.append(
            f"inner: median QP reduction {median_qp_reduction:.6g}% is below "
            f"{arguments.minimum_median_qp_reduction_percent:.6g}%"
        )

    return {
        "baseline": baseline,
        "pairs": len(rows),
        "selectors": sorted(selectors),
        "candidate_strict_convergences": candidate_strict,
        "baseline_strict_convergences": baseline_strict,
        "maximum_absolute_objective_gap_percent": maximum_objective_gap,
        "median_wall_runtime_reduction_percent": median_runtime_reduction,
        "median_local_qp_reduction_percent": median_qp_reduction,
        "runtime_wins": wins,
        "runtime_losses": losses,
        "runtime_ties": ties,
        "runtime_exact_sign_p": sign_p,
        "rows_by_selector": {row.get("selector", ""): row for row in rows},
        "issues": issues,
    }


def audit_manifest(path, arguments):
    with path.open(encoding="utf-8") as stream:
        manifest = json.load(stream)
    configuration = manifest.get("configuration", {})
    issues = []
    expected = {
        "protocol_id": arguments.expected_protocol,
        "git_commit": arguments.expected_commit,
        "suite": "paper_development",
        "repetitions": 1,
        "schedule": "interleaved",
        "schedule_seed": 20260805,
        "fixed_rho": True,
        "exact_admm": True,
    }
    for field, value in expected.items():
        if configuration.get(field) != value:
            issues.append(
                f"manifest {field}={configuration.get(field)!r}; expected {value!r}"
            )
    if set(configuration.get("methods", [])) != {"full", "inner", "qp_continuation"}:
        issues.append("manifest method set is not full, inner, qp_continuation")
    if len(configuration.get("task_order", [])) != 3 * arguments.expected_pairs:
        issues.append("manifest task count does not match the frozen 3-method matrix")
    return issues


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("full_vs_inner", type=Path)
    parser.add_argument("full_vs_qp_continuation", type=Path)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--expected-commit", required=True)
    parser.add_argument(
        "--expected-protocol", default="ral-continuation-ablation-development-v1"
    )
    parser.add_argument("--expected-pairs", type=int, default=240)
    parser.add_argument(
        "--maximum-absolute-objective-gap-percent", type=float, default=5.0
    )
    parser.add_argument(
        "--minimum-median-runtime-reduction-percent", type=float, default=20.0
    )
    parser.add_argument(
        "--minimum-median-qp-reduction-percent", type=float, default=20.0
    )
    parser.add_argument("--maximum-runtime-sign-p-value", type=float, default=0.05)
    parser.add_argument("--output", type=Path)
    arguments = parser.parse_args()

    inner = audit_pairs(arguments.full_vs_inner, "inner", arguments)
    qp = audit_pairs(arguments.full_vs_qp_continuation, "qp_continuation", arguments)
    issues = audit_manifest(arguments.manifest, arguments)
    issues.extend(inner.pop("issues"))
    issues.extend(qp.pop("issues"))

    if inner["selectors"] != qp["selectors"]:
        issues.append("the two comparisons do not contain identical scenario sets")
    shared_fields = (
        "candidate_success",
        "candidate_converged",
        "candidate_wall_time_s",
        "candidate_qp_solves",
    )
    for selector in set(inner["selectors"]) & set(qp["selectors"]):
        inner_row = inner["rows_by_selector"][selector]
        qp_row = qp["rows_by_selector"][selector]
        for field in shared_fields:
            if inner_row.get(field) != qp_row.get(field):
                issues.append(f"{selector}: inconsistent full-method field {field}")

    inner.pop("selectors")
    qp.pop("selectors")
    inner.pop("rows_by_selector")
    qp.pop("rows_by_selector")
    report = {
        "status": "passed" if not issues else "failed",
        "thresholds": {
            "expected_pairs": arguments.expected_pairs,
            "maximum_absolute_objective_gap_percent": arguments.maximum_absolute_objective_gap_percent,
            "minimum_median_runtime_reduction_percent": arguments.minimum_median_runtime_reduction_percent,
            "minimum_median_qp_reduction_percent": arguments.minimum_median_qp_reduction_percent,
            "maximum_runtime_sign_p_value": arguments.maximum_runtime_sign_p_value,
        },
        "full_vs_inner": inner,
        "full_vs_qp_continuation": qp,
        "issues": issues,
    }
    rendered = json.dumps(report, indent=2, sort_keys=True)
    if arguments.output:
        arguments.output.parent.mkdir(parents=True, exist_ok=True)
        arguments.output.write_text(rendered + "\n", encoding="utf-8")
    print(rendered)
    raise SystemExit(0 if not issues else 1)


if __name__ == "__main__":
    main()
