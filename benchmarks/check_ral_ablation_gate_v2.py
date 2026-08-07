#!/usr/bin/env python3
"""Audit the causal layers of the TurboADMM-NL continuation ablation."""

import argparse
import json
import statistics
from pathlib import Path

from check_ral_ablation_gate import as_float, audit_manifest, exact_sign_p, read_csv


def audit_pairs(
    path,
    candidate,
    baseline,
    minimum_runtime_reduction,
    minimum_qp_reduction,
    minimum_largest_scale_runtime_reduction,
    arguments,
):
    rows = read_csv(path)
    issues = []
    if len(rows) != arguments.expected_pairs:
        issues.append(
            f"{candidate} versus {baseline}: expected {arguments.expected_pairs} "
            f"pairs, found {len(rows)}"
        )

    selectors = [row.get("selector", "") for row in rows]
    if len(set(selectors)) != len(selectors) or "" in selectors:
        issues.append(f"{candidate} versus {baseline}: invalid selectors")

    runtime_reductions = []
    runtime_reductions_by_n = {}
    qp_reductions = []
    objective_gaps = []
    wins = losses = ties = 0
    candidate_strict = baseline_strict = 0

    for row in rows:
        selector = row.get("selector", "<unknown>")
        if row.get("candidate") != candidate or row.get("baseline") != baseline:
            issues.append(f"{selector}: expected {candidate} versus {baseline}")
        for role, method in (("candidate", candidate), ("baseline", baseline)):
            if row.get(f"{role}_execution_status") != "completed":
                issues.append(f"{selector}: {method} execution did not complete")
            if row.get(f"{role}_success") != "1":
                issues.append(f"{selector}: {method} trajectory is not independently valid")
            if row.get(f"{role}_converged") != "1":
                issues.append(f"{selector}: {method} did not strictly converge")
        candidate_strict += row.get("candidate_converged") == "1"
        baseline_strict += row.get("baseline_converged") == "1"

        try:
            candidate_wall = as_float(row, "candidate_wall_time_s")
            baseline_wall = as_float(row, "baseline_wall_time_s")
            gap = abs(as_float(row, "candidate_objective_gap_percent"))
            candidate_qp = as_float(row, "candidate_qp_solves")
            baseline_qp = as_float(row, "baseline_qp_solves")
            agent_count = int(row["n"])
        except (KeyError, TypeError, ValueError) as error:
            issues.append(f"{selector}: invalid numeric evidence: {error}")
            continue
        if candidate_wall <= 0.0 or baseline_wall <= 0.0:
            issues.append(f"{selector}: wall times must be positive")
            continue
        if candidate_qp < 0.0 or baseline_qp <= 0.0:
            issues.append(f"{selector}: local QP counts are invalid")
            continue

        runtime_reduction = 100.0 * (baseline_wall - candidate_wall) / baseline_wall
        runtime_reductions.append(runtime_reduction)
        runtime_reductions_by_n.setdefault(agent_count, []).append(runtime_reduction)
        qp_reductions.append(100.0 * (baseline_qp - candidate_qp) / baseline_qp)
        objective_gaps.append(gap)
        if baseline_wall > candidate_wall:
            wins += 1
        elif baseline_wall < candidate_wall:
            losses += 1
        else:
            ties += 1

    median_runtime_reduction = statistics.median(runtime_reductions)
    median_qp_reduction = statistics.median(qp_reductions)
    maximum_objective_gap = max(objective_gaps)
    largest_agent_count = max(runtime_reductions_by_n)
    largest_scale_reduction = statistics.median(
        runtime_reductions_by_n[largest_agent_count]
    )
    sign_p = exact_sign_p(wins, losses)

    if maximum_objective_gap > arguments.maximum_absolute_objective_gap_percent:
        issues.append(
            f"{candidate} versus {baseline}: maximum objective gap "
            f"{maximum_objective_gap:.6g}% exceeds "
            f"{arguments.maximum_absolute_objective_gap_percent:.6g}%"
        )
    if median_runtime_reduction < minimum_runtime_reduction:
        issues.append(
            f"{candidate} versus {baseline}: median wall reduction "
            f"{median_runtime_reduction:.6g}% is below {minimum_runtime_reduction:.6g}%"
        )
    if sign_p > arguments.maximum_runtime_sign_p_value:
        issues.append(
            f"{candidate} versus {baseline}: paired runtime sign p={sign_p:.6g} "
            f"exceeds {arguments.maximum_runtime_sign_p_value:.6g}"
        )
    if minimum_qp_reduction is not None and median_qp_reduction < minimum_qp_reduction:
        issues.append(
            f"{candidate} versus {baseline}: median QP reduction "
            f"{median_qp_reduction:.6g}% is below {minimum_qp_reduction:.6g}%"
        )
    if (
        minimum_largest_scale_runtime_reduction is not None
        and largest_scale_reduction < minimum_largest_scale_runtime_reduction
    ):
        issues.append(
            f"{candidate} versus {baseline}: N={largest_agent_count} median wall "
            f"reduction {largest_scale_reduction:.6g}% is below "
            f"{minimum_largest_scale_runtime_reduction:.6g}%"
        )

    return {
        "candidate": candidate,
        "baseline": baseline,
        "pairs": len(rows),
        "selectors": sorted(selectors),
        "candidate_strict_convergences": candidate_strict,
        "baseline_strict_convergences": baseline_strict,
        "maximum_absolute_objective_gap_percent": maximum_objective_gap,
        "median_wall_runtime_reduction_percent": median_runtime_reduction,
        "largest_agent_count": largest_agent_count,
        "largest_scale_median_wall_runtime_reduction_percent": largest_scale_reduction,
        "median_local_qp_reduction_percent": median_qp_reduction,
        "runtime_wins": wins,
        "runtime_losses": losses,
        "runtime_ties": ties,
        "runtime_exact_sign_p": sign_p,
        "rows_by_selector": {row.get("selector", ""): row for row in rows},
        "issues": issues,
    }


def compare_method_evidence(left, left_role, right, right_role, method, issues):
    selectors = set(left["selectors"]) & set(right["selectors"])
    fields = ("success", "converged", "wall_time_s", "qp_solves")
    for selector in selectors:
        left_row = left["rows_by_selector"][selector]
        right_row = right["rows_by_selector"][selector]
        for field in fields:
            if left_row.get(f"{left_role}_{field}") != right_row.get(
                f"{right_role}_{field}"
            ):
                issues.append(f"{selector}: inconsistent {method} field {field}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("full_vs_inner", type=Path)
    parser.add_argument("full_vs_qp_continuation", type=Path)
    parser.add_argument("qp_continuation_vs_inner", type=Path)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--expected-commit", required=True)
    parser.add_argument(
        "--expected-protocol", default="ral-continuation-ablation-development-v2"
    )
    parser.add_argument("--expected-pairs", type=int, default=240)
    parser.add_argument(
        "--maximum-absolute-objective-gap-percent", type=float, default=5.0
    )
    parser.add_argument("--minimum-total-runtime-reduction-percent", type=float, default=20.0)
    parser.add_argument("--minimum-matrix-runtime-reduction-percent", type=float, default=20.0)
    parser.add_argument("--minimum-pair-runtime-reduction-percent", type=float, default=5.0)
    parser.add_argument(
        "--minimum-pair-largest-scale-runtime-reduction-percent", type=float, default=10.0
    )
    parser.add_argument("--minimum-total-qp-reduction-percent", type=float, default=20.0)
    parser.add_argument("--minimum-pair-qp-reduction-percent", type=float, default=20.0)
    parser.add_argument("--maximum-runtime-sign-p-value", type=float, default=0.05)
    parser.add_argument("--output", type=Path)
    arguments = parser.parse_args()

    total = audit_pairs(
        arguments.full_vs_inner, "full", "inner",
        arguments.minimum_total_runtime_reduction_percent,
        arguments.minimum_total_qp_reduction_percent, None, arguments,
    )
    pair = audit_pairs(
        arguments.full_vs_qp_continuation, "full", "qp_continuation",
        arguments.minimum_pair_runtime_reduction_percent,
        arguments.minimum_pair_qp_reduction_percent,
        arguments.minimum_pair_largest_scale_runtime_reduction_percent, arguments,
    )
    matrix = audit_pairs(
        arguments.qp_continuation_vs_inner, "qp_continuation", "inner",
        arguments.minimum_matrix_runtime_reduction_percent, None, None, arguments,
    )

    issues = audit_manifest(arguments.manifest, arguments)
    for comparison in (total, pair, matrix):
        issues.extend(comparison.pop("issues"))
    if not (total["selectors"] == pair["selectors"] == matrix["selectors"]):
        issues.append("the three comparisons do not contain identical scenario sets")
    compare_method_evidence(total, "candidate", pair, "candidate", "full", issues)
    compare_method_evidence(total, "baseline", matrix, "baseline", "inner", issues)
    compare_method_evidence(
        pair, "baseline", matrix, "candidate", "qp_continuation", issues
    )
    for comparison in (total, pair, matrix):
        comparison.pop("selectors")
        comparison.pop("rows_by_selector")

    report = {
        "gate_version": 2,
        "status": "passed" if not issues else "failed",
        "thresholds": {
            "expected_pairs": arguments.expected_pairs,
            "maximum_absolute_objective_gap_percent": arguments.maximum_absolute_objective_gap_percent,
            "minimum_total_runtime_reduction_percent": arguments.minimum_total_runtime_reduction_percent,
            "minimum_matrix_runtime_reduction_percent": arguments.minimum_matrix_runtime_reduction_percent,
            "minimum_pair_runtime_reduction_percent": arguments.minimum_pair_runtime_reduction_percent,
            "minimum_pair_largest_scale_runtime_reduction_percent": arguments.minimum_pair_largest_scale_runtime_reduction_percent,
            "minimum_total_qp_reduction_percent": arguments.minimum_total_qp_reduction_percent,
            "minimum_pair_qp_reduction_percent": arguments.minimum_pair_qp_reduction_percent,
            "maximum_runtime_sign_p_value": arguments.maximum_runtime_sign_p_value,
        },
        "full_vs_inner": total,
        "full_vs_qp_continuation": pair,
        "qp_continuation_vs_inner": matrix,
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
