#!/usr/bin/env python3
"""Create paired, timeout-aware RA-L scaling summaries."""

import argparse
import csv
import math
import statistics
from collections import defaultdict
from pathlib import Path


SCENARIO_FIELDS = ("family", "density", "composition", "seed", "n", "m")


def read_csv(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def as_float(row, field):
    try:
        return float(row[field])
    except (KeyError, TypeError, ValueError):
        return math.nan


def quantile(values, probability):
    ordered = sorted(value for value in values if math.isfinite(value))
    if not ordered:
        return math.nan
    location = probability * (len(ordered) - 1)
    lower = int(math.floor(location))
    upper = int(math.ceil(location))
    if lower == upper:
        return ordered[lower]
    fraction = location - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def median(values):
    finite = [value for value in values if math.isfinite(value)]
    return statistics.median(finite) if finite else math.nan


def finite_min(values, upper_bound=math.inf):
    finite = [value for value in values if math.isfinite(value) and value < upper_bound]
    return min(finite) if finite else math.nan


def finite_max(values):
    finite = [value for value in values if math.isfinite(value)]
    return max(finite) if finite else math.nan


def wilson_interval(successes, count, z=1.959963984540054):
    if count == 0:
        return math.nan, math.nan
    rate = successes / count
    denominator = 1.0 + z * z / count
    center = (rate + z * z / (2.0 * count)) / denominator
    radius = z * math.sqrt(
        rate * (1.0 - rate) / count + z * z / (4.0 * count * count)
    ) / denominator
    return max(0.0, center - radius), min(1.0, center + radius)


def exact_mcnemar(candidate_only, baseline_only):
    discordant = candidate_only + baseline_only
    if discordant == 0:
        return 1.0
    tail = min(candidate_only, baseline_only)
    probability = sum(
        math.comb(discordant, value) for value in range(tail + 1)
    ) / (2.0 ** discordant)
    return min(1.0, 2.0 * probability)


def scenario_key(row):
    return tuple(row[field] for field in SCENARIO_FIELDS)


def aggregate_summary(scope, value, group):
    candidate_successes = sum(int(row["candidate_success"]) for row in group)
    baseline_successes = sum(int(row["baseline_success"]) for row in group)
    candidate_converged = sum(int(row["candidate_converged"]) for row in group)
    baseline_converged = sum(int(row["baseline_converged"]) for row in group)
    both = sum(row["paired_outcome"] == "both_success" for row in group)
    candidate_only = sum(row["paired_outcome"] == "candidate_only" for row in group)
    baseline_only = sum(row["paired_outcome"] == "baseline_only" for row in group)
    candidate_ci = wilson_interval(candidate_successes, len(group))
    baseline_ci = wilson_interval(baseline_successes, len(group))
    candidate_times = [
        as_float(row, "candidate_time_s") for row in group if row["candidate_success"]
    ]
    baseline_times = [
        as_float(row, "baseline_time_s") for row in group if row["baseline_success"]
    ]
    speedups = [as_float(row, "baseline_over_candidate") for row in group]
    absolute_gaps = [
        abs(as_float(row, "candidate_objective_gap_percent")) for row in group
    ]
    return {
        "scope": scope,
        "value": value,
        "paired_attempts": len(group),
        "candidate_successes": candidate_successes,
        "candidate_success_rate": candidate_successes / len(group),
        "candidate_success_ci95_low": candidate_ci[0],
        "candidate_success_ci95_high": candidate_ci[1],
        "candidate_strict_convergences": candidate_converged,
        "candidate_strict_convergence_rate": candidate_converged / len(group),
        "baseline_successes": baseline_successes,
        "baseline_success_rate": baseline_successes / len(group),
        "baseline_success_ci95_low": baseline_ci[0],
        "baseline_success_ci95_high": baseline_ci[1],
        "baseline_strict_convergences": baseline_converged,
        "baseline_strict_convergence_rate": baseline_converged / len(group),
        "both_success": both,
        "candidate_only": candidate_only,
        "baseline_only": baseline_only,
        "neither_success": len(group) - both - candidate_only - baseline_only,
        "mcnemar_exact_p": exact_mcnemar(candidate_only, baseline_only),
        "candidate_time_median_s": median(candidate_times),
        "baseline_time_median_s": median(baseline_times),
        "baseline_over_candidate_median": median(speedups),
        "baseline_over_candidate_q25": quantile(speedups, 0.25),
        "baseline_over_candidate_q75": quantile(speedups, 0.75),
        "candidate_absolute_objective_gap_median_percent": median(absolute_gaps),
        "candidate_absolute_objective_gap_p95_percent": quantile(absolute_gaps, 0.95),
        "candidate_absolute_objective_gap_max_percent": finite_max(absolute_gaps),
        "candidate_maximum_terminal_error": finite_max(
            as_float(row, "candidate_terminal_error") for row in group
            if row["candidate_success"]
        ),
        "candidate_minimum_pair_clearance": finite_min(
            as_float(row, "candidate_pair_clearance") for row in group
            if row["candidate_success"]
        ),
        "candidate_minimum_obstacle_clearance": finite_min((
            as_float(row, "candidate_obstacle_clearance") for row in group
            if row["candidate_success"]
        ), 1.0e10),
        "candidate_maximum_dynamics_defect": finite_max(
            as_float(row, "candidate_dynamics_defect") for row in group
            if row["candidate_success"]
        ),
        "candidate_maximum_agent_degree": finite_max(
            as_float(row, "candidate_maximum_agent_degree") for row in group
        ),
        "candidate_maximum_local_qp_variables": finite_max(
            as_float(row, "candidate_local_qp_variables") for row in group
        ),
        "centralized_maximum_qp_variables": finite_max(
            as_float(row, "centralized_qp_variables") for row in group
        ),
        "centralized_maximum_qp_constraints": finite_max(
            as_float(row, "centralized_qp_constraints") for row in group
        ),
    }


def write_csv(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        return
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    temporary.replace(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("results", type=Path)
    parser.add_argument("--inventory", type=Path, required=True)
    parser.add_argument("--execution-status", type=Path, required=True)
    parser.add_argument("--candidate", default="full")
    parser.add_argument("--baseline", default="centralized_osqp")
    parser.add_argument("--pairs-output", type=Path, required=True)
    parser.add_argument("--summary-output", type=Path, required=True)
    parser.add_argument("--aggregate-output", type=Path)
    arguments = parser.parse_args()

    results = read_csv(arguments.results)
    inventory = read_csv(arguments.inventory)
    statuses = read_csv(arguments.execution_status)
    width = max(3, len(str(max(int(row["scenario_index"]) for row in inventory))))
    inventory_by_selector = {
        f"scenario_{int(row['scenario_index']):0{width}d}": row for row in inventory
    }
    result_by_key_method = {
        (scenario_key(row), row["method"]): row for row in results
    }
    status_by_selector_method = {
        (row["selector"], row["method"]): row for row in statuses
    }

    paired_rows = []
    for selector, metadata in sorted(inventory_by_selector.items()):
        candidate_status = status_by_selector_method.get((selector, arguments.candidate))
        baseline_status = status_by_selector_method.get((selector, arguments.baseline))
        if candidate_status is None or baseline_status is None:
            continue
        key = scenario_key(metadata)
        candidate = result_by_key_method.get((key, arguments.candidate))
        baseline = result_by_key_method.get((key, arguments.baseline))
        candidate_converged = candidate is not None and candidate.get("converged") == "1"
        baseline_converged = baseline is not None and baseline.get("converged") == "1"
        candidate_success = candidate is not None and candidate.get("protocol_success") == "1"
        baseline_success = baseline is not None and baseline.get("protocol_success") == "1"
        if candidate_success and baseline_success:
            outcome = "both_success"
        elif candidate_success:
            outcome = "candidate_only"
        elif baseline_success:
            outcome = "baseline_only"
        else:
            outcome = "neither_success"
        candidate_time = as_float(candidate or {}, "solve_time_ms") / 1000.0
        baseline_time = as_float(baseline or {}, "solve_time_ms") / 1000.0
        speedup = (
            baseline_time / candidate_time
            if candidate_success and baseline_success and candidate_time > 0.0
            else math.nan
        )
        baseline_objective = as_float(baseline or {}, "objective")
        candidate_objective = as_float(candidate or {}, "objective")
        objective_gap = (
            100.0 * (candidate_objective - baseline_objective) / abs(baseline_objective)
            if candidate_success and baseline_success and baseline_objective != 0.0
            else math.nan
        )
        paired_rows.append({
            "selector": selector,
            **{field: metadata[field] for field in SCENARIO_FIELDS},
            "candidate": arguments.candidate,
            "baseline": arguments.baseline,
            "candidate_execution_status": candidate_status["execution_status"],
            "baseline_execution_status": baseline_status["execution_status"],
            "candidate_success": int(candidate_success),
            "candidate_converged": int(candidate_converged),
            "baseline_success": int(baseline_success),
            "baseline_converged": int(baseline_converged),
            "paired_outcome": outcome,
            "candidate_time_s": candidate_time,
            "baseline_time_s": baseline_time,
            "baseline_over_candidate": speedup,
            "candidate_objective_gap_percent": objective_gap,
            "candidate_active_pairs": (candidate or {}).get("maximum_active_pairs", ""),
            "candidate_potential_pairs": (candidate or {}).get("maximum_potential_pairs", ""),
            "candidate_maximum_agent_degree": (candidate or {}).get("maximum_agent_degree", ""),
            "candidate_terminal_error": (candidate or {}).get("maximum_terminal_error", ""),
            "candidate_pair_clearance": (candidate or {}).get("minimum_pairwise_clearance", ""),
            "candidate_obstacle_clearance": (candidate or {}).get("minimum_obstacle_clearance", ""),
            "candidate_dynamics_defect": (candidate or {}).get("maximum_dynamics_defect", ""),
            "baseline_terminal_error": (baseline or {}).get("maximum_terminal_error", ""),
            "baseline_pair_clearance": (baseline or {}).get("minimum_pairwise_clearance", ""),
            "baseline_obstacle_clearance": (baseline or {}).get("minimum_obstacle_clearance", ""),
            "baseline_dynamics_defect": (baseline or {}).get("maximum_dynamics_defect", ""),
            "candidate_local_qp_variables": (candidate or {}).get("maximum_local_qp_variables", ""),
            "candidate_local_qp_constraints": (candidate or {}).get("maximum_local_qp_constraints", ""),
            "centralized_qp_variables": (baseline or {}).get("centralized_qp_variables", ""),
            "centralized_qp_constraints": (baseline or {}).get("centralized_qp_constraints", ""),
        })

    groups = defaultdict(list)
    for row in paired_rows:
        groups[(row["n"], row["m"], row["composition"], row["family"])].append(row)
    summaries = []
    for (n, m, composition, family), group in sorted(groups.items()):
        candidate_successes = sum(int(row["candidate_success"]) for row in group)
        candidate_converged = sum(int(row["candidate_converged"]) for row in group)
        baseline_successes = sum(int(row["baseline_success"]) for row in group)
        baseline_converged = sum(int(row["baseline_converged"]) for row in group)
        both = sum(row["paired_outcome"] == "both_success" for row in group)
        candidate_only = sum(row["paired_outcome"] == "candidate_only" for row in group)
        baseline_only = sum(row["paired_outcome"] == "baseline_only" for row in group)
        candidate_ci = wilson_interval(candidate_successes, len(group))
        baseline_ci = wilson_interval(baseline_successes, len(group))
        candidate_times = [as_float(row, "candidate_time_s") for row in group if row["candidate_success"]]
        baseline_times = [as_float(row, "baseline_time_s") for row in group if row["baseline_success"]]
        speedups = [as_float(row, "baseline_over_candidate") for row in group]
        objective_gaps = [as_float(row, "candidate_objective_gap_percent") for row in group]
        absolute_gaps = [
            abs(as_float(row, "candidate_objective_gap_percent")) for row in group
        ]
        summaries.append({
            "n": n,
            "m": m,
            "composition": composition,
            "family": family,
            "paired_attempts": len(group),
            "candidate_successes": candidate_successes,
            "candidate_success_rate": candidate_successes / len(group),
            "candidate_success_ci95_low": candidate_ci[0],
            "candidate_success_ci95_high": candidate_ci[1],
            "candidate_strict_convergences": candidate_converged,
            "candidate_strict_convergence_rate": candidate_converged / len(group),
            "baseline_successes": baseline_successes,
            "baseline_success_rate": baseline_successes / len(group),
            "baseline_success_ci95_low": baseline_ci[0],
            "baseline_success_ci95_high": baseline_ci[1],
            "baseline_strict_convergences": baseline_converged,
            "baseline_strict_convergence_rate": baseline_converged / len(group),
            "both_success": both,
            "candidate_only": candidate_only,
            "baseline_only": baseline_only,
            "neither_success": len(group) - both - candidate_only - baseline_only,
            "mcnemar_exact_p": exact_mcnemar(candidate_only, baseline_only),
            "candidate_time_median_s": median(candidate_times),
            "baseline_time_median_s": median(baseline_times),
            "baseline_over_candidate_median": median(speedups),
            "baseline_over_candidate_q25": quantile(speedups, 0.25),
            "baseline_over_candidate_q75": quantile(speedups, 0.75),
            "candidate_objective_gap_median_percent": median(objective_gaps),
            "candidate_absolute_objective_gap_p95_percent": quantile(absolute_gaps, 0.95),
            "candidate_absolute_objective_gap_max_percent": finite_max(absolute_gaps),
            "candidate_maximum_agent_degree_median": median(
                as_float(row, "candidate_maximum_agent_degree") for row in group
            ),
            "candidate_local_qp_variables_median": median(
                as_float(row, "candidate_local_qp_variables") for row in group
            ),
            "centralized_qp_variables_median": median(
                as_float(row, "centralized_qp_variables") for row in group
            ),
        })

    aggregate_groups = [("overall", "all", paired_rows)]
    for n in sorted({row["n"] for row in paired_rows}, key=int):
        aggregate_groups.append(("n", n, [row for row in paired_rows if row["n"] == n]))
    for composition in sorted({row["composition"] for row in paired_rows}):
        aggregate_groups.append((
            "composition", composition,
            [row for row in paired_rows if row["composition"] == composition],
        ))
    obstacle_groups = {label: [] for label in ("0", "n", "2n")}
    for row in paired_rows:
        n = int(row["n"])
        m = int(row["m"])
        ratio = "0" if m == 0 else "n" if m == n else "2n"
        obstacle_groups[ratio].append(row)
    for ratio, group in obstacle_groups.items():
        if group:
            aggregate_groups.append(("obstacle_ratio", ratio, group))
    aggregates = [
        aggregate_summary(scope, value, group)
        for scope, value, group in aggregate_groups
    ]

    write_csv(arguments.pairs_output, paired_rows)
    write_csv(arguments.summary_output, summaries)
    if arguments.aggregate_output:
        write_csv(arguments.aggregate_output, aggregates)
    print(f"paired={len(paired_rows)} cells={len(summaries)}")
    print(f"pairs={arguments.pairs_output}")
    print(f"summary={arguments.summary_output}")
    if arguments.aggregate_output:
        print(f"aggregate={arguments.aggregate_output}")


if __name__ == "__main__":
    main()
