#!/usr/bin/env python3
"""Summarize TurboADMM-NL benchmark CSVs without third-party packages."""

import argparse
import csv
import math
import statistics
from collections import defaultdict


GROUP_FIELDS = ("case_id", "difficulty", "family", "density", "composition", "n", "m", "method")
MEDIAN_FIELDS = (
    "scp_iterations", "admm_iterations", "qp_solves",
    "maximum_active_pairs", "maximum_potential_pairs", "maximum_agent_degree",
    "maximum_active_obstacles_per_agent", "maximum_potential_obstacles_per_agent",
    "maximum_local_qp_variables", "maximum_local_qp_constraints",
    "centralized_qp_variables", "centralized_qp_constraints",
    "qp_build_time_ms", "pair_build_time_ms", "admm_assembly_time_ms",
    "local_qp_batch_time_ms", "local_qp_solve_time_ms",
    "consensus_time_ms", "globalization_time_ms",
)


def quantile(values, probability):
    if not values:
        return math.nan
    ordered = sorted(values)
    location = probability * (len(ordered) - 1)
    lower = int(math.floor(location))
    upper = int(math.ceil(location))
    if lower == upper:
        return ordered[lower]
    fraction = location - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


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


def as_float(row, field):
    try:
        return float(row[field])
    except (KeyError, TypeError, ValueError):
        return math.nan


def finite(values):
    return [value for value in values if math.isfinite(value)]


def summarize(rows):
    groups = defaultdict(list)
    for row in rows:
        groups[tuple(row[field] for field in GROUP_FIELDS)].append(row)

    output = []
    for key in sorted(groups):
        group = groups[key]
        successful = [
            row for row in group
            if row.get("protocol_success", "0") == "1"
        ]
        success_count = len(successful)
        ci_low, ci_high = wilson_interval(success_count, len(group))
        times = finite(as_float(row, "solve_time_ms") for row in successful)
        objectives = finite(as_float(row, "objective") for row in successful)
        backend_work = finite(as_float(row, "backend_iterations") for row in group)
        qp_work = finite(as_float(row, "qp_working_set_recalculations") for row in group)
        record = dict(zip(GROUP_FIELDS, key))
        record.update({
            "runs": len(group),
            "successes": success_count,
            "success_rate": success_count / len(group),
            "success_ci95_low": ci_low,
            "success_ci95_high": ci_high,
            "time_median_ms": statistics.median(times) if times else math.nan,
            "time_q25_ms": quantile(times, 0.25),
            "time_q75_ms": quantile(times, 0.75),
            "objective_median": statistics.median(objectives) if objectives else math.nan,
            "backend_iterations_median": statistics.median(backend_work) if backend_work else math.nan,
            "working_set_recalculations_median": statistics.median(qp_work) if qp_work else math.nan,
        })
        for field in MEDIAN_FIELDS:
            values = finite(as_float(row, field) for row in successful)
            record[f"{field}_median"] = statistics.median(values) if values else math.nan
        pair_fractions = finite(
            as_float(row, "maximum_active_pairs") / as_float(row, "maximum_potential_pairs")
            for row in successful
            if as_float(row, "maximum_potential_pairs") > 0.0
        )
        record["active_pair_fraction_median"] = (
            statistics.median(pair_fractions) if pair_fractions else math.nan
        )
        output.append(record)
    return output


def summarize_methods(rows, status_rows):
    results_by_method = defaultdict(list)
    status_by_method = defaultdict(list)
    for row in rows:
        results_by_method[row["method"]].append(row)
    for row in status_rows:
        status_by_method[row["method"]].append(row)
    output = []
    for method in sorted(set(results_by_method) | set(status_by_method)):
        results = results_by_method[method]
        statuses = status_by_method[method]
        successful = [row for row in results if row.get("protocol_success", "0") == "1"]
        attempted = len(statuses) if statuses else len(results)
        success_count = len(successful)
        ci_low, ci_high = wilson_interval(success_count, attempted)
        times = finite(as_float(row, "solve_time_ms") for row in successful)
        output.append({
            "method": method,
            "attempted": attempted,
            "completed": len(results),
            "timeouts": sum(row.get("execution_status") == "timeout" for row in statuses),
            "execution_errors": sum(row.get("execution_status") == "error" for row in statuses),
            "solver_successes": sum(row.get("solver_success", "0") == "1" for row in results),
            "validator_successes": sum(row.get("validator_success", "0") == "1" for row in results),
            "protocol_successes": success_count,
            "success_rate": success_count / attempted if attempted else math.nan,
            "success_ci95_low": ci_low,
            "success_ci95_high": ci_high,
            "successful_time_median_ms": statistics.median(times) if times else math.nan,
            "successful_time_q25_ms": quantile(times, 0.25),
            "successful_time_q75_ms": quantile(times, 0.75),
        })
    return output


def enrich_objective_gaps(rows):
    scenario_fields = (
        "suite", "case_id", "family", "density", "composition", "seed",
        "n", "m"
    )
    best = {}
    for row in rows:
        if row.get("protocol_success", "0") != "1":
            continue
        value = as_float(row, "objective")
        if not math.isfinite(value):
            continue
        key = tuple(row[field] for field in scenario_fields)
        best[key] = min(value, best.get(key, value))
    enriched = []
    for row in rows:
        copy = dict(row)
        key = tuple(row[field] for field in scenario_fields)
        objective = as_float(row, "objective")
        reference = best.get(key, math.nan)
        if math.isfinite(objective) and math.isfinite(reference):
            scale = max(abs(reference), 1.0e-12)
            copy["objective_gap_percent"] = 100.0 * (objective - reference) / scale
        else:
            copy["objective_gap_percent"] = math.nan
        enriched.append(copy)
    return enriched


def write_rows(path, rows):
    if not rows:
        return
    with open(path, "w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input", help="benchmark CSV produced by nonlinear_benchmark")
    parser.add_argument("--summary", default="nonlinear_benchmark_summary.csv")
    parser.add_argument("--enriched", default="", help="optional run-level CSV with objective gaps")
    parser.add_argument("--execution-status", default="",
                        help="optional execution_status.csv produced by the matrix runner")
    parser.add_argument("--method-summary", default="",
                        help="optional output with timeout-aware method aggregates")
    arguments = parser.parse_args()
    with open(arguments.input, newline="", encoding="utf-8-sig") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise SystemExit("benchmark CSV has no rows")
    summary = summarize(rows)
    write_rows(arguments.summary, summary)
    if arguments.enriched:
        write_rows(arguments.enriched, enrich_objective_gaps(rows))
    if arguments.method_summary:
        status_rows = []
        if arguments.execution_status:
            with open(arguments.execution_status, newline="", encoding="utf-8-sig") as stream:
                status_rows = list(csv.DictReader(stream))
        write_rows(arguments.method_summary, summarize_methods(rows, status_rows))
    total_successes = sum(row.get("protocol_success", "0") == "1" for row in rows)
    print(f"rows={len(rows)} successful={total_successes} groups={len(summary)}")
    print(f"summary={arguments.summary}")
    if arguments.method_summary:
        print(f"method_summary={arguments.method_summary}")


if __name__ == "__main__":
    main()
