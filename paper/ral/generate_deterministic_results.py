#!/usr/bin/env python3
"""Generate a timeout-aware deterministic table from a complete frozen run."""

import argparse
import csv
import hashlib
import math
import statistics
from pathlib import Path


LABELS = {
    "easy_open": "Easy open",
    "easy_single_blocker": "Easy blocker",
    "medium_doorway": "Medium doorway",
    "hard_heterogeneous_doorway": "Heterog. doorway",
    "hard_warehouse": "Warehouse",
    "very_hard_maze": "Maze",
}
METHOD_LABELS = {
    "full": "TurboADMM-NL",
    "centralized_osqp": "OSQP",
    "centralized_qpoases": "qpOASES",
}


def comma_list(value):
    values = tuple(item.strip() for item in value.split(",") if item.strip())
    if not values or len(values) != len(set(values)):
        raise argparse.ArgumentTypeError("expected a nonempty unique comma-separated list")
    return values


def read_csv(path):
    with path.open(newline="", encoding="utf-8-sig") as stream:
        return list(csv.DictReader(stream))


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def scientific_latex(value):
    if value == 0.0:
        return "0"
    exponent = math.floor(math.log10(abs(value)))
    coefficient = value / (10.0 ** exponent)
    return rf"{coefficient:.2f}\times10^{{{exponent}}}"


def number(row, field):
    try:
        value = float(row[field])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"missing numeric field {field}") from error
    if not math.isfinite(value):
        raise ValueError(f"non-finite numeric field {field}")
    return value


def integer(row, field):
    value = number(row, field)
    if value != round(value):
        raise ValueError(f"field {field} is not an integer")
    return int(value)


def index_unique(rows, key_fields, description):
    indexed = {}
    for row in rows:
        key = tuple(row.get(field, "") for field in key_fields)
        if key in indexed:
            raise ValueError(f"duplicate {description}: {key}")
        indexed[key] = row
    return indexed


def format_cell(successes, repetitions, wall_times):
    median = f"{statistics.median(wall_times):.2f}" if wall_times else "--"
    return f"{successes}/{repetitions}; {median}"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("results", type=Path)
    parser.add_argument("execution_status", type=Path)
    parser.add_argument("--cases", type=comma_list, required=True)
    parser.add_argument("--expected-methods", type=comma_list, required=True)
    parser.add_argument(
        "--reported-methods", type=comma_list,
        default=comma_list("full,centralized_osqp,centralized_qpoases"))
    parser.add_argument("--repetitions", type=int, required=True)
    parser.add_argument("--output-csv", type=Path, required=True)
    parser.add_argument("--output-tex", type=Path, required=True)
    arguments = parser.parse_args()
    if arguments.repetitions <= 0:
        parser.error("--repetitions must be positive")
    if not set(arguments.reported_methods) <= set(arguments.expected_methods):
        parser.error("reported methods must be a subset of expected methods")
    if arguments.reported_methods[0] != "full":
        parser.error("the first reported method must be full")

    result_rows = read_csv(arguments.results)
    status_rows = read_csv(arguments.execution_status)
    expected = {
        (case, method, str(repetition))
        for case in arguments.cases
        for method in arguments.expected_methods
        for repetition in range(1, arguments.repetitions + 1)
    }
    relevant_status = [
        row for row in status_rows
        if row.get("selector") in arguments.cases
        and row.get("method") in arguments.expected_methods
    ]
    status_by_key = index_unique(
        relevant_status, ("selector", "method", "repetition"), "status row")
    missing = sorted(expected - set(status_by_key))
    unexpected = sorted(set(status_by_key) - expected)
    if missing or unexpected:
        raise ValueError(
            f"incomplete deterministic schedule: missing={len(missing)}, "
            f"unexpected={len(unexpected)}")
    errors = [
        (key, row.get("execution_status"))
        for key, row in status_by_key.items()
        if row.get("execution_status") not in ("completed", "timeout")
    ]
    if errors:
        raise ValueError(f"deterministic schedule contains execution errors: {errors[:5]}")

    relevant_results = [
        row for row in result_rows
        if row.get("case_id") in arguments.cases
        and row.get("method") in arguments.expected_methods
    ]
    results_by_key = index_unique(
        relevant_results, ("case_id", "method", "repetition"), "result row")
    completed = {
        key for key, row in status_by_key.items()
        if row.get("execution_status") == "completed"
    }
    if set(results_by_key) != completed:
        raise ValueError(
            "result rows do not match completed executions: "
            f"missing={len(completed - set(results_by_key))}, "
            f"unexpected={len(set(results_by_key) - completed)}")

    summaries = []
    tex_rows = []
    for case in arguments.cases:
        metadata = [row for row in relevant_results if row.get("case_id") == case]
        if not metadata:
            raise ValueError(f"case {case} has no completed execution")
        dimensions = {(integer(row, "n"), integer(row, "m")) for row in metadata}
        if len(dimensions) != 1:
            raise ValueError(f"case {case} has inconsistent dimensions")
        agents, obstacles = dimensions.pop()
        record = {"case_id": case, "n": agents, "m": obstacles}
        cells = []
        for method in arguments.reported_methods:
            rows = [
                results_by_key[(case, method, str(repetition))]
                for repetition in range(1, arguments.repetitions + 1)
                if (case, method, str(repetition)) in results_by_key
            ]
            successful = [row for row in rows if row.get("protocol_success") == "1"]
            walls = [number(row, "wall_time_seconds") for row in successful]
            strict = sum(row.get("converged") == "1" for row in rows)
            record.update({
                f"{method}_successes": len(successful),
                f"{method}_strict_convergences": strict,
                f"{method}_timeouts": sum(
                    status_by_key[(case, method, str(repetition))].get(
                        "execution_status") == "timeout"
                    for repetition in range(1, arguments.repetitions + 1)),
                f"{method}_successful_wall_median_seconds": (
                    statistics.median(walls) if walls else math.nan),
            })
            cells.append(format_cell(len(successful), arguments.repetitions, walls))

        gaps = []
        for repetition in range(1, arguments.repetitions + 1):
            candidate = results_by_key.get((case, "full", str(repetition)))
            baseline = results_by_key.get(
                (case, "centralized_osqp", str(repetition)))
            if (candidate is None or baseline is None
                    or candidate.get("protocol_success") != "1"
                    or baseline.get("protocol_success") != "1"):
                continue
            candidate_objective = number(candidate, "objective")
            baseline_objective = number(baseline, "objective")
            if baseline_objective == 0.0:
                raise ValueError("cannot compute an objective gap from a zero objective")
            gaps.append(
                100.0 * (candidate_objective - baseline_objective)
                / abs(baseline_objective))
        record["joint_osqp_comparisons"] = len(gaps)
        record["maximum_absolute_osqp_objective_gap_percent"] = (
            max(map(abs, gaps)) if gaps else math.nan)
        summaries.append(record)
        label = LABELS.get(case, case.replace("_", " ").title())
        tex_rows.append(
            f"{label} ({agents}/{obstacles}) & " + " & ".join(cells) + r" \\")

    candidate_rows = [
        row for row in relevant_results
        if row.get("method") == "full" and row.get("protocol_success") == "1"
    ]
    candidate_expected = len(arguments.cases) * arguments.repetitions
    if len(candidate_rows) != candidate_expected:
        raise ValueError(
            f"TurboADMM-NL has {len(candidate_rows)}/{candidate_expected} valid runs")
    strict = sum(row.get("converged") == "1" for row in candidate_rows)
    if strict != candidate_expected:
        raise ValueError(
            f"TurboADMM-NL has {strict}/{candidate_expected} strict convergences")
    objective_gaps = [
        row["maximum_absolute_osqp_objective_gap_percent"] for row in summaries
        if math.isfinite(row["maximum_absolute_osqp_objective_gap_percent"])
    ]
    if not objective_gaps:
        raise ValueError("no jointly valid TurboADMM-NL/OSQP objective comparisons")

    arguments.output_csv.parent.mkdir(parents=True, exist_ok=True)
    with arguments.output_csv.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(summaries[0]))
        writer.writeheader()
        writer.writerows(summaries)

    minimum_pair = min(number(row, "minimum_pairwise_clearance")
                       for row in candidate_rows)
    minimum_obstacle = min(number(row, "minimum_obstacle_clearance")
                           for row in candidate_rows)
    maximum_dynamics = max(number(row, "maximum_dynamics_defect")
                           for row in candidate_rows)
    maximum_terminal = max(number(row, "maximum_terminal_error")
                           for row in candidate_rows)
    lines = [
        "% Generated by paper/ral/generate_deterministic_results.py; do not edit.",
        f"% {arguments.results.name} sha256={sha256(arguments.results)}",
        f"% {arguments.execution_status.name} sha256={sha256(arguments.execution_status)}",
        rf"\newcommand{{\DeterministicRuns}}{{{candidate_expected}}}",
        rf"\newcommand{{\DeterministicStrict}}{{{strict}}}",
        rf"\newcommand{{\DeterministicMaximumObjectiveGapPercent}}{{{max(objective_gaps):.3f}}}",
        rf"\newcommand{{\DeterministicMinimumPairClearance}}{{{scientific_latex(minimum_pair)}}}",
        rf"\newcommand{{\DeterministicMinimumObstacleClearance}}{{{scientific_latex(minimum_obstacle)}}}",
        rf"\newcommand{{\DeterministicMaximumDynamicsDefect}}{{{scientific_latex(maximum_dynamics)}}}",
        rf"\newcommand{{\DeterministicMaximumTerminalError}}{{{scientific_latex(maximum_terminal)}}}",
        "% Each cell is valid runs/repetitions; median wall time [s] on valid runs.",
        r"\newcommand{\DeterministicTableRows}{%",
        *tex_rows,
        "}",
    ]
    arguments.output_tex.parent.mkdir(parents=True, exist_ok=True)
    arguments.output_tex.write_text("\n".join(lines) + "\n", encoding="ascii")
    print(
        f"generated {arguments.output_csv} and {arguments.output_tex}; "
        f"TurboADMM-NL strict={strict}/{candidate_expected}, "
        f"max |objective gap|={max(objective_gaps):.6g}%")


if __name__ == "__main__":
    main()
