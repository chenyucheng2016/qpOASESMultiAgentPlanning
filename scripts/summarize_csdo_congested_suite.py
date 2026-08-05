#!/usr/bin/env python3
"""Summarize every frozen congested case, including Turbo timeouts."""

import argparse
import collections
import csv
import pathlib

from run_csdo_interaction_comparison import (
    independently_valid,
    interaction_metrics,
)
from run_csdo_turbo_comparison import load_yaml


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", required=True, type=pathlib.Path)
    parser.add_argument("--work-root", required=True, type=pathlib.Path)
    parser.add_argument("--output-csv", required=True, type=pathlib.Path)
    parser.add_argument("--status-csv", type=pathlib.Path)
    parser.add_argument("--results-csv", type=pathlib.Path)
    return parser.parse_args()


def solution_metrics(instance, guess, path):
    if guess is None or not path.exists():
        return None
    solution = load_yaml(path)
    if not solution:
        return None
    schedule = solution.get("schedule")
    if not schedule or any(name not in schedule for name in
                           (agent.get("name", f"agent{index}")
                            for index, agent in enumerate(instance["agents"]))):
        return None
    return interaction_metrics(instance, guess, solution)


def optional_yaml(path):
    return load_yaml(path) if path.exists() else None


def paired_outcome(csdo_valid, turbo_valid):
    if turbo_valid and not csdo_valid:
        return "turbo_only"
    if csdo_valid and turbo_valid:
        return "both_valid"
    if csdo_valid:
        return "csdo_only"
    return "neither_valid"


def main():
    args = parse_args()
    with args.manifest.open(newline="", encoding="utf-8") as stream:
        cases = list(csv.DictReader(stream))
    manifest_fields = list(cases[0]) if cases else []
    status_path = args.status_csv or args.output_csv.with_name("results_status.csv")
    statuses = {}
    if status_path.exists():
        with status_path.open(newline="", encoding="utf-8") as stream:
            statuses = {row["instance"]: row for row in csv.DictReader(stream)}
    result_path = args.results_csv or args.output_csv.with_name("results.csv")
    case_results = {}
    if result_path.exists():
        with result_path.open(newline="", encoding="utf-8") as stream:
            case_results = {row["instance"]: row for row in csv.DictReader(stream)}
    fields = manifest_fields + [
        "runner_state",
        "runner_return_code",
        "csdo_return_code",
        "turbo_return_code",
        "csdo_wall_time",
        "turbo_primary_return_code",
        "turbo_primary_wall_time",
        "turbo_recovery_used",
        "turbo_corridor_recovery_window",
        "turbo_wall_time",
        "paired_outcome",
        "pbs_success",
        "warmstart_source",
        "root_conflicting_pairs",
        "warmstart_conflicting_pairs",
        "csdo_output",
        "csdo_valid",
        "turbo_output",
        "turbo_valid",
        "csdo_dynamics_defect",
        "csdo_pair_clearance",
        "csdo_obstacle_clearance",
        "csdo_terminal_position_error",
        "csdo_terminal_yaw_error",
        "turbo_dynamics_defect",
        "turbo_pair_clearance",
        "turbo_obstacle_clearance",
        "turbo_terminal_position_error",
        "turbo_terminal_yaw_error",
        "csdo_arrival_sum",
        "turbo_arrival_sum",
        "csdo_path_length",
        "turbo_path_length",
    ]
    rows = []
    for case in cases:
        instance_path = args.manifest.parent / case["instance"]
        work_dir = args.work_root / instance_path.stem
        instance = load_yaml(instance_path)
        guess_path = work_dir / "root_guesses.yaml"
        if not guess_path.exists():
            guess_path = work_dir / "csdo_guesses.yaml"
        guess = optional_yaml(guess_path)
        metadata = optional_yaml(work_dir / "root_metadata.yaml") or {}
        csdo_metrics = solution_metrics(
            instance, guess, work_dir / "csdo.yaml")
        turbo_metrics = solution_metrics(
            instance, guess, work_dir / "turbo.yaml")
        csdo_valid = (independently_valid(csdo_metrics)
                      if csdo_metrics is not None else False)
        turbo_valid = (independently_valid(turbo_metrics)
                       if turbo_metrics is not None else False)
        status = statuses.get(instance_path.name, {})
        case_result = case_results.get(instance_path.name, {})
        row = dict(case)
        row.update({
            "runner_state": status.get("state", "missing"),
            "runner_return_code": status.get("runner_return_code"),
            "csdo_return_code": case_result.get("csdo_return_code"),
            "turbo_return_code": case_result.get("turbo_return_code"),
            "turbo_primary_return_code": case_result.get(
                "turbo_primary_return_code"),
            "turbo_primary_wall_time": case_result.get(
                "turbo_primary_wall_time"),
            "turbo_recovery_used": case_result.get("turbo_recovery_used"),
            "turbo_corridor_recovery_window": case_result.get(
                "turbo_corridor_recovery_window"),
            "csdo_wall_time": case_result.get("csdo_wall_time"),
            "turbo_wall_time": case_result.get("turbo_wall_time"),
            "paired_outcome": paired_outcome(csdo_valid, turbo_valid),
            "pbs_success": metadata.get("pbs_success"),
            "warmstart_source": case_result.get(
                "warmstart_source") or metadata.get("warmstart_source"),
            "root_conflicting_pairs": case_result.get(
                "root_conflicting_pairs",
                metadata.get("root_conflicting_pairs")),
            "warmstart_conflicting_pairs": case_result.get(
                "warmstart_conflicting_pairs",
                metadata.get("warmstart_conflicting_pairs")),
            "csdo_output": csdo_metrics is not None,
            "csdo_valid": csdo_valid,
            "turbo_output": turbo_metrics is not None,
            "turbo_valid": turbo_valid,
            "csdo_dynamics_defect": (csdo_metrics["maximum_dynamics_defect"]
                                     if csdo_metrics is not None else None),
            "csdo_pair_clearance": (csdo_metrics["minimum_pairwise_clearance"]
                                    if csdo_metrics is not None else None),
            "csdo_obstacle_clearance": (
                csdo_metrics["minimum_exact_obstacle_clearance"]
                if csdo_metrics is not None else None),
            "csdo_terminal_position_error": (
                csdo_metrics["maximum_terminal_position_error"]
                if csdo_metrics is not None else None),
            "csdo_terminal_yaw_error": (csdo_metrics["maximum_terminal_yaw_error"]
                                        if csdo_metrics is not None else None),
            "turbo_dynamics_defect": (turbo_metrics["maximum_dynamics_defect"]
                                      if turbo_metrics is not None else None),
            "turbo_pair_clearance": (turbo_metrics["minimum_pairwise_clearance"]
                                     if turbo_metrics is not None else None),
            "turbo_obstacle_clearance": (
                turbo_metrics["minimum_exact_obstacle_clearance"]
                if turbo_metrics is not None else None),
            "turbo_terminal_position_error": (
                turbo_metrics["maximum_terminal_position_error"]
                if turbo_metrics is not None else None),
            "turbo_terminal_yaw_error": (turbo_metrics["maximum_terminal_yaw_error"]
                                         if turbo_metrics is not None else None),
            "csdo_arrival_sum": (csdo_metrics["sum_arrival_stages"]
                                 if csdo_metrics is not None else None),
            "turbo_arrival_sum": (turbo_metrics["sum_arrival_stages"]
                                  if turbo_metrics is not None else None),
            "csdo_path_length": (csdo_metrics["total_path_length"]
                                 if csdo_metrics is not None else None),
            "turbo_path_length": (turbo_metrics["total_path_length"]
                                  if turbo_metrics is not None else None),
        })
        rows.append(row)
    args.output_csv.parent.mkdir(parents=True, exist_ok=True)
    with args.output_csv.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    counts = collections.Counter(row["paired_outcome"] for row in rows)
    print(f"paired rows: {len(rows)}")
    for outcome, count in sorted(counts.items()):
        print(f"{outcome}: {count}")


if __name__ == "__main__":
    main()
