#!/usr/bin/env python3
"""Measure CSDO warm-start difficulty before TurboADMM-NL is invoked."""

import argparse
import csv
import math
import pathlib

from run_csdo_turbo_comparison import (
    circle_positions,
    load_yaml,
    trajectory_metrics,
)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--case-dirs", required=True, nargs="+", type=pathlib.Path)
    parser.add_argument("--output-csv", required=True, type=pathlib.Path)
    parser.add_argument("--substeps", type=int, default=10)
    return parser.parse_args()


def worst_pair(schedule, names, substeps):
    minimum = math.inf
    location = ""
    horizon = len(schedule[names[0]]) - 1
    for stage in range(horizon):
        for substep in range(substeps + 1):
            alpha = substep / substeps
            circles = {}
            for name in names:
                first = circle_positions(schedule[name][stage])
                second = circle_positions(schedule[name][stage + 1])
                circles[name] = [
                    (
                        a[0] + alpha * (b[0] - a[0]),
                        a[1] + alpha * (b[1] - a[1]),
                        a[2],
                    )
                    for a, b in zip(first, second)
                ]
            for first_index, first_name in enumerate(names):
                for second_name in names[first_index + 1:]:
                    for first_circle_index, first_circle in enumerate(
                            circles[first_name]):
                        for second_circle_index, second_circle in enumerate(
                                circles[second_name]):
                            clearance = math.hypot(
                                first_circle[0] - second_circle[0],
                                first_circle[1] - second_circle[1],
                            ) - first_circle[2] - second_circle[2]
                            if clearance < minimum:
                                minimum = clearance
                                location = (
                                    f"{first_name}:{first_circle_index}-"
                                    f"{second_name}:{second_circle_index}@"
                                    f"{stage}+{substep}/{substeps}")
    return minimum, location


def corridor_metrics(corridors):
    minimum_width = math.inf
    maximum_width = 0.0
    minimum_nominal_slack = math.inf
    zero_width_axes = 0
    total_axes = 0
    for boxes in corridors.values():
        for box in boxes:
            nominal_x, nominal_y, lower_x, upper_x, lower_y, upper_y = map(
                float, box)
            for nominal, lower, upper in (
                    (nominal_x, lower_x, upper_x),
                    (nominal_y, lower_y, upper_y)):
                width = upper - lower
                minimum_width = min(minimum_width, width)
                maximum_width = max(maximum_width, width)
                minimum_nominal_slack = min(
                    minimum_nominal_slack,
                    nominal - lower,
                    upper - nominal,
                )
                zero_width_axes += width <= 1.0e-9
                total_axes += 1
    return minimum_width, maximum_width, minimum_nominal_slack, zero_width_axes, total_axes


def per_agent_dynamics(instance, guess):
    schedule = guess["schedule"]
    defects = []
    for index, agent in enumerate(instance["agents"]):
        name = agent.get("name", f"agent{index}")
        single_instance = dict(instance)
        single_instance["agents"] = [agent]
        single_guess = {"schedule": {name: schedule[name]}}
        defects.append(trajectory_metrics(
            single_instance, single_guess, single_guess
        )["maximum_dynamics_defect"])
    return defects


def schedule_corridor_slack(schedule, corridors):
    minimum = math.inf
    for name, nodes in schedule.items():
        boxes = corridors[name]
        if len(boxes) != 2 * len(nodes):
            raise ValueError(
                f"{name} corridor has {len(boxes)} rows for {len(nodes)} states")
        for stage, node in enumerate(nodes):
            for circle, position in enumerate(circle_positions(node)):
                box = boxes[2 * stage + circle]
                x, y = position[:2]
                lower_x, upper_x, lower_y, upper_y = map(float, box[2:])
                minimum = min(
                    minimum,
                    x - lower_x,
                    upper_x - x,
                    y - lower_y,
                    upper_y - y,
                )
    return minimum


def main():
    args = parse_args()
    if args.substeps <= 0:
        raise SystemExit("--substeps must be positive")
    rows = []
    for case_dir in args.case_dirs:
        instance = load_yaml(case_dir / "instance.yaml")
        guess = load_yaml(case_dir / "root_guesses.yaml")
        corridors = load_yaml(case_dir / "root_corridors.yaml")
        metadata = load_yaml(case_dir / "root_metadata.yaml")
        schedule = guess["schedule"]
        names = [agent.get("name", f"agent{index}")
                 for index, agent in enumerate(instance["agents"])]
        metrics = trajectory_metrics(instance, guess, guess, args.substeps)
        pair_clearance, pair_location = worst_pair(
            schedule, names, args.substeps)
        widths = corridor_metrics(corridors)
        defects = per_agent_dynamics(instance, guess)
        actual_corridor_slack = schedule_corridor_slack(schedule, corridors)
        starts = ";".join(
            f"{name}=({float(agent['start'][0]):.1f},{float(agent['start'][1]):.1f})"
            for name, agent in zip(names, instance["agents"]))
        rows.append({
            "case": case_dir.name,
            "starts_by_assigned_name": starts,
            "horizon": metadata["horizon"],
            "pbs_success": metadata["pbs_success"],
            "warmstart_conflicting_pairs": metadata["warmstart_conflicting_pairs"],
            "static_corridors_legal": metadata["static_corridors_legal"],
            "objective": metrics["objective"],
            "maximum_dynamics_defect": metrics["maximum_dynamics_defect"],
            "per_agent_dynamics_defect": ";".join(
                f"{name}={defect:.9g}" for name, defect in zip(names, defects)),
            "minimum_pairwise_clearance": pair_clearance,
            "worst_pair_location": pair_location,
            "minimum_obstacle_clearance": metrics["minimum_exact_obstacle_clearance"],
            "minimum_corridor_width": widths[0],
            "maximum_corridor_width": widths[1],
            "minimum_nominal_corridor_slack": widths[2],
            "minimum_schedule_corridor_slack": actual_corridor_slack,
            "zero_width_corridor_axes": widths[3],
            "total_corridor_axes": widths[4],
        })
    args.output_csv.parent.mkdir(parents=True, exist_ok=True)
    with args.output_csv.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    for row in rows:
        print(
            f"{row['case']}: N={row['horizon']}, "
            f"dynamics={row['maximum_dynamics_defect']:.6g}, "
            f"pair={row['minimum_pairwise_clearance']:.6g}, "
            f"corridor_width={row['minimum_corridor_width']:.6g}")


if __name__ == "__main__":
    main()
