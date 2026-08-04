#!/usr/bin/env python3
"""Generate predeclared CSDO passing-bay development/evaluation families."""

import argparse
import csv
import math
import pathlib

from generate_csdo_congested_scenarios import write_instance


FAMILIES = {
    "development": (
        (7.2, 8.1, 9.0, 9.9),
        (-1.0, 0.0, 1.0),
        "passing_bay_recovery_development",
    ),
    "evaluation": (
        (7.5, 8.4, 9.3, 10.2),
        (-1.5, -0.5, 0.5, 1.5),
        "passing_bay_recovery_evaluation",
    ),
}
PRIORITIES = (("forward", (0, 1)), ("reverse", (1, 0)))
CENTER_X = 15.05


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output-dir",
        type=pathlib.Path,
        default=pathlib.Path("benchmarks/instances/csdo/passing_bay_sweep"),
    )
    parser.add_argument(
        "--split", choices=tuple(FAMILIES), default="development")
    return parser.parse_args()


def wall_x_coordinates():
    return [0.8 + 1.5 * index for index in range(19)] + [29.2]


def obstacles(bay_length):
    left = CENTER_X - 0.5 * bay_length
    right = CENTER_X + 0.5 * bay_length
    lower = [(x, 4.0, 0.8) for x in wall_x_coordinates()]
    upper = [(x, 10.0, 0.8) for x in wall_x_coordinates()
             if x < left or x > right]
    ceiling = []
    x = left + 0.3
    while x <= right - 0.3 + 1.0e-9:
        ceiling.append((x, 14.2, 0.8))
        x += 1.5
    left_side = [(left, y, 0.8) for y in (11.0, 12.5, 14.0)]
    right_side = [(right, y, 0.8) for y in (11.0, 12.5, 14.0)]
    return lower + upper + ceiling + left_side + right_side


def agents(arrival_offset, order):
    physical_agents = [
        ((5.0, 7.0, 0.0), (25.0, 7.0, 0.0)),
        ((25.0 + arrival_offset, 7.0, math.pi),
         (5.0, 7.0, math.pi)),
    ]
    return [physical_agents[index] for index in order]


def tenths_label(value):
    prefix = "m" if value < 0.0 else "p"
    return f"{prefix}{abs(round(10.0 * value)):02d}"


def main():
    args = parse_args()
    output_dir = args.output_dir
    bay_lengths, arrival_offsets, family = FAMILIES[args.split]
    output_dir.mkdir(parents=True, exist_ok=True)
    rows = []
    for bay_length in bay_lengths:
        for arrival_offset in arrival_offsets:
            for priority, order in PRIORITIES:
                stem = (
                    f"passing_bay_l{round(10.0 * bay_length):02d}_"
                    f"o{tenths_label(arrival_offset)}_{priority}"
                )
                filename = stem + ".yaml"
                write_instance(
                    output_dir / filename,
                    (30, 18),
                    agents(arrival_offset, order),
                    obstacles(bay_length),
                )
                rows.append((
                    family,
                    bay_length,
                    arrival_offset,
                    priority,
                    2,
                    filename,
                    "recovery",
                ))
    with (output_dir / f"{args.split}_manifest.csv").open(
            "w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow((
            "family", "bay_length", "arrival_offset", "priority",
            "agents", "instance", "mode",
        ))
        writer.writerows(rows)


if __name__ == "__main__":
    main()
