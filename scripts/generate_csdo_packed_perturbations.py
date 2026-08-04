#!/usr/bin/env python3
"""Generate frozen goal perturbations of the packed passing-bay case."""

import argparse
import csv
import math
import pathlib

from generate_csdo_congested_scenarios import write_instance


BASE_AGENTS = [
    ((2.5, 7.0, 0.0), (29.5, 3.0, 0.0)),
    ((6.5, 7.0, 0.0), (33.5, 12.0, 0.0)),
    ((10.5, 7.0, 0.0), (37.5, 3.0, 0.0)),
    ((37.5, 7.0, math.pi), (2.5, 12.0, math.pi)),
    ((33.5, 7.0, math.pi), (6.5, 3.0, math.pi)),
]

GOAL_Y_OFFSETS = [
    (0.0, 0.0, 0.0, 0.0, 0.0),
    (0.2, -0.2, 0.2, -0.2, 0.2),
    (-0.3, 0.3, 0.0, 0.3, -0.3),
    (0.0, 0.25, -0.25, 0.25, -0.25),
    (0.4, 0.0, -0.4, -0.4, 0.4),
]


def original_obstacles():
    lower = [(x, 4.0, 0.8) for x in
             (13.3, 14.8, 16.3, 17.8, 19.3,
              20.8, 22.3, 23.8, 25.3, 26.8)]
    upper = [(x, 10.0, 0.8) for x in (13.3, 14.8, 25.3, 26.8)]
    ceiling = [(x, 14.2, 0.8) for x in
               (16.3, 17.8, 19.3, 20.8, 22.3, 23.8)]
    left = [(16.0, y, 0.8) for y in (11.0, 12.5, 14.0)]
    right = [(24.1, y, 0.8) for y in (11.0, 12.5, 14.0)]
    return lower + upper + ceiling + left + right


def perturbed_agents(offsets):
    agents = []
    for offset, (start, goal) in zip(offsets, BASE_AGENTS):
        agents.append((start, (goal[0], goal[1] + offset, goal[2])))
    return agents


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output-dir",
        type=pathlib.Path,
        default=pathlib.Path("benchmarks/instances/csdo/congested"),
    )
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    rows = []
    for seed, offsets in enumerate(GOAL_Y_OFFSETS):
        name = f"packed_passing_bay_goal_seed{seed}.yaml"
        write_instance(
            args.output_dir / name,
            (40, 18),
            perturbed_agents(offsets),
            original_obstacles(),
        )
        rows.append((seed, name, 5, "recovery"))
    with (args.output_dir / "goal_perturbation_manifest.csv").open(
            "w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(("seed", "instance", "agents", "mode"))
        writer.writerows(rows)


if __name__ == "__main__":
    main()
