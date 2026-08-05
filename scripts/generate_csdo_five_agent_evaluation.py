#!/usr/bin/env python3
"""Generate the disjoint, frozen five-agent CSDO evaluation family."""

import argparse
import csv
import pathlib

from generate_csdo_congested_scenarios import write_instance
from generate_csdo_packed_perturbations import BASE_AGENTS, original_obstacles


PRIORITY_ORDERS = (
    (0, 1, 2, 3, 4),
    (0, 1, 2, 4, 3),
    (0, 2, 1, 3, 4),
    (1, 0, 2, 4, 3),
    (1, 2, 0, 3, 4),
    (2, 0, 1, 4, 3),
    (2, 1, 0, 3, 4),
    (2, 1, 0, 4, 3),
)

# These perturbations are absent from every development instance. Convoy-wide
# start shifts preserve the zero-clearance but noncolliding four-metre initial
# spacing. Goal perturbations remain well inside the map boundary.
PROFILES = (
    {
        "name": "toward_gate",
        "east_start_x": 0.20,
        "west_start_x": -0.20,
        "goal_y": (0.15, -0.10, 0.20, -0.15, 0.10),
    },
    {
        "name": "away_from_gate",
        "east_start_x": -0.25,
        "west_start_x": 0.25,
        "goal_y": (-0.20, 0.15, -0.10, 0.20, -0.15),
    },
    {
        "name": "asymmetric",
        "east_start_x": 0.10,
        "west_start_x": 0.20,
        "goal_y": (0.10, 0.20, -0.15, -0.20, 0.15),
    },
)


def perturbed_agents(profile):
    agents = []
    for physical_id, (start, goal) in enumerate(BASE_AGENTS):
        start_shift = (profile["east_start_x"] if physical_id < 3
                       else profile["west_start_x"])
        shifted_start = (start[0] + start_shift, start[1], start[2])
        shifted_goal = (
            goal[0],
            goal[1] + profile["goal_y"][physical_id],
            goal[2],
        )
        agents.append((shifted_start, shifted_goal))
    return agents


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output-dir",
        type=pathlib.Path,
        default=pathlib.Path(
            "benchmarks/instances/csdo/five_agent_evaluation"),
    )
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    rows = []
    case = 0
    for profile in PROFILES:
        physical_agents = perturbed_agents(profile)
        for order in PRIORITY_ORDERS:
            filename = (
                f"five_agent_{profile['name']}_order{case:02d}.yaml")
            write_instance(
                args.output_dir / filename,
                (40, 18),
                [physical_agents[index] for index in order],
                original_obstacles(),
            )
            rows.append((
                case,
                "five_agent_packed",
                profile["name"],
                "-".join(map(str, order)),
                filename,
                5,
                "recovery",
                "auto",
                90,
            ))
            case += 1

    with (args.output_dir / "evaluation_manifest.csv").open(
            "w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow((
            "case", "family", "profile", "order", "instance", "agents",
            "mode", "warmstart_policy", "minimum_horizon",
        ))
        writer.writerows(rows)


if __name__ == "__main__":
    main()
