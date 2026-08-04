#!/usr/bin/env python3
"""Generate exact symmetries and priority orders of the packed CSDO case."""

import argparse
import csv
import math
import pathlib

from generate_csdo_packed_perturbations import BASE_AGENTS, original_obstacles
from generate_csdo_congested_scenarios import write_instance


WIDTH = 40.0
HEIGHT = 18.0


def wrap_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle


def transform_state(state, transform):
    x, y, yaw = state
    if transform == "mirror_x":
        return WIDTH - x, y, wrap_angle(math.pi - yaw)
    if transform == "mirror_y":
        return x, HEIGHT - y, wrap_angle(-yaw)
    if transform == "rotate_180":
        return WIDTH - x, HEIGHT - y, wrap_angle(yaw + math.pi)
    return state


def transform_obstacle(obstacle, transform):
    x, y, radius = obstacle
    if transform in ("mirror_x", "rotate_180"):
        x = WIDTH - x
    if transform in ("mirror_y", "rotate_180"):
        y = HEIGHT - y
    return x, y, radius


def transformed_agents(transform, order):
    transformed = [
        (transform_state(start, transform), transform_state(goal, transform))
        for start, goal in BASE_AGENTS
    ]
    return [transformed[index] for index in order]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output-dir",
        type=pathlib.Path,
        default=pathlib.Path("benchmarks/instances/csdo/congested"),
    )
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    cases = [
        ("base", "identity", (0, 1, 2, 3, 4)),
        ("mirror_x", "mirror_x", (0, 1, 2, 3, 4)),
        ("mirror_y", "mirror_y", (0, 1, 2, 3, 4)),
        ("rotate_180", "rotate_180", (0, 1, 2, 3, 4)),
        ("priority_reverse", "identity", (4, 3, 2, 1, 0)),
        ("priority_interleaved", "identity", (0, 3, 1, 4, 2)),
    ]
    rows = []
    for name, transform, order in cases:
        filename = f"packed_passing_bay_{name}.yaml"
        obstacles = [transform_obstacle(obstacle, transform)
                     for obstacle in original_obstacles()]
        write_instance(
            args.output_dir / filename,
            (int(WIDTH), int(HEIGHT)),
            transformed_agents(transform, order),
            obstacles,
        )
        rows.append((name, filename, 5, "recovery"))
    with (args.output_dir / "symmetry_manifest.csv").open(
            "w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(("variant", "instance", "agents", "mode"))
        writer.writerows(rows)


if __name__ == "__main__":
    main()
