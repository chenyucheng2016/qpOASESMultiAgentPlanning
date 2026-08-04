#!/usr/bin/env python3
"""Generate the frozen packed CSDO/TurboADMM-NL interaction suite."""

import argparse
import csv
import math
import pathlib


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output-dir",
        type=pathlib.Path,
        default=pathlib.Path("benchmarks/instances/csdo/congested"),
    )
    return parser.parse_args()


def inclusive_range(first, last, step=1.5):
    values = []
    value = first
    while value <= last + 1.0e-9:
        values.append(round(value, 10))
        value += step
    return values


def passing_bay_obstacles():
    obstacles = []
    for x in inclusive_range(0.8, 41.2):
        obstacles.append((x, 4.0, 0.8))
        if not 16.0 <= x <= 26.0:
            obstacles.append((x, 10.0, 0.8))
    for x in inclusive_range(17.3, 24.8):
        obstacles.append((x, 14.2, 0.8))
    for y in (11.0, 12.5, 14.0):
        obstacles.append((17.0, y, 0.8))
        obstacles.append((25.1, y, 0.8))
    return obstacles


def room_passage_obstacles():
    obstacles = []
    for x in inclusive_range(13.3, 26.8):
        obstacles.append((x, 4.0, 0.8))
    for x in (13.3, 14.8, 25.3, 26.8):
        obstacles.append((x, 10.0, 0.8))
    for x in inclusive_range(16.3, 23.8):
        obstacles.append((x, 14.2, 0.8))
    for y in (11.0, 12.5, 14.0):
        obstacles.append((16.0, y, 0.8))
        obstacles.append((24.1, y, 0.8))
    return obstacles


def convoy_agents(count):
    east_count = (count + 1) // 2
    west_count = count // 2
    east_starts = [4.0 + 5.0 * index for index in range(east_count)]
    west_starts = [38.0 - 5.0 * index for index in range(west_count)]
    east_goals = [38.0 - 5.0 * (east_count - 1 - index)
                  for index in range(east_count)]
    west_goals = [4.0 + 5.0 * (west_count - 1 - index)
                  for index in range(west_count)]
    agents = []
    for start, goal in zip(east_starts, east_goals):
        agents.append(((start, 7.0, 0.0), (goal, 7.0, 0.0)))
    for start, goal in zip(west_starts, west_goals):
        agents.append(((start, 7.0, math.pi), (goal, 7.0, math.pi)))
    return agents


def room_agents(count):
    slots = {
        4: [(2.5, 3.0), (2.5, 12.0)],
        6: [(2.5, 3.0), (2.5, 12.0), (8.0, 7.0)],
        8: [(2.5, 3.0), (2.5, 12.0), (8.0, 3.0), (8.0, 12.0)],
    }[count]
    left = slots
    right = [(40.0 - x, y) for x, y in slots]
    agents = []
    for start, goal in zip(left, right):
        agents.append(((start[0], start[1], 0.0),
                       (goal[0], goal[1], 0.0)))
    for start, goal in zip(right, left):
        agents.append(((start[0], start[1], math.pi),
                       (goal[0], goal[1], math.pi)))
    return agents


def write_instance(path, dimensions, agents, obstacles):
    lines = ["agents:"]
    for index, (start, goal) in enumerate(agents):
        lines.extend([
            "  - start: [{:.10g}, {:.10g}, {:.16g}]".format(*start),
            f"    name: agent{index}",
            "    goal: [{:.10g}, {:.10g}, {:.16g}]".format(*goal),
        ])
    lines.extend([
        "map:",
        f"  dimensions: [{dimensions[0]}, {dimensions[1]}]",
        "  obstacles:",
    ])
    for obstacle in obstacles:
        lines.append("    - [{:.10g}, {:.10g}, {:.10g}]".format(*obstacle))
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main():
    output_dir = parse_args().output_dir
    output_dir.mkdir(parents=True, exist_ok=True)
    rows = []
    for count in (3, 4, 5):
        name = f"convoy_passing_bay_n{count}.yaml"
        write_instance(
            output_dir / name,
            (42, 18),
            convoy_agents(count),
            passing_bay_obstacles(),
        )
        rows.append(("convoy_passing_bay", count, name, "recovery"))
    for count in (4, 6, 8):
        name = f"packed_room_swap_n{count}.yaml"
        write_instance(
            output_dir / name,
            (40, 18),
            room_agents(count),
            room_passage_obstacles(),
        )
        rows.append(("packed_room_swap", count, name, "recovery"))

    with (output_dir / "manifest.csv").open(
            "w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(("family", "agents", "instance", "mode"))
        writer.writerows(rows)


if __name__ == "__main__":
    main()
