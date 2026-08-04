#!/usr/bin/env python3
"""Generate a frozen priority-order sweep of one packed physical problem."""

import argparse
import csv
import pathlib

from generate_csdo_packed_perturbations import BASE_AGENTS, original_obstacles
from generate_csdo_congested_scenarios import write_instance


ORDERS = [
    ("east_first", (0, 1, 2, 3, 4)),
    ("east_reverse", (2, 1, 0, 3, 4)),
    ("west_first", (3, 4, 0, 1, 2)),
    ("west_reverse_first", (4, 3, 0, 1, 2)),
    ("full_reverse", (4, 3, 2, 1, 0)),
    ("alternate_east", (0, 3, 1, 4, 2)),
    ("alternate_west", (3, 0, 4, 1, 2)),
    ("center_out", (1, 4, 2, 3, 0)),
]


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
    for name, order in ORDERS:
        filename = f"packed_priority_{name}.yaml"
        agents = [BASE_AGENTS[index] for index in order]
        write_instance(
            args.output_dir / filename,
            (40, 18),
            agents,
            original_obstacles(),
        )
        rows.append((name, filename, 5, "recovery"))
    with (args.output_dir / "priority_manifest.csv").open(
            "w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(("priority", "instance", "agents", "mode"))
        writer.writerows(rows)


if __name__ == "__main__":
    main()
