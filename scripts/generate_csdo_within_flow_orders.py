#!/usr/bin/env python3
"""Generate all within-convoy priority orders for the packed CSDO case."""

import argparse
import csv
import itertools
import pathlib

from generate_csdo_packed_perturbations import BASE_AGENTS, original_obstacles
from generate_csdo_congested_scenarios import write_instance


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
    case = 0
    for east in itertools.permutations((0, 1, 2)):
        for west in itertools.permutations((3, 4)):
            order = east + west
            filename = f"packed_within_flow_order{case:02d}.yaml"
            write_instance(
                args.output_dir / filename,
                (40, 18),
                [BASE_AGENTS[index] for index in order],
                original_obstacles(),
            )
            rows.append((case, "-".join(map(str, order)), filename, 5,
                         "recovery", "auto", 81))
            case += 1
    with (args.output_dir / "within_flow_manifest.csv").open(
            "w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow((
            "case", "order", "instance", "agents", "mode",
            "warmstart_policy", "minimum_horizon",
        ))
        writer.writerows(rows)


if __name__ == "__main__":
    main()
