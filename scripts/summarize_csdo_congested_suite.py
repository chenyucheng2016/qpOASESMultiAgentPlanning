#!/usr/bin/env python3
"""Summarize every frozen congested case, including Turbo timeouts."""

import argparse
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
    return parser.parse_args()


def solution_metrics(instance, guess, path):
    if not path.exists():
        return None
    solution = load_yaml(path)
    schedule = solution.get("schedule")
    if not schedule or any(name not in schedule for name in
                           (agent.get("name", f"agent{index}")
                            for index, agent in enumerate(instance["agents"]))):
        return None
    return interaction_metrics(instance, guess, solution)


def main():
    args = parse_args()
    with args.manifest.open(newline="", encoding="utf-8") as stream:
        cases = list(csv.DictReader(stream))
    fields = [
        "instance",
        "csdo_output",
        "csdo_valid",
        "turbo_output",
        "turbo_valid",
        "csdo_dynamics_defect",
        "csdo_pair_clearance",
        "turbo_dynamics_defect",
        "turbo_pair_clearance",
        "csdo_arrival_sum",
        "turbo_arrival_sum",
    ]
    rows = []
    for case in cases:
        instance_path = args.manifest.parent / case["instance"]
        work_dir = args.work_root / instance_path.stem
        instance = load_yaml(instance_path)
        guess_path = work_dir / "root_guesses.yaml"
        if not guess_path.exists():
            guess_path = work_dir / "csdo_guesses.yaml"
        guess = load_yaml(guess_path)
        csdo_metrics = solution_metrics(
            instance, guess, work_dir / "csdo.yaml")
        turbo_metrics = solution_metrics(
            instance, guess, work_dir / "turbo.yaml")
        rows.append({
            "instance": instance_path.name,
            "csdo_output": csdo_metrics is not None,
            "csdo_valid": (independently_valid(csdo_metrics)
                           if csdo_metrics is not None else False),
            "turbo_output": turbo_metrics is not None,
            "turbo_valid": (independently_valid(turbo_metrics)
                            if turbo_metrics is not None else False),
            "csdo_dynamics_defect": (csdo_metrics["maximum_dynamics_defect"]
                                     if csdo_metrics is not None else None),
            "csdo_pair_clearance": (csdo_metrics["minimum_pairwise_clearance"]
                                    if csdo_metrics is not None else None),
            "turbo_dynamics_defect": (turbo_metrics["maximum_dynamics_defect"]
                                      if turbo_metrics is not None else None),
            "turbo_pair_clearance": (turbo_metrics["minimum_pairwise_clearance"]
                                     if turbo_metrics is not None else None),
            "csdo_arrival_sum": (csdo_metrics["sum_arrival_stages"]
                                 if csdo_metrics is not None else None),
            "turbo_arrival_sum": (turbo_metrics["sum_arrival_stages"]
                                  if turbo_metrics is not None else None),
        })
    args.output_csv.parent.mkdir(parents=True, exist_ok=True)
    with args.output_csv.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


if __name__ == "__main__":
    main()
