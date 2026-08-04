#!/usr/bin/env python3
"""Compare a CSDO interpolated warm start with its exact bicycle rollout."""

import argparse
import copy
import math
import pathlib

from run_csdo_turbo_comparison import (
    DEG_TO_RAD,
    DT,
    load_yaml,
    trajectory_metrics,
)


def exact_rollout(instance, guess):
    result = {"schedule": {}}
    for index, agent in enumerate(instance["agents"]):
        name = agent.get("name", f"agent{index}")
        reference = guess["schedule"][name]
        nodes = [copy.deepcopy(reference[0])]
        for stage in range(len(reference) - 1):
            node = nodes[-1]
            control = reference[stage]
            x = float(node["x"])
            y = float(node["y"])
            yaw = float(node["yaw"])
            steer = float(node["steer"]) * DEG_TO_RAD
            speed = float(control["v"])
            omega = float(control["omega"]) * DEG_TO_RAD
            following = {
                "x": x + DT * speed * math.cos(yaw),
                "y": y + DT * speed * math.sin(yaw),
                "yaw": yaw + DT * speed * math.tan(steer),
                "steer": (steer + DT * omega) / DEG_TO_RAD,
                "t": stage + 1,
            }
            if stage + 1 < len(reference) - 1:
                following["v"] = float(reference[stage + 1]["v"])
                following["omega"] = float(reference[stage + 1]["omega"])
            nodes.append(following)
        result["schedule"][name] = nodes
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--instance", required=True, type=pathlib.Path)
    parser.add_argument("--guess", required=True, type=pathlib.Path)
    args = parser.parse_args()
    instance = load_yaml(args.instance)
    guess = load_yaml(args.guess)
    rollout = exact_rollout(instance, guess)
    for label, trajectory in (("interpolated", guess), ("exact_rollout", rollout)):
        metrics = trajectory_metrics(instance, guess, trajectory)
        print(label)
        for key, value in metrics.items():
            print(f"  {key}: {value}")


if __name__ == "__main__":
    main()
