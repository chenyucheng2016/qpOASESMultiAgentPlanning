#!/usr/bin/env python3
"""Run CSDO/Turbo interaction experiments with a shared CSDO front end."""

import argparse
import csv
import json
import math
import pathlib

from run_csdo_turbo_comparison import load_yaml, run, trajectory_metrics


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--mode",
        choices=("recovery", "conservatism", "joint_repair"),
        required=True)
    parser.add_argument("--instance", required=True, type=pathlib.Path)
    parser.add_argument("--csdo-executable", required=True, type=pathlib.Path)
    parser.add_argument("--turbo-executable", required=True, type=pathlib.Path)
    parser.add_argument("--root-exporter", required=True, type=pathlib.Path)
    parser.add_argument("--csdo-config", required=True, type=pathlib.Path)
    parser.add_argument("--work-dir", required=True, type=pathlib.Path)
    parser.add_argument("--output-csv", required=True, type=pathlib.Path)
    parser.add_argument("--threads", type=int, default=0)
    parser.add_argument(
        "--warmstart-policy",
        choices=("auto", "pbs_root", "independent"),
        default="auto")
    parser.add_argument("--pad-stages", type=int, default=0)
    parser.add_argument("--delay-from-agent", type=int, default=-1)
    parser.add_argument("--delay-stages", type=int, default=0)
    parser.add_argument("--sequential-delay-stages", type=int, default=0)
    parser.add_argument("--independent", action="store_true")
    parser.add_argument("--corridor-recovery-window", type=int, default=0)
    parser.add_argument("--minimum-horizon", type=int, default=0)
    parser.add_argument("--timeout", type=float, default=7200.0)
    return parser.parse_args()


def append_row(path, row):
    path.parent.mkdir(parents=True, exist_ok=True)
    exists = path.exists() and path.stat().st_size > 0
    with path.open("a", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(row))
        if not exists:
            writer.writeheader()
        writer.writerow(row)


def interaction_metrics(instance, guess, solution):
    metrics = trajectory_metrics(instance, guess, solution)
    path_lengths = []
    arrival_stages = []
    reference_deviations = []
    gate_intervals = []
    for index, agent in enumerate(instance["agents"]):
        name = agent.get("name", f"agent{index}")
        nodes = solution["schedule"][name]
        reference_nodes = guess["schedule"][name]
        reference_deviations.append(max(math.hypot(
            float(node["x"]) - float(reference["x"]),
            float(node["y"]) - float(reference["y"]),
        ) for node, reference in zip(nodes, reference_nodes)))
        path_lengths.append(sum(math.hypot(
            float(nodes[stage + 1]["x"]) - float(nodes[stage]["x"]),
            float(nodes[stage + 1]["y"]) - float(nodes[stage]["y"]),
        ) for stage in range(len(nodes) - 1)))
        goal_x, goal_y = map(float, agent["goal"][:2])
        arrival = len(nodes) - 1
        for stage in range(len(nodes)):
            if all(math.hypot(
                    float(node["x"]) - goal_x,
                    float(node["y"]) - goal_y) <= 0.05
                    for node in nodes[stage:]):
                arrival = stage
                break
        arrival_stages.append(arrival)

        occupied = [stage for stage, node in enumerate(nodes)
                    if 18.0 <= float(node["x"]) <= 24.0]
        gate_intervals.append(
            (min(occupied), max(occupied)) if occupied else None)

    horizon = len(next(iter(solution["schedule"].values())))
    occupancy = [sum(interval is not None
                     and interval[0] <= stage <= interval[1]
                     for interval in gate_intervals)
                 for stage in range(horizon)]
    active_intervals = [interval for interval in gate_intervals
                        if interval is not None]
    metrics.update({
        "total_path_length": sum(path_lengths),
        "maximum_arrival_stage": max(arrival_stages),
        "sum_arrival_stages": sum(arrival_stages),
        "first_agent_arrival_stage": arrival_stages[0],
        "second_agent_arrival_stage": arrival_stages[1]
            if len(arrival_stages) > 1 else arrival_stages[0],
        "first_agent_reference_deviation": reference_deviations[0],
        "second_agent_reference_deviation": reference_deviations[1]
            if len(reference_deviations) > 1 else reference_deviations[0],
        "gate_peak_occupancy": max(occupancy) if occupancy else 0,
        "gate_active_span": (max(interval[1] for interval in active_intervals)
                             - min(interval[0] for interval in active_intervals))
            if active_intervals else 0,
    })
    return metrics


def prefixed(row, prefix, values):
    for key, value in values.items():
        row[f"{prefix}_{key}"] = value


def independently_valid(metrics):
    return (
        metrics["maximum_dynamics_defect"] <= 1.0e-2
        and metrics["maximum_terminal_position_error"] <= 1.0e-3
        and metrics["maximum_terminal_yaw_error"] <= 2.0e-3
        and metrics["minimum_pairwise_clearance"] >= -5.0e-3
        and metrics["minimum_exact_obstacle_clearance"] >= -5.0e-3
    )


def main():
    args = parse_args()
    args.work_dir.mkdir(parents=True, exist_ok=True)
    instance = args.instance.resolve()
    csdo_output = (args.work_dir / "csdo.yaml").resolve()
    csdo_guess = args.work_dir / "csdo_guesses.yaml"
    csdo_corridor = args.work_dir / "csdo_corridors.yaml"
    root_guess = args.work_dir / "root_guesses.yaml"
    root_corridor = args.work_dir / "root_corridors.yaml"
    root_metadata = args.work_dir / "root_metadata.yaml"
    turbo_output = (args.work_dir / "turbo.yaml").resolve()
    turbo_primary_output = (args.work_dir / "turbo_primary.yaml").resolve()
    for path in (csdo_output, csdo_guess, csdo_corridor, root_guess,
                 root_corridor, root_metadata, turbo_output,
                 turbo_primary_output, args.work_dir / "turbo.log",
                 args.work_dir / "turbo_primary.log"):
        if path.exists():
            path.unlink()
    warmstart_policy = (
        "independent" if args.independent else args.warmstart_policy)
    if args.corridor_recovery_window < 0:
        raise RuntimeError("corridor recovery window must be nonnegative")
    if args.minimum_horizon < 0:
        raise RuntimeError("minimum horizon must be nonnegative")

    csdo_command = [
        str(args.csdo_executable.resolve()),
        "--input", str(instance),
        "--output", str(csdo_output),
        "--initial_guess", "--corridor", "--screen", "1",
        "--qp_backend", "osqp", "--timeLimit", str(args.timeout),
    ]
    csdo_code, csdo_wall, csdo_log = run(
        csdo_command, args.csdo_executable.resolve().parent, args.timeout)
    (args.work_dir / "csdo.log").write_text(csdo_log, encoding="utf-8")

    csdo_result = load_yaml(csdo_output) if csdo_output.exists() else None
    csdo_statistics = csdo_result.get("statistics", {}) if csdo_result else {}
    csdo_output_available = bool(
        csdo_result and csdo_result.get("schedule") and csdo_corridor.exists())
    csdo_success = bool(
        csdo_output_available
        and int(csdo_statistics.get("search_status", 0)) > 0
        and int(csdo_statistics.get("solver_status", 0)) == 1)

    if args.mode == "recovery":
        exporter_command = [
            str(args.root_exporter.resolve()),
            "--input", str(instance),
            "--config", str(args.csdo_config.resolve()),
            "--guess", str(root_guess.resolve()),
            "--corridor", str(root_corridor.resolve()),
            "--metadata", str(root_metadata.resolve()),
            "--pad-stages", str(args.pad_stages),
            "--delay-from-agent", str(args.delay_from_agent),
            "--delay-stages", str(args.delay_stages),
            "--sequential-delay-stages",
            str(args.sequential_delay_stages),
            "--time-limit", str(args.timeout), "--screen", "1",
        ]
        if warmstart_policy == "independent":
            exporter_command.append("--independent")
        elif warmstart_policy == "pbs_root":
            exporter_command.append("--root")
        export_code, export_wall, export_log = run(
            exporter_command, args.root_exporter.resolve().parent, args.timeout)
        (args.work_dir / "root_exporter.log").write_text(
            export_log, encoding="utf-8")
        if export_code != 0:
            raise RuntimeError("PBS-root export failed; see root_exporter.log")
        metadata = load_yaml(root_metadata)
        reference_guess_path = root_guess
        corridor_path = root_corridor
        if csdo_output_available:
            guess_path = csdo_output
            metadata["warmstart_source"] = "csdo_solution_repair"
            metadata["warmstart_conflicting_pairs"] = None
        else:
            guess_path = root_guess
    elif args.mode == "joint_repair":
        if not csdo_output_available:
            raise RuntimeError(
                "CSDO did not emit a repairable trajectory and corridors")
        guess_path, corridor_path = csdo_output, csdo_corridor
        metadata = {
            "pbs_success": int(csdo_statistics.get("search_status", 0)) > 0,
            "warmstart_source": "csdo_fixed_plane_solution",
            "root_conflicting_pairs": None,
            "warmstart_conflicting_pairs": None,
            "search_time": csdo_statistics.get("runtime_search"),
        }
        reference_guess_path = guess_path
        export_wall = 0.0
    else:
        if not csdo_success or not csdo_guess.exists() or not csdo_corridor.exists():
            raise RuntimeError("CSDO did not produce a successful shared front end")
        guess_path, corridor_path = csdo_guess, csdo_corridor
        metadata = {
            "pbs_success": True,
            "warmstart_source": "pbs_goal",
            "root_conflicting_pairs": 0,
            "warmstart_conflicting_pairs": 0,
            "search_time": csdo_result["statistics"].get("runtime_search"),
        }
        reference_guess_path = guess_path
        export_wall = 0.0

    turbo_command = [
        str(args.turbo_executable.resolve()),
        "--input", str(instance),
        "--guess", str(guess_path.resolve()),
        "--corridor", str(corridor_path.resolve()),
        "--output", str(turbo_output),
        "--threads", str(args.threads),
        "--minimum-horizon", str(args.minimum_horizon),
    ]
    turbo_code, turbo_wall, turbo_log = run(
        turbo_command, args.turbo_executable.resolve().parent, args.timeout)
    turbo_primary_code = turbo_code
    turbo_primary_wall = turbo_wall
    turbo_recovery_used = False
    primary_result = load_yaml(turbo_output) if turbo_output.exists() else None
    primary_valid = bool(
        primary_result
        and primary_result.get("schedule")
        and primary_result.get("statistics", {}).get("validated"))
    if args.corridor_recovery_window > 0 and not primary_valid:
        turbo_recovery_used = True
        if turbo_output.exists():
            turbo_output.replace(turbo_primary_output)
        (args.work_dir / "turbo_primary.log").write_text(
            turbo_log, encoding="utf-8")
        recovery_command = turbo_command + [
            "--corridor-window", str(args.corridor_recovery_window),
        ]
        recovery_code, recovery_wall, recovery_log = run(
            recovery_command,
            args.turbo_executable.resolve().parent,
            args.timeout)
        turbo_code = recovery_code
        turbo_wall += recovery_wall
        turbo_log = recovery_log
    (args.work_dir / "turbo.log").write_text(turbo_log, encoding="utf-8")
    turbo_result = load_yaml(turbo_output) if turbo_output.exists() else None
    guess = load_yaml(reference_guess_path)
    instance_data = load_yaml(instance)
    names = [agent.get("name", f"agent{index}")
             for index, agent in enumerate(instance_data["agents"])]
    turbo_schedule = turbo_result.get("schedule", {}) if turbo_result else {}
    turbo_output_available = bool(
        turbo_schedule and all(name in turbo_schedule for name in names))
    turbo_success = bool(
        turbo_result
        and turbo_output_available
        and turbo_result.get("statistics", {}).get("validated"))

    row = {
        "mode": args.mode,
        "instance": instance.name,
        "warmstart_policy": warmstart_policy,
        "minimum_horizon": args.minimum_horizon,
        "input_horizon": metadata.get("horizon"),
        "normalized_horizon": args.minimum_horizon,
        "pbs_success": metadata.get("pbs_success"),
        "warmstart_source": metadata.get("warmstart_source"),
        "root_conflicting_pairs": metadata.get("root_conflicting_pairs"),
        "warmstart_conflicting_pairs": metadata.get("warmstart_conflicting_pairs"),
        "csdo_success": csdo_success,
        "turbo_success": turbo_success,
        "csdo_return_code": csdo_code,
        "turbo_return_code": turbo_code,
        "turbo_primary_return_code": turbo_primary_code,
        "csdo_wall_time": csdo_wall,
        "root_export_wall_time": export_wall,
        "turbo_wall_time": turbo_wall,
        "turbo_primary_wall_time": turbo_primary_wall,
        "turbo_recovery_used": turbo_recovery_used,
        "turbo_corridor_recovery_window": args.corridor_recovery_window,
    }
    if turbo_result:
        turbo_statistics = turbo_result.get("statistics", {})
        row["input_horizon"] = turbo_statistics.get("input_horizon")
        row["normalized_horizon"] = turbo_statistics.get(
            "normalized_horizon")
    metric_template = interaction_metrics(instance_data, guess, guess)
    if turbo_output_available:
        turbo_metrics = interaction_metrics(instance_data, guess, turbo_result)
    else:
        turbo_metrics = {key: None for key in metric_template}
    if csdo_output_available:
        csdo_metrics = interaction_metrics(instance_data, guess, csdo_result)
    else:
        csdo_metrics = {key: None for key in metric_template}
    row["csdo_independently_validated"] = bool(
        csdo_output_available and independently_valid(csdo_metrics))
    prefixed(row, "csdo", csdo_metrics)
    prefixed(row, "turbo", turbo_metrics)
    append_row(args.output_csv, row)
    print(json.dumps(row, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
