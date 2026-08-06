#!/usr/bin/env python3
"""Run CSDO/Turbo interaction experiments with a shared CSDO front end."""

import argparse
import csv
import hashlib
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


SCHEDULE_FIELDS = ("t", "x", "y", "yaw", "steer", "v", "omega")


def file_sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def canonical_schedule(schedule):
    normalized = {}
    for name in sorted(schedule):
        nodes = []
        for node in schedule[name]:
            nodes.append({
                field: (format(float(node[field]), ".17g")
                        if field in node else None)
                for field in SCHEDULE_FIELDS
            })
        normalized[name] = nodes
    return json.dumps(
        normalized, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")


def schedule_sha256(document):
    schedule = document.get("schedule") if document else None
    if not schedule:
        raise RuntimeError("trajectory artifact has no schedule")
    return hashlib.sha256(canonical_schedule(schedule)).hexdigest()


def compare_schedules(first, second, tolerance=1.0e-9):
    first_schedule = first.get("schedule") if first else None
    second_schedule = second.get("schedule") if second else None
    if not first_schedule or not second_schedule:
        return False, math.inf
    if set(first_schedule) != set(second_schedule):
        return False, math.inf
    maximum_difference = 0.0
    for name in first_schedule:
        first_nodes = first_schedule[name]
        second_nodes = second_schedule[name]
        if len(first_nodes) != len(second_nodes):
            return False, math.inf
        for first_node, second_node in zip(first_nodes, second_nodes):
            for field in SCHEDULE_FIELDS:
                first_has_field = field in first_node
                second_has_field = field in second_node
                if first_has_field != second_has_field:
                    return False, math.inf
                if not first_has_field:
                    continue
                difference = abs(
                    float(first_node[field]) - float(second_node[field]))
                maximum_difference = max(maximum_difference, difference)
    return maximum_difference <= tolerance, maximum_difference


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
        "--timeLimit", str(args.timeout),
    ]
    csdo_code, csdo_wall, csdo_log = run(
        csdo_command, args.csdo_executable.resolve().parent, args.timeout)
    (args.work_dir / "csdo.log").write_text(csdo_log, encoding="utf-8")

    csdo_result = load_yaml(csdo_output) if csdo_output.exists() else None
    csdo_statistics = csdo_result.get("statistics", {}) if csdo_result else {}
    csdo_output_available = bool(
        csdo_result and csdo_result.get("schedule"))
    csdo_initial_guess_available = csdo_guess.exists()
    csdo_success = bool(
        csdo_output_available
        and int(csdo_statistics.get("search_status", 0)) > 0
        and int(csdo_statistics.get("solver_status", 0)) == 1)
    shared_schedule_match = None
    shared_schedule_max_abs_difference = None
    csdo_initial_schedule_sha256 = None
    root_schedule_sha256 = None
    csdo_initial_guess_sha256 = None
    root_guess_sha256 = None
    root_corridor_sha256 = None

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
        root_document = load_yaml(root_guess)
        root_guess_sha256 = file_sha256(root_guess)
        root_corridor_sha256 = file_sha256(root_corridor)
        root_schedule_sha256 = schedule_sha256(root_document)
        if (warmstart_policy == "auto"
                and bool(metadata.get("pbs_success"))):
            if not csdo_initial_guess_available:
                raise RuntimeError(
                    "exporter found a PBS goal but CSDO emitted no initial guess")
            csdo_initial_document = load_yaml(csdo_guess)
            csdo_initial_guess_sha256 = file_sha256(csdo_guess)
            csdo_initial_schedule_sha256 = schedule_sha256(
                csdo_initial_document)
            (shared_schedule_match,
             shared_schedule_max_abs_difference) = compare_schedules(
                 csdo_initial_document, root_document)
            if not shared_schedule_match:
                raise RuntimeError(
                    "CSDO and reconstructed PBS schedules differ: "
                    f"maximum absolute difference "
                    f"{shared_schedule_max_abs_difference}")
            guess_path = csdo_guess
            reference_guess_path = csdo_guess
            corridor_path = root_corridor
            turbo_guess_artifact = "csdo_initial_guess"
            turbo_corridor_artifact = (
                "csdo_initial_corridor_reconstruction")
        else:
            if (warmstart_policy == "auto"
                    and csdo_initial_guess_available):
                raise RuntimeError(
                    "CSDO and exporter disagree on PBS success")
            guess_path = root_guess
            reference_guess_path = root_guess
            corridor_path = root_corridor
            source = metadata.get("warmstart_source")
            if source == "pbs_root":
                turbo_guess_artifact = "pbs_root_guess"
                turbo_corridor_artifact = "pbs_root_corridor"
            elif source == "independent_single_agent":
                turbo_guess_artifact = "independent_single_agent_guess"
                turbo_corridor_artifact = (
                    "independent_single_agent_corridor")
            else:
                raise RuntimeError(
                    f"unsupported recovery warm-start source: {source}")
    elif args.mode == "joint_repair":
        if not csdo_output_available:
            raise RuntimeError(
                "CSDO did not emit a repairable trajectory and corridors")
        guess_path, corridor_path = csdo_output, csdo_corridor
        turbo_guess_artifact = "csdo_fixed_plane_solution"
        turbo_corridor_artifact = "csdo_dumped_initial_corridor"
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
        if (not csdo_success or not csdo_guess.exists()
                or not csdo_corridor.exists()):
            raise RuntimeError("CSDO did not produce a successful shared front end")
        guess_path, corridor_path = csdo_guess, csdo_corridor
        turbo_guess_artifact = "csdo_initial_guess"
        turbo_corridor_artifact = "csdo_dumped_initial_corridor"
        metadata = {
            "pbs_success": True,
            "warmstart_source": "pbs_goal",
            "root_conflicting_pairs": 0,
            "warmstart_conflicting_pairs": 0,
            "search_time": csdo_result["statistics"].get("runtime_search"),
        }
        reference_guess_path = guess_path
        export_wall = 0.0

    if csdo_guess.exists() and csdo_initial_guess_sha256 is None:
        csdo_initial_guess_sha256 = file_sha256(csdo_guess)
        csdo_initial_schedule_sha256 = schedule_sha256(load_yaml(csdo_guess))
    turbo_guess_sha256 = file_sha256(guess_path)
    turbo_corridor_sha256 = file_sha256(corridor_path)
    turbo_schedule_sha256 = schedule_sha256(load_yaml(guess_path))

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
        "search_seed": metadata.get("search_seed"),
        "turbo_input_artifact": turbo_guess_artifact,
        "turbo_guess_artifact": turbo_guess_artifact,
        "turbo_corridor_artifact": turbo_corridor_artifact,
        "shared_schedule_match": shared_schedule_match,
        "shared_schedule_max_abs_difference": (
            shared_schedule_max_abs_difference),
        "csdo_initial_guess_sha256": csdo_initial_guess_sha256,
        "root_guess_sha256": root_guess_sha256,
        "root_corridor_sha256": root_corridor_sha256,
        "turbo_guess_sha256": turbo_guess_sha256,
        "turbo_corridor_sha256": turbo_corridor_sha256,
        "csdo_initial_schedule_sha256": csdo_initial_schedule_sha256,
        "root_schedule_sha256": root_schedule_sha256,
        "turbo_schedule_sha256": turbo_schedule_sha256,
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
