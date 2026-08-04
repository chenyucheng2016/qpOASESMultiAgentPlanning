#!/usr/bin/env python3
"""Run original CSDO and TurboADMM-NL from one CSDO-generated warm start."""

import argparse
import csv
import math
import pathlib
import subprocess
import time

import yaml


DT = 3.0 * 0.706 / 3.0 / 0.8
DEG_TO_RAD = 3.14 / 180.0
FOOTPRINTS = ((1.25, 1.25), (-0.25, 1.25))


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--instance", required=True, type=pathlib.Path)
    parser.add_argument("--csdo-executable", required=True, type=pathlib.Path)
    parser.add_argument("--turbo-executable", required=True, type=pathlib.Path)
    parser.add_argument("--work-dir", required=True, type=pathlib.Path)
    parser.add_argument("--output-csv", required=True, type=pathlib.Path)
    parser.add_argument("--threads", type=int, default=0)
    parser.add_argument(
        "--csdo-backend", choices=("osqp", "turbo_qpoases"), default="osqp")
    parser.add_argument("--timeout", type=float, default=7200.0)
    return parser.parse_args()


def run(command, cwd, timeout):
    start = time.perf_counter()
    try:
        completed = subprocess.run(
            command,
            cwd=cwd,
            check=False,
            timeout=timeout,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        return completed.returncode, time.perf_counter() - start, completed.stdout
    except subprocess.TimeoutExpired as error:
        output = error.stdout or ""
        if isinstance(output, bytes):
            output = output.decode(errors="replace")
        return 124, time.perf_counter() - start, output


def load_yaml(path):
    with path.open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def angle_error(first, second):
    return abs(math.atan2(math.sin(first - second), math.cos(first - second)))


def state(node):
    return (
        float(node["x"]),
        float(node["y"]),
        float(node["yaw"]),
        float(node["steer"]) * DEG_TO_RAD,
    )


def control(node):
    return float(node["v"]), float(node["omega"]) * DEG_TO_RAD


def circle_positions(node):
    x, y, yaw, _ = state(node)
    return [
        (x + offset * math.cos(yaw), y + offset * math.sin(yaw), radius)
        for offset, radius in FOOTPRINTS
    ]


def trajectory_metrics(instance, guess, solution, interpolation_substeps=10):
    schedule = solution["schedule"]
    guess_schedule = guess["schedule"]
    names = [agent.get("name", f"agent{index}")
             for index, agent in enumerate(instance["agents"])]
    metrics = {
        "objective": 0.0,
        "maximum_dynamics_defect": 0.0,
        "maximum_terminal_position_error": 0.0,
        "maximum_terminal_yaw_error": 0.0,
        "minimum_pairwise_clearance": math.inf,
        "minimum_exact_obstacle_clearance": math.inf,
    }
    for name in names:
        nodes = schedule[name]
        for stage in range(len(nodes) - 1):
            x, y, yaw, steer = state(nodes[stage])
            speed, omega = control(nodes[stage])
            predicted = (
                x + DT * speed * math.cos(yaw),
                y + DT * speed * math.sin(yaw),
                yaw + DT * speed * math.tan(steer),
                steer + DT * omega,
            )
            following = state(nodes[stage + 1])
            for predicted_value, actual_value in zip(predicted, following):
                metrics["maximum_dynamics_defect"] = max(
                    metrics["maximum_dynamics_defect"],
                    abs(predicted_value - actual_value),
                )
            metrics["objective"] += omega * omega
            if stage + 1 < len(nodes) - 1:
                next_speed, _ = control(nodes[stage + 1])
                metrics["objective"] += (next_speed - speed) ** 2
        goal = state(guess_schedule[name][-1])
        terminal = state(nodes[-1])
        metrics["maximum_terminal_position_error"] = max(
            metrics["maximum_terminal_position_error"],
            math.hypot(terminal[0] - goal[0], terminal[1] - goal[1]),
        )
        metrics["maximum_terminal_yaw_error"] = max(
            metrics["maximum_terminal_yaw_error"],
            angle_error(terminal[2], goal[2]),
        )

    obstacles = instance["map"].get("obstacles") or []
    width, height = map(float, instance["map"]["dimensions"])
    horizon = len(schedule[names[0]]) - 1
    for stage in range(horizon):
        endpoint_circles = {}
        for name in names:
            endpoint_circles[name] = (
                circle_positions(schedule[name][stage]),
                circle_positions(schedule[name][stage + 1]),
            )
        for substep in range(interpolation_substeps + 1):
            alpha = substep / interpolation_substeps
            circles = {}
            for name in names:
                circles[name] = []
                first, second = endpoint_circles[name]
                for first_circle, second_circle in zip(first, second):
                    circles[name].append((
                        first_circle[0] + alpha
                        * (second_circle[0] - first_circle[0]),
                        first_circle[1] + alpha
                        * (second_circle[1] - first_circle[1]),
                        first_circle[2],
                    ))
            for first_index, first_name in enumerate(names):
                for second_name in names[first_index + 1:]:
                    for first_circle in circles[first_name]:
                        for second_circle in circles[second_name]:
                            clearance = math.hypot(
                                first_circle[0] - second_circle[0],
                                first_circle[1] - second_circle[1],
                            ) - first_circle[2] - second_circle[2]
                            metrics["minimum_pairwise_clearance"] = min(
                                metrics["minimum_pairwise_clearance"], clearance)
            for name in names:
                for x, y, radius in circles[name]:
                    boundary_clearance = min(
                        x - radius,
                        width - x - radius,
                        y - radius,
                        height - y - radius,
                    )
                    metrics["minimum_exact_obstacle_clearance"] = min(
                        metrics["minimum_exact_obstacle_clearance"],
                        boundary_clearance,
                    )
                    for obstacle_x, obstacle_y, obstacle_radius in obstacles:
                        clearance = math.hypot(
                            x - float(obstacle_x),
                            y - float(obstacle_y),
                        ) - radius - float(obstacle_radius)
                        metrics["minimum_exact_obstacle_clearance"] = min(
                            metrics["minimum_exact_obstacle_clearance"],
                            clearance,
                        )
    return metrics


def append_row(path, row):
    path.parent.mkdir(parents=True, exist_ok=True)
    exists = path.exists() and path.stat().st_size > 0
    with path.open("a", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(row))
        if not exists:
            writer.writeheader()
        writer.writerow(row)


def main():
    args = parse_args()
    if args.threads < 0:
        raise SystemExit("--threads must be nonnegative")
    args.csdo_executable = args.csdo_executable.resolve()
    args.turbo_executable = args.turbo_executable.resolve()
    args.work_dir.mkdir(parents=True, exist_ok=True)
    instance = load_yaml(args.instance)
    stem = args.instance.stem
    csdo_output = args.work_dir / f"{stem}_csdo.yaml"
    guess_output = args.work_dir / f"{stem}_csdo_guesses.yaml"
    corridor_output = args.work_dir / f"{stem}_csdo_corridors.yaml"
    turbo_output = args.work_dir / f"{stem}_turbo.yaml"
    csdo_log = args.work_dir / f"{stem}_csdo.log"
    turbo_log = args.work_dir / f"{stem}_turbo.log"

    csdo_command = [
        str(args.csdo_executable),
        "--input", str(args.instance.resolve()),
        "--output", str(csdo_output.resolve()),
        "--initial_guess",
        "--corridor",
        "--screen", "0",
        "--qp_backend", args.csdo_backend,
        "--timeLimit", str(args.timeout),
    ]
    csdo_code, csdo_wall, csdo_stdout = run(
        csdo_command, args.csdo_executable.parent, args.timeout + 30.0)
    csdo_log.write_text(csdo_stdout, encoding="utf-8")
    if (not csdo_output.exists() or not guess_output.exists()
            or not corridor_output.exists()):
        raise SystemExit(
            f"CSDO did not produce output, warm start, and corridors (code {csdo_code}); "
            f"see {csdo_log}")

    turbo_command = [
        str(args.turbo_executable),
        "--input", str(args.instance.resolve()),
        "--guess", str(guess_output.resolve()),
        "--corridor", str(corridor_output.resolve()),
        "--output", str(turbo_output.resolve()),
        "--threads", str(args.threads),
    ]
    turbo_code, turbo_wall, turbo_stdout = run(
        turbo_command, args.turbo_executable.parent, args.timeout)
    turbo_log.write_text(turbo_stdout, encoding="utf-8")
    if not turbo_output.exists():
        raise SystemExit(
            f"Turbo bridge did not produce output (code {turbo_code}); "
            f"see {turbo_log}")

    guess = load_yaml(guess_output)
    csdo = load_yaml(csdo_output)
    turbo = load_yaml(turbo_output)
    csdo_statistics = csdo["statistics"]
    turbo_statistics = turbo["statistics"]
    csdo_metrics = trajectory_metrics(instance, guess, csdo)
    turbo_metrics = trajectory_metrics(instance, guess, turbo)
    first_name = instance["agents"][0].get("name", "agent0")
    shared_frontend_time = (
        float(csdo_statistics["runtime_search"])
        + float(csdo_statistics["runtime_preprocess"])
    )
    row = {
        "instance": str(args.instance),
        "agents": len(instance["agents"]),
        "obstacles": len(instance["map"].get("obstacles") or []),
        "horizon": len(guess["schedule"][first_name]) - 1,
        "csdo_return_code": csdo_code,
        "csdo_process_wall_s": csdo_wall,
        "csdo_reported_end_to_end_s": csdo_statistics["runtime"],
        "csdo_optimizer_sequential_s": csdo_statistics["runtime_optimization"],
        "csdo_optimizer_ideal_parallel_s":
            csdo_statistics["runtime_decentralized_optimization"],
        "csdo_solver_status": csdo_statistics["solver_status"],
        "csdo_qp_backend": csdo_statistics.get("qp_backend", args.csdo_backend),
        "csdo_qp_backend_time_s":
            csdo_statistics.get("qp_backend_runtime", math.nan),
        "csdo_qp_cold_starts": csdo_statistics.get("qp_cold_starts", 0),
        "csdo_riccati_initializations":
            csdo_statistics.get("riccati_initializations", 0),
        "csdo_riccati_failures": csdo_statistics.get("riccati_failures", 0),
        "csdo_matrix_hotstarts": csdo_statistics.get("matrix_hotstarts", 0),
        "csdo_hotstart_fallbacks":
            csdo_statistics.get("hotstart_fallbacks", 0),
        "csdo_objective": csdo_metrics["objective"],
        "csdo_min_pair_clearance": csdo_metrics["minimum_pairwise_clearance"],
        "csdo_min_obstacle_clearance":
            csdo_metrics["minimum_exact_obstacle_clearance"],
        "csdo_dynamics_defect": csdo_metrics["maximum_dynamics_defect"],
        "csdo_terminal_position_error":
            csdo_metrics["maximum_terminal_position_error"],
        "csdo_terminal_yaw_error": csdo_metrics["maximum_terminal_yaw_error"],
        "turbo_return_code": turbo_code,
        "turbo_process_wall_s": turbo_wall,
        "turbo_solver_s": turbo_statistics["solver_time"],
        "turbo_shared_end_to_end_s":
            shared_frontend_time + float(turbo_statistics["solver_time"]),
        "turbo_success": turbo_statistics["success"],
        "turbo_converged": turbo_statistics["converged"],
        "turbo_validated": turbo_statistics["validated"],
        "turbo_objective": turbo_metrics["objective"],
        "turbo_min_pair_clearance": turbo_metrics["minimum_pairwise_clearance"],
        "turbo_min_obstacle_clearance":
            turbo_metrics["minimum_exact_obstacle_clearance"],
        "turbo_dynamics_defect": turbo_metrics["maximum_dynamics_defect"],
        "turbo_terminal_position_error":
            turbo_metrics["maximum_terminal_position_error"],
        "turbo_terminal_yaw_error": turbo_metrics["maximum_terminal_yaw_error"],
        "turbo_scp_iterations": turbo_statistics["scp_iterations"],
        "turbo_admm_iterations": turbo_statistics["admm_iterations"],
        "turbo_last_qp_status": turbo_statistics.get("last_qp_status", 0),
        "turbo_failed_agent": turbo_statistics.get("failed_agent", -1),
        "turbo_restoration_attempts":
            turbo_statistics.get("restoration_attempts", 0),
        "turbo_successful_restorations":
            turbo_statistics.get("successful_restorations", 0),
        "turbo_line_search_recovery_attempts":
            turbo_statistics.get("line_search_recovery_attempts", 0),
        "turbo_maximum_restoration_slack":
            turbo_statistics.get("maximum_restoration_slack", 0.0),
        "turbo_final_restoration_slack":
            turbo_statistics.get("final_restoration_slack", 0.0),
        "turbo_qp_variables": turbo_statistics["maximum_local_qp_variables"],
        "turbo_qp_constraints": turbo_statistics["maximum_local_qp_constraints"],
        "turbo_corridor_rows":
            turbo_statistics["maximum_corridor_rows_per_agent"],
        "turbo_cold_starts": turbo_statistics["cold_starts"],
        "turbo_riccati_initializations":
            turbo_statistics["riccati_initializations"],
        "turbo_riccati_failures": turbo_statistics["riccati_failures"],
        "turbo_hotstart_fallbacks": turbo_statistics["hotstart_fallbacks"],
        "turbo_matrix_hotstarts": turbo_statistics["matrix_hotstarts"],
        "turbo_vector_hotstarts": turbo_statistics["vector_hotstarts"],
        "turbo_riccati_s": turbo_statistics["riccati_time"],
        "turbo_cold_start_qp_s": turbo_statistics["cold_start_qp_time"],
        "turbo_matrix_hotstart_qp_s":
            turbo_statistics["matrix_hotstart_qp_time"],
        "turbo_vector_hotstart_qp_s":
            turbo_statistics["vector_hotstart_qp_time"],
        "turbo_threads": turbo_statistics["parallel_threads"],
    }
    append_row(args.output_csv, row)
    print(
        f"{stem}: CSDO optimizer {row['csdo_optimizer_sequential_s']} s, "
        f"Turbo {row['turbo_solver_s']} s, "
        f"Turbo validated={row['turbo_validated']}")


if __name__ == "__main__":
    main()
