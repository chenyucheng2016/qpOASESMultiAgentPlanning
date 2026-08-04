#!/usr/bin/env python3
"""Test TurboADMM-NL agent-order equivariance from one fixed CSDO warm start."""

import argparse
import csv
import math
import pathlib
import subprocess
import time

import yaml

from run_csdo_turbo_comparison import load_yaml, trajectory_metrics


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-instance", required=True, type=pathlib.Path)
    parser.add_argument("--guess", required=True, type=pathlib.Path)
    parser.add_argument("--corridor", required=True, type=pathlib.Path)
    parser.add_argument("--turbo-executable", required=True, type=pathlib.Path)
    parser.add_argument("--work-dir", required=True, type=pathlib.Path)
    parser.add_argument("--output-csv", required=True, type=pathlib.Path)
    parser.add_argument(
        "--orders",
        nargs="+",
        default=("0-1-2-3-4", "0-1-2-4-3", "1-0-2-3-4"),
        help="Agent-index permutations, such as 0-1-2-4-3.",
    )
    parser.add_argument("--threads", type=int, default=2)
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--reuse-existing", action="store_true")
    return parser.parse_args()


def parse_order(specification, agent_count):
    try:
        order = tuple(int(value) for value in specification.split("-"))
    except ValueError as error:
        raise SystemExit(f"invalid order {specification!r}") from error
    if sorted(order) != list(range(agent_count)):
        raise SystemExit(
            f"order {specification!r} is not a permutation of 0..{agent_count - 1}")
    return order


def run_streaming(command, cwd, log_path, timeout):
    start = time.perf_counter()
    with log_path.open("w", encoding="utf-8") as stream:
        process = subprocess.Popen(
            command,
            cwd=cwd,
            stdout=stream,
            stderr=subprocess.STDOUT,
            text=True,
        )
        try:
            code = process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            process.terminate()
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
            code = 124
    return code, time.perf_counter() - start


def trajectory_delta(reference, candidate):
    maximum_state_delta = 0.0
    maximum_control_delta = 0.0
    reference_schedule = reference["schedule"]
    candidate_schedule = candidate["schedule"]
    if set(reference_schedule) != set(candidate_schedule):
        return math.inf, math.inf
    for name, reference_nodes in reference_schedule.items():
        candidate_nodes = candidate_schedule[name]
        if len(reference_nodes) != len(candidate_nodes):
            return math.inf, math.inf
        for reference_node, candidate_node in zip(reference_nodes, candidate_nodes):
            for key in ("x", "y", "yaw", "steer"):
                maximum_state_delta = max(
                    maximum_state_delta,
                    abs(float(reference_node[key]) - float(candidate_node[key])),
                )
            for key in ("v", "omega"):
                if key not in reference_node or key not in candidate_node:
                    continue
                maximum_control_delta = max(
                    maximum_control_delta,
                    abs(float(reference_node[key]) - float(candidate_node[key])),
                )
    return maximum_state_delta, maximum_control_delta


def write_rows(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main():
    args = parse_args()
    if args.threads < 0:
        raise SystemExit("--threads must be nonnegative")
    if args.timeout <= 0:
        raise SystemExit("--timeout must be positive")

    base_instance = load_yaml(args.base_instance)
    guess = load_yaml(args.guess)
    agents = base_instance["agents"]
    orders = [parse_order(specification, len(agents))
              for specification in args.orders]
    args.work_dir.mkdir(parents=True, exist_ok=True)
    executable = args.turbo_executable.resolve()
    guess_path = args.guess.resolve()
    corridor_path = args.corridor.resolve()

    rows = []
    solutions = {}
    for specification, order in zip(args.orders, orders):
        case_dir = args.work_dir / f"order_{specification.replace('-', '')}"
        case_dir.mkdir(parents=True, exist_ok=True)
        instance_path = case_dir / "instance.yaml"
        output_path = case_dir / "turbo.yaml"
        log_path = case_dir / "turbo.log"

        instance = dict(base_instance)
        instance["agents"] = [agents[index] for index in order]
        with instance_path.open("w", encoding="utf-8") as stream:
            yaml.safe_dump(instance, stream, sort_keys=False)
        if output_path.exists() and not args.reuse_existing:
            output_path.unlink()

        command = [
            str(executable),
            "--input", str(instance_path.resolve()),
            "--guess", str(guess_path),
            "--corridor", str(corridor_path),
            "--output", str(output_path.resolve()),
            "--threads", str(args.threads),
        ]
        if args.reuse_existing and output_path.exists():
            return_code, wall_time = 0, 0.0
        else:
            return_code, wall_time = run_streaming(
                command, executable.parent, log_path, args.timeout)

        solution = load_yaml(output_path) if output_path.exists() else None
        statistics = solution.get("statistics", {}) if solution else {}
        metrics = (trajectory_metrics(instance, guess, solution)
                   if solution and "schedule" in solution else {})
        solutions[specification] = solution
        rows.append({
            "order": specification,
            "physical_names": "-".join(
                str(agents[index].get("name", f"agent{index}")) for index in order),
            "return_code": return_code,
            "wall_time_s": wall_time,
            "output_produced": solution is not None,
            "success": statistics.get("success", False),
            "converged": statistics.get("converged", False),
            "validated": statistics.get("validated", False),
            "solver_time_s": statistics.get("solver_time", math.nan),
            "scp_iterations": statistics.get("scp_iterations", 0),
            "admm_iterations": statistics.get("admm_iterations", 0),
            "objective": metrics.get("objective", math.nan),
            "minimum_pairwise_clearance": metrics.get(
                "minimum_pairwise_clearance", math.nan),
            "minimum_obstacle_clearance": metrics.get(
                "minimum_exact_obstacle_clearance", math.nan),
            "maximum_dynamics_defect": metrics.get(
                "maximum_dynamics_defect", math.nan),
            "terminal_position_error": metrics.get(
                "maximum_terminal_position_error", math.nan),
            "terminal_yaw_error": metrics.get(
                "maximum_terminal_yaw_error", math.nan),
            "terminal_state_error": statistics.get(
                "maximum_terminal_state_error", math.nan),
            "cold_starts": statistics.get("cold_starts", 0),
            "matrix_hotstarts": statistics.get("matrix_hotstarts", 0),
            "vector_hotstarts": statistics.get("vector_hotstarts", 0),
            "maximum_active_pairs": statistics.get("maximum_active_pairs", 0),
            "maximum_agent_degree": statistics.get("maximum_agent_degree", 0),
            "state_delta_from_first": math.nan,
            "control_delta_from_first": math.nan,
        })
        print(
            f"order {specification}: code={return_code}, "
            f"validated={statistics.get('validated', False)}, "
            f"wall={wall_time:.3f} s")

    reference = solutions[args.orders[0]]
    if reference and "schedule" in reference:
        for row in rows:
            candidate = solutions[row["order"]]
            if candidate and "schedule" in candidate:
                state_delta, control_delta = trajectory_delta(reference, candidate)
                row["state_delta_from_first"] = state_delta
                row["control_delta_from_first"] = control_delta
    write_rows(args.output_csv, rows)


if __name__ == "__main__":
    main()
