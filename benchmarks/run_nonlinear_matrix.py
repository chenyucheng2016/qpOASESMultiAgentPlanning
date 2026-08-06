#!/usr/bin/env python3
"""Run nonlinear benchmarks one scenario at a time with resume and timeouts."""

import argparse
import csv
import hashlib
import json
import os
import re
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path


METHODS = (
    "cold",
    "inner",
    "qp_continuation",
    "full",
    "centralized_qpoases",
    "centralized_osqp",
)
MANUAL_CASES = (
    "easy_open",
    "easy_single_blocker",
    "medium_doorway",
    "medium_heterogeneous_open",
    "hard_heterogeneous_doorway",
    "hard_warehouse",
    "very_hard_maze",
)
STATUS_FIELDS = (
    "suite",
    "track",
    "selector",
    "method",
    "repetition",
    "execution_status",
    "return_code",
    "wall_time_seconds",
    "peak_memory_kib",
    "result_file",
    "log_file",
    "message",
)


def parse_methods(value):
    methods = tuple(item.strip() for item in value.split(",") if item.strip())
    unknown = sorted(set(methods) - set(METHODS))
    if unknown:
        raise argparse.ArgumentTypeError("unknown methods: " + ", ".join(unknown))
    if not methods:
        raise argparse.ArgumentTypeError("methods must not be empty")
    return methods

def parse_manual_cases(value):
    cases = tuple(item.strip() for item in value.split(",") if item.strip())
    unknown = sorted(set(cases) - set(MANUAL_CASES))
    if unknown:
        raise argparse.ArgumentTypeError("unknown manual cases: " + ", ".join(unknown))
    if not cases:
        raise argparse.ArgumentTypeError("manual cases must not be empty")
    return cases


def parse_indices(value):
    try:
        indices = tuple(int(item.strip()) for item in value.split(",") if item.strip())
    except ValueError as error:
        raise argparse.ArgumentTypeError("scenario indices must be comma-separated integers") from error
    if not indices or any(index < 0 for index in indices):
        raise argparse.ArgumentTypeError("scenario indices must be nonnegative")
    return indices


def build_tasks(selectors, methods, repetitions, run_dir, schedule, schedule_seed):
    tasks = []

    def append_task(selector, selector_arguments, method, repetition):
        suffix = "" if repetitions == 1 else f"__rep_{repetition:02d}"
        result_path = run_dir / f"{selector}__{method}{suffix}.csv"
        tasks.append((selector, method, repetition, result_path, selector_arguments))

    if schedule == "grouped":
        for selector, selector_arguments in selectors:
            for method in methods:
                for repetition in range(1, repetitions + 1):
                    append_task(selector, selector_arguments, method, repetition)
        return tasks

    for repetition in range(1, repetitions + 1):
        for selector_index, (selector, selector_arguments) in enumerate(selectors):
            offset = (schedule_seed + selector_index + repetition - 1) % len(methods)
            method_order = methods[offset:] + methods[:offset]
            for method in method_order:
                append_task(selector, selector_arguments, method, repetition)
    return tasks


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def write_run_manifest(path, arguments, executable, tasks, status_path):
    inventory = path.parent / "inventory.csv"
    configuration = {
        "protocol_id": arguments.protocol_id,
        "git_commit": arguments.git_commit,
        "executable": str(executable),
        "executable_sha256": sha256(executable),
        "runner": str(Path(__file__).resolve()),
        "runner_sha256": sha256(Path(__file__).resolve()),
        "suite": arguments.suite,
        "track": arguments.track,
        "manual_cases": list(arguments.manual_cases),
        "scenario_indices": list(arguments.scenario_indices),
        "methods": list(arguments.methods),
        "repetitions": arguments.repetitions,
        "timeout_seconds": arguments.timeout_seconds,
        "threads": arguments.threads,
        "fixed_rho": arguments.fixed_rho,
        "exact_admm": arguments.exact_admm,
        "schedule": arguments.schedule,
        "schedule_seed": arguments.schedule_seed,
        "omp_dynamic": "FALSE",
        "omp_proc_bind": "close",
        "omp_places": "cores",
        "task_order": [
            {"selector": task[0], "method": task[1], "repetition": task[2]}
            for task in tasks
        ],
    }
    if inventory.is_file():
        configuration["inventory"] = str(inventory.resolve())
        configuration["inventory_sha256"] = sha256(inventory)
    if path.exists():
        with path.open(encoding="utf-8") as stream:
            existing = json.load(stream)
        if existing.get("configuration") != configuration:
            raise RuntimeError(
                "output directory contains a different run manifest; use a new directory"
            )
        return
    if arguments.protocol_id != "development-unfrozen" and status_path.exists():
        raise RuntimeError(
            "frozen protocol output predates its manifest; use a new output directory"
        )
    manifest = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "configuration": configuration,
    }
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump(manifest, stream, indent=2, sort_keys=True)
        stream.write("\n")
    os.replace(temporary, path)


def read_single_result(path):
    try:
        with path.open(newline="", encoding="utf-8") as stream:
            rows = list(csv.DictReader(stream))
    except (OSError, csv.Error):
        return None
    return rows[0] if len(rows) == 1 else None


def discover_count(executable, suite, track, output_dir):
    inventory = output_dir / "inventory.csv"
    command = [
        str(executable), "--suite", suite, "--track", track,
        "--dry-run", "--output", str(inventory),
    ]
    completed = subprocess.run(command, capture_output=True, text=True, check=False)
    match = re.search(r"generated (\d+) scenarios", completed.stdout)
    if completed.returncode != 0 or not match:
        detail = completed.stderr.strip() or completed.stdout.strip()
        raise RuntimeError("could not discover scenario count: " + detail)
    return int(match.group(1))


def sample_peak_memory_kib(pid):
    """Return Linux VmHWM for one process, falling back to VmRSS."""
    try:
        lines = Path(f"/proc/{pid}/status").read_text(
            encoding="utf-8", errors="replace").splitlines()
    except OSError:
        return None
    resident = None
    for line in lines:
        if line.startswith("VmHWM:"):
            return int(line.split()[1])
        if line.startswith("VmRSS:"):
            resident = int(line.split()[1])
    return resident


def run_monitored(command, log_stream, timeout_seconds, environment):
    """Run one benchmark while sampling its direct-process peak memory."""
    process = subprocess.Popen(
        command,
        stdout=log_stream,
        stderr=subprocess.STDOUT,
        env=environment,
    )
    deadline = time.monotonic() + timeout_seconds
    peak_memory_kib = None
    while True:
        sample = sample_peak_memory_kib(process.pid)
        if sample is not None:
            peak_memory_kib = max(peak_memory_kib or 0, sample)
        return_code = process.poll()
        if return_code is not None:
            return return_code, peak_memory_kib, False
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            process.kill()
            process.wait()
            return process.returncode, peak_memory_kib, True
        time.sleep(min(0.02, remaining))


def read_statuses(path):
    if not path.exists():
        return {}
    with path.open(newline="", encoding="utf-8") as stream:
        return {
            (row["selector"], row["method"], int(row.get("repetition") or "1")): row
            for row in csv.DictReader(stream)
        }


def write_statuses(path, statuses):
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=STATUS_FIELDS)
        writer.writeheader()
        for key in sorted(statuses):
            writer.writerow(statuses[key])
    os.replace(temporary, path)


def combine_results(path, tasks, statuses):
    rows = []
    fields = None
    for selector, method, repetition, result_path, _ in tasks:
        row = read_single_result(result_path)
        if row is None:
            continue
        status = statuses.get((selector, method, repetition), {})
        row["repetition"] = repetition
        row["execution_status"] = status.get("execution_status", "completed")
        row["wall_time_seconds"] = status.get("wall_time_seconds", "")
        row["peak_memory_kib"] = status.get("peak_memory_kib", "")
        fields = list(row)
        rows.append(row)
    if fields is None:
        return
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    os.replace(temporary, path)


def status_record(arguments, selector, method, repetition, state, return_code,
                  elapsed, peak_memory_kib, result_path, log_path, message):
    return {
        "suite": arguments.suite,
        "track": arguments.track,
        "selector": selector,
        "method": method,
        "repetition": repetition,
        "execution_status": state,
        "return_code": return_code,
        "wall_time_seconds": f"{elapsed:.6f}",
        "peak_memory_kib": ("" if peak_memory_kib is None
                            else str(peak_memory_kib)),
        "result_file": str(result_path),
        "log_file": str(log_path),
        "message": message,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("executable", type=Path)
    parser.add_argument("--suite", default="manual",
                        choices=("manual", "ci", "smoke", "development", "final",
                                 "paper_development", "paper_final"))
    parser.add_argument("--track", default="all",
                        choices=("all", "scaling", "models", "families", "primary"))
    parser.add_argument("--scenario-indices", type=parse_indices, default=(),
                        help="optional comma-separated scenario indices")
    parser.add_argument("--manual-cases", type=parse_manual_cases, default=(),
                        help="optional comma-separated deterministic case ids")
    parser.add_argument("--methods", type=parse_methods, default=METHODS)
    parser.add_argument("--protocol-id", default="development-unfrozen")
    parser.add_argument("--git-commit", default="unknown")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--combined", default="results.csv")
    parser.add_argument("--timeout-seconds", type=float, default=300.0)
    parser.add_argument("--threads", type=int, default=0,
                        help="positive fixed thread count; zero uses the frozen automatic policy")
    parser.add_argument("--repetitions", type=int, default=1,
                        help="number of independent process executions per scenario-method pair")
    parser.add_argument("--schedule", choices=("grouped", "interleaved"),
                        default="grouped")
    parser.add_argument("--schedule-seed", type=int, default=0)
    parser.add_argument("--fixed-rho", action="store_true")
    parser.add_argument("--exact-admm", action="store_true")
    parser.add_argument("--rerun-failures", action="store_true")
    arguments = parser.parse_args()

    executable = arguments.executable.resolve()
    if not executable.is_file():
        parser.error(f"benchmark executable not found: {executable}")
    if arguments.timeout_seconds <= 0.0:
        parser.error("--timeout-seconds must be positive")
    if arguments.threads < 0:
        parser.error("--threads must be nonnegative")
    if arguments.repetitions <= 0:
        parser.error("--repetitions must be positive")

    output_dir = arguments.output_dir.resolve()
    run_dir = output_dir / "runs"
    run_dir.mkdir(parents=True, exist_ok=True)
    status_path = output_dir / "execution_status.csv"
    combined_path = output_dir / arguments.combined
    statuses = read_statuses(status_path)

    if arguments.suite == "manual":
        if arguments.scenario_indices:
            parser.error("--scenario-indices is only valid for generated suites")
        selected_cases = arguments.manual_cases or MANUAL_CASES
        selectors = tuple((case_id, ("--case", case_id)) for case_id in selected_cases)
    else:
        if arguments.manual_cases:
            parser.error("--manual-cases is only valid for the manual suite")
        count = discover_count(executable, arguments.suite, arguments.track, output_dir)
        indices = arguments.scenario_indices or tuple(range(count))
        if any(index >= count for index in indices):
            parser.error(f"scenario index must be smaller than {count}")
        width = max(3, len(str(max(0, count - 1))))
        selectors = tuple(
            (f"scenario_{index:0{width}d}", ("--scenario-index", str(index)))
            for index in indices
        )

    tasks = build_tasks(
        selectors, arguments.methods, arguments.repetitions, run_dir,
        arguments.schedule, arguments.schedule_seed,
    )
    total = len(tasks)
    environment = os.environ.copy()
    environment["OMP_DYNAMIC"] = "FALSE"
    environment["OMP_PROC_BIND"] = "close"
    environment["OMP_PLACES"] = "cores"
    write_run_manifest(
        output_dir / "run_manifest.json", arguments, executable, tasks, status_path)
    for index, (selector, method, repetition, result_path, selector_arguments) in enumerate(tasks, 1):
        key = (selector, method, repetition)
        attempt_token = f"{os.getpid()}.{time.monotonic_ns()}"
        partial = result_path.with_name(
            f"{result_path.stem}.{attempt_token}.partial{result_path.suffix}"
        )
        partial_trace = Path(str(partial) + ".scp.csv")
        final_trace = Path(str(result_path) + ".scp.csv")
        existing = read_single_result(result_path)
        if existing is not None:
            if key not in statuses or statuses[key].get("execution_status") != "completed":
                log_path = result_path.with_suffix(".log")
                statuses[key] = status_record(
                    arguments, selector, method, repetition, "completed", 0,
                    0.0, None,
                    result_path, log_path, "resumed result"
                )
                write_statuses(status_path, statuses)
            if partial_trace.is_file() and not final_trace.exists():
                os.replace(partial_trace, final_trace)
            print(f"[{index}/{total}] resume {selector} {method}", flush=True)
            continue
        previous = statuses.get(key)
        if previous and previous.get("execution_status") != "completed" and not arguments.rerun_failures:
            print(f"[{index}/{total}] skip {selector} {method} ({previous['execution_status']})", flush=True)
            continue

        log_path = result_path.with_suffix(".log")
        command = [
            str(executable), "--suite", arguments.suite, "--track", arguments.track,
            "--method", method, "--output", str(partial),
        ]
        if arguments.threads > 0:
            command += ["--threads", str(arguments.threads)]
        if arguments.fixed_rho:
            command.append("--fixed-rho")
        if arguments.exact_admm:
            command.append("--exact-admm")
        command += list(selector_arguments)
        print(f"[{index}/{total}] run {selector} {method}", flush=True)
        started = time.monotonic()
        with log_path.open("wb") as log_stream:
            return_code, peak_memory_kib, timed_out = run_monitored(
                command, log_stream, arguments.timeout_seconds, environment)
        elapsed = time.monotonic() - started
        if timed_out:
            statuses[key] = status_record(
                arguments, selector, method, repetition, "timeout", return_code,
                elapsed, peak_memory_kib, result_path, log_path,
                f"exceeded {arguments.timeout_seconds:g} seconds",
            )
        else:
            row = read_single_result(partial)
            if return_code == 0 and row is not None:
                os.replace(partial, result_path)
                if partial_trace.is_file():
                    os.replace(partial_trace, final_trace)
                state = "completed"
                message = "completed"
            else:
                state = "error"
                message = "process failed or did not produce exactly one result row"
            statuses[key] = status_record(
                arguments, selector, method, repetition, state, return_code,
                elapsed, peak_memory_kib, result_path, log_path, message,
            )
        write_statuses(status_path, statuses)
        combine_results(combined_path, tasks, statuses)
        print(f"  {statuses[key]['execution_status']} in {elapsed:.2f} s", flush=True)

    combine_results(combined_path, tasks, statuses)
    counts = {state: 0 for state in ("completed", "timeout", "error")}
    for selector, method, repetition, _, _ in tasks:
        state = statuses.get((selector, method, repetition), {}).get("execution_status", "error")
        counts[state] = counts.get(state, 0) + 1
    print(
        f"completed={counts.get('completed', 0)} timeout={counts.get('timeout', 0)} "
        f"error={counts.get('error', 0)} combined={combined_path}",
        flush=True,
    )
    return 0 if counts.get("completed", 0) == total else 1


if __name__ == "__main__":
    sys.exit(main())
