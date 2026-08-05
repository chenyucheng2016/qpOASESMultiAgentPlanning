#!/usr/bin/env python3
"""Run every frozen CSDO/Turbo congested case without selecting outcomes."""

import argparse
import csv
import hashlib
import json
import pathlib
import random
import subprocess
import sys
import time


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", required=True, type=pathlib.Path)
    parser.add_argument("--csdo-executable", required=True, type=pathlib.Path)
    parser.add_argument("--turbo-executable", required=True, type=pathlib.Path)
    parser.add_argument("--root-exporter", required=True, type=pathlib.Path)
    parser.add_argument("--csdo-config", required=True, type=pathlib.Path)
    parser.add_argument("--work-root", required=True, type=pathlib.Path)
    parser.add_argument("--output-csv", required=True, type=pathlib.Path)
    parser.add_argument("--protocol-id", required=True)
    parser.add_argument("--git-commit", required=True)
    parser.add_argument("--schedule-seed", type=int, default=20260804)
    parser.add_argument(
        "--warmstart-policy",
        choices=("auto", "pbs_root", "independent"),
        default="auto")
    parser.add_argument("--corridor-recovery-window", type=int, default=0)
    parser.add_argument("--minimum-horizon", type=int, default=0)
    parser.add_argument("--timeout", type=float, default=600.0)
    parser.add_argument("--resume", action="store_true")
    return parser.parse_args()


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def file_record(path):
    resolved = path.resolve()
    if not resolved.is_file():
        raise RuntimeError(f"required file does not exist: {resolved}")
    return {"path": str(resolved), "sha256": sha256(resolved)}


def write_json(path, value):
    with path.open("w", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, sort_keys=True)
        stream.write("\n")


def append_status(path, row, fields):
    exists = path.exists() and path.stat().st_size > 0
    with path.open("a", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        if not exists:
            writer.writeheader()
        writer.writerow(row)


def load_statuses(path):
    if not path.exists():
        return {}
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    return {row["task_id"]: row for row in rows}


def load_case_result(path):
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) != 1:
        raise RuntimeError(f"expected one result row in {path}")
    return rows[0]


def write_aggregate(path, tasks, work_root):
    rows = []
    for task in tasks:
        case_path = work_root / pathlib.Path(task["task_id"]).stem / "result.csv"
        if case_path.exists():
            rows.append(load_case_result(case_path))
    if not rows:
        return
    fields = list(rows[0])
    if any(list(row) != fields for row in rows[1:]):
        raise RuntimeError("per-case result schemas do not match")
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def main():
    args = parse_args()
    status_csv = args.output_csv.with_name(
        f"{args.output_csv.stem}_status.csv")
    run_manifest = args.output_csv.with_name(
        f"{args.output_csv.stem}_manifest.json")
    args.output_csv.parent.mkdir(parents=True, exist_ok=True)
    args.work_root.mkdir(parents=True, exist_ok=True)
    if not args.resume:
        for path in (args.output_csv, status_csv, run_manifest):
            if path.exists():
                raise RuntimeError(f"refusing to overwrite existing {path}")
    with args.manifest.open(newline="", encoding="utf-8") as stream:
        cases = list(csv.DictReader(stream))
    if not cases:
        raise RuntimeError("manifest contains no cases")
    if any(not case.get("instance") for case in cases):
        raise RuntimeError("every manifest row must define instance")
    task_ids = [case["instance"] for case in cases]
    if len(task_ids) != len(set(task_ids)):
        raise RuntimeError("manifest instance names must be unique")

    runner = pathlib.Path(__file__).with_name(
        "run_csdo_interaction_comparison.py")
    comparison_library = pathlib.Path(__file__).with_name(
        "run_csdo_turbo_comparison.py")
    summarizer = pathlib.Path(__file__).with_name(
        "summarize_csdo_congested_suite.py")
    ordered_cases = list(cases)
    random.Random(args.schedule_seed).shuffle(ordered_cases)
    tasks = []
    for sequence, case in enumerate(ordered_cases):
        instance = args.manifest.parent / case["instance"]
        warmstart_policy = (
            case.get("warmstart_policy") or args.warmstart_policy)
        minimum_horizon = int(
            case.get("minimum_horizon") or args.minimum_horizon)
        tasks.append({
            "task_id": case["instance"],
            "sequence": sequence,
            "instance": file_record(instance),
            "case": case,
            "threads": max(1, int(case["agents"]) // 2),
            "warmstart_policy": warmstart_policy,
            "minimum_horizon": minimum_horizon,
        })
    manifest_value = {
        "schema_version": 1,
        "protocol_id": args.protocol_id,
        "git_commit": args.git_commit,
        "schedule_seed": args.schedule_seed,
        "timeout_s": args.timeout,
        "corridor_recovery_window": args.corridor_recovery_window,
        "warmstart_policy": args.warmstart_policy,
        "minimum_horizon": args.minimum_horizon,
        "files": {
            "case_manifest": file_record(args.manifest),
            "suite_runner": file_record(pathlib.Path(__file__)),
            "case_runner": file_record(runner),
            "comparison_library": file_record(comparison_library),
            "summarizer": file_record(summarizer),
            "csdo_executable": file_record(args.csdo_executable),
            "turbo_executable": file_record(args.turbo_executable),
            "root_exporter": file_record(args.root_exporter),
            "csdo_config": file_record(args.csdo_config),
        },
        "tasks": tasks,
    }
    if args.resume:
        if not run_manifest.exists():
            raise RuntimeError("resume requested without a run manifest")
        with run_manifest.open(encoding="utf-8") as stream:
            previous_manifest = json.load(stream)
        if previous_manifest != manifest_value:
            raise RuntimeError("resume configuration does not match run manifest")
    else:
        write_json(run_manifest, manifest_value)

    status_fields = (
        "task_id", "sequence", "instance", "family", "mode", "agents",
        "threads", "warmstart_policy", "minimum_horizon", "state",
        "runner_return_code", "elapsed_s",
    )
    completed_tasks = load_statuses(status_csv)
    noncompleted = []
    for task in tasks:
        task_id = task["task_id"]
        if task_id in completed_tasks:
            print(f"skipping recorded task {task_id}", flush=True)
            continue
        case = task["case"]
        instance = args.manifest.parent / case["instance"]
        work_dir = args.work_root / instance.stem
        work_dir.mkdir(parents=True, exist_ok=True)
        case_result = work_dir / "result.csv"
        if case_result.exists():
            row = {
                "task_id": task_id,
                "sequence": task["sequence"],
                "instance": instance.name,
                "family": case.get("family", ""),
                "mode": case["mode"],
                "agents": case["agents"],
                "threads": task["threads"],
                "warmstart_policy": task["warmstart_policy"],
                "minimum_horizon": task["minimum_horizon"],
                "state": "recovered",
                "runner_return_code": 0,
                "elapsed_s": "",
            }
            append_status(status_csv, row, status_fields)
            print(f"recovered completed task {task_id}", flush=True)
            continue
        command = [
            sys.executable,
            str(runner),
            "--mode", case["mode"],
            "--instance", str(instance),
            "--csdo-executable", str(args.csdo_executable),
            "--turbo-executable", str(args.turbo_executable),
            "--root-exporter", str(args.root_exporter),
            "--csdo-config", str(args.csdo_config),
            "--work-dir", str(work_dir),
            "--output-csv", str(case_result),
            "--threads", str(task["threads"]),
            "--warmstart-policy", task["warmstart_policy"],
            "--minimum-horizon",
            str(task["minimum_horizon"]),
            "--corridor-recovery-window",
            str(args.corridor_recovery_window),
            "--timeout", str(args.timeout),
        ]
        print(f"running {instance.name}", flush=True)
        start = time.perf_counter()
        completed = subprocess.run(
            command, check=False, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, text=True)
        elapsed = time.perf_counter() - start
        (work_dir / "suite_runner.log").write_text(
            completed.stdout, encoding="utf-8")
        state = "completed" if completed.returncode == 0 else "failed"
        row = {
            "task_id": task_id,
            "sequence": task["sequence"],
            "instance": instance.name,
            "family": case.get("family", ""),
            "mode": case["mode"],
            "agents": case["agents"],
            "threads": task["threads"],
            "warmstart_policy": task["warmstart_policy"],
            "minimum_horizon": task["minimum_horizon"],
            "state": state,
            "runner_return_code": completed.returncode,
            "elapsed_s": f"{elapsed:.9f}",
        }
        append_status(status_csv, row, status_fields)
        if state != "completed":
            noncompleted.append(instance.name)
    write_aggregate(args.output_csv, tasks, args.work_root)
    if noncompleted:
        print("recorded non-completed outcomes: " + ", ".join(noncompleted))


if __name__ == "__main__":
    main()
