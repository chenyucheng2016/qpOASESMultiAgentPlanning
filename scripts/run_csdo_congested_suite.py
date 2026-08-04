#!/usr/bin/env python3
"""Run every frozen CSDO/Turbo congested case without selecting outcomes."""

import argparse
import csv
import pathlib
import subprocess
import sys


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", required=True, type=pathlib.Path)
    parser.add_argument("--csdo-executable", required=True, type=pathlib.Path)
    parser.add_argument("--turbo-executable", required=True, type=pathlib.Path)
    parser.add_argument("--root-exporter", required=True, type=pathlib.Path)
    parser.add_argument("--csdo-config", required=True, type=pathlib.Path)
    parser.add_argument("--work-root", required=True, type=pathlib.Path)
    parser.add_argument("--output-csv", required=True, type=pathlib.Path)
    parser.add_argument("--timeout", type=float, default=7200.0)
    return parser.parse_args()


def main():
    args = parse_args()
    status_csv = args.output_csv.with_name(
        f"{args.output_csv.stem}_status.csv")
    for path in (args.output_csv, status_csv):
        if path.exists():
            raise RuntimeError(f"refusing to append to existing {path}")
    with args.manifest.open(newline="", encoding="utf-8") as stream:
        cases = list(csv.DictReader(stream))
    if not cases:
        raise RuntimeError("manifest contains no cases")

    runner = pathlib.Path(__file__).with_name(
        "run_csdo_interaction_comparison.py")
    statuses = []
    for case in cases:
        instance = args.manifest.parent / case["instance"]
        work_dir = args.work_root / instance.stem
        threads = max(1, int(case["agents"]) // 2)
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
            "--output-csv", str(args.output_csv),
            "--threads", str(threads),
            "--timeout", str(args.timeout),
        ]
        print(f"running {instance.name}", flush=True)
        completed = subprocess.run(command, check=False)
        statuses.append((instance.name, completed.returncode))

    with status_csv.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(("instance", "runner_return_code"))
        writer.writerows(statuses)
    failures = [name for name, code in statuses if code != 0]
    if failures:
        raise RuntimeError("suite runner failures: " + ", ".join(failures))


if __name__ == "__main__":
    main()
