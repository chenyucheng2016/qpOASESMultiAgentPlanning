#!/usr/bin/env python3
"""Enforce the frozen completeness and paired-success gates for CSDO."""

import argparse
import csv
import math
from pathlib import Path


def as_float(row, field):
    try:
        return float(row[field])
    except (KeyError, TypeError, ValueError):
        return math.nan


def as_int(row, field):
    value = as_float(row, field)
    return int(value) if math.isfinite(value) else -1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("summary", type=Path)
    parser.add_argument("paired_statistics", type=Path)
    parser.add_argument("--expected-cases", type=int, default=32)
    parser.add_argument("--minimum-turbo-successes", type=int, default=30)
    parser.add_argument("--minimum-pbs-recovery-rate", type=float, default=0.8)
    parser.add_argument("--maximum-mcnemar-p", type=float, default=0.05)
    arguments = parser.parse_args()
    if arguments.expected_cases <= 0:
        parser.error("--expected-cases must be positive")
    if not 0.0 <= arguments.minimum_pbs_recovery_rate <= 1.0:
        parser.error("--minimum-pbs-recovery-rate must be in [0, 1]")
    if not 0.0 <= arguments.maximum_mcnemar_p <= 1.0:
        parser.error("--maximum-mcnemar-p must be in [0, 1]")

    with arguments.summary.open(newline="", encoding="utf-8-sig") as stream:
        cases = list(csv.DictReader(stream))
    with arguments.paired_statistics.open(
            newline="", encoding="utf-8-sig") as stream:
        statistics_rows = list(csv.DictReader(stream))
    failures = []
    if len(cases) != arguments.expected_cases:
        failures.append(
            f"expected {arguments.expected_cases} cases, found {len(cases)}")
    instances = [row.get("instance", "") for row in cases]
    if any(not instance for instance in instances):
        failures.append("one or more cases have no instance identifier")
    if len(instances) != len(set(instances)):
        failures.append("summary contains duplicate instances")
    incomplete = [
        instance for instance, row in zip(instances, cases)
        if row.get("runner_state") not in ("completed", "recovered")
    ]
    if incomplete:
        failures.append(f"{len(incomplete)} suite tasks are incomplete")
    if len(statistics_rows) != 1:
        failures.append(
            f"expected one paired-statistics row, found {len(statistics_rows)}")
        statistics = {}
    else:
        statistics = statistics_rows[0]

    turbo_successes = as_int(statistics, "turbo_successes")
    csdo_successes = as_int(statistics, "csdo_successes")
    turbo_only = as_int(statistics, "turbo_only")
    csdo_only = as_int(statistics, "csdo_only")
    recovery_rate = as_float(statistics, "turbo_pbs_recovery_rate")
    mcnemar_p = as_float(statistics, "mcnemar_exact_p_value")
    if turbo_successes < arguments.minimum_turbo_successes:
        failures.append(
            f"Turbo successes {turbo_successes} are below {arguments.minimum_turbo_successes}")
    if turbo_successes <= csdo_successes or turbo_only <= csdo_only:
        failures.append("paired outcomes do not favor TurboADMM-NL")
    if (not math.isfinite(recovery_rate)
            or recovery_rate < arguments.minimum_pbs_recovery_rate):
        failures.append(
            f"PBS recovery rate {recovery_rate:.6g} is below "
            f"{arguments.minimum_pbs_recovery_rate:.6g}")
    if not math.isfinite(mcnemar_p) or mcnemar_p > arguments.maximum_mcnemar_p:
        failures.append(
            f"McNemar p-value {mcnemar_p:.6g} exceeds "
            f"{arguments.maximum_mcnemar_p:.6g}")

    print(
        f"cases={len(cases)}/{arguments.expected_cases} "
        f"turbo={turbo_successes} csdo={csdo_successes} "
        f"turbo_only={turbo_only} csdo_only={csdo_only} "
        f"pbs_recovery_rate={recovery_rate:.9g} mcnemar_p={mcnemar_p:.9g}")
    if failures:
        raise SystemExit("RA-L CSDO gate failed:\n- " + "\n- ".join(failures))


if __name__ == "__main__":
    main()
