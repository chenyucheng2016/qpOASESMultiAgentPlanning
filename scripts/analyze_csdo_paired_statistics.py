#!/usr/bin/env python3
"""Compute paired success statistics for a frozen CSDO comparison."""

import argparse
import csv
import math
from pathlib import Path


FIELDS = (
    "cases",
    "turbo_successes",
    "turbo_success_rate",
    "turbo_wilson_lower",
    "turbo_wilson_upper",
    "csdo_successes",
    "csdo_success_rate",
    "csdo_wilson_lower",
    "csdo_wilson_upper",
    "success_rate_difference",
    "both_valid",
    "turbo_only",
    "csdo_only",
    "neither_valid",
    "mcnemar_exact_p_value",
    "pbs_failure_cases",
    "turbo_pbs_recoveries",
    "turbo_pbs_recovery_rate",
    "turbo_pbs_recovery_wilson_lower",
    "turbo_pbs_recovery_wilson_upper",
    "csdo_pbs_recoveries",
    "csdo_pbs_recovery_rate",
)


def boolean(value):
    normalized = value.strip().lower()
    if normalized in ("1", "true", "yes"):
        return True
    if normalized in ("0", "false", "no"):
        return False
    raise ValueError(f"invalid boolean value: {value!r}")


def wilson(successes, total):
    if total == 0:
        return float("nan"), float("nan")
    z = 1.959963984540054
    proportion = successes / total
    denominator = 1.0 + z * z / total
    center = (proportion + z * z / (2.0 * total)) / denominator
    margin = z * math.sqrt(
        proportion * (1.0 - proportion) / total
        + z * z / (4.0 * total * total)
    ) / denominator
    return center - margin, center + margin


def exact_mcnemar(turbo_only, csdo_only):
    discordant = turbo_only + csdo_only
    if discordant == 0:
        return 1.0
    tail = sum(
        math.comb(discordant, index)
        for index in range(min(turbo_only, csdo_only) + 1)
    ) / (2.0 ** discordant)
    return min(1.0, 2.0 * tail)


def analyze(rows):
    cases = len(rows)
    paired = {(True, True): 0, (True, False): 0,
              (False, True): 0, (False, False): 0}
    pbs_failures = []
    for row in rows:
        csdo_valid = boolean(row["csdo_valid"])
        turbo_valid = boolean(row["turbo_valid"])
        paired[(turbo_valid, csdo_valid)] += 1
        if not boolean(row["pbs_success"]):
            pbs_failures.append((turbo_valid, csdo_valid))

    both_valid = paired[(True, True)]
    turbo_only = paired[(True, False)]
    csdo_only = paired[(False, True)]
    neither_valid = paired[(False, False)]
    turbo_successes = both_valid + turbo_only
    csdo_successes = both_valid + csdo_only
    turbo_lower, turbo_upper = wilson(turbo_successes, cases)
    csdo_lower, csdo_upper = wilson(csdo_successes, cases)
    turbo_recoveries = sum(turbo for turbo, _ in pbs_failures)
    csdo_recoveries = sum(csdo for _, csdo in pbs_failures)
    recovery_lower, recovery_upper = wilson(
        turbo_recoveries, len(pbs_failures))

    return {
        "cases": cases,
        "turbo_successes": turbo_successes,
        "turbo_success_rate": turbo_successes / cases,
        "turbo_wilson_lower": turbo_lower,
        "turbo_wilson_upper": turbo_upper,
        "csdo_successes": csdo_successes,
        "csdo_success_rate": csdo_successes / cases,
        "csdo_wilson_lower": csdo_lower,
        "csdo_wilson_upper": csdo_upper,
        "success_rate_difference": (turbo_successes - csdo_successes) / cases,
        "both_valid": both_valid,
        "turbo_only": turbo_only,
        "csdo_only": csdo_only,
        "neither_valid": neither_valid,
        "mcnemar_exact_p_value": exact_mcnemar(turbo_only, csdo_only),
        "pbs_failure_cases": len(pbs_failures),
        "turbo_pbs_recoveries": turbo_recoveries,
        "turbo_pbs_recovery_rate": turbo_recoveries / len(pbs_failures),
        "turbo_pbs_recovery_wilson_lower": recovery_lower,
        "turbo_pbs_recovery_wilson_upper": recovery_upper,
        "csdo_pbs_recoveries": csdo_recoveries,
        "csdo_pbs_recovery_rate": csdo_recoveries / len(pbs_failures),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("summary", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    arguments = parser.parse_args()
    with arguments.summary.open(newline="", encoding="utf-8") as stream:
        result = analyze(list(csv.DictReader(stream)))
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    with arguments.output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerow(result)
    print(
        f"Turbo {result['turbo_successes']}/{result['cases']}, "
        f"CSDO {result['csdo_successes']}/{result['cases']}, "
        f"exact McNemar p={result['mcnemar_exact_p_value']:.8g}"
    )


if __name__ == "__main__":
    main()
