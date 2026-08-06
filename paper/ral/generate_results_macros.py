#!/usr/bin/env python3
"""Generate manuscript result macros from manifest-locked analysis CSVs."""

import argparse
import csv
import hashlib
import math
import statistics
from pathlib import Path


def read_csv(path):
    with path.open(newline="", encoding="utf-8-sig") as stream:
        return list(csv.DictReader(stream))


def number(row, field):
    try:
        value = float(row[field])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"missing numeric field {field}") from error
    if not math.isfinite(value):
        raise ValueError(f"non-finite numeric field {field}")
    return value


def integer(row, field):
    value = number(row, field)
    if value != round(value):
        raise ValueError(f"field {field} is not an integer")
    return int(value)


def unique(rows, description, **fields):
    matches = [
        row for row in rows
        if all(row.get(field) == value for field, value in fields.items())
    ]
    if len(matches) != 1:
        raise ValueError(f"expected one {description} row, found {len(matches)}")
    return matches[0]


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def scientific_latex(value):
    if value <= 0.0:
        raise ValueError("p-value must be positive")
    exponent = math.floor(math.log10(value))
    coefficient = value / (10.0 ** exponent)
    return rf"{coefficient:.2f}\times10^{{{exponent}}}"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--evidence-stage", choices=("development", "final"),
                        required=True)
    parser.add_argument("--paired-aggregate", type=Path, required=True)
    parser.add_argument("--full-vs-inner", type=Path, required=True)
    parser.add_argument("--full-vs-qp-continuation", type=Path, required=True)
    parser.add_argument("--csdo-statistics", type=Path, required=True)
    parser.add_argument("--csdo-summary", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    arguments = parser.parse_args()

    paired = read_csv(arguments.paired_aggregate)
    overall = unique(paired, "primary overall", scope="overall", value="all")
    scale = {
        agents: unique(paired, f"N={agents}", scope="n", value=str(agents))
        for agents in (4, 8, 12, 20)
    }
    inner = unique(read_csv(arguments.full_vs_inner), "inner ablation overall",
                   scope="overall", value="all")
    qp_continuation = unique(
        read_csv(arguments.full_vs_qp_continuation),
        "QP-continuation ablation overall", scope="overall", value="all")
    if integer(inner, "paired_attempts") != integer(
            qp_continuation, "paired_attempts"):
        raise ValueError("ablation paired-attempt counts do not match")
    csdo_rows = read_csv(arguments.csdo_statistics)
    if len(csdo_rows) != 1:
        raise ValueError("expected one CSDO statistics row")
    csdo = csdo_rows[0]
    csdo_summary = read_csv(arguments.csdo_summary)
    if len(csdo_summary) != integer(csdo, "cases"):
        raise ValueError("CSDO summary row count does not match paired statistics")
    for row in csdo_summary:
        provenance_valid = (
            row.get("mode") == "recovery"
            and row.get("search_seed") == "0")
        pbs_success = row.get("pbs_success", "").strip().lower()
        if pbs_success == "true":
            schedule_hashes = (
                row.get("csdo_initial_schedule_sha256"),
                row.get("root_schedule_sha256"),
                row.get("turbo_schedule_sha256"),
            )
            provenance_valid = provenance_valid and (
                row.get("warmstart_source") == "pbs_goal"
                and row.get("turbo_guess_artifact")
                    == "csdo_initial_guess"
                and row.get("turbo_corridor_artifact")
                    == "csdo_initial_corridor_reconstruction"
                and row.get("shared_schedule_match", "").lower() == "true"
                and len(set(schedule_hashes)) == 1
                and all(schedule_hashes)
                and row.get("turbo_guess_sha256")
                    == row.get("csdo_initial_guess_sha256")
                and row.get("turbo_corridor_sha256")
                    == row.get("root_corridor_sha256"))
        elif pbs_success == "false":
            root_conflicts = max(
                integer(row, "root_conflicting_pairs"),
                integer(row, "warmstart_conflicting_pairs"))
            provenance_valid = provenance_valid and (
                row.get("warmstart_source") == "pbs_root"
                and row.get("turbo_guess_artifact") == "pbs_root_guess"
                and row.get("turbo_corridor_artifact")
                    == "pbs_root_corridor"
                and row.get("turbo_guess_sha256")
                    == row.get("root_guess_sha256")
                and row.get("turbo_corridor_sha256")
                    == row.get("root_corridor_sha256")
                and row.get("turbo_schedule_sha256")
                    == row.get("root_schedule_sha256")
                and bool(row.get("root_schedule_sha256"))
                and root_conflicts > 0)
        else:
            raise ValueError("CSDO summary contains an invalid PBS status")
        if not provenance_valid:
            raise ValueError(
                "CSDO summary does not prove shared-front-end provenance for "
                f"{row.get('instance', '<unknown>')}")

    csdo_walls = [number(row, "csdo_wall_time") for row in csdo_summary]
    turbo_walls = [number(row, "turbo_wall_time") for row in csdo_summary]
    commands = {
        "EvidenceStage": arguments.evidence_stage,
        "PrimaryCases": str(integer(overall, "paired_attempts")),
        "PrimaryStrictConvergences": str(
            integer(overall, "candidate_strict_convergences")),
        "PrimaryOSQPStrictConvergences": str(
            integer(overall, "baseline_strict_convergences")),
        "PrimaryMaximumObjectiveGapPercent": f'{number(overall, "candidate_absolute_objective_gap_max_percent"):.3f}',
        "WallRatioNFour": f'{number(scale[4], "baseline_over_candidate_wall_median"):.3f}',
        "WallRatioNEight": f'{number(scale[8], "baseline_over_candidate_wall_median"):.3f}',
        "WallRatioNTwelve": f'{number(scale[12], "baseline_over_candidate_wall_median"):.3f}',
        "WallRatioNTwenty": f'{number(scale[20], "baseline_over_candidate_wall_median"):.3f}',
        "AblationCases": str(integer(inner, "paired_attempts")),
        "InnerRuntimeReductionPercent": f'{100.0 * (1.0 - 1.0 / number(inner, "baseline_over_candidate_wall_median")):.1f}',
        "QPContinuationRuntimeReductionPercent": f'{100.0 * (1.0 - 1.0 / number(qp_continuation, "baseline_over_candidate_wall_median")):.1f}',
        "ContinuationQPReductionPercent": f'{number(inner, "candidate_qp_solve_reduction_median_percent"):.1f}',
        "CSDOCases": str(integer(csdo, "cases")),
        "TurboCSDOSuccesses": str(integer(csdo, "turbo_successes")),
        "CSDOSuccesses": str(integer(csdo, "csdo_successes")),
        "CSDOBothValid": str(integer(csdo, "both_valid")),
        "TurboOnlyValid": str(integer(csdo, "turbo_only")),
        "CSDOOnlyValid": str(integer(csdo, "csdo_only")),
        "CSDONeitherValid": str(integer(csdo, "neither_valid")),
        "CSDOMcNemarP": scientific_latex(number(csdo, "mcnemar_exact_p_value")),
        "PBSFailureCases": str(integer(csdo, "pbs_failure_cases")),
        "TurboPBSRecoveries": str(integer(csdo, "turbo_pbs_recoveries")),
        "TurboPBSRecoveryPercent": f'{100.0 * number(csdo, "turbo_pbs_recovery_rate"):.1f}',
        "CSDOWilsonLowPercent": f'{100.0 * number(csdo, "csdo_wilson_lower"):.1f}',
        "CSDOWilsonHighPercent": f'{100.0 * number(csdo, "csdo_wilson_upper"):.1f}',
        "TurboWilsonLowPercent": f'{100.0 * number(csdo, "turbo_wilson_lower"):.1f}',
        "TurboWilsonHighPercent": f'{100.0 * number(csdo, "turbo_wilson_upper"):.1f}',
        "TurboCSDOMedianWallSeconds": f"{statistics.median(turbo_walls):.2f}",
        "CSDOMedianWallSeconds": f"{statistics.median(csdo_walls):.3f}",
    }

    lines = ["% Generated by paper/ral/generate_results_macros.py; do not edit."]
    for path in (arguments.paired_aggregate, arguments.full_vs_inner,
                 arguments.full_vs_qp_continuation, arguments.csdo_statistics,
                 arguments.csdo_summary):
        lines.append(f"% {path.name} sha256={sha256(path)}")
    for name, value in commands.items():
        lines.append(rf"\newcommand{{\{name}}}{{{value}}}")
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    arguments.output.write_text("\n".join(lines) + "\n", encoding="ascii")
    print(f"generated {arguments.output}")


if __name__ == "__main__":
    main()
