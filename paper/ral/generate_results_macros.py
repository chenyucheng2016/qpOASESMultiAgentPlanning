#!/usr/bin/env python3
"""Generate manuscript result macros from manifest-locked analysis CSVs."""

import argparse
import csv
import hashlib
import json
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
    parser.add_argument("--primary-manifest", type=Path, required=True)
    parser.add_argument("--primary-scale-method-summary", type=Path,
                        required=True)
    parser.add_argument("--paired-aggregate", type=Path, required=True)
    parser.add_argument("--full-vs-inner", type=Path, required=True)
    parser.add_argument("--qp-continuation-vs-inner", type=Path,
                        required=True)
    parser.add_argument("--full-vs-qp-continuation", type=Path, required=True)
    parser.add_argument("--csdo-statistics", type=Path, required=True)
    parser.add_argument("--csdo-summary", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    arguments = parser.parse_args()

    with arguments.primary_manifest.open(encoding="utf-8") as stream:
        primary_manifest = json.load(stream)
    configuration = primary_manifest.get("configuration", {})
    expected_primary = {
        "development": {
            "cases": 240,
            "suite": "paper_development",
            "protocol_id": "ral-monte-carlo-primary-development",
            "methods": {"full", "centralized_osqp"},
        },
        "final": {
            "cases": 720,
            "suite": "paper_final",
            "protocol_id": "ral-monte-carlo-primary-final",
            "methods": {
                "full", "centralized_osqp", "centralized_qpoases"},
        },
    }[arguments.evidence_stage]
    manifest_methods = set(configuration.get("methods", ()))
    if (configuration.get("suite") != expected_primary["suite"]
            or configuration.get("protocol_id")
            != expected_primary["protocol_id"]
            or manifest_methods != expected_primary["methods"]
            or configuration.get("repetitions") != 1
            or configuration.get("scenario_indices") != []
            or configuration.get("fixed_rho") is not True
            or configuration.get("exact_admm") is not True):
        raise ValueError(
            f"primary manifest does not match the frozen "
            f"{arguments.evidence_stage} protocol")
    expected_tasks = expected_primary["cases"] * len(manifest_methods)
    if len(configuration.get("task_order", ())) != expected_tasks:
        raise ValueError(
            f"primary manifest has {len(configuration.get('task_order', ()))} "
            f"tasks; expected {expected_tasks}")

    paired = read_csv(arguments.paired_aggregate)
    overall = unique(paired, "primary overall", scope="overall", value="all")
    scale = {
        agents: unique(paired, f"N={agents}", scope="n", value=str(agents))
        for agents in (4, 8, 12, 20)
    }
    if integer(overall, "paired_attempts") != expected_primary["cases"]:
        raise ValueError(
            f"primary aggregate has {integer(overall, 'paired_attempts')} "
            f"pairs; expected {expected_primary['cases']}")
    expected_scale_cases = expected_primary["cases"] // len(scale)
    if any(integer(row, "paired_attempts") != expected_scale_cases
           for row in scale.values()):
        raise ValueError("primary aggregate has an incomplete team-size stratum")
    if (integer(overall, "candidate_successes") != expected_primary["cases"]
            or integer(overall, "candidate_strict_convergences")
            != expected_primary["cases"]
            or integer(overall, "baseline_successes")
            != expected_primary["cases"]
            or integer(overall, "baseline_strict_convergences")
            != expected_primary["cases"]):
        raise ValueError("primary evidence is not complete, valid, and strict")
    if number(overall, "candidate_absolute_objective_gap_max_percent") > 5.0:
        raise ValueError("primary objective gap exceeds the frozen 5 percent gate")
    primary_method_scale_rows = read_csv(
        arguments.primary_scale_method_summary)
    expected_scale_rows = len(scale) * len(manifest_methods)
    if len(primary_method_scale_rows) != expected_scale_rows:
        raise ValueError(
            f"primary scale-method summary has {len(primary_method_scale_rows)} "
            f"rows; expected {expected_scale_rows}")
    primary_method_scale = {}
    for agents in scale:
        for method in sorted(manifest_methods):
            row = unique(
                primary_method_scale_rows, f"N={agents} {method}",
                n=str(agents), method=method)
            attempted = integer(row, "attempted")
            accounted = integer(row, "completed") + integer(row, "timeouts")
            if (attempted != expected_scale_cases
                    or accounted != attempted
                    or integer(row, "execution_errors") != 0):
                raise ValueError(
                    f"primary scale-method row N={agents} {method} is incomplete")
            primary_method_scale[(agents, method)] = row
    for agents, paired_row in scale.items():
        full_row = primary_method_scale[(agents, "full")]
        osqp_row = primary_method_scale[(agents, "centralized_osqp")]
        if (integer(full_row, "protocol_successes")
                != integer(paired_row, "candidate_successes")
                or integer(osqp_row, "protocol_successes")
                != integer(paired_row, "baseline_successes")
                or not math.isclose(
                    number(full_row, "successful_wall_time_median_ms") / 1000.0,
                    number(paired_row, "candidate_wall_time_median_s"))
                or not math.isclose(
                    number(osqp_row, "successful_wall_time_median_ms") / 1000.0,
                    number(paired_row, "baseline_wall_time_median_s"))):
            raise ValueError(
                f"primary scale summaries disagree for N={agents}")
    inner = unique(read_csv(arguments.full_vs_inner), "inner ablation overall",
                   scope="overall", value="all")
    matrix_continuation = unique(
        read_csv(arguments.qp_continuation_vs_inner),
        "matrix-continuation ablation overall", scope="overall", value="all")
    qp_continuation = unique(
        read_csv(arguments.full_vs_qp_continuation),
        "QP-continuation ablation overall", scope="overall", value="all")
    if len({integer(row, "paired_attempts") for row in
            (inner, matrix_continuation, qp_continuation)}) != 1:
        raise ValueError("ablation paired-attempt counts do not match")
    pair_transport_n20 = unique(
        read_csv(arguments.full_vs_qp_continuation),
        "pair-transport N=20 ablation", scope="n", value="20")
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
            source = row.get("warmstart_source")
            failure_artifacts = {
                "pbs_root": (
                    "pbs_root_guess", "pbs_root_corridor"),
                "independent_single_agent": (
                    "independent_single_agent_guess",
                    "independent_single_agent_corridor"),
            }
            expected_artifacts = failure_artifacts.get(source)
            provenance_valid = provenance_valid and (
                expected_artifacts is not None
                and row.get("turbo_guess_artifact") == expected_artifacts[0]
                and row.get("turbo_corridor_artifact")
                    == expected_artifacts[1]
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
                "CSDO summary does not prove shared-front-end or "
                "failure-artifact provenance for "
                f"{row.get('instance', '<unknown>')}")
    root_failure_rows = [
        row for row in csdo_summary
        if row.get("warmstart_source") == "pbs_root"]
    independent_failure_rows = [
        row for row in csdo_summary
        if row.get("warmstart_source") == "independent_single_agent"]
    if (len(root_failure_rows) != integer(csdo, "pbs_root_failure_cases")
            or len(independent_failure_rows)
            != integer(csdo, "independent_path_failure_cases")):
        raise ValueError("CSDO recovery cohort counts do not match the summary")
    shared_rows = [
        row for row in csdo_summary
        if row.get("csdo_valid", "").strip().lower() == "true"
        and row.get("turbo_valid", "").strip().lower() == "true"]
    if not shared_rows:
        raise ValueError("CSDO comparison contains no shared valid outcomes")

    csdo_walls = [number(row, "csdo_wall_time") for row in csdo_summary]
    turbo_walls = [number(row, "turbo_wall_time") for row in csdo_summary]
    csdo_shared_paths = [
        number(row, "csdo_path_length") for row in shared_rows]
    turbo_shared_paths = [
        number(row, "turbo_path_length") for row in shared_rows]
    csdo_shared_path_median = statistics.median(csdo_shared_paths)
    turbo_shared_path_median = statistics.median(turbo_shared_paths)
    shared_path_reduction = (
        1.0 - turbo_shared_path_median / csdo_shared_path_median)
    commands = {
        "EvidenceStage": arguments.evidence_stage,
        "EvidenceStageLabel": arguments.evidence_stage.title(),
        "PrimaryCases": str(integer(overall, "paired_attempts")),
        "PrimaryCasesPerScale": str(expected_scale_cases),
        "PrimaryStrictConvergences": str(
            integer(overall, "candidate_strict_convergences")),
        "PrimaryOSQPStrictConvergences": str(
            integer(overall, "baseline_strict_convergences")),
        "PrimaryMedianObjectiveGapPercent": f'{number(overall, "candidate_absolute_objective_gap_median_percent"):.3f}',
        "PrimaryPninetyfiveObjectiveGapPercent": f'{number(overall, "candidate_absolute_objective_gap_p95_percent"):.3f}',
        "PrimaryTimeoutSeconds": f'{number(configuration, "timeout_seconds"):.0f}',
        "PrimaryMaximumObjectiveGapPercent": f'{number(overall, "candidate_absolute_objective_gap_max_percent"):.3f}',
        "WallRatioNFour": f'{number(scale[4], "baseline_over_candidate_wall_median"):.3f}',
        "WallRatioNEight": f'{number(scale[8], "baseline_over_candidate_wall_median"):.3f}',
        "WallRatioNTwelve": f'{number(scale[12], "baseline_over_candidate_wall_median"):.3f}',
        "WallRatioNTwenty": f'{number(scale[20], "baseline_over_candidate_wall_median"):.3f}',
        "MemoryRatioNTwenty": f'{number(scale[20], "baseline_peak_memory_median_kib") / number(scale[20], "candidate_peak_memory_median_kib"):.1f}',
        "WallSignPNtwelve": scientific_latex(
            number(scale[12], "wall_speedup_exact_sign_p")),
        "WallSignPNtwenty": scientific_latex(
            number(scale[20], "wall_speedup_exact_sign_p")),
        "AblationCases": str(integer(inner, "paired_attempts")),
        "FullContinuationRuntimeReductionPercent": f'{100.0 * (1.0 - 1.0 / number(inner, "baseline_over_candidate_wall_median")):.1f}',
        "MatrixContinuationRuntimeReductionPercent": f'{100.0 * (1.0 - 1.0 / number(matrix_continuation, "baseline_over_candidate_wall_median")):.1f}',
        "PairTransportRuntimeReductionPercent": f'{100.0 * (1.0 - 1.0 / number(qp_continuation, "baseline_over_candidate_wall_median")):.1f}',
        "PairTransportRuntimeReductionNTwentyPercent": f'{100.0 * (1.0 - 1.0 / number(pair_transport_n20, "baseline_over_candidate_wall_median")):.1f}',
        "PairTransportQPReductionPercent": f'{number(qp_continuation, "candidate_qp_solve_reduction_median_percent"):.1f}',
        "CSDOCases": str(integer(csdo, "cases")),
        "TurboCSDOSuccesses": str(integer(csdo, "turbo_successes")),
        "CSDOSuccesses": str(integer(csdo, "csdo_successes")),
        "CSDOBothValid": str(integer(csdo, "both_valid")),
        "TurboOnlyValid": str(integer(csdo, "turbo_only")),
        "CSDOOnlyValid": str(integer(csdo, "csdo_only")),
        "CSDONeitherValid": str(integer(csdo, "neither_valid")),
        "CSDOMcNemarP": scientific_latex(number(csdo, "mcnemar_exact_p_value")),
        "PBSFailureCases": str(integer(csdo, "pbs_failure_cases")),
        "PBSRootFailureCases": str(
            integer(csdo, "pbs_root_failure_cases")),
        "IndependentPathFailureCases": str(
            integer(csdo, "independent_path_failure_cases")),
        "TurboPBSRecoveries": str(integer(csdo, "turbo_pbs_recoveries")),
        "TurboPBSRecoveryPercent": f'{100.0 * number(csdo, "turbo_pbs_recovery_rate"):.1f}',
        "CSDOWilsonLowPercent": f'{100.0 * number(csdo, "csdo_wilson_lower"):.1f}',
        "CSDOWilsonHighPercent": f'{100.0 * number(csdo, "csdo_wilson_upper"):.1f}',
        "TurboWilsonLowPercent": f'{100.0 * number(csdo, "turbo_wilson_lower"):.1f}',
        "TurboWilsonHighPercent": f'{100.0 * number(csdo, "turbo_wilson_upper"):.1f}',
        "TurboCSDOMedianWallSeconds": f"{statistics.median(turbo_walls):.2f}",
        "CSDOMedianWallSeconds": f"{statistics.median(csdo_walls):.3f}",
        "CSDOSharedPathMedianMeters": f"{csdo_shared_path_median:.2f}",
        "TurboSharedPathMedianMeters": f"{turbo_shared_path_median:.2f}",
        "SharedPathReductionPercent": f"{100.0 * shared_path_reduction:.1f}",
    }

    scale_labels = {4: "Four", 8: "Eight", 12: "Twelve", 20: "Twenty"}
    method_prefixes = {
        "full": "Turbo",
        "centralized_osqp": "OSQP",
        "centralized_qpoases": "QpOASES",
    }
    for agents, scale_label in scale_labels.items():
        for method in sorted(manifest_methods):
            row = primary_method_scale[(agents, method)]
            prefix = method_prefixes[method]
            successes = integer(row, "protocol_successes")
            attempts = integer(row, "attempted")
            wall_ms = float(row["successful_wall_time_median_ms"])
            wall_seconds = (
                f"{wall_ms / 1000.0:.2f}" if math.isfinite(wall_ms) else "--")
            commands[f"Primary{prefix}SuccessN{scale_label}"] = (
                f"{successes}/{attempts}")
            commands[f"Primary{prefix}WallN{scale_label}"] = wall_seconds

    lines = ["% Generated by paper/ral/generate_results_macros.py; do not edit."]
    for path in (arguments.primary_manifest,
                 arguments.primary_scale_method_summary,
                 arguments.paired_aggregate,
                 arguments.full_vs_inner,
                 arguments.qp_continuation_vs_inner,
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
