#!/usr/bin/env python3
"""Generate RA-L figures from manifest-locked benchmark CSV artifacts."""

import argparse
import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import yaml


COLORS = ("#0072B2", "#D55E00", "#009E73", "#CC79A7")


def read_csv(path):
    with path.open(newline="", encoding="utf-8-sig") as stream:
        return list(csv.DictReader(stream))


def number(row, field):
    try:
        return float(row[field])
    except (KeyError, TypeError, ValueError):
        return math.nan


def finite(row, preferred, fallback):
    value = number(row, preferred)
    return value if math.isfinite(value) else number(row, fallback)


def save_figure(figure, output_dir, stem):
    for suffix in ("pdf", "png"):
        figure.savefig(
            output_dir / f"{stem}.{suffix}", bbox_inches="tight",
            dpi=300, metadata={"Creator": "paper/ral/generate_figures.py"})
    plt.close(figure)


def configure_style():
    plt.rcParams.update({
        "font.family": "serif",
        "font.size": 8,
        "axes.labelsize": 8,
        "axes.titlesize": 8,
        "legend.fontsize": 7,
        "xtick.labelsize": 7,
        "ytick.labelsize": 7,
        "axes.linewidth": 0.7,
        "lines.linewidth": 1.4,
        "pdf.fonttype": 42,
        "ps.fonttype": 42,
    })


def scaling_figure(rows, output_dir):
    rows = sorted(
        (row for row in rows if row.get("scope") == "n"),
        key=lambda row: int(row["value"]))
    if not rows:
        raise ValueError("paired aggregate has no scale rows")
    agents = np.asarray([int(row["value"]) for row in rows])
    medians = np.asarray([
        finite(row, "baseline_over_candidate_wall_median",
               "baseline_over_candidate_median") for row in rows])
    lower = np.asarray([
        finite(row, "baseline_over_candidate_wall_q25",
               "baseline_over_candidate_q25") for row in rows])
    upper = np.asarray([
        finite(row, "baseline_over_candidate_wall_q75",
               "baseline_over_candidate_q75") for row in rows])
    if not np.all(np.isfinite(np.r_[medians, lower, upper])):
        raise ValueError("scaling aggregate has missing speedup quantiles")

    figure, axis = plt.subplots(figsize=(3.45, 2.25), constrained_layout=True)
    axis.axhline(1.0, color="0.35", linewidth=0.8, linestyle="--", zorder=1)
    axis.fill_between(agents, lower, upper, color=COLORS[0], alpha=0.18,
                      linewidth=0, label="Interquartile range")
    axis.plot(agents, medians, color=COLORS[0], marker="o", markersize=4,
              label="Median OSQP / TurboADMM-NL")
    for index, (x_value, value) in enumerate(zip(agents, medians)):
        if index == 0:
            offset, alignment = (0, -15), "center"
        elif index + 1 == len(agents):
            offset, alignment = (-4, 5), "right"
        else:
            offset, alignment = (0, 5), "center"
        axis.annotate(f"{value:.2f}x", (x_value, value), xytext=offset,
                      textcoords="offset points", ha=alignment, fontsize=7)
    axis.set_xlabel("Number of agents")
    axis.set_ylabel("Process-wall speedup")
    axis.set_xticks(agents)
    axis.set_ylim(bottom=max(0.0, min(lower) - 0.25),
                  top=max(upper) + 0.35)
    axis.grid(axis="y", color="0.88", linewidth=0.5)
    axis.legend(frameon=False, loc="upper left")
    save_figure(figure, output_dir, "scaling-crossover")


def overall_row(rows):
    matches = [
        row for row in rows
        if row.get("scope") == "overall" and row.get("value") == "all"]
    if len(matches) != 1:
        raise ValueError("expected exactly one overall aggregate row")
    return matches[0]


def ablation_figure(inner_rows, matrix_rows, qp_rows, output_dir):
    rows = [overall_row(matrix_rows), overall_row(qp_rows),
            overall_row(inner_rows)]
    labels = ("Matrix/WS\ncontinuation", "Pair-state\ntransport",
              "Full cross-SCP\ncontinuation")
    speedup = np.asarray([
        finite(row, "baseline_over_candidate_wall_median",
               "baseline_over_candidate_median") for row in rows])
    qp_reduction = np.asarray([
        number(row, "candidate_qp_solve_reduction_median_percent")
        for row in rows])
    if not np.all(np.isfinite(np.r_[speedup, qp_reduction])):
        raise ValueError("ablation aggregate has missing metrics")

    figure, axes = plt.subplots(1, 2, figsize=(7.0, 2.45),
                                constrained_layout=True)
    x_values = np.arange(len(labels))
    bars = axes[0].bar(x_values, speedup, width=0.58, color=COLORS[0])
    axes[0].axhline(1.0, color="0.35", linewidth=0.8, linestyle="--")
    axes[0].bar_label(bars, fmt="%.2fx", padding=2, fontsize=7)
    axes[0].set_ylabel("Comparison speedup")
    axes[0].set_xticks(x_values, labels)
    axes[0].set_title("Continuation speedup")
    axes[0].set_ylim(0.0, max(speedup) * 1.28)
    axes[0].grid(axis="y", color="0.88", linewidth=0.5)

    bars = axes[1].bar(x_values, qp_reduction, width=0.58, color=COLORS[2])
    axes[1].bar_label(bars, fmt="%.1f%%", padding=2, fontsize=7)
    axes[1].set_ylabel("Median local-QP reduction")
    axes[1].set_xticks(x_values, labels)
    axes[1].set_title("Solver work reduction")
    axes[1].set_ylim(0.0, max(qp_reduction) * 1.28)
    axes[1].grid(axis="y", color="0.88", linewidth=0.5)
    save_figure(figure, output_dir, "continuation-ablation")


def read_yaml(path):
    with path.open(encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def csdo_repair_figure(instance_path, root_path, turbo_path, output_dir):
    instance = read_yaml(instance_path)
    root = read_yaml(root_path)
    turbo = read_yaml(turbo_path)
    root_schedule = root.get("schedule", {})
    turbo_schedule = turbo.get("schedule", {})
    if not root_schedule or set(root_schedule) != set(turbo_schedule):
        raise ValueError("CSDO recovery schedules do not contain the same agents")

    figure, axes = plt.subplots(1, 2, figsize=(7.0, 2.45), sharex=True,
                                sharey=True, constrained_layout=True)
    panels = (
        (root_schedule, "(a) Conflicted PBS root: CSDO fails"),
        (turbo_schedule, "(b) TurboADMM-NL repair: validated"),
    )
    for axis, (schedule, title) in zip(axes, panels):
        for obstacle in instance["map"]["obstacles"]:
            axis.add_patch(plt.Circle(
                (float(obstacle[0]), float(obstacle[1])), float(obstacle[2]),
                facecolor="0.72", edgecolor="0.45", linewidth=0.35))
        for index, agent in enumerate(sorted(schedule)):
            states = schedule[agent]
            x_values = np.asarray([number(state, "x") for state in states])
            y_values = np.asarray([number(state, "y") for state in states])
            if not np.all(np.isfinite(np.r_[x_values, y_values])):
                raise ValueError(f"non-finite trajectory for {agent}")
            color = COLORS[index]
            axis.plot(x_values, y_values, color=color, label=agent)
            axis.scatter(x_values[::3], y_values[::3], s=7, color=color,
                         edgecolors="none", zorder=3)
            axis.scatter(x_values[0], y_values[0], s=25, marker="o",
                         facecolors="white", edgecolors=color, linewidths=1.0,
                         zorder=4)
            axis.scatter(x_values[-1], y_values[-1], s=28, marker="*",
                         color=color, zorder=4)
        axis.set_xlim(3.5, 26.5)
        axis.set_ylim(3.0, 11.5)
        axis.set_aspect("equal", adjustable="box")
        axis.set_xlabel("x (m)")
        axis.set_title(title)
        axis.grid(color="0.9", linewidth=0.4)
    axes[0].set_ylabel("y (m)")
    axes[0].legend(frameon=False, loc="upper left", ncol=2)
    save_figure(figure, output_dir, "csdo-repair")


def csdo_figure(rows, output_dir):
    if len(rows) != 1:
        raise ValueError("expected one CSDO paired-statistics row")
    row = rows[0]
    cases = number(row, "cases")
    pbs_cases = number(row, "pbs_failure_cases")
    rates = np.asarray([
        100.0 * number(row, "csdo_success_rate"),
        100.0 * number(row, "turbo_success_rate"),
        100.0 * number(row, "csdo_pbs_recovery_rate"),
        100.0 * number(row, "turbo_pbs_recovery_rate"),
    ])
    lower = np.asarray([
        100.0 * number(row, "csdo_wilson_lower"),
        100.0 * number(row, "turbo_wilson_lower"),
        0.0,
        100.0 * number(row, "turbo_pbs_recovery_wilson_lower"),
    ])
    upper = np.asarray([
        100.0 * number(row, "csdo_wilson_upper"),
        100.0 * number(row, "turbo_wilson_upper"),
        0.0,
        100.0 * number(row, "turbo_pbs_recovery_wilson_upper"),
    ])
    if not np.all(np.isfinite(np.r_[cases, pbs_cases, rates, lower, upper])):
        raise ValueError("CSDO statistics contain missing values")

    figure, axis = plt.subplots(figsize=(3.45, 2.45), constrained_layout=True)
    x_values = np.arange(2)
    width = 0.34
    csdo_bars = axis.bar(x_values - width / 2, rates[[0, 2]], width,
                         color=COLORS[1], label="CSDO")
    turbo_bars = axis.bar(x_values + width / 2, rates[[1, 3]], width,
                          color=COLORS[0], label="TurboADMM-NL")
    axis.errorbar(
        x_values[0] - width / 2, rates[0],
        yerr=[[rates[0] - lower[0]], [upper[0] - rates[0]]],
        fmt="none", ecolor="0.2", capsize=2, linewidth=0.8)
    axis.errorbar(
        x_values[0] + width / 2, rates[1],
        yerr=[[rates[1] - lower[1]], [upper[1] - rates[1]]],
        fmt="none", ecolor="0.2", capsize=2, linewidth=0.8)
    axis.errorbar(
        x_values[1] + width / 2, rates[3],
        yerr=[[rates[3] - lower[3]], [upper[3] - rates[3]]],
        fmt="none", ecolor="0.2", capsize=2, linewidth=0.8)
    label_heights = (upper[0] + 2.0, upper[1] + 2.0,
                     rates[2] + 2.0, upper[3] + 2.0)
    for x_value, value, height in zip(
            (x_values[0] - width / 2, x_values[0] + width / 2,
             x_values[1] - width / 2, x_values[1] + width / 2),
            rates, label_heights):
        axis.text(x_value, height, f"{value:.1f}%", ha="center", va="bottom",
                  fontsize=7)
    axis.set_xticks(x_values, (f"All cases\n(n={int(cases)})",
                              f"PBS failures\n(n={int(pbs_cases)})"))
    axis.set_ylabel("Independent success rate (%)")
    axis.set_ylim(0.0, 125.0)
    axis.grid(axis="y", color="0.88", linewidth=0.5)
    axis.legend(frameon=False, loc="upper center", ncol=2)
    save_figure(figure, output_dir, "csdo-recovery")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--paired-aggregate", type=Path, required=True)
    parser.add_argument("--full-vs-inner", type=Path, required=True)
    parser.add_argument("--qp-continuation-vs-inner", type=Path,
                        required=True)
    parser.add_argument("--full-vs-qp-continuation", type=Path, required=True)
    parser.add_argument("--csdo-statistics", type=Path, required=True)
    parser.add_argument("--csdo-recovery-instance", type=Path, required=True)
    parser.add_argument("--csdo-recovery-root", type=Path, required=True)
    parser.add_argument("--csdo-recovery-turbo", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    arguments = parser.parse_args()
    arguments.output_dir.mkdir(parents=True, exist_ok=True)
    configure_style()
    scaling_figure(read_csv(arguments.paired_aggregate), arguments.output_dir)
    ablation_figure(
        read_csv(arguments.full_vs_inner),
        read_csv(arguments.qp_continuation_vs_inner),
        read_csv(arguments.full_vs_qp_continuation),
        arguments.output_dir)
    csdo_figure(read_csv(arguments.csdo_statistics), arguments.output_dir)
    csdo_repair_figure(
        arguments.csdo_recovery_instance, arguments.csdo_recovery_root,
        arguments.csdo_recovery_turbo, arguments.output_dir)
    print(f"generated figures in {arguments.output_dir}")


if __name__ == "__main__":
    main()
