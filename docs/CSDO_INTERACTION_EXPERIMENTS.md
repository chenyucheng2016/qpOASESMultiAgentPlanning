# CSDO interaction experiments

These experiments compare the official CSDO pipeline with TurboADMM-NL on
interactive cases. They are quality and recovery experiments, not runtime-win
claims. All reported pilot runs were executed in WSL and are single runs; the
paper evaluation must repeat them under the frozen timing protocol.

## Common interface

`csdo_root_warmstart_exporter` exposes three explicit warm-start sources:

- `pbs_goal`: CSDO's successful PBS/Hybrid-A* result;
- `pbs_root`: the conflicted PBS root when PBS cannot resolve the interaction;
- `independent_single_agent`: independently feasible Hybrid-A* paths when PBS
  does not construct a reusable root, or when `--independent` is requested.

The exporter records conflicts before and after optional temporal staggering.
`--pad-stages`, `--delay-from-agent`/`--delay-stages`, and
`--sequential-delay-stages` change only the exported warm start and are written
to the metadata. CSDO itself always runs unmodified on the original instance.

`--mode joint_repair` adds a failure-recovery comparison. It first runs the
unmodified CSDO optimizer, then gives its emitted fixed-plane trajectory and
corridors to TurboADMM-NL for joint negotiation. CSDO is counted as solver
successful only when its solver status is exactly `1`; status `2` is the
original implementation's inaccurate/unsuccessful result, even if a schedule
is available for Turbo to repair.

Both back ends use CSDO's vehicle model, two-disc footprint, static corridors,
time step, terminal state, and objective. Turbo validates continuous-time
samples at ten substeps per interval.

## Retained scenarios and pilot results

| Scenario | CSDO result | Turbo result | CSDO / Turbo wall time (s) | Main evidence |
|---|---:|---:|---:|---|
| `narrow_passing_bay.yaml` | PBS failure | Valid | 0.095 / 7.477 | Turbo repairs one conflicted PBS-root pair; pair clearance 0.066 m and exact obstacle clearance 0.050 m. |
| `pinned_vs_free_crossing.yaml` | Valid | Valid | 0.280 / 0.569 | Interaction-zone span 12 to 8 stages and summed arrival 84 to 76; the pinned agent arrives at stage 34 instead of 42. |
| `five_vehicle_two_capacity_passage.yaml` | Valid | Valid | 1.650 / 22.870 | Summed arrival 480 to 356 and path length 178.42 to 171.69 m while peak passage occupancy remains two. |
| `five_vehicle_passing_bay_rooms.yaml` (`joint_repair`) | Solver success, independently invalid | Valid | 0.962 / 67.724 | CSDO has -0.0797 m pair clearance. Turbo repairs it to -0.0000619 m under the unchanged -0.001 m tolerance, with dynamics defect `1.42e-14` and terminal error `3.68e-6` m. |

The CSDO independent validator allows for its three-decimal YAML serialization:
dynamics defect at most `1e-2`, terminal position error at most `1e-3`, terminal
yaw error at most `2e-3`, and pair/obstacle clearance at least `-5e-3`. The
passing-bay-room CSDO output fails the safety criterion by an order of
magnitude. Turbo's success flag comes from the stricter in-process validator.
Turbo plans with a uniform 5 mm pairwise buffer, capped for each disc pair by
its available start/goal clearance so tangent fixed endpoints remain feasible.
Reported metrics still use CSDO's physical 1.25 m discs and the unchanged
validator tolerances.

## Rejection rule

A scenario is retained only when Turbo validates and it demonstrates either a
qualitative success difference or a meaningful coordination-quality change.
The attempted reciprocal four-way case was rejected because Turbo could not
repair its two remaining head-on conflicts. A turning-only four-way case was
also rejected because its independent paths had zero conflicts. Failed pilot
definitions are not part of the maintained benchmark set.

## WSL runner

The paired runner is `scripts/run_csdo_interaction_comparison.py`. Recovery
experiments use `--mode recovery`; failed fixed-plane solutions use
`--mode joint_repair`; shared-PBS quality experiments use
`--mode conservatism`. Add `--independent` only when the experiment explicitly
studies the cheap independent initializer. Use a separate CSV path for each
pilot or a fresh aggregate file so every row follows the same schema.

Before promoting a repair case, run `scripts/analyze_csdo_warmstarts.py` on
both its search warm start and any emitted CSDO solution. The diagnostic reports
the actual schedule-to-corridor slack separately from the nominal coordinates
stored in CSDO's corridor dump, because CSDO may update the bounds without
updating those embedded nominal coordinates.
