# CSDO congested benchmark families

These WSL experiments test strict feasibility and recovery, not a runtime-win
claim. CSDO is unmodified at the PBS/Hybrid-A*/fixed-plane level. Both outputs
are checked by the independent continuous validator. Runtime is recorded for
reproducibility but is not a primary comparison metric.

## Frozen families

### Exact packed symmetries

The physical five-vehicle passing-bay problem is evaluated in its original
orientation, horizontal and vertical reflections, and a 180-degree rotation.
The transformations preserve vehicle geometry, obstacle dimensions, starts,
goals, and density.

| Variant | CSDO | TurboADMM-NL | CSDO / Turbo arrival sum | Decisive metric |
|---|---|---|---:|---|
| Base | Invalid | Valid | 367 / 305 | CSDO pair clearance `-0.0797` m. |
| Mirror x | Valid | Invalid | 369 / 316 | Turbo pair clearance `-0.0239` m. |
| Mirror y | Valid | Valid | 397 / 292 | Neutral feasibility control. |
| Rotate 180 | Invalid | Valid | 381 / 323 | CSDO dynamics defect `0.0623`; Turbo defect `1.42e-14`. |

The exact-symmetry family therefore contains two strict Turbo wins, one
both-valid control, and one CSDO-only success. It does not support a universal
recovery claim.

### Eight priority patterns

The map and physical agent tasks are identical. Only the YAML agent order,
which determines CSDO/PBS priority, changes. The frozen patterns are east-first,
east-reversed, west-first, west-reversed, full reverse, two alternating orders,
and center-out.

| Paired outcome | Count |
|---|---:|
| CSDO invalid, Turbo valid | 2 |
| Both valid | 1 |
| Both invalid | 4 |
| CSDO invalid, Turbo timeout | 1 |

Turbo repairs only two of the seven CSDO-invalid outputs. This is a development
gate, not positive paper evidence.

### Twelve within-flow permutations

All `3! * 2! = 12` permutations within the eastbound and westbound convoys are
frozen. The two flow groups are not interleaved. A uniform 120-second cap is
used.

| Paired outcome | Count |
|---|---:|
| CSDO invalid, Turbo valid | 1 |
| CSDO valid, Turbo no output under cap | 8 |
| Both invalid/no Turbo output | 3 |

This family exposes the current principal blocker: physically equivalent agent
relabelings produce different PBS horizons and Turbo convergence behavior.
The exporter horizons range from 66 to 81 stages. More packed maps should not
be promoted to paper benchmarks until this permutation/initializer sensitivity
is fixed.

A delayed pair-feasibility polishing phase now recovers the targeted order04
`joint_repair` case. With two WSL threads it returns an independently valid
trajectory in 370.45 s: pair clearance is zero, obstacle clearance is
`1.03e-7` m, dynamics defect is `1.42e-14`, and terminal-state error is
`1.26e-4`. The run uses 31 SCP and 1,286 ADMM iterations. The solver does not
claim strict convergence because the final line-search recovery exhausts its
budget after feasibility is reached. This is positive recovery evidence, but
not yet a paper-facing strict-convergence result.

## Existing recovery control

`narrow_passing_bay.yaml` remains the clean PBS-recovery example: CSDO PBS
fails, while Turbo repairs one conflicted root pair and returns an independently
valid trajectory. It is a two-agent mechanism test, not a density study.

## Reproduction

Generate the frozen instances:

```bash
python3 scripts/generate_csdo_packed_symmetries.py
python3 scripts/generate_csdo_priority_sweep.py
python3 scripts/generate_csdo_within_flow_orders.py
```

Run a complete manifest with `scripts/run_csdo_congested_suite.py`. The runner
writes a separate status CSV and continues after failed cases. Then use
`scripts/summarize_csdo_congested_suite.py` to reconstruct a row for every
manifest entry, including Turbo timeouts.

## Publication gate

The CSDO comparison becomes paper-facing only when all of the following hold:

1. Turbo validates on at least 80% of the exact-symmetry and priority families.
2. Turbo recovers at least 70% of CSDO-invalid cases under a frozen cap.
3. No physically equivalent within-flow relabeling changes Turbo success.
4. At least one multi-agent case starts from a locally conflicted PBS/root
   trajectory and ends strictly feasible.
5. Every frozen row is reported. Runtime may be secondary, but its trade-off is
   disclosed in supplementary material or limitations.
