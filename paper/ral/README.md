# TurboADMM-NL RA-L manuscript

This directory contains the working RA-L manuscript. The paper deliberately
positions TurboADMM-NL as an extension of TurboADMM rather than an unrelated
planner.

The results section currently distinguishes development evidence from final
evidence. Do not remove that distinction until all machine-enforced gates pass
and the untouched 720-case split has been executed exactly once.

The LaTeX distribution must provide `IEEEtran`, `amsmath`, `booktabs`, `cite`,
`graphicx`, and `multirow`. Build with `latexmk` when Perl is available:

```bash
latexmk -pdf -interaction=nonstopmode -halt-on-error main.tex
```

The equivalent explicit build, which was used for the rendered QA copy, is:

```bash
pdflatex -interaction=nonstopmode -halt-on-error main.tex
bibtex main
pdflatex -interaction=nonstopmode -halt-on-error main.tex
pdflatex -interaction=nonstopmode -halt-on-error main.tex
```

Generate the timeout-aware deterministic table only after all 360 scheduled
executions are present. The generator verifies the complete six-method,
ten-repetition ledger before writing either output:

```bash
python generate_deterministic_results.py \
  <deterministic>/results.csv \
  <deterministic>/execution_status.csv \
  --cases easy_open,easy_single_blocker,medium_doorway,hard_heterogeneous_doorway,hard_warehouse,very_hard_maze \
  --expected-methods full,centralized_osqp,centralized_qpoases,qp_continuation,inner,cold \
  --repetitions 10 \
  --output-csv <deterministic>/paper_summary.csv \
  --output-tex deterministic_results.tex
```

Generate `results_macros.tex` from the manifest-locked analysis CSVs before
building the manuscript. Use `--evidence-stage development` until the untouched
final split has been executed; change it to `final` only for the frozen final
artifacts. The generated file records the SHA-256 digest of every input:

```bash
python generate_results_macros.py \
  --evidence-stage development \
  --primary-manifest <primary>/run_manifest.json \
  --paired-aggregate <primary>/paired_aggregate.csv \
  --full-vs-inner <ablation>/full_vs_inner_aggregate.csv \
  --qp-continuation-vs-inner \
    <ablation>/qp_continuation_vs_inner_aggregate.csv \
  --full-vs-qp-continuation \
    <ablation>/full_vs_qp_continuation_aggregate.csv \
  --csdo-statistics <csdo>/paired_statistics.csv \
  --csdo-summary <csdo>/summary.csv \
  --output results_macros.tex
```

The figure generator requires Matplotlib, NumPy, and PyYAML.

Generate the four benchmark figures only from manifest-locked analysis artifacts:

```bash
python generate_figures.py \
  --paired-aggregate <primary>/paired_aggregate.csv \
  --full-vs-inner <ablation>/full_vs_inner_aggregate.csv \
  --qp-continuation-vs-inner \
    <ablation>/qp_continuation_vs_inner_aggregate.csv \
  --full-vs-qp-continuation \
    <ablation>/full_vs_qp_continuation_aggregate.csv \
  --csdo-statistics <csdo>/paired_statistics.csv \
  --csdo-recovery-instance <instances>/representative.yaml \
  --csdo-recovery-root <csdo-case>/root_guesses.yaml \
  --csdo-recovery-turbo <csdo-case>/turbo.yaml \
  --output-dir <figures>
```

Development figures are for layout and analysis checks only. Regenerate them
from the untouched final artifacts after the solver and protocol are frozen.

Before submission:

1. replace the incomplete author block with the complete author list and
   affiliations;
2. run and archive the 240-case strict-convergence development gate;
3. rerun the continuation ablations with the frozen solver;
4. run the six deterministic cases for ten interleaved repetitions;
5. run the disjoint 32-case CSDO v4 suite with the same frozen solver and pass
   its exact shared-front-end and failure-artifact provenance gate;
6. execute the untouched 720-case final matrix once;
7. replace development-only scaling text with final paired statistics;
8. generate final figures from the manifest-locked CSV artifacts;
9. verify every numerical claim against the generated tables; and
10. compile, render, and visually inspect the complete PDF.

The current manuscript is a technical draft, not a submission artifact. It
contains no fabricated final numbers; the final primary results remain locked.
