# TurboADMM-NL RA-L manuscript

This directory contains the working RA-L manuscript. The paper deliberately
positions TurboADMM-NL as an extension of TurboADMM rather than an unrelated
planner.

The results section currently distinguishes development evidence from final
evidence. Do not remove that distinction until all machine-enforced gates pass
and the untouched 720-case split has been executed exactly once.

Build with:

```bash
latexmk -pdf -interaction=nonstopmode -halt-on-error main.tex
```

Before submission:

1. replace the incomplete author block with the complete author list and
   affiliations;
2. run and archive the 240-case strict-convergence development gate;
3. rerun the continuation ablations with the frozen solver;
4. run the six deterministic cases for ten interleaved repetitions;
5. run the disjoint 32-case CSDO suite with the same frozen solver;
6. execute the untouched 720-case final matrix once;
7. replace development-only scaling text with final paired statistics;
8. generate final figures from the manifest-locked CSV artifacts;
9. verify every numerical claim against the generated tables; and
10. compile, render, and visually inspect the complete PDF.

The current manuscript is a technical draft, not a submission artifact. It
contains no fabricated final numbers; the final primary results remain locked.
