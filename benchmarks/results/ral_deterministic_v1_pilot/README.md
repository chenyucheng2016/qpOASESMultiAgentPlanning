# RA-L deterministic protocol pilot

This directory archives the one-repetition `ral-deterministic-v1-pilot` run
collected in WSL on 2026-08-04 from commit `faa8c18`. The protocol preflight
passed all 13 CTest regressions before executing the 36 interleaved tasks.

The publication-facing aggregate inputs are:

- `run_manifest.json`: executable hash, solver flags, schedule, and task order;
- `execution_status.csv`: completed, timeout, and error status for all tasks;
- `results.csv`: 28 completed solver rows;
- `summary.csv`, `method_summary.csv`, and `enriched.csv`: derived aggregates;
- `runs/`: per-task result rows, SCP traces, and captured logs.

Eight tasks reached the frozen 300-second process cap. The status table, rather
than absence from `results.csv`, is authoritative for these censored runs.

The runner version used for this pilot left completed trace files with the
legacy suffix `.partial.csv.scp.csv`; the associated completed result row has
the same stem without `.partial`. Later runner revisions finalize successful
traces as `.csv.scp.csv`. No archived file was renamed after collection.

These data are a configuration gate, not the ten-repetition paper timing table.
