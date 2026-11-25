"""Validate the complete v15 cell grids and compute the pre-specified primary contrasts.

Confirmation (seeds 70-99): BACS+ (plus_0.30_6) vs random ordering, with TOST.
Comparison (seeds 100-129): BACS+ vs the EMRMF reference (emrmf_published).
Writes comparison.csv, paired_seed_effects.csv, comparison.png and analysis_inputs.json
into the run directory, and the experiment manifests if they do not exist yet.
"""
import argparse
import hashlib
import itertools
import json
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from stats import difference_test, paired_seed_means, tost_log_ratio

ROOT = Path(__file__).resolve().parents[1]


def checksum(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main(run):
    from datetime import datetime, timezone
    plan = json.loads((ROOT / "config/sim/v15.json").read_text())
    sources = json.loads((run / "sources.json").read_text())
    if any(checksum(ROOT / path) != digest for path, digest in sources.items()):
        raise ValueError("run snapshot differs from current source")
    output, seed_outputs, inputs = [], [], dict(sources)
    for name, reference in (("confirmation", "random"), ("bridge", "emrmf_published")):
        block = plan[name]
        files = sorted((run / name).glob("*.csv"))
        if len(files) != 480:
            raise ValueError(f"{name}: {len(files)} of 480 cells available")
        for path in files:
            inputs[str(path.relative_to(ROOT))] = checksum(path)
        df = pd.concat(map(pd.read_csv, files), ignore_index=True)
        keys = ["seed", "n_robots", "condition", "arm"]
        start, end = block["seeds"]
        expected = set(itertools.product(range(start, end + 1), plan["team_sizes"], block["conditions"], block["arms"]))
        if len(df) != len(expected) or set(df[keys].itertuples(index=False, name=None)) != expected:
            raise ValueError(f"{name} cell grid mismatch")
        if not np.isfinite(df[["pose_rmse", "align_rmse"]]).all().all() or (df.align_rmse <= 0).any():
            raise ValueError("invalid metric")
        paired = paired_seed_means(df, "arm", "plus_0.30_6", reference, keys[:3])
        d = difference_test(paired, "plus_0.30_6", reference)
        seed_outputs.append(paired.rename(columns={"plus_0.30_6": "treatment_rmse", reference: "reference_rmse"}).assign(experiment=name, version="v15"))
        record = {"experiment": name, "version": "v15", **d,
                  "reduction_pct": 100 * (1 - d["treatment_mean"] / d["reference_mean"])}
        if name == "confirmation":
            record.update(tost_log_ratio(paired))
        output.append(record)
    summary = pd.DataFrame(output)
    pd.concat(seed_outputs, ignore_index=True).to_csv(run / "paired_seed_effects.csv", index=False)
    summary.to_csv(run / "comparison.csv", index=False)
    fig, ax = plt.subplots(figsize=(5, 3.5))
    ax.bar(["Confirmation\nBACS+ vs Random", "Comparison\nBACS+ vs EMRMF"], summary.reduction_pct)
    ax.axhline(0, color="k", lw=0.7)
    ax.set(ylabel="Reduction in mean alignment RMSE (%)")
    fig.tight_layout()
    fig.savefig(run / "comparison.png", dpi=180)
    inputs["scripts/analyze_v15.py"] = checksum(Path(__file__))
    inputs["scripts/stats.py"] = checksum(ROOT / "scripts/stats.py")
    (run / "analysis_inputs.json").write_text(json.dumps(inputs, indent=2))
    for experiment in ("confirmation", "bridge"):
        provenance_path = run / f"{experiment}_manifest.json"
        if provenance_path.exists():
            continue  # preserve the timestamps of the original execution
        paths = sorted((run / experiment).glob("*.csv"))
        stamps = [p.stat().st_mtime for p in paths]
        manifest = {
            "experiment": experiment, "source_snapshot": run.name,
            "config_path": "config/sim/v15.json", "config_sha256": sources["config/sim/v15.json"],
            "seeds_inclusive": plan[experiment]["seeds"], "team_sizes": plan["team_sizes"],
            "conditions": plan[experiment]["conditions"], "arms": plan[experiment]["arms"],
            "cell_count": len(paths), "analysis_inputs": "analysis_inputs.json",
            "execution_command": "python3 scripts/run_v15.py --workers 12",
            "first_cell_written_utc": datetime.fromtimestamp(min(stamps), timezone.utc).isoformat(),
            "last_cell_written_utc": datetime.fromtimestamp(max(stamps), timezone.utc).isoformat(),
        }
        provenance_path.write_text(json.dumps(manifest, indent=2) + "\n")
    print(summary.to_string(index=False))


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("run", type=Path)
    main(p.parse_args().run.resolve())
