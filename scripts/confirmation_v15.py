"""Generate the corrected confirmation figure, table and 30 paired seed effects."""
import argparse
import itertools
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from stats import paired_seed_means, difference_test, tost_log_ratio
from analyze_v15 import ROOT, checksum


def generate(run):
    sources=json.loads((run/"sources.json").read_text())
    for path,digest in sources.items():
        if checksum(ROOT/path)!=digest:
            raise ValueError(f"Source mismatch: {path}")
    files=sorted((run/"confirmation").glob("*.csv"))
    config=json.loads((ROOT/"config/sim/v15.json").read_text())
    block=config["confirmation"]
    keys=["seed","condition","n_robots","arm"]
    grid=set(itertools.product(range(block["seeds"][0],block["seeds"][1]+1),
                               block["conditions"],config["team_sizes"],block["arms"]))
    raw=pd.concat([pd.read_csv(f) for f in files],ignore_index=True)
    if len(raw)!=len(grid) or set(raw[keys].itertuples(index=False,name=None))!=grid:
        raise ValueError("Incomplete or duplicate confirmation observations")
    if not np.isfinite(raw.align_rmse).all() or (raw.align_rmse<=0).any():
        raise ValueError("Invalid alignment error")
    paired=paired_seed_means(raw,"arm","plus_0.30_6","random",keys[:3])
    summary={**difference_test(paired,"plus_0.30_6","random"),**tost_log_ratio(paired)}
    out=run/"confirmation_analysis"
    out.mkdir(exist_ok=True)
    paired.to_csv(out/"paired_seeds.csv",index=False)
    pd.DataFrame([summary]).to_csv(out/"table.csv",index=False)
    ratio=summary["geometric_mean_ratio"]
    lo,hi=summary["ratio_ci90_lo"],summary["ratio_ci90_hi"]
    fig,ax=plt.subplots(figsize=(6,2.1),layout="constrained")
    ax.axvspan(.9,1.1,color="#dceadf")
    ax.axvline(1,color="gray",linestyle=":")
    ax.errorbar(ratio,0,xerr=[[ratio-lo],[hi-ratio]],fmt="o",capsize=5,color="#174c6a")
    ax.set(xlabel="BACS+ / Random alignment RMSE ratio",yticks=[],xlim=(.8,2))
    ax.set_title("Corrected confirmation · 30 seeds · 90% confidence interval")
    fig.savefig(out/"figure.png",dpi=200)
    plt.close(fig)
    sources.update({str(f.relative_to(ROOT)):checksum(f) for f in files})
    sources.update({str(f.relative_to(ROOT)):checksum(f) for f in
                    (Path(__file__).resolve(), ROOT/"scripts/stats.py", ROOT/"scripts/analyze_v15.py")})
    (out/"inputs.json").write_text(json.dumps(sources,indent=2))


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run",type=Path)
    generate(parser.parse_args().run.resolve())
