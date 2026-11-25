import json
from pathlib import Path

run = Path("runs/v15/409d1b1da6673e913e2fee281b74ecc422f7bb1fe2e29d9f90da710cf8d55213")
with open(run / "analysis_inputs.json") as f:
    data = json.load(f)

print("Total entries in ledger:", len(data))
prefixes = set(x.split("/")[0] for x in data)
print("Top level dirs:", sorted(prefixes))
for p in sorted(prefixes):
    count = sum(1 for x in data if x.startswith(p + "/"))
    print(f"  {p}: {count}")

import subprocess
import sys

# Test running pytest in a mock clean directory with only certain files
print("Checking what files pytest needs...")


