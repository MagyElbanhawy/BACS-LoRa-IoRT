import hashlib
import json
from pathlib import Path

ROOT = Path(".")
run = ROOT / "runs/v15/409d1b1da6673e913e2fee281b74ecc422f7bb1fe2e29d9f90da710cf8d55213"
sources = json.loads((run / "sources.json").read_text())

diffs = []
for p, exp_hash in sources.items():
    file_path = ROOT / p
    if not file_path.exists():
        diffs.append((p, "MISSING", exp_hash))
        continue
    act_hash = hashlib.sha256(file_path.read_bytes()).hexdigest()
    if act_hash != exp_hash:
        diffs.append((p, act_hash, exp_hash))

print(f"Total checked: {len(sources)}, Differences: {len(diffs)}")
for p, act, exp in diffs:
    print(f"  {p}:")
    print(f"    current:  {act}")
    print(f"    expected: {exp}")
