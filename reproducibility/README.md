# Reproducibility checks

```bash
python reproducibility/verify_environment.py   # Python modules and repository layout
python tools/check_sources.py                  # SHA-256 of the simulator files vs runs/v15/409d…/sources.json
python -m pytest -q tests                      # automated tests
```

`tools/check_sources.py` should report `Differences: 0`. Rerunning stored cells of `runs/v15/409d1b1da6673e913e2fee281b74ecc422f7bb1fe2e29d9f90da710cf8d55213/` with `scripts/run_v15.py` reproduces their values exactly. See the root [`README.md`](../README.md), Section 4, for the full rerun commands.
