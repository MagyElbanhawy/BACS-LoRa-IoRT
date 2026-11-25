"""Print or save a balanced FIFO/BACS/BACS+/RANDOM hardware block plan."""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

from .experiment_plan import block_plan


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--block", type=int, required=True, help="one-based experimental block")
    result.add_argument("--duration", type=int, default=600, help="seconds per policy (default: 600)")
    result.add_argument("--format", choices=("table", "json", "csv"), default="table")
    result.add_argument("--output", type=Path, help="write the plan to this file instead of stdout")
    return result


def render(args: argparse.Namespace) -> str:
    rows = block_plan(args.block, args.duration)
    if args.format == "json":
        return json.dumps([item.__dict__ for item in rows], indent=2) + "\n"
    if args.format == "csv":
        import io
        handle = io.StringIO()
        writer = csv.DictWriter(handle, fieldnames=rows[0].__dict__.keys(), lineterminator="\n")
        writer.writeheader()
        writer.writerows(item.__dict__ for item in rows)
        return handle.getvalue()
    header = "POSITION  SESSION             POLICY   RUN  DURATION"
    lines = [header]
    for item in rows:
        lines.append(f"{item.position:>8}  {item.session:<18}  {item.policy:<7}  {item.run:>3}  {item.duration_s:>7}s")
    return "\n".join(lines) + "\n"


def main(argv: list[str] | None = None) -> None:
    args = parser().parse_args(argv)
    try:
        text = render(args)
    except ValueError as error:
        raise SystemExit(str(error)) from error
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")
    else:
        sys.stdout.write(text)


if __name__ == "__main__":
    main()
