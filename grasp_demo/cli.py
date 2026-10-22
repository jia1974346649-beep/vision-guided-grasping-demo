"""Repeatable local CLI; no GUI, model downloads, or network requests."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from .reporting import write_report
from .simulation import CASES, run_episode


def main(argv=None):
    parser = argparse.ArgumentParser(description="Run the anonymous PyBullet RGB-D grasp simulation.")
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--case", choices=CASES, default="nominal")
    parser.add_argument("--output", type=Path, default=Path("runs/demo"), help="Directory for the five generated result files")
    args = parser.parse_args(argv)
    if not 0 <= args.seed < 2**32:
        parser.error("--seed must be between 0 and 4294967295")
    result = run_episode(seed=args.seed, case=args.case, record=True)
    report = write_report(result, args.output)
    print(json.dumps({"case": args.case, "seed": args.seed, "success": result["metrics"]["success"],
                      "failure_reason": result["metrics"]["failure_reason"], "report": str(report),
                      "grasp_mode": result["metrics"]["grasp_mode"]}, ensure_ascii=False))
    # Expected negative scenarios generate evidence and exit normally. An
    # unexpectedly failed nominal demonstration is a nonzero CLI result.
    return 0 if result["metrics"]["success"] or args.case != "nominal" else 1
