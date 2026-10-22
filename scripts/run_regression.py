"""Regenerate a small fixed-seed regression record, never a general success rate."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from grasp_demo.simulation import run_episode


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("runs/regression.json"))
    args = parser.parse_args()
    rows = [run_episode(seed=seed, case="nominal", record=False)["metrics"]
            for seed in (7, 11, 23, 41, 97)]
    negative = [run_episode(seed=7, case=case, record=False)["metrics"]
                for case in ("no-target", "out-of-workspace")]
    passed = all(row["success"] for row in rows) and all(not row["success"] for row in negative)
    record = {"schema_version": 1, "synthetic_scene": True,
              "interpretation": "Five fixed-seed constraint-assisted regressions and two expected failures; not a general grasp success rate or real-robot benchmark.",
              "nominal": rows, "expected_failures": negative, "regression_passed": passed}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_bytes((json.dumps(record, ensure_ascii=False, indent=2, allow_nan=False) + "\n").encode("utf-8"))
    print(json.dumps({"nominal_cases": len(rows), "expected_failure_cases": len(negative), "regression_passed": passed}))
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
