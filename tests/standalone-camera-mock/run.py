#!/usr/bin/env python3
"""Run the contained camera experiment from the enclosing Monado checkout."""
import argparse
import datetime as dt
import json
from pathlib import Path
import sys
import subprocess
import unittest

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
sys.dont_write_bytecode = True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", choices=("four-l8", "four-rgb"),
                        help="Run only the selected four-camera demonstration")
    parser.add_argument("--output", type=Path, help="Fresh destination; existing directories are refused")
    args = parser.parse_args()

    from experiment import monado_identity, source_hashes, snapshot_sources
    identity = monado_identity()
    try:
        from capture_e2e.__main__ import BUILD, build, run_case, save, suite
    except ImportError as error:
        raise RuntimeError("Install dependencies using .venv/bin/python -m pip install -r "
                           "tests/standalone-camera-mock/capture_mock/requirements.lock") from error

    stamp = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    output = (args.output or ROOT / "results" / "validation" / stamp).resolve()
    output.mkdir(parents=True, exist_ok=False)
    manifest = {"schema_version": 1, "role": "Local mock Capture-to-CV integration check",
                "created_at_utc": stamp, "command": [sys.executable, *sys.argv],
                "monado": identity, "source_hashes": source_hashes(), "status": "prepared",
                "scope": "No production receiver/Builder, full tracking, Qualcomm stack or boards"}
    snapshot_sources(output / "source_snapshot", manifest["source_hashes"])
    save(output / "manifest.json", manifest)
    print("Building isolated native test components (runtime is not started).", flush=True)
    build()
    native = json.loads(subprocess.check_output([
        str(Path(sys.executable).parent / "ctest"), "--test-dir", str(BUILD),
        "--show-only=json-v1"], text=True))["tests"]
    result = {"schema_version": 1, "scenario": args.scenario, "python_tests_run": 0,
              "native_tests": len(native), "native_test_names": [t["name"] for t in native],
              "passed": False}
    if args.scenario:
        case = run_case(args.scenario, output=output / "demo", do_build=False)
        result["passed"] = case["local_prototype_passed"]
        report = output / "demo/index.html"
    else:
        tests = unittest.defaultTestLoader.discover(str(ROOT / "tests"))
        with (output / "python-tests.txt").open("w") as log:
            tested = unittest.TextTestRunner(stream=log, verbosity=2).run(tests)
        result.update(python_tests_run=tested.testsRun, python_failures=len(tested.failures),
                      python_errors=len(tested.errors), python_skipped=len(tested.skipped))
        print(f"Python checks: {tested.testsRun} run; {len(tested.failures)} failures; "
              f"{len(tested.errors)} errors.", flush=True)
        if not tested.wasSuccessful():
            print((output / "python-tests.txt").read_text(), file=sys.stderr)
        else:
            result["passed"] = suite(output / "suite", do_build=False)
        report = output / "suite/index.html"
    result["report"] = str(report) if report.exists() else None
    result["monado_after"] = monado_identity()
    save(output / "validation.json", result)
    manifest["status"] = "complete" if result["passed"] else "failed"
    save(output / "manifest.json", manifest)
    print(json.dumps({"passed": result["passed"], "report": result["report"],
                      "validation": str(output / "validation.json")}), flush=True)
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, ValueError, FileExistsError) as error:
        raise SystemExit(f"standalone-camera-mock: {error}")
