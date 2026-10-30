"""Supporting R8 harness; use the repository-root run.py for the main demonstration."""
import argparse
import datetime as dt
import html
import json
from pathlib import Path
from .bindings import ROOT
from .build import ensure_build
from .runner import SCENARIOS, load_case, run_case, save_json

def main():
    parser = argparse.ArgumentParser(description="Native mock Capture / Monado queue / CV validation prototype")
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("build")
    run = sub.add_parser("run")
    run.add_argument("--scenario", default="one-camera")
    run.add_argument("--output", type=Path)
    suite = sub.add_parser("suite")
    suite.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.command == "build":
        print(json.dumps(ensure_build(), indent=2))
        return 0
    ensure_build()
    if args.command == "run":
        result = run_case(args.scenario, output=args.output, build=False)
        print(json.dumps({"output": result["output"], "prototype_path_verified": result["prototype_path_verified"]}))
        return 0 if result["prototype_path_verified"] else 1
    timestamp = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    output = (args.output or ROOT / "results" / "capture-mock" / "suites" / timestamp).resolve()
    if output.exists():
        raise FileExistsError(f"Refusing to overwrite {output}")
    output.mkdir(parents=True)
    catalog = json.loads(SCENARIOS.read_text())
    save_json(output / "suite-plan.json", {
        "evidence_role": "Synthetic input, measured local native-component/CV prototype",
        "cases": {name: load_case(name) for name in catalog["cases"]},
        "production_receiver": "not integrated", "board_execution": "not tested"})
    results = []
    for name in catalog["cases"]:
        result = run_case(name, output=output / name, build=False)
        results.append({"scenario": name, "prototype_path_verified": result["prototype_path_verified"],
                        "report": f"{name}/report.html", "streams": result["streams"]})
        print(json.dumps({"scenario": name, "passed": result["prototype_path_verified"]}), flush=True)
    save_json(output / "suite-summary.json", results)
    rows = []
    for result in results:
        total = lambda name: sum(x[name] for x in result["streams"].values())
        rows.append(f'<tr><td><a href="{result["report"]}">{html.escape(result["scenario"])}</a></td>'
                    f'<td>{"PASS" if result["prototype_path_verified"] else "FAIL"}</td>'
                    f'<td>{total("generated")}</td><td>{total("processed")}</td>'
                    f'<td>{total("rejected")}</td><td>{total("queue_dropped")}</td></tr>')
    report = """<!doctype html><meta charset="utf-8"><title>Mock capture results</title>
<style>body{font:17px system-ui;max-width:1100px;margin:40px auto;padding:0 20px;color:#172638}
table{border-collapse:collapse;width:100%}td,th{padding:14px;border-bottom:1px solid #ccd5df;text-align:left}
aside{padding:20px;background:#fff3d7;border-radius:10px}a{color:#174ea6}</style>
<h1>Mock capture → native Monado queue → CV</h1>
<aside>Local prototype. The source and CV validation receiver are new; the native Monado queue is unchanged.
Team tracker receiver/Builder, full runtime, RB5/EB5 and physical camera synchronization remain untested.</aside>
<p>Normal cases expect complete delivery. Invalid-frame cases expect one clear rejection.
The slow-consumer case expects measured queue drops, with retained buffers staying valid.</p>
<table><tr><th>Scenario</th><th>Expected outcome</th><th>Generated</th><th>Processed</th>
<th>Rejected</th><th>Queue drops</th></tr>""" + "".join(rows) + "</table>"
    (output / "index.html").write_text(report)
    print(json.dumps({"overview": str(output / "index.html"), "all_expected_outcomes": all(r["prototype_path_verified"] for r in results)}))
    return 0 if all(r["prototype_path_verified"] for r in results) else 1

if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (ValueError, RuntimeError, FileExistsError) as error:
        raise SystemExit(f"capture_mock: {error}")
