"""Repository-local paths, source protection and reproducible experiment metadata."""
import hashlib
import json
import os
import shutil
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parent
REPO_ROOT = ROOT.parents[1]
MONADO_SOURCE_DIR = REPO_ROOT
GENERATED_DIRECTORIES = {"__pycache__", "build", "results"}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def validate_protected_sources(repository, expected):
    """Fail closed on a changed or missing reviewed input; authored tests may change."""
    actual = {}
    mismatches = []
    for relative, digest in sorted(expected.items()):
        path = Path(repository) / relative
        if not path.is_file():
            mismatches.append(relative + " (missing)")
            continue
        actual[relative] = sha(path)
        if actual[relative] != digest:
            mismatches.append(relative)
    if mismatches:
        raise RuntimeError("Reviewed Monado input changed; review compatibility before updating "
                           "reviewed-sources.json: " + ", ".join(mismatches))
    return actual


def monado_identity():
    reviewed = json.loads((ROOT / "reviewed-sources.json").read_text())
    hashes = validate_protected_sources(REPO_ROOT, reviewed["source_hashes"])

    def git(*args):
        return subprocess.check_output(["git", *args], cwd=REPO_ROOT, text=True).strip()

    dirty = git("status", "--porcelain=v1", "--untracked-files=all")
    return {"origin": git("remote", "get-url", "origin"),
            "revision": git("rev-parse", "HEAD"),
            "branch": git("rev-parse", "--abbrev-ref", "HEAD"),
            "working_tree_clean": not dirty,
            "working_tree_changes": dirty.splitlines(),
            "reviewed_revision": reviewed["reviewed_revision"],
            "reviewed_sources_match": True,
            "source_hashes": hashes}


def authored_source_hashes():
    """Use checkout-relative keys so snapshots work without a nested checkout."""
    hashes = {}
    for directory, subdirs, names in os.walk(ROOT):
        subdirs[:] = sorted(name for name in subdirs if name not in GENERATED_DIRECTORIES)
        for name in sorted(names):
            path = Path(directory) / name
            if not path.is_file() or path.suffix == ".pyc":
                continue
            hashes[str(path.relative_to(REPO_ROOT))] = sha(path)
    hashes[".gitignore"] = sha(REPO_ROOT / ".gitignore")
    return hashes


def source_hashes():
    identity = monado_identity()
    return {**authored_source_hashes(), **identity["source_hashes"]}


def snapshot_sources(destination, hashes):
    """Preserve exact bytes before source startup; validate a reused suite snapshot."""
    destination = Path(destination)
    for relative, expected in hashes.items():
        source = REPO_ROOT / relative
        if sha(source) != expected:
            raise RuntimeError("Source changed while preparing manifest: " + relative)
        target = destination / relative
        if target.exists():
            if sha(target) != expected:
                raise RuntimeError("Refusing to overwrite a different snapshot: " + relative)
            continue
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, target)
        if sha(target) != expected:
            raise RuntimeError("Source changed while copying snapshot: " + relative)


def compare_normal_pixels(summary, baseline=None):
    """Compare observed post-CV pixels with the saved pre-port recordings."""
    baseline = baseline or json.loads((ROOT / "fixtures/normal-pixels.json").read_text())
    case = baseline["cases"].get(summary["scenario"])
    if case is None:
        return {"applicable": False, "reason": "Not a normal historical baseline case"}
    if summary["config"] != case["config"]:
        raise RuntimeError("Historical pixel comparison requires the original normal fixture config")
    records = [json.loads(line) for line in
               (Path(summary["output"]) / "frames.jsonl").read_text().splitlines()]
    actual = {}
    for row in records:
        key = (str(row["camera_id"]), row["sequence"])
        if key in actual:
            raise RuntimeError("Duplicate frame in baseline comparison: " + str(key))
        actual[key] = row["image_sha256"]
    expected = {(camera, sequence): digest for camera in case["cameras"]
                for sequence, digest in enumerate(baseline["pixel_sha256_by_camera"][camera])}
    if actual != expected:
        raise RuntimeError("Normal output pixels differ from the historical observed baseline")
    return {"applicable": True, "matched_frames": len(expected),
            "baseline_report": baseline["prototype_report_id"],
            "baseline_fixture_sha256": sha(ROOT / "fixtures/normal-pixels.json")}
