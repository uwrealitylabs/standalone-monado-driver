"""Execute a real native component path with an explicitly new CV validation sink."""
import copy
import ctypes as ct
import datetime as dt
import hashlib
import html
import json
import platform
import subprocess
import sys
import threading
import time
import zlib
from pathlib import Path

import cv2
import numpy as np

from .bindings import ROOT, LIBRARY, CALLBACK, Stats, library, native_config
from .build import ensure_build, monado_identity, sha
from experiment import source_hashes as experiment_hashes, snapshot_sources

SCENARIOS = ROOT / "capture_mock" / "scenarios.json"
ROLE = "PROTOTYPE: synthetic Capture -> unchanged native Monado queue -> new CV validation sink"
CLOCK = "CLOCK_MONOTONIC on this host"

def clock_ns():
    return time.clock_gettime_ns(time.CLOCK_MONOTONIC)

def load_case(name):
    catalog = json.loads(SCENARIOS.read_text())
    if name not in catalog["cases"]:
        raise ValueError(f"Unknown scenario {name!r}")
    result = copy.deepcopy(catalog["defaults"])
    result.update(catalog["cases"][name])
    return result

def validate_config(config):
    if config.get("source_mode") not in ("individual", "image-bundle"):
        raise ValueError("source_mode must be individual or image-bundle")
    for key in ("width", "height"):
        if not isinstance(config[key], int) or not 64 <= config[key] <= 4096:
            raise ValueError(f"{key} must be an integer in [64,4096]")
    bounds = {"padding": (0, 256), "fps_num": (1, 1000), "fps_den": (1, 1000),
              "frames": (1, 100000), "queue_capacity": (1, 1024), "cameras": (1, 4),
              "session_id": (1, 255), "seed": (0, 65535), "timeout_ms": (1, 120000),
              "consumer_delay_ms": (0, 1000), "malformed_kind": (0, 3)}
    for key, (low, high) in bounds.items():
        value = config[key]
        if not isinstance(value, int) or not low <= value <= high:
            raise ValueError(f"{key} must be an integer in [{low},{high}]")
    for key in ("interrupt_camera", "malformed_camera"):
        if config[key] is not None and not 0 <= config[key] < config["cameras"]:
            raise ValueError(f"{key} is outside configured cameras")
    if config["interrupt_camera"] is not None:
        if not isinstance(config["interrupt_after"], int) or not 0 <= config["interrupt_after"] <= config["frames"]:
            raise ValueError("interrupt_after must be within the frame sequence")
    if config["malformed_kind"] and config["malformed_camera"] is None:
        raise ValueError("Malformed-frame test requires a camera")
    if not 0 <= config["malformed_sequence"] < config["frames"]:
        raise ValueError("malformed_sequence must be within the frame sequence")
    if config.get("manual_stop_after_ms") is not None and config["manual_stop_after_ms"] < 0:
        raise ValueError("manual_stop_after_ms must be nonnegative")
    cv = config["cv"]
    if cv["max_corners"] <= 0 or not 0 < cv["quality_level"] <= 1 or cv["min_distance"] <= 0:
        raise ValueError("Invalid corner detector parameters")

def source_hashes():
    return experiment_hashes()

def percentiles(values):
    if not values:
        return {"median_ms": None, "p95_ms": None, "max_ms": None}
    data = np.asarray(values, dtype=np.float64) / 1e6
    return {"median_ms": float(np.median(data)), "p95_ms": float(np.percentile(data, 95)),
            "max_ms": float(np.max(data))}

def save_json(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")

def render_report(output, summary):
    rows = []
    for camera, metrics in summary["streams"].items():
        sample = f'camera-{camera}-session-{summary["config"]["session_id"]}-annotated.png'
        preview = (f'<a href="{sample}"><img width="240" src="{sample}" alt="camera {camera} annotated"/></a>'
                   if (output / sample).exists() else "No valid sample")
        rows.append(f"<tr><td>{camera}</td><td>{metrics['generated']}</td>"
                    f"<td>{metrics['received']}</td><td>{metrics['processed']}</td>"
                    f"<td>{metrics['rejected']}</td><td>{metrics['queue_dropped']}</td>"
                    f"<td>{metrics['processing_time']['median_ms']}</td><td>{preview}</td></tr>")
    body = f"""<!doctype html><meta charset="utf-8"><title>Mock capture: {html.escape(summary['scenario'])}</title>
<style>body{{font:16px system-ui;max-width:1200px;margin:32px auto;padding:0 20px;color:#172638}}
table{{border-collapse:collapse;width:100%}}td,th{{padding:12px;border-bottom:1px solid #ccd5df;text-align:left}}
aside{{padding:16px;background:#fff3d7;border-radius:8px}}img{{max-width:240px}}pre{{white-space:pre-wrap}}</style>
<h1>{html.escape(summary['scenario'])}: {'PASS' if summary['prototype_path_verified'] else 'FAIL'}</h1>
<aside>{ROLE}. Team tracker receiver, Builder, full Monado runtime and boards are untested.</aside>
<p>The first frame comes from the native capture source and reaches OpenCV through the original Monado queue.</p>
<table><tr><th>Camera</th><th>Generated</th><th>Received</th><th>Processed</th><th>Rejected</th>
<th>Queue drops</th><th>Median receive-to-CV ms</th><th>Annotated sample</th></tr>{''.join(rows)}</table>
<h2>Checks</h2><pre>{html.escape(json.dumps(summary['checks'], indent=2))}</pre>
<p><a href="manifest.json">Pre-run manifest</a> · <a href="config.json">Effective config</a> ·
<a href="frames.jsonl">Frame-stage logs</a> · <a href="summary.json">Measurements and gaps</a></p>"""
    (output / "report.html").write_text(body)

def run_case(name, config=None, output=None, build=True):
    config = copy.deepcopy(config if config is not None else load_case(name))
    validate_config(config)
    if build:
        ensure_build()
    if not LIBRARY.exists():
        raise RuntimeError("Native library missing; run the component build first")
    cv2.setNumThreads(1)
    lib = library()
    native_format = lib.wrl_format_r8()
    encoded_config = json.dumps(config, sort_keys=True, separators=(",", ":")).encode()
    fixture_id = hashlib.sha256(encoded_config).hexdigest()[:16]
    timestamp = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    if output is None:
        output = ROOT / "results" / "capture-mock" / f"{name}-{timestamp}-{fixture_id}"
    output = Path(output).resolve()
    if output.exists():
        raise FileExistsError(f"Refusing to overwrite {output}")
    output.mkdir(parents=True)
    save_json(output / "config.json", config)
    manifest = {
        "schema_version": 1, "evidence_role": ROLE, "scenario": name, "fixture_id": fixture_id,
        "created_at_utc": timestamp, "config": config, "seed": config["seed"],
        "source_hashes": source_hashes(), "native_library_sha256": sha(LIBRARY),
        "monado": monado_identity(), "python": sys.version, "platform": platform.platform(),
        "opencv": cv2.__version__, "numpy": np.__version__, "opencv_threads": cv2.getNumThreads(),
        "command": [sys.executable, *sys.argv], "clock": CLOCK,
        "entry": "during Capture: native xrt_fs replacement",
        "pixel_format": "XRT_FORMAT_R8, one grayscale byte per pixel; prototype fixture, not sensor RAW",
        "modules": ["new native Capture/xrt_fs", "unchanged u_sink_queue.c",
                    "new native observation sink/ctypes callback", "OpenCV goodFeaturesToTrack"],
        "source_start_interface": ("xrt_fs_slam_stream_start / image-only xrt_slam_sinks bundle"
                                   if config["source_mode"] == "image-bundle" else "xrt_fs_stream_start per camera"),
        "missing": ["team-specific receiver/Builder integration", "full Monado runtime",
                    "RB5/EB5 execution", "physical cameras/CSI/ISP/synchronization"],
        "native_component_build": json.loads((LIBRARY.parent / "build-record.json").read_text())
            if (LIBRARY.parent / "build-record.json").exists() else None,
        "status": "prepared",
    }
    # Preserve authored source, not merely hashes of uncommitted files.
    snapshot_sources(output / "source_snapshot", manifest["source_hashes"])
    manifest["authored_source_snapshot"] = "source_snapshot/tests/standalone-camera-mock/capture_mock"
    # Written before the first capture thread is started.
    save_json(output / "manifest.json", manifest)
    records = []
    artifact_cameras = set()
    record_lock = threading.Lock()
    callback_errors = []
    log = (output / "frames.jsonl").open("w")
    configs, callbacks, handles, stats = [], [], [], {}
    finish_status = {}

    def receive(_userdata, ptr):
        view = ptr.contents
        record = {
            "schema_version": 1, "camera_id": int(view.camera_id), "session_id": int(view.session_id),
            "sequence_number": int(view.sequence), "source_id": int(view.source_id),
            "simulated_capture_ns": int(view.simulated_capture_ns),
            "published_synthetic_timestamp_ns": int(view.published_timestamp_ns),
            "frame_ready_ns": int(view.ready_ns), "host_arrival_ns": int(view.arrival_ns),
            "python_callback_ns": clock_ns(), "host_clock": CLOCK,
            "width": int(view.width), "height": int(view.height), "stride_bytes": int(view.stride),
            "format_enum": int(view.format), "payload_size": int(view.payload_size),
            "producer_payload_crc32": int(view.payload_crc),
        }
        result = 1
        try:
            if config["consumer_delay_ms"]:
                time.sleep(config["consumer_delay_ms"] / 1000)
            if view.format != native_format:
                raise ValueError(f"Unsupported frame format {view.format}; expected native R8 {native_format}")
            if (view.width, view.height) != (config["width"], config["height"]):
                raise ValueError("Unexpected frame dimensions")
            if view.stride < view.width or view.stride != config["width"] + config["padding"]:
                raise ValueError("Invalid row stride")
            if view.payload_size != view.stride * view.height or not view.data:
                raise ValueError("Invalid payload size or null data")
            raw = ct.string_at(view.data, view.payload_size)
            if zlib.crc32(raw) != view.payload_crc:
                raise ValueError("Pixel CRC mismatch between producer and receiver")
            expected_header = bytes([87, 82, 76, 1, view.camera_id,
                                     *int(view.sequence).to_bytes(4, "little"),
                                     config["seed"] & 255, config["seed"] >> 8,
                                     config["session_id"], 0, 0, 0, 0])
            if raw[:16] != expected_header:
                raise ValueError("Camera/frame identity in pixels does not match metadata")
            rows = np.frombuffer(raw, dtype=np.uint8).reshape(view.height, view.stride)
            if config["padding"] and not np.all(rows[:, view.width:] == 165):
                raise ValueError("Row padding was overwritten")
            gray = rows[:, :view.width].copy()
            cv = config["cv"]
            corners = cv2.goodFeaturesToTrack(
                gray, maxCorners=cv["max_corners"], qualityLevel=cv["quality_level"],
                minDistance=cv["min_distance"], blockSize=cv["block_size"])
            record["cv_done_ns"] = clock_ns()
            record["receive_to_cv_ns"] = record["cv_done_ns"] - record["host_arrival_ns"]
            record["frame_ready_to_receive_ns"] = record["host_arrival_ns"] - record["frame_ready_ns"]
            record["corners"] = 0 if corners is None else len(corners)
            record["image_sha256"] = hashlib.sha256(gray.tobytes()).hexdigest()
            record["status"] = "processed"
            with record_lock:
                save_sample = view.camera_id not in artifact_cameras
                artifact_cameras.add(view.camera_id)
            if save_sample:
                prefix = f"camera-{view.camera_id}-session-{view.session_id}"
                if not cv2.imwrite(str(output / f"{prefix}-input.png"), gray):
                    raise RuntimeError("Failed to save input image")
                annotated = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
                if corners is not None:
                    for xy in corners.reshape(-1, 2):
                        cv2.circle(annotated, tuple(np.rint(xy).astype(int)), 3, (0, 210, 40), 1)
                if not cv2.imwrite(str(output / f"{prefix}-annotated.png"), annotated):
                    raise RuntimeError("Failed to save annotated image")
                record["sample_prefix"] = prefix
            result = 0
        except BaseException as error:
            record["status"] = "rejected"
            record["error"] = f"{type(error).__name__}: {error}"
            record["rejected_ns"] = clock_ns()
        try:
            with record_lock:
                records.append(record)
                log.write(json.dumps(record, allow_nan=False) + "\n")
                log.flush()
        except BaseException as error:
            callback_errors.append(str(error))
            result = 1
        return result

    try:
        for camera in range(config["cameras"]):
            native = native_config(config, camera)
            callback = CALLBACK(receive)
            handle = lib.wrl_harness_create(ct.byref(native), callback, None)
            if not handle:
                raise RuntimeError(lib.wrl_last_error().decode())
            configs.append(native)
            callbacks.append(callback)  # Retain callbacks until all native threads are joined.
            handles.append(handle)
        start_ns = clock_ns() + 50_000_000
        manifest["scheduled_monotonic_epoch_ns"] = start_ns
        manifest["status"] = "running"
        save_json(output / "manifest.json", manifest)
        if config["source_mode"] == "image-bundle":
            native_handles = (ct.c_void_p * len(handles))(*handles)
            if lib.wrl_harness_group_start(native_handles, len(handles), start_ns):
                raise RuntimeError("Native image-only Capture bundle failed to start")
        else:
            for handle in handles:
                if lib.wrl_harness_start(handle, start_ns):
                    raise RuntimeError("Native Capture failed to start")
        if config.get("manual_stop_after_ms") is not None:
            time.sleep(config["manual_stop_after_ms"] / 1000)
            for handle in handles:
                lib.wrl_harness_stop(handle)
        for camera, handle in enumerate(handles):
            finish_status[str(camera)] = lib.wrl_harness_finish(handle, config["timeout_ms"])
            value = Stats()
            lib.wrl_harness_stats(handle, ct.byref(value))
            stats[str(camera)] = {n: int(getattr(value, n)) for n, _ in value._fields_}
    finally:
        for handle in handles:
            lib.wrl_harness_close(handle)
        log.close()

    streams = {}
    checks = {"native_finish_and_cleanup": all(x == 0 for x in finish_status.values()),
              "callback_logging": not callback_errors, "upstream_source_unchanged": True}
    for camera in range(config["cameras"]):
        native = stats[str(camera)]
        observed = sorted([r for r in records if r["camera_id"] == camera],
                          key=lambda r: r["host_arrival_ns"])
        processed = [r for r in observed if r["status"] == "processed"]
        sequences = [r["sequence_number"] for r in observed]
        missing = sorted(set(range(native["pushed"])) - set(sequences))
        duplicates = len(sequences) - len(set(sequences))
        expected_generated = config["frames"]
        if config["interrupt_camera"] == camera:
            expected_generated = min(expected_generated, config["interrupt_after"])
        manual_stop = config.get("manual_stop_after_ms") is not None
        expected_rejected = int(config["malformed_camera"] == camera and config["malformed_kind"] > 0)
        durations = [r["receive_to_cv_ns"] for r in processed]
        elapsed = (processed[-1]["cv_done_ns"] - processed[0]["cv_done_ns"]) if len(processed) > 1 else None
        fps = (len(processed) - 1) * 1e9 / elapsed if elapsed and elapsed > 0 else None
        camera_checks = {
            "generated_as_expected": native["generated"] <= expected_generated if manual_stop else native["generated"] == expected_generated,
            "native_no_error": native["native_error"] == 0,
            "sent_received_accounted": native["generated"] == native["pushed"] == len(observed) + len(missing),
            "receiver_cv_accounted": native["received"] == len(observed) == native["processed"] + native["rejected"],
            "expected_rejections": native["rejected"] == expected_rejected,
            "no_duplicates": duplicates == 0,
            "ordering": sequences == sorted(sequences),
            "buffers_released": native["live_frames"] == 0,
            "retained_frame_unchanged": native["retained_valid"] == 1,
            "bounded_live_buffers": native["peak_live_frames"] <= config["queue_capacity"] + 3,
            "clock_ordering": all(r["frame_ready_ns"] <= r["host_arrival_ns"] <= r["python_callback_ns"] <=
                                  r.get("cv_done_ns", r.get("rejected_ns")) for r in observed),
            "visible_features": all(r["corners"] > 0 for r in processed),
        }
        if not config["consumer_delay_ms"]:
            camera_checks["no_unexplained_queue_drops"] = not missing
        else:
            camera_checks["slow_consumer_drops_observed"] = bool(missing)
        checks[f"camera_{camera}"] = camera_checks
        streams[str(camera)] = {
            **native, "queue_dropped": len(missing), "missing_sequences": missing,
            "duplicates": duplicates, "processing_time": percentiles(durations),
            "frame_ready_to_receive": percentiles([r["frame_ready_to_receive_ns"] for r in processed]),
            "observed_processed_fps": fps, "injected_consumer_delay_ms": config["consumer_delay_ms"],
            "rejection_messages": [r["error"] for r in observed if r["status"] == "rejected"],
        }
    if config["cameras"] > 1:
        intervals = [(min(r["frame_ready_ns"] for r in records if r["camera_id"] == c),
                      max(r["frame_ready_ns"] for r in records if r["camera_id"] == c))
                     for c in range(config["cameras"]) if any(r["camera_id"] == c for r in records)]
        overlap = min(end for _, end in intervals) - max(begin for begin, _ in intervals) if intervals else 0
        checks["four_streams_overlap"] = len(intervals) == config["cameras"] and overlap > 0
    if config["source_mode"] == "image-bundle":
        checks["shared_source_id_with_distinct_routes"] = (
            bool(records) and {r["source_id"] for r in records} == {0x57524C47} and
            {r["camera_id"] for r in records} == set(range(config["cameras"])))
    monado_identity()
    flat_checks = []
    for value in checks.values():
        flat_checks.extend(value.values() if isinstance(value, dict) else [value])
    summary = {
        "schema_version": 1, "scenario": name, "evidence_role": ROLE, "config": config,
        "fixture_id": fixture_id, "prototype_path_verified": all(flat_checks),
        "production_tracker_receiver": "not integrated; this receiver is a new validation sink",
        "builder_and_full_runtime": "not exercised", "board_execution": "not tested",
        "physical_camera_verdicts": {"support": "unverified", "concurrent_capture": "unverified",
                                    "exposure_synchronization": "unverified"},
        "clock": CLOCK, "timing_definition": "native sink arrival to CV completion; includes FFI/GIL wait, validation and injected consumer delay; excludes annotation/file encoding",
        "streams": streams, "checks": checks, "callback_errors": callback_errors,
        "output": str(output),
    }
    save_json(output / "summary.json", summary)
    render_report(output, summary)
    manifest["status"] = "complete"
    manifest["artifact_hashes"] = {str(p.relative_to(output)): sha(p) for p in output.rglob("*") if p.is_file() and p != output / "manifest.json"}
    save_json(output / "manifest.json", manifest)
    return summary
