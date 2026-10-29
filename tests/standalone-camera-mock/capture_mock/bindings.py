"""Small ctypes bridge; no copy of Monado's structs is maintained in Python."""
import ctypes as ct
import sys
import platform
from pathlib import Path

from experiment import ROOT
BUILD = ROOT / "build" / f"capture-mock-{sys.platform}-{platform.machine()}"
LIBRARY = BUILD / ("libwrl_mock_capture.dylib" if sys.platform == "darwin" else "libwrl_mock_capture.so")
UINT_MAX = 2**32 - 1

class Config(ct.Structure):
    _fields_ = [(n, ct.c_uint32) for n in (
        "width", "height", "padding", "fps_num", "fps_den", "frames", "camera_id",
        "seed", "queue_capacity", "interrupt_after", "malformed_sequence",
        "malformed_kind", "hold_first", "session_id")]

class FrameView(ct.Structure):
    _fields_ = ([(n, ct.c_uint32) for n in (
        "camera_id", "session_id", "width", "height", "stride", "format",
        "payload_crc", "reserved")] +
        [(n, ct.c_uint64) for n in (
            "sequence", "source_id", "simulated_capture_ns", "published_timestamp_ns",
            "ready_ns", "arrival_ns", "payload_size")] +
        [("data", ct.POINTER(ct.c_uint8))])

class Stats(ct.Structure):
    _fields_ = [(n, ct.c_uint64) for n in (
        "generated", "pushed", "received", "processed", "rejected", "live_frames",
        "peak_live_frames", "retained_valid", "producer_done", "interrupted",
        "started_ns", "ended_ns", "native_error")]

CALLBACK = ct.CFUNCTYPE(ct.c_int, ct.c_void_p, ct.POINTER(FrameView))
_LIBRARY = None

def library():
    global _LIBRARY
    if _LIBRARY is not None:
        return _LIBRARY
    lib = ct.CDLL(str(LIBRARY))
    lib.wrl_config_size.restype = ct.c_uint32
    lib.wrl_frame_view_size.restype = ct.c_uint32
    lib.wrl_format_r8.restype = ct.c_uint32
    lib.wrl_stats_size.restype = ct.c_uint32
    if lib.wrl_config_size() != ct.sizeof(Config) or lib.wrl_frame_view_size() != ct.sizeof(FrameView):
        raise RuntimeError("Native/Python ABI size mismatch; rebuild this source revision")
    if lib.wrl_stats_size() != ct.sizeof(Stats):
        raise RuntimeError("Native/Python stats ABI size mismatch")
    lib.wrl_harness_create.argtypes = [ct.POINTER(Config), CALLBACK, ct.c_void_p]
    lib.wrl_harness_create.restype = ct.c_void_p
    lib.wrl_harness_start.argtypes = [ct.c_void_p, ct.c_uint64]
    lib.wrl_harness_start.restype = ct.c_int
    lib.wrl_harness_group_start.argtypes = [ct.POINTER(ct.c_void_p), ct.c_uint32, ct.c_uint64]
    lib.wrl_harness_group_start.restype = ct.c_int
    lib.wrl_harness_finish.argtypes = [ct.c_void_p, ct.c_uint32]
    lib.wrl_harness_finish.restype = ct.c_int
    lib.wrl_harness_stop.argtypes = [ct.c_void_p]
    lib.wrl_harness_stats.argtypes = [ct.c_void_p, ct.POINTER(Stats)]
    lib.wrl_harness_close.argtypes = [ct.c_void_p]
    for name in ("wrl_harness_stop", "wrl_harness_stats", "wrl_harness_close"):
        getattr(lib, name).restype = None
    lib.wrl_last_error.restype = ct.c_char_p
    _LIBRARY = lib
    return lib

def native_config(config, camera):
    interrupt = config["frames"]
    if config["interrupt_camera"] == camera:
        interrupt = config["interrupt_after"]
    malformed = config["malformed_camera"] == camera
    return Config(
        config["width"], config["height"], config["padding"],
        config["fps_num"], config["fps_den"], config["frames"], camera,
        config["seed"], config["queue_capacity"], interrupt,
        config["malformed_sequence"] if malformed else UINT_MAX,
        config["malformed_kind"] if malformed else 0,
        int(config["hold_first"]), config["session_id"])
