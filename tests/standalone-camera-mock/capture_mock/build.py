"""Build only native capture/queue components, never install a Monado runtime."""
import json
import platform
import subprocess
import sys
from pathlib import Path
from .bindings import ROOT, BUILD

from experiment import MONADO_SOURCE_DIR, monado_identity, sha

def ensure_build():
    monado = monado_identity()
    bin_dir = Path(sys.executable).parent
    cmake, ninja, ctest = (bin_dir / n for n in ("cmake", "ninja", "ctest"))
    if not all(p.exists() for p in (cmake, ninja, ctest)):
        raise RuntimeError("Use the project .venv with capture_mock/requirements.lock installed")
    BUILD.mkdir(parents=True, exist_ok=True)
    commands = [
        [str(cmake), "-S", str(ROOT / "capture_mock" / "native"), "-B", str(BUILD),
         "-G", "Ninja", f"-DCMAKE_MAKE_PROGRAM={ninja}", "-DCMAKE_BUILD_TYPE=RelWithDebInfo", f"-DMONADO_SOURCE_DIR={MONADO_SOURCE_DIR}"],
        [str(cmake), "--build", str(BUILD), "--parallel", "2"],
        [str(ctest), "--test-dir", str(BUILD), "--output-on-failure"],
    ]
    for command in commands:
        result = subprocess.run(command, cwd=ROOT, text=True, capture_output=True, timeout=120)
        if result.returncode:
            raise RuntimeError(result.stdout + result.stderr)
    record = {"role": "Native component build, not full Monado runtime",
              "commands": commands, "monado": monado,
              "python": sys.version, "host": platform.platform(),
              "cmake": subprocess.check_output([cmake, "--version"], text=True).splitlines()[0],
              "ninja": subprocess.check_output([ninja, "--version"], text=True).strip(),
              "configuration": "CPU frame queue only; upstream config templates with optional tracing/runtime modules disabled"}
    (BUILD / "build-record.json").write_text(json.dumps(record, indent=2) + "\n")
    return record
