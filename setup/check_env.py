"""
Environment check for BrainSync, used by the setup and start scripts (Windows and Linux/Mac).

  python setup/check_env.py            full check: Python version, libraries, MIDI outputs,
                                       serial ports, audio inputs, prediction model
  python setup/check_env.py --sync     reinstall the requirements only if requirements.txt
                                       changed since the last install (used by start_app)
  python setup/check_env.py --mark     record requirements.txt as installed (used by setup_env)

Exit code 1 only when something the app cannot run without is missing.
"""
import argparse
import hashlib
import importlib
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
REQUIREMENTS = ROOT / "requirements.txt"
MARKER = ROOT / ".venv" / ".requirements.sha256"
MODEL = ROOT / "server" / "utils" / "prediction_model" / "results" / "models" / "lgbm_model.joblib"

# Python versions with ready-made packages for every library (python-rtmidi needs a compiler otherwise)
TESTED_PYTHON = ((3, 10), (3, 11), (3, 12))

# (import name, pip name, needed to run the app)
LIBRARIES = [
    ("numpy", "numpy", True),
    ("pandas", "pandas", True),
    ("matplotlib", "matplotlib", True),
    ("serial", "pyserial", True),
    ("mido", "mido", True),
    ("rtmidi", "python-rtmidi", True),
    ("sklearn", "scikit-learn", True),
    ("lightgbm", "lightgbm", True),
    ("joblib", "joblib", True),
    ("tkinter", "tkinter (comes with Python)", True),
    ("sounddevice", "sounddevice", False),  # only the latency measurement needs it
    ("optuna", "optuna", False),            # only model training
    ("plotly", "plotly", False),
    ("PIL", "Pillow", False),
]


def ok(msg):
    print(f"  [OK]   {msg}")


def warn(msg):
    print(f"  [WARN] {msg}")


def fail(msg):
    print(f"  [FAIL] {msg}")


def requirements_hash() -> str:
    return hashlib.sha256(REQUIREMENTS.read_bytes()).hexdigest()


def mark_installed():
    MARKER.parent.mkdir(parents=True, exist_ok=True)
    MARKER.write_text(requirements_hash())


def sync_requirements() -> int:
    """Reinstall only when requirements.txt changed since the last successful install."""
    if MARKER.exists() and MARKER.read_text().strip() == requirements_hash():
        return 0
    print("requirements.txt changed since the last install: updating the libraries...")
    result = subprocess.run([sys.executable, "-m", "pip", "install", "-r", str(REQUIREMENTS)])
    if result.returncode != 0:
        print("Installing the requirements failed. Run setup_env again to see the details.")
        return result.returncode
    mark_installed()
    return 0


def check_python() -> bool:
    v = sys.version_info[:2]
    text = f"Python {v[0]}.{v[1]} ({sys.executable})"
    if v in TESTED_PYTHON:
        ok(text)
    elif v < TESTED_PYTHON[0]:
        fail(text + f": too old, install Python {TESTED_PYTHON[-1][0]}.{TESTED_PYTHON[-1][1]}")
        return False
    else:
        warn(text + ": newer than tested. If python-rtmidi fails to install, use Python 3.11 or 3.12.")
    return True


def check_libraries() -> bool:
    all_needed = True
    for module, pip_name, needed in LIBRARIES:
        try:
            importlib.import_module(module)
            ok(pip_name)
        except Exception as e:
            if needed:
                fail(f"{pip_name} missing ({e.__class__.__name__}): the app cannot run without it")
                all_needed = False
            else:
                warn(f"{pip_name} missing: only needed for extra tools")
    return all_needed


def check_midi():
    try:
        import mido
        mido.set_backend("mido.backends.rtmidi")
        outputs = mido.get_output_names()
    except Exception as e:
        warn(f"MIDI outputs could not be listed: {e}")
        return
    if not outputs:
        warn("No MIDI output found: the music cannot play")
        return
    ok(f"MIDI outputs: {', '.join(outputs)}  (the first one is used)")
    if any("GS Wavetable" in name for name in outputs):
        print("         Note: the Windows GS Wavetable synth is slow; measure its delay with")
        print("         'Measure latency' in the GUI (or server/utils/analysis/measure_latency.py).")


def check_serial():
    try:
        from serial.tools import list_ports
        ports = [f"{p.device} ({p.description})" for p in list_ports.comports()]
    except Exception as e:
        warn(f"Serial ports could not be listed: {e}")
        return
    if ports:
        ok("Serial ports: " + "; ".join(ports))
    else:
        warn("No serial port found: plug in the receiver ESP32 before starting a session")


def check_audio():
    try:
        import sounddevice
        inputs = [d["name"] for d in sounddevice.query_devices() if d["max_input_channels"] > 0]
    except Exception as e:
        warn(f"Audio inputs could not be listed ({e.__class__.__name__}): only the latency measurement needs them")
        return
    if inputs:
        ok(f"Audio inputs (for the latency measurement): {len(inputs)} found, e.g. {inputs[0]}")
    else:
        warn("No microphone found: the latency measurement needs one")


def check_model():
    if MODEL.exists():
        ok(f"Prediction model: {MODEL.relative_to(ROOT)}")
    else:
        warn("No trained prediction model yet: the app runs without it; train one in the Model Learning tab")


def full_check() -> int:
    print("\nChecking the environment...")
    good = check_python()
    good = check_libraries() and good
    if good:
        check_midi()
        check_serial()
        check_audio()
    check_model()
    print()
    if good:
        print("Environment ready.")
        return 0
    print("Some required parts are missing (see [FAIL] above).")
    return 1


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--sync", action="store_true", help="reinstall the requirements if they changed")
    parser.add_argument("--mark", action="store_true", help="record the requirements as installed")
    args = parser.parse_args()
    if args.mark:
        mark_installed()
        return 0
    if args.sync:
        return sync_requirements()
    return full_check()


if __name__ == "__main__":
    sys.exit(main())
