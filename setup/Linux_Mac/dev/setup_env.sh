#!/bin/bash

# ======================================================================================
# BRAIN MUSIC INTERFACE - SETUP WIZARD (Linux/Mac)
# ======================================================================================

# Navigate to project root (three levels up from setup/Linux_Mac/dev/)
cd "$(dirname "$0")/../../.." || exit 1

# SETUP NOTE:
# If you want to do a full test run on Windows (using WSL), create a separate copy of 
# your project folder (e.g., test_linux), open WSL in that folder, and run it there.
# Do NOT run this in the same folder as your Windows setup, or it will break your .venv!

echo "=================================================="
echo "      BRAIN MUSIC INTERFACE - SETUP WIZARD"
echo "=================================================="
echo
echo "This script will:"
echo "  1. Find Python 3.10-3.12 (and offer to install 3.12 if there is none)"
echo "     and create a Python Virtual Environment (.venv)"
echo "  2. Upgrade pip"
echo "  3. Install all dependencies"
echo "  4. Check the environment (libraries, MIDI, serial ports, model)"
echo
read -p "Press Enter to continue..."

# 1. Create Virtual Environment
echo
echo "[1/4] Creating virtual environment..."

# The MIDI library (python-rtmidi) has ready-made packages only for Python 3.10-3.12;
# on newer versions pip would have to build it, which needs a compiler.
PYTHON_CMD=""
GOOD_PYTHON=""
for candidate in python3.11 python3.12 python3.10; do
    if command -v "$candidate" &> /dev/null; then
        PYTHON_CMD="$candidate"
        GOOD_PYTHON=1
        break
    fi
done

if [ -z "$PYTHON_CMD" ]; then
    echo "No Python 3.10, 3.11 or 3.12 found. The MIDI library (python-rtmidi) only has ready-made"
    echo "packages for these versions, so Python 3.12 is needed. It is installed next to any other"
    echo "Python; nothing is removed."
    if command -v brew &> /dev/null; then
        read -p "Install Python 3.12 now with 'brew install python@3.12 python-tk@3.12'? [y/N]: " REPLY
        if [[ "$REPLY" =~ ^[Yy]$ ]]; then
            brew install python@3.12 python-tk@3.12 && command -v python3.12 &> /dev/null && PYTHON_CMD="python3.12" && GOOD_PYTHON=1
        fi
    elif command -v apt-get &> /dev/null; then
        read -p "Install Python 3.12 now with 'sudo apt-get install -y python3.12 python3.12-venv python3.12-tk'? [y/N]: " REPLY
        if [[ "$REPLY" =~ ^[Yy]$ ]]; then
            sudo apt-get update && sudo apt-get install -y python3.12 python3.12-venv python3.12-tk \
                && PYTHON_CMD="python3.12" && GOOD_PYTHON=1
            if [ -z "$GOOD_PYTHON" ]; then
                echo "python3.12 is not in this system's package list (older Ubuntu/Debian):"
                echo "install it from python.org or the deadsnakes PPA, then rerun this setup."
            fi
        fi
    else
        echo "Install Python 3.12 from https://www.python.org/downloads/ and rerun this setup."
    fi
fi

if [ -z "$PYTHON_CMD" ]; then
    for candidate in python3 python; do
        if command -v "$candidate" &> /dev/null; then
            PYTHON_CMD="$candidate"
            break
        fi
    done
fi
if [ -z "$PYTHON_CMD" ]; then
    echo "Error: no Python found. Please install Python 3.12."
    exit 1
fi
echo "Using: $PYTHON_CMD ($($PYTHON_CMD --version))"
if [ -z "$GOOD_PYTHON" ]; then
    echo "WARNING: this is not Python 3.10-3.12, so installing the MIDI library may fail."
fi

# Ensure the venv module is available (needed on Debian/Ubuntu: python3-venv)
if ! $PYTHON_CMD -c "import venv, ensurepip" 2> /dev/null; then
    echo "Error: Python venv/ensurepip modules not available."
    echo "Install them (e.g., 'sudo apt install -y python3.10-venv') and rerun."
    exit 1
fi

# An existing .venv made with an unsuitable Python cannot install the MIDI library
if [ -n "$GOOD_PYTHON" ] && [ -x ".venv/bin/python" ]; then
    if ! .venv/bin/python -c "import sys; sys.exit(0 if (3, 10) <= sys.version_info[:2] <= (3, 12) else 1)"; then
        echo "The existing .venv uses a Python version the MIDI library has no package for."
        read -p "Delete .venv and make it again with $PYTHON_CMD? [y/N]: " REPLY
        if [[ "$REPLY" =~ ^[Yy]$ ]]; then
            rm -rf .venv
        fi
    fi
fi

if [ ! -d ".venv" ]; then
    $PYTHON_CMD -m venv .venv
    if [ $? -ne 0 ]; then
        echo "Error creating venv!"
        exit 1
    fi
else
    echo "Virtual environment already exists, skipping creation."
fi

if [ ! -f ".venv/bin/activate" ]; then
    echo "Virtual environment is missing '.venv/bin/activate'."
    echo "Remove '.venv' and rerun this script."
    exit 1
fi

# 2. Activate Environment
echo
echo "[2/4] Activating environment and upgrading pip..."
source .venv/bin/activate
echo "Upgrading pip..."
pip install --upgrade pip

# 2b. Ensure Tkinter is available (system package, not installed via pip)
echo
echo "[2b] Checking Tkinter (python3-tk)..."
python - <<'PY'
try:
    import tkinter  # noqa: F401
    print("Tkinter available.")
except Exception:
    raise SystemExit(1)
PY

if [ $? -ne 0 ]; then
    echo "Tkinter not available."
    if command -v apt-get >/dev/null 2>&1; then
        read -p "Install python3-tk via 'sudo apt-get install -y python3-tk'? [y/N]: " REPLY
        if [[ "$REPLY" =~ ^[Yy]$ ]]; then
            sudo apt-get update && sudo apt-get install -y python3-tk
            if [ $? -ne 0 ]; then
                echo "Failed to install python3-tk. Please install it manually and rerun."
                exit 1
            fi
        else
            echo "Please install python3-tk manually and rerun the setup."
            exit 1
        fi
    else
        echo "Install Tkinter for your platform (e.g., python3-tk on Debian/Ubuntu) and rerun."
        exit 1
    fi
fi

# 3. Install Dependencies
echo
echo "[3/4] Installing dependencies..."
python -m pip install -r requirements.txt
if [ $? -ne 0 ]; then
    echo "Error installing dependencies!"
    echo "If python-rtmidi failed to build, remove .venv and rerun: it offers to install Python 3.12."
    exit 1
fi
python setup/check_env.py --mark
echo "Dependencies installed successfully."

# 4. Check the environment
echo
echo "[4/4] Checking the environment..."
python setup/check_env.py
if [ $? -ne 0 ]; then
    echo
    echo "Setup finished with problems, see [FAIL] above."
    exit 1
fi

echo
echo "=================================================="
echo "      SETUP COMPLETE!"
echo "=================================================="
echo
echo "You can now use './start_app.sh' to run the application."
echo "Note: You may need to run 'chmod +x setup_env.sh' and 'chmod +x start_app.sh' first."
echo
