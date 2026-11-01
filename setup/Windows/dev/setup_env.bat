@echo off
:: ======================================================================================
:: BRAIN MUSIC INTERFACE - SETUP WIZARD
:: ======================================================================================
:: Finds (or installs) a suitable Python, creates the virtual environment (.venv), installs
:: all dependencies and checks that everything the app needs is there.
::
:: Why the Python version matters: the MIDI library (python-rtmidi) has ready-made packages
:: only for Python 3.10, 3.11 and 3.12. On newer versions pip would have to build it from
:: source, which needs a C++ compiler most computers do not have.

:: Navigate to project root (three levels up from setup/Windows/dev/)
cd /d "%~dp0..\..\..\"

title BRAIN MUSIC INTERFACE - SETUP
cls
echo ==================================================
echo       BRAIN MUSIC INTERFACE - SETUP WIZARD
echo ==================================================
echo.
echo This script will:
echo   1. Find Python 3.10-3.12 (and offer to install 3.12 if there is none)
echo   2. Create a Python Virtual Environment (.venv) and upgrade pip
echo   3. Install all dependencies
echo   4. Check the environment (libraries, MIDI, serial ports, model)
echo.
pause

:: --------------------------
:: Step 1: Find a suitable Python, install 3.12 if there is none
:: --------------------------
echo.
echo [1/4] Finding Python...
set PYTHON_CMD=
set GOOD_PYTHON=
for %%V in (3.11 3.12 3.10) do (
    if not defined PYTHON_CMD (
        py -%%V --version >nul 2>&1 && set "PYTHON_CMD=py -%%V" && set GOOD_PYTHON=1
    )
)
if not defined PYTHON_CMD call :find_installed_312
if not defined PYTHON_CMD call :install_python
if not defined PYTHON_CMD (
    python --version >nul 2>&1 && set "PYTHON_CMD=python"
)
if not defined PYTHON_CMD (
    py --version >nul 2>&1 && set "PYTHON_CMD=py"
)
if not defined PYTHON_CMD (
    echo No Python found. Please install Python 3.12 from https://www.python.org/downloads/ and run this again.
    pause
    exit /b 1
)
echo Using: %PYTHON_CMD%
%PYTHON_CMD% --version
if not defined GOOD_PYTHON (
    echo WARNING: this is not Python 3.10-3.12, so installing the MIDI library may fail.
)

:: --------------------------
:: Step 2: Create and activate the Virtual Environment
:: --------------------------
echo.
echo [2/4] Creating the virtual environment...

:: An existing .venv made with an unsuitable Python cannot install the MIDI library
if defined GOOD_PYTHON if exist .venv\Scripts\python.exe (
    .venv\Scripts\python.exe -c "import sys; sys.exit(0 if (3, 10) <= sys.version_info[:2] <= (3, 12) else 1)"
    if errorlevel 1 call :rebuild_venv
)

if not exist .venv\Scripts\activate.bat (
    %PYTHON_CMD% -m venv .venv
    if errorlevel 1 (
        echo Error creating venv!
        pause
        exit /b 1
    )
) else (
    echo Virtual environment already exists, skipping creation.
)

call .venv\Scripts\activate.bat
echo Upgrading pip...
python -m pip install --upgrade pip

:: --------------------------
:: Step 3: Install Dependencies
:: --------------------------
echo.
echo [3/4] Installing dependencies...
python -m pip install -r requirements.txt
if errorlevel 1 (
    echo.
    echo Error installing dependencies!
    echo If python-rtmidi failed to build, delete the .venv folder and run this setup again:
    echo it will offer to install Python 3.12.
    pause
    exit /b 1
)
python setup\check_env.py --mark
echo Dependencies installed successfully.

:: --------------------------
:: Step 4: Check the environment
:: --------------------------
echo.
echo [4/4] Checking the environment...
python setup\check_env.py
if errorlevel 1 (
    echo.
    echo Setup finished with problems, see [FAIL] above.
    pause
    exit /b 1
)

:: --------------------------
:: Completion
:: --------------------------
echo.
echo ==================================================
echo       SETUP COMPLETE!
echo ==================================================
echo.
echo You can now use 'start_app.bat' to run the application.
echo.
pause
exit /b 0


:: ======================================================================================
:: Helpers (called with "call :name")
:: ======================================================================================

:: Python 3.12 installed but not on the PATH yet (e.g. right after installing it)
:find_installed_312
for %%P in ("%LOCALAPPDATA%\Programs\Python\Python312\python.exe" "%ProgramFiles%\Python312\python.exe") do (
    if not defined PYTHON_CMD if exist %%P (
        set PYTHON_CMD=%%P
        set GOOD_PYTHON=1
    )
)
exit /b 0

:: Offers to install Python 3.12 with winget (built into Windows 10 and 11)
:install_python
echo.
echo No Python 3.10, 3.11 or 3.12 was found on this computer.
echo The MIDI library ^(python-rtmidi^) only has ready-made packages for these versions,
echo so Python 3.12 is needed. It is installed next to any other Python; nothing is removed.
where winget >nul 2>&1
if errorlevel 1 (
    echo winget is not available here. Install Python 3.12 from https://www.python.org/downloads/
    echo and run this setup again.
    exit /b 0
)
choice /c YN /m "Install Python 3.12 now"
if errorlevel 2 exit /b 0
winget install -e --id Python.Python.3.12 --accept-package-agreements --accept-source-agreements
call :find_installed_312
if not defined PYTHON_CMD (
    py -3.12 --version >nul 2>&1 && set "PYTHON_CMD=py -3.12" && set GOOD_PYTHON=1
)
if not defined PYTHON_CMD (
    echo Python 3.12 was installed but cannot be found yet.
    echo Close this window and run setup_env.bat again.
)
exit /b 0

:: Deletes a .venv made with an unsuitable Python, so it is made again with the right one
:rebuild_venv
echo.
echo The existing .venv uses a Python version the MIDI library has no package for.
choice /c YN /m "Delete .venv and make it again with the right Python"
if errorlevel 2 exit /b 0
rmdir /s /q .venv
exit /b 0
