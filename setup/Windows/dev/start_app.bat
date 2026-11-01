@echo off
:: ======================================================================================
:: BRAIN MUSIC INTERFACE - LAUNCHER SCRIPT
:: ======================================================================================
:: Runs the GUI inside the project's virtual environment (.venv). If requirements.txt
:: changed since the last install (for example after pulling a new branch), the libraries
:: are updated first.

:: Navigate to project root (three levels up from setup/Windows/dev/)
cd /d "%~dp0..\..\..\"

title BRAIN MUSIC INTERFACE
cls

echo ==================================================
echo       STARTING BRAIN MUSIC INTERFACE...
echo ==================================================
echo.

:: 1. Check for Virtual Environment
if exist .venv\Scripts\activate.bat (
    call .venv\Scripts\activate.bat
) else (
    echo Error: Virtual environment '.venv' not found.
    echo Please run 'setup_env.bat' first!
    pause
    exit /b 1
)

:: 2. Update the libraries if requirements.txt changed
python setup\check_env.py --sync
if errorlevel 1 (
    pause
    exit /b 1
)

:: 3. Launch Application
echo Launching GUI...
python server\gui_app.py

:: 4. Exit Handling: keep the window open after a crash so the error can be read
if errorlevel 1 (
    echo.
    echo Application crashed or exited with error.
    pause
)
