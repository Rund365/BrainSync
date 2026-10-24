import time
import serial
from ..session.logger import Logger

BAUD = 921600  # receiver firmware with the IMUs (protocol 2)
LEGACY_BAUD = 115200  # receiver firmware before the IMUs (protocol 1)
PROTOCOL_VERSION = 2  # newest serial protocol this code understands


def open_receiver(port: str, logger: Logger, baud: int | None = None, timeout: float = 0.2) -> serial.Serial:
    """Open the receiver's serial port. Without `baud`, finds the rate the firmware uses:
    the IMU firmware's first, then the old one's. Raises IOError if neither answers."""
    for rate in ([baud] if baud else [BAUD, LEGACY_BAUD]):
        ser = serial.Serial(port, rate, timeout=timeout)
        if baud or _answers(ser):
            logger.log(f"Receiver on {port} at {rate} baud")
            return ser
        logger.log(f"No answer from the receiver at {rate} baud")
        ser.close()
    raise IOError(f"No answer from the receiver on {port} at {BAUD} or {LEGACY_BAUD} baud")


def _answers(ser, attempts: int = 3) -> bool:
    time.sleep(0.5)  # the ESP32 reboots when the port opens
    ser.reset_input_buffer()
    for _ in range(attempts):
        ser.write(b"RESET\n")  # every firmware version acknowledges RESET
        deadline = time.monotonic() + 0.4
        while time.monotonic() < deadline:
            if ser.readline().decode("utf-8", errors="ignore").strip().startswith("ACK,"):
                return True
    return False


def query_version(ser, logger: Logger, attempts: int = 3) -> int:
    """Serial protocol of the receiver firmware. Firmware from before the VERSION command
    does not answer it: that is protocol 1 (steps only, no IMU)."""
    for _ in range(attempts):
        ser.write(b"VERSION\n")
        deadline = time.monotonic() + 0.3
        while time.monotonic() < deadline:
            resp = ser.readline().decode("utf-8", errors="ignore").strip()
            if resp.startswith("ACK,VERSION,"):
                try:
                    return int(resp.split(",")[2])
                except (IndexError, ValueError):
                    logger.log(f"Unreadable VERSION answer: {resp!r}")
    return 1

# communication with the ESP32
def send_config_command(ser: serial.Serial, logger: Logger, cmd_prefix: str,
                        value: int, desc: str, expected_ack_prefix: str,
                        retries: int = 3):
    """Helper to send configuration for the parameters in the ESP32."""
    #creating the command string
    cmd = f"{cmd_prefix},{value}\n"
    #encoding the command string
    encoded_cmd = cmd.encode("utf-8")
    #sending the command to the ESP32
    for _ in range(retries):
        logger.log(f"Setting {desc} to {value}")
        ser.write(encoded_cmd)
        time.sleep(0.1)
        resp = ser.readline()
        resp_str = resp.decode('utf-8', errors='ignore').strip() if resp else ""
        
        # We check if the response starts with the specific expected ACK prefix
        if resp_str.startswith(expected_ack_prefix):
            logger.log(f"{desc.capitalize()} set response: {resp_str}")
            return True
        
        logger.log(f"Unexpected or no ACK for {desc} (got: {resp_str!r}, expected: {expected_ack_prefix}, retrying...")
        
    logger.log(f"Failed to set {desc} after {retries} attempts")
    return False

def send_calibration_command(ser: serial.Serial, logger: Logger | None = None, margin: int = 150, retries: int = 3,
                             wait_s: float = 4.0) -> bool:
    """Send weight calibration command and wait until the sender reports its new threshold.
    The receiver's first ACK,CAL_WEIGHT only means the command was handed to the radio;
    ACK,CAL_WEIGHT,<threshold> comes from the sender after it calibrated (about 1 s)."""
    log = logger.log if logger else print
    cmd = f"CAL_WEIGHT,{int(margin)}\n".encode("utf-8")
    for _ in range(retries):
        log(f"Calibrating weight with margin={margin}...")
        ser.write(cmd)
        deadline = time.monotonic() + wait_s
        while time.monotonic() < deadline:
            resp = ser.readline()
            resp_str = resp.decode("utf-8", errors="ignore").strip() if resp else ""
            if resp_str == "ERR,CAL_WEIGHT":
                log("Calibration failed: the receiver could not reach the sender, or the sender refused")
                break
            parts = resp_str.split(",")
            if len(parts) == 3 and parts[:2] == ["ACK", "CAL_WEIGHT"] and parts[2].lstrip("-").isdigit():
                log(f"Calibration success, new threshold {parts[2]}")
                return True
        else:
            log("No answer from the sender: is it on and in range?")
    log("Calibration failed after retries")
    return False

def send_handshake_command(ser: serial.Serial, logger: Logger, command: bytes, expected_ack: str, retries: int = 5) -> bool:
    """Helper to send handshake commands and wait for ACK."""
    for _ in range(retries):
        ser.write(command)
        time.sleep(0.1)
        resp = ser.readline()
        resp_str = resp.decode("utf-8", errors="ignore").strip() if resp else ""
        if resp_str == expected_ack:
            logger.log(f"{expected_ack} received")
            return True
        logger.log(f"Unexpected or no ACK (got: {resp_str!r}), retrying...")
    return False

def session_handshake(ser: serial.Serial, logger: Logger, smoothing_window: int = 3, stride: int = 2) -> int:
    """Perform RESET/START handshake with bounded retries.
    Returns the receiver's protocol version (1 = no IMU, 2 = IMU firmware), or 0 on failure."""
    # allow the ESP to finish boot messages, then clear the buffer
    time.sleep(0.5)
    ser.reset_input_buffer()

    version = query_version(ser, logger)
    if version > PROTOCOL_VERSION:
        logger.log(f"Receiver firmware speaks protocol {version}, this code only up to {PROTOCOL_VERSION}: "
                   "update the Python code")
        return 0
    logger.log(f"Receiver firmware: protocol {version}" + (" (no IMU data)" if version < 2 else " (IMU)"))
    ser.reset_input_buffer()

    # Always send: the receiver keeps the previous session's values until it reboots.
    # Window before stride, because the receiver rejects a stride larger than the window.
    if not send_config_command(ser, logger, "SET_WINDOW", smoothing_window, "smoothing window", "ACK,WINDOW"):
        logger.log("Warning: Failed to set smoothing window")

    if not send_config_command(ser, logger, "SET_STRIDE", stride, "update stride", "ACK,STRIDE"):
        logger.log("Warning: Failed to set update stride")

    if not send_handshake_command(ser, logger, b"RESET\n", "ACK,RESET"):
        logger.log("RESET handshake failed")
        return False

    if not send_handshake_command(ser, logger, b"START\n", "ACK,START"):
        logger.log("START handshake failed")
        return False

    logger.log("Session handshake completed")
    return version

def handle_step(raw_line: str, walking_bpm: float):
    step = raw_line.decode("utf-8", errors="ignore").strip()

    # Protocol 2: STEP,<seq>,<t_us>,<foot>,<interval_ms>,<fsr>,<instant_bpm>,<avg_bpm>
    # Protocol 1: <millis>, <foot>, <instant_bpm>, <avg_bpm>
    # Any other line (IMU, LIFT, STATUS, ...) raises ValueError, like a malformed step.
    if step.startswith("STEP,"):
        _, _, ts_str, foot_str, _, _, instant_bpm_str, bpm_str = step.split(",")
    else:
        ts_str, foot_str, instant_bpm_str, bpm_str = step.split(",")

    ts = int(ts_str)
    foot = int(foot_str)
    instant_bpm = float(instant_bpm_str)  # available if you need it later
    bpm = float(bpm_str)

    # If bpm is zero (e.g., initial step), fall back to current walking bpm.
    if bpm == 0.0:
        bpm = walking_bpm

    return bpm, instant_bpm, ts, foot

def send_session_command(ser, logger: Logger, command: str):
    """During a session the reader thread owns the serial input, and IMU lines arrive between
    the answers: only send here. The receiver's ACK comes back as a normal line (main._process_line)."""
    logger.log(f"Sending {command} to the receiver")
    ser.write(f"{command}\n".encode("utf-8"))


# communication with the GUI
def handle_engine_command(cmd, ser, logger, bpm_estimation, player):
    """
    Process a command coming from the GUI/STDIN during a session.
    Returns True if a QUIT signal was handled (caller should break).
    """
    # String commands (from stdin)
    if isinstance(cmd, str):
        if ":" in cmd:
            key, val = cmd.split(":", 1)
            key = key.upper()
            try:
                if key == "SET_ALPHA_UP":
                    bpm_estimation.set_smoothing_alpha_up(float(val))
                    logger.log(f"Config: Alpha UP set to {val}")
                elif key == "SET_ALPHA_DOWN":
                    bpm_estimation.set_smoothing_alpha_down(float(val))
                    logger.log(f"Config: Alpha DOWN set to {val}")
                elif key == "SET_MANUAL_BPM":
                    bpm_estimation.set_manual_bpm(float(val))
                    logger.log(f"Config: Manual BPM set to {val}")
                elif key == "SET_WINDOW":
                    send_session_command(ser, logger, f"SET_WINDOW,{int(val)}")
                elif key == "SET_STRIDE":
                    send_session_command(ser, logger, f"SET_STRIDE,{int(val)}")
                elif key == "SET_RANDOM_SPAN":
                    if hasattr(bpm_estimation, 'set_random_span'):
                        bpm_estimation.set_random_span(float(val))
                elif key == "SET_RANDOM_GAMIFIED":
                    if hasattr(bpm_estimation, 'set_random_gamified'):
                        bpm_estimation.set_random_gamified(int(val) == 1)
                elif key == "SET_RANDOM_SIMPLE_THRESHOLD":
                    if hasattr(bpm_estimation, 'set_random_simple_threshold'):
                        bpm_estimation.set_random_simple_threshold(float(val))
                elif key == "SET_RANDOM_SIMPLE_STEPS":
                    if hasattr(bpm_estimation, 'set_random_simple_steps'):
                        bpm_estimation.set_random_simple_steps(int(val))
                elif key == "SET_RANDOM_SIMPLE_TIMEOUT":
                    if hasattr(bpm_estimation, 'set_random_simple_timeout'):
                        bpm_estimation.set_random_simple_timeout(float(val))
                elif key == "SET_MODE":
                    mode = val.lower()
                    if mode == "manual":
                        bpm_estimation.set_manual_mode(True)
                    elif mode == "random":
                        bpm_estimation.set_random_mode(True)
                    elif mode == "hybrid":
                        bpm_estimation.set_hybrid_mode(True)
                    elif mode == "dynamic":
                        bpm_estimation.set_dynamic_mode(True)
                    logger.log(f"Mode switched to: {mode}")
                elif key == "SET_HYBRID_LOCK_STEPS":
                    bpm_estimation.set_hybrid_lock_steps(int(val))
                elif key == "SET_HYBRID_UNLOCK_TIME":
                    bpm_estimation.set_hybrid_unlock_time(float(val))
                elif key == "SET_HYBRID_STABILITY_THRESHOLD":
                    bpm_estimation.set_hybrid_stability_threshold(float(val))
                elif key == "SET_HYBRID_UNLOCK_THRESHOLD":
                    bpm_estimation.set_hybrid_unlock_threshold(float(val))
                elif key == "CAL_WEIGHT":
                    send_session_command(ser, logger, f"CAL_WEIGHT,{int(val) if val else 200}")
            except ValueError:
                logger.log(f"Invalid command format: {cmd}")
        elif cmd == "QUIT":
            return True
    return False


