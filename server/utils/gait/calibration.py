"""
Per-foot IMU calibration, following Sabatini et al. 2005:
  A. M. Sabatini, C. Martelloni, S. Scapellato, F. Cavallo, "Assessment of Walking Features From
  Foot Inertial Sensing", IEEE Trans. Biomed. Eng. 52(3):486-494, 2005, DOI 10.1109/TBME.2004.840727

- Axis and sign: the gyro axis we use is the one perpendicular to the sagittal plane
  (mediolateral), so it reads the foot's pitch rate directly. Positive = the swing direction
  (their Fig. 2: the swing is a broad positive pulse, toe-off and heel strike are negative peaks).
  Sabatini sets the axis by mounting the sensor by hand (§II-A). We measure it instead, once per
  mounting, from a calibration walk: utils/analysis/calibrate_imus.py (gaitmap) writes
  imu_calibration.json, read here. MOUNTING is only for the simulator and the unit tests.
- Gyro bias ("nulling", §II-C.2): mean of the gyro over the first 0.5 s, while the user stands
  still before walking (Sabatini: N = 100 samples at 200 Hz). The session must start standing still:
  a window whose gyro spread is above STILL_STD_DPS (ours) is thrown away and the next one tried.

gaitmap: Kuderle et al., "Gaitmap - An Open Ecosystem for IMU-Based Human Gait Analysis and
Algorithm Benchmarking", IEEE Open J. Eng. Med. Biol., 2024.
"""
import json
from pathlib import Path

import numpy as np

G = 9.81
AXES = {"x": 0, "y": 1, "z": 2}

# Foot id (1 = right, 2 = left) -> (board axis across the foot, as printed on the BNO055 board,
# sign that makes the swing positive). Mounted the same way on both shoes, the left IMU is turned
# 180 degrees relative to the right one, so its sign is the opposite.
# These values are the simulator's (utils/simulation/legs.py: pitch rate is sensor y, left
# sensor mirrored). Real shoes use imu_calibration.json instead (mounting_from below).
MOUNTING = {1: ("y", +1), 2: ("y", -1)}

CALIBRATION_FILE = Path(__file__).resolve().parents[2] / "imu_calibration.json"  # server/
IMU_OF_FOOT = {1: "right", 2: "left"}  # IMU label (I2C address) that feeds each foot's tracker
MOVED_DEG = 10.0  # OURS, placeholder: gravity while standing this far from the calibration's "up" = the IMU
                  # has moved on the shoe. Sabatini 2005 (§II-D, §IV) checks the mounting the same way
                  # before each trial and saw 1.2 deg (SD) of change over about 2 hours

SAMPLE_RATE_HZ = 100
BIAS_S = 0.5  # Sabatini §II-C.2: N = 100 samples at 200 Hz
BIAS_SAMPLES = round(BIAS_S * SAMPLE_RATE_HZ)
STILL_STD_DPS = 2.0  # OURS, placeholder: a 0.5 s window counts as standing still only if the gyro
                     # spread on every axis is below this. To be set from the table and standing
                     # recordings (Allan variance, Woodman 2007 §5.1).


def load_calibration(path: Path = CALIBRATION_FILE) -> dict | None:
    """imu_calibration.json as written by utils/analysis/calibrate_imus.py, or None if there is none."""
    if not path.exists():
        return None
    return json.loads(path.read_text(encoding="utf-8"))


def mounting_from(calibration: dict | None) -> dict:
    """Foot id -> (axis, sign). An IMU in the calibration uses its measured pitch axis (the sign is
    already in the axis); an IMU that is not there falls back to MOUNTING."""
    mounting = dict(MOUNTING)
    for foot, label in IMU_OF_FOOT.items():
        imu = (calibration or {}).get("imus", {}).get(label)
        if imu:
            mounting[foot] = (imu["pitch_axis"], +1)
    return mounting


def up_from(calibration: dict | None) -> dict:
    """Foot id -> the 'up' direction its IMU had in the calibration walk (sensor axes), where stored."""
    imus = (calibration or {}).get("imus", {})
    return {foot: np.asarray(imus[label]["up"], float) for foot, label in IMU_OF_FOOT.items()
            if "up" in imus.get(label, {})}


def tilt_deg(standing_acc: np.ndarray, up: np.ndarray) -> float:
    """Angle between gravity measured while standing still and the calibration's 'up'."""
    cos = float(standing_acc @ up) / (np.linalg.norm(standing_acc) * np.linalg.norm(up))
    return float(np.degrees(np.arccos(min(1.0, max(-1.0, cos)))))


class FootCalibration:
    def __init__(self, axis="y", sign: int = +1):
        # axis: a board axis name ("x", "y", "z") or a unit vector in the sensor frame
        self.axis = np.eye(3)[AXES[axis]] if isinstance(axis, str) else np.asarray(axis, float)
        self.sign = int(sign)
        self.gyro_bias = np.zeros(3)
        self.bias_std = np.zeros(3)  # gyro spread over the accepted window
        self.still_samples = 0  # samples seen while measuring the bias (names kept for phase_controller)
        self.swing_samples = 0  # samples after the bias is known
        self.rejected_windows = 0  # 0.5 s windows thrown away because the foot was moving
        self.standing_acc = None  # accelerometer mean over the accepted window = gravity while standing
        self._window = []
        self._acc_window = []
        self._bias_ok = False

    @property
    def ready(self) -> bool:
        return self._bias_ok

    def add(self, acc: np.ndarray, gyr: np.ndarray):
        if self.ready:
            self.swing_samples += 1
            return
        self.still_samples += 1
        self._window.append(np.asarray(gyr, float))
        self._acc_window.append(np.asarray(acc, float))
        if len(self._window) == BIAS_SAMPLES:
            w = np.array(self._window)
            std = w.std(axis=0)
            if (std < STILL_STD_DPS).all():  # standing still: Sabatini §II-C.2 nulling
                self.gyro_bias, self.bias_std, self._bias_ok = w.mean(axis=0), std, True
                self.standing_acc = np.mean(self._acc_window, axis=0)
            else:  # moving: try the next 0.5 s
                self.rejected_windows += 1
            self._window, self._acc_window = [], []

    def pitch_rate(self, gyr: np.ndarray) -> float:
        """Foot pitch rate in dps, positive = swing direction."""
        return self.sign * float((gyr - self.gyro_bias) @ self.axis)

    def state(self) -> dict:
        return {
            "gyro_bias_dps": self.gyro_bias.round(3).tolist(),
            "bias_std_dps": self.bias_std.round(3).tolist(),
            "standing_acc_ms2": None if self.standing_acc is None else self.standing_acc.round(3).tolist(),
            "rejected_windows": self.rejected_windows,
            "pitch_axis": (self.axis * self.sign).round(4).tolist(),
            "still_samples": self.still_samples,
            "swing_samples": self.swing_samples,
        }
