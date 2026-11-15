"""
Per-foot IMU calibration and foot orientation.

The IMU calibration file (imu_calibration.json, written once per mounting by
utils/analysis/calibrate_imus.py from a calibration walk, with gaitmap) gives each IMU's pitch axis
(with its sign) and "up". See Documentation/HOW_IT_WORKS.md §4. Without it there is no pitch axis,
so Sabatini's events are not detected: the session's IMU events are not valid (main.py warns).

Source for the gait-event method: A. M. Sabatini, C. Martelloni, S. Scapellato, F. Cavallo,
"Assessment of Walking Features From Foot Inertial Sensing", IEEE Trans. Biomed. Eng.
52(3):486-494, 2005, DOI 10.1109/TBME.2004.840727.

- Gyro bias: Sabatini §II-C.2, the mean of 0.5 s standing still at the session start (a window whose
  gyro spread is above STILL_STD_DPS is thrown away and the next one tried).
- Gravity: the accelerometer over that same window. Its size is gravity as this sensor reads it (the
  recorded IMU reads ~10.4 m/s², not 9.81; Sabatini §II-D checks |a| = 1 g for the same reason). Its
  direction is "up" if the file has none.
- Foot orientation: "up" is carried through every sample by the gyro (a vector fixed in the room,
  seen from the turning sensor) and pulled back to gravity while the foot is flat after a landing
  (UP_WINDOW_S). So the vertical acceleration is right whether the foot stays level or tips 70° in a swing.
- Has the IMU moved since its calibration walk? At the session start, gravity while standing still
  is compared with the file's "up" (engine/sensor_health.py, as Sabatini §II-D checks before each trial).

gaitmap: Kuderle et al., "Gaitmap - An Open Ecosystem for IMU-Based Human Gait Analysis and
Algorithm Benchmarking", IEEE Open J. Eng. Med. Biol., 2024.
"""
import json
from pathlib import Path

import numpy as np

G = 9.81
AXES = {"x": 0, "y": 1, "z": 2}

CALIBRATION_FILE = Path(__file__).resolve().parents[2] / "imu_calibration.json"  # server/
# IMU label (I2C address, the "left" / "right" slot of each IMU line) that sits on each foot. The
# shoes are wired crossed: the IMU labelled "left" is on the right foot (the recordings of
# session_2026-11-01: each IMU's swings line up with the other foot's FSR). Everything that turns IMU
# slots into feet goes through this (imu_of_foot), so the hardware stays as it is.
IMU_OF_FOOT = {1: "left", 2: "right"}
MOVED_DEG = 10.0  # placeholder: gravity while standing this far from the calibration's "up" = the IMU
                  # has moved on the shoe. Sabatini 2005 (§II-D, §IV) checks the mounting the same way
                  # before each trial and saw 1.2 deg (SD) of change over about 2 hours

SAMPLE_RATE_HZ = 100
SAMPLE_S = 1.0 / SAMPLE_RATE_HZ
BIAS_S = 0.5  # Sabatini §II-C.2: N = 100 samples at 200 Hz
BIAS_SAMPLES = round(BIAS_S * SAMPLE_RATE_HZ)
STILL_STD_DPS = 2.0  # placeholder: a 0.5 s window counts as standing still only if the gyro
                     # spread on every axis is below this. To be set from the table and standing
                     # recordings (Allan variance, Woodman 2007 §5.1).
UP_WINDOW_S = (0.20, 0.30)  # after a landing, past the impact and before the foot lifts (foot down
                            # ~0.55 s in the recordings, heel-off ~0.35 s after heel strike when walking
                            # at 130 steps/min): the foot is flat. Starts when the landing is confirmed
                            # (events.py SETTLE_S)
FLAT_GAIN = 0.2  # per sample, how far the tracked "up" is pulled to gravity while the foot is flat


def _unit(v):
    n = np.linalg.norm(v)
    return v / n if n > 0 else v


def load_calibration(path: Path = CALIBRATION_FILE) -> dict | None:
    """imu_calibration.json as written by utils/analysis/calibrate_imus.py, or None if there is none."""
    if not path.exists():
        return None
    return json.loads(path.read_text(encoding="utf-8"))


def mounting_from(calibration: dict | None) -> dict:
    """Foot id -> (axis, sign) for each IMU in the calibration (the sign is already in the axis).
    An IMU that is not there is left out."""
    mounting = {}
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


def imu_of_foot(msg, foot: int):
    """(valid, raw values) of the IMU on this foot, from an IMU message (hardware.sensor_stream.ImuMsg)."""
    return (msg.valid_left, msg.left) if IMU_OF_FOOT[foot] == "left" else (msg.valid_right, msg.right)


def tilt_deg(standing_acc: np.ndarray, up: np.ndarray) -> float:
    """Angle between gravity measured while standing still and the calibration's 'up'."""
    cos = float(standing_acc @ up) / (np.linalg.norm(standing_acc) * np.linalg.norm(up))
    return float(np.degrees(np.arccos(min(1.0, max(-1.0, cos)))))


class FootCalibration:
    def __init__(self, axis=None, sign: int | None = None, up=None):
        """axis / sign / up: from the IMU calibration file (mounting_from, up_from). axis is a board
        axis name ("x", "y", "z") or a unit vector in the sensor frame; None = no pitch axis."""
        if axis is not None:
            self.axis = np.eye(3)[AXES[axis]] if isinstance(axis, str) else _unit(np.asarray(axis, float))
            self.sign = int(sign or 1)
        else:
            self.axis, self.sign = None, 1
        self.up_given = up is not None
        self.gyro_bias = np.zeros(3)
        self.bias_std = np.zeros(3)
        self.up = None if up is None else _unit(np.asarray(up, float))  # "up" while the foot is flat
        self.up_now = None if up is None else self.up.copy()  # "up" in the sensor frame now, carried by the gyro
        self.g_level = G  # |acceleration| this sensor reads at rest (m/s²)
        self.still_samples = 0  # samples before the bias is known (names kept for phase_controller)
        self.swing_samples = 0  # samples after it is known
        self.rejected_windows = 0  # 0.5 s windows thrown away because the foot was moving
        self.standing_acc = None  # accelerometer mean over the accepted still window = gravity while standing
        self._window = []
        self._acc_window = []
        self._bias_ok = False
        self._last_t = None
        self._landed = None  # time of this foot's last landing (events.py LAND)

    @property
    def ready(self) -> bool:
        """Gyro bias known (the IMU counts as calibrated; Sabatini's events also need the axis)."""
        return self._bias_ok

    def add(self, acc: np.ndarray, gyr: np.ndarray, t: float | None = None):
        acc, gyr = np.asarray(acc, float), np.asarray(gyr, float)
        dt = SAMPLE_S if self._last_t is None or t is None else min(max(t - self._last_t, 0.0), 0.05)
        self._last_t = t
        flat = (self._landed is not None and t is not None
                and UP_WINDOW_S[0] <= t - self._landed <= UP_WINDOW_S[1])
        self._track_up(acc, gyr, dt, flat)

        if self.ready:
            self.swing_samples += 1
            return
        self.still_samples += 1
        self._window.append(gyr)
        self._acc_window.append(acc)
        if len(self._window) == BIAS_SAMPLES:
            w = np.array(self._window)
            std = w.std(axis=0)
            if (std < STILL_STD_DPS).all():  # standing still: Sabatini §II-C.2 nulling
                self.gyro_bias, self.bias_std, self._bias_ok = w.mean(axis=0), std, True
                self.standing_acc = np.mean(self._acc_window, axis=0)
                self.g_level = float(np.linalg.norm(self.standing_acc))
                if self.up is None:
                    self.up = self.standing_acc / self.g_level
                    self.up_now = self.up.copy()
            else:  # moving: try the next 0.5 s
                self.rejected_windows += 1
            self._window, self._acc_window = [], []

    def _track_up(self, acc, gyr, dt, flat):
        if self.up_now is None:
            return
        w = np.radians(gyr - self.gyro_bias)
        self.up_now = _unit(self.up_now - np.cross(w, self.up_now) * dt)  # a room-fixed vector, sensor frame
        if flat:  # foot flat and still: the accelerometer shows where up is
            self.up_now = _unit(self.up_now + FLAT_GAIN * (_unit(acc) - self.up_now))

    def on_land(self, t: float):
        """Landing of this foot (events.py): the foot is flat UP_WINDOW_S after it."""
        self._landed = t

    def tilt_now_deg(self) -> float | None:
        """How far the foot is tilted now from flat (degrees), or None before "up" and the bias are known."""
        if self.up is None or self.up_now is None or not self._bias_ok:
            return None
        return float(np.degrees(np.arccos(min(1.0, max(-1.0, float(self.up @ self.up_now))))))

    def vertical_acc(self, acc: np.ndarray) -> float | None:
        """Acceleration along "up" (tracked) minus gravity as this sensor reads it (m/s², + = up),
        or None before "up" is known."""
        if self.up_now is None:
            return None
        return float(np.asarray(acc, float) @ self.up_now) - self.g_level

    def pitch_rate(self, gyr: np.ndarray) -> float:
        """Foot pitch rate in dps, positive = swing direction."""
        return self.sign * float((gyr - self.gyro_bias) @ self.axis)

    def state(self) -> dict:
        r = lambda v: None if v is None else np.round(v, 4).tolist()
        return {
            "gyro_bias_dps": self.gyro_bias.round(3).tolist(),
            "bias_std_dps": self.bias_std.round(3).tolist(),
            "up": r(self.up),
            "up_source": "given" if self.up_given else ("standing" if self.up is not None else None),
            "standing_acc_ms2": r(self.standing_acc),
            "g_level_ms2": round(self.g_level, 3),
            "rejected_windows": self.rejected_windows,
            "pitch_axis": None if self.axis is None else r(self.axis * self.sign),
            "still_samples": self.still_samples,
            "swing_samples": self.swing_samples,
        }
