"""
Per-foot IMU calibration, learned while walking (Integration Plan §7.3, §15.1).

- Gyro bias and gravity: averaged over moments the foot is still (flat on the ground).
- Pitch axis: the main rotation axis during swings (largest principal component of the
  gyro), so a crooked or mirrored sensor still gives the foot's pitch rate.
- Sign: chosen so the mid-swing peak is positive. Before each FSR strike, the largest
  rotation of the last half second is the mid-swing peak; its sign votes.
"""
import numpy as np

G = 9.81
STILL_GYRO_DPS = 20.0  # raw rotation below this (bias is only a few dps) ...
STILL_ACC_TOL = 0.6  # ... and |acceleration| within this of gravity = foot still
SWING_GYRO_DPS = 120.0  # rotation above this = foot swinging
SIGN_VOTES_NEEDED = 3


class FootCalibration:
    def __init__(self):
        self.gyro_bias = np.zeros(3)
        self.gravity = np.array([0.0, 0.0, G])
        self.still_samples = 0
        self._cov = np.zeros((3, 3))
        self.swing_samples = 0
        self.axis = None  # unit vector in the sensor frame
        self._votes = 0
        self.sign = 0  # +1 / -1 once decided

    @property
    def ready(self) -> bool:
        return self.axis is not None and self.sign != 0

    def add(self, acc: np.ndarray, gyr: np.ndarray):
        if np.linalg.norm(gyr) < STILL_GYRO_DPS and abs(np.linalg.norm(acc) - G) < STILL_ACC_TOL:
            w = 1.0 / min(self.still_samples + 1, 500)
            self.gyro_bias += w * (gyr - self.gyro_bias)
            self.gravity += w * (acc - self.gravity)
            self.still_samples += 1
            return
        g = gyr - self.gyro_bias
        if np.linalg.norm(g) > SWING_GYRO_DPS:
            self._cov += np.outer(g, g)
            self.swing_samples += 1
            if self.swing_samples % 25 == 0:
                self._update_axis()

    def _update_axis(self):
        axis = np.linalg.eigh(self._cov)[1][:, -1]
        if self.axis is not None and axis @ self.axis < 0:
            axis = -axis  # keep the same orientation, so earlier sign votes stay valid
        self.axis = axis

    def raw_pitch_rate(self, gyr: np.ndarray) -> float:
        """Pitch rate along the axis, before the sign is known."""
        return float((gyr - self.gyro_bias) @ self.axis)

    def vote_sign(self, raw_pitch_before_strike: list):
        if self.sign or not raw_pitch_before_strike:
            return
        peak = max(raw_pitch_before_strike, key=abs)
        self._votes += 1 if peak > 0 else -1
        if abs(self._votes) >= SIGN_VOTES_NEEDED:
            self.sign = 1 if self._votes > 0 else -1

    def state(self) -> dict:
        return {
            "gyro_bias_dps": self.gyro_bias.round(3).tolist(),
            "gravity_ms2": self.gravity.round(3).tolist(),
            "pitch_axis": None if self.axis is None else (self.axis * (self.sign or 1)).round(4).tolist(),
            "still_samples": self.still_samples,
            "swing_samples": self.swing_samples,
        }
