"""
Per-foot IMU calibration, following Sabatini et al. 2005:
  A. M. Sabatini, C. Martelloni, S. Scapellato, F. Cavallo, "Assessment of Walking Features From
  Foot Inertial Sensing", IEEE Trans. Biomed. Eng. 52(3):486-494, 2005, DOI 10.1109/TBME.2004.840727

- Axis and sign come from how the IMU is mounted (§II-A): the gyro axis we use is the one
  perpendicular to the sagittal plane (mediolateral), so it reads the foot's pitch rate directly.
  Set per foot in MOUNTING. Positive = the swing direction (their Fig. 2: the swing is a broad
  positive pulse, toe-off and heel strike are negative peaks).
- Gyro bias ("nulling", §II-C.2): mean of the gyro over the first 0.5 s, while the user stands
  still before walking (Sabatini: N = 100 samples at 200 Hz). The session must start standing still.
  The spread of those samples is logged (bias_std_dps) so it can be checked offline that the
  user really stood still.

The mounting itself is checked offline after each session with gaitmap's automatic sensor
alignment (Kuderle et al., "Gaitmap - An Open Ecosystem for IMU-Based Human Gait Analysis and
Algorithm Benchmarking", IEEE Open J. Eng. Med. Biol., 2024). See
Documentation/IMU_sources_what_we_use.md.
"""
import numpy as np

G = 9.81
AXES = {"x": 0, "y": 1, "z": 2}

# Foot id (1 = right, 2 = left) -> (board axis across the foot, as printed on the BNO055 board,
# sign that makes the swing positive). Mounted the same way on both shoes, the left IMU is turned
# 180 degrees relative to the right one, so its sign is the opposite.
# These defaults match the simulator (utils/simulation/legs.py: pitch rate is sensor y, left
# sensor mirrored). TODO: set to the real mounting on the shoes.
MOUNTING = {1: ("y", +1), 2: ("y", -1)}

SAMPLE_RATE_HZ = 100
BIAS_S = 0.5  # Sabatini §II-C.2: N = 100 samples at 200 Hz
BIAS_SAMPLES = round(BIAS_S * SAMPLE_RATE_HZ)


class FootCalibration:
    def __init__(self, axis="y", sign: int = +1):
        # axis: a board axis name ("x", "y", "z") or a unit vector in the sensor frame
        self.axis = np.eye(3)[AXES[axis]] if isinstance(axis, str) else np.asarray(axis, float)
        self.sign = int(sign)
        self.gyro_bias = np.zeros(3)
        self.still_samples = 0  # samples averaged into the bias
        self.swing_samples = 0  # samples after the bias is known (name kept for phase_controller)
        self._sum = np.zeros(3)
        self._sum_sq = np.zeros(3)

    @property
    def ready(self) -> bool:
        return self.still_samples >= BIAS_SAMPLES

    def add(self, acc: np.ndarray, gyr: np.ndarray):
        if self.ready:
            self.swing_samples += 1
            return
        self._sum += gyr
        self._sum_sq += gyr * gyr
        self.still_samples += 1
        if self.ready:
            self.gyro_bias = self._sum / self.still_samples

    def pitch_rate(self, gyr: np.ndarray) -> float:
        """Foot pitch rate in dps, positive = swing direction."""
        return self.sign * float((gyr - self.gyro_bias) @ self.axis)

    def state(self) -> dict:
        n = max(self.still_samples, 1)
        std = np.sqrt(np.maximum(self._sum_sq / n - (self._sum / n) ** 2, 0.0))
        return {
            "gyro_bias_dps": self.gyro_bias.round(3).tolist(),
            "bias_std_dps": std.round(3).tolist(),
            "pitch_axis": (self.axis * self.sign).round(4).tolist(),
            "still_samples": self.still_samples,
            "swing_samples": self.swing_samples,
        }
