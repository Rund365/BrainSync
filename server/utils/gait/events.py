"""
Online gait event detection for one foot.

Gait phase segmentation from the foot's sagittal angular velocity, following
  A. M. Sabatini, C. Martelloni, S. Scapellato, F. Cavallo, "Assessment of Walking Features From
  Foot Inertial Sensing", IEEE Trans. Biomed. Eng. 52(3):486-494, 2005, §II-C.1, Eq. 2-3, Fig. 2.
Positive = swing direction (calibration.py):
  STANCE   --|w| >= 30 dps (Eq. 2)--------------------------->  heel-off (HO), PUSH_OFF
  PUSH_OFF --largest negative peak, ends when w turns positive->  toe-off (TO)
  SWING    --largest positive peak, ends when w turns negative->  mid-swing (MS)
  LANDING  --largest negative peak, ends when |w| < 30 dps (Eq. 3, foot-flat)
                                                       -->  heel strike (HS), back to STANCE
The sign checks on each peak (a TO must be negative, an MS positive, an HS negative) are ours:
the paper describes clean cycles only.
Events are timestamped with the sender time of the extreme, not the time they were
noticed, so detection delay does not shift them. A timeout returns to STANCE (stopping).

Also measures how late the FSR fires after the real contact: the heel-strike impact is
the first sample whose |acceleration| leaves gravity sharply just before the FSR strike.
"""
from collections import deque
from dataclasses import dataclass
from statistics import median

import numpy as np

from .calibration import FootCalibration, G, MOUNTING

FILTER_ALPHA = 0.5  # one-pole low-pass on the pitch rate (~11 Hz at 100 Hz)
HO_DPS = 30.0  # heel-off when |pitch rate| >= 30 dps (Sabatini 2005, Eq. 2, TH_HO)
FF_DPS = 30.0  # foot-flat when |pitch rate| < 30 dps (Sabatini 2005, Eq. 3, TH_FF)
SWING_TIMEOUT_S = 1.0
IMPACT_MS2 = 4.0  # |acceleration| this far from gravity = impact
# ... and this much away from the sample before: an impact is a jump, while the braking just
# before landing (up to ~3 g on real feet) builds up over several samples
IMPACT_JUMP_MS2 = 8.0
IMPACT_SAMPLE_LAG_S = 0.005  # first sample after contact is on average half a sample late

STANCE, PUSH_OFF, SWING, LANDING = "stance", "push_off", "swing", "landing"


@dataclass
class GaitEvent:
    foot: int
    kind: str  # "HO", "TO", "MS" or "HS"
    sender_s: float
    value_dps: float


class FootTracker:
    def __init__(self, foot: int, axis=None, sign: int | None = None):
        """axis / sign override MOUNTING[foot] (used to replay recordings with a known axis)."""
        self.foot = foot
        mount_axis, mount_sign = MOUNTING[foot]
        self.calib = FootCalibration(mount_axis if axis is None else axis,
                                     mount_sign if sign is None else sign)
        self.state = STANCE
        self._state_since = 0.0
        self._filtered = None
        self._extreme = None  # (value, time) of the peak being tracked
        self._last_strike = None
        self._acc_dev = deque(maxlen=150)  # (t, |a| - g), for the impact search
        self._pending_strikes = []
        self._fsr_delays = deque(maxlen=20)
        self.last_t = None

    @property
    def fsr_delay_s(self):
        """How long the FSR fires after the real contact (median), or None if unknown yet."""
        return median(self._fsr_delays) if len(self._fsr_delays) >= 3 else None

    def add_sample(self, t: float, acc: np.ndarray, gyr: np.ndarray) -> list:
        self.last_t = t
        self._acc_dev.append((t, float(np.linalg.norm(acc)) - G))
        self._check_impacts(t)

        self.calib.add(acc, gyr)
        if not self.calib.ready:
            return []
        w = self.calib.pitch_rate(gyr)
        self._filtered = w if self._filtered is None else self._filtered + FILTER_ALPHA * (w - self._filtered)
        return self._detect(t, self._filtered)

    def on_strike(self, t: float):
        """FSR strike of this foot (sender time). The IMU finds its own heel strike."""
        self._pending_strikes.append(t)
        self._last_strike = t

    def _enter(self, state: str, t: float):
        self.state = state
        self._state_since = t
        self._extreme = None

    def _detect(self, t: float, w: float) -> list:
        if self.state != STANCE and t - self._state_since > SWING_TIMEOUT_S:
            self._enter(STANCE, t)

        events = []
        if self.state == STANCE:  # foot flat
            if abs(w) >= HO_DPS:  # Eq. 2: heel-off
                self._enter(PUSH_OFF, t)
                events.append(GaitEvent(self.foot, "HO", t, w))

        if self.state == PUSH_OFF:  # toe-off = largest clockwise (negative) peak
            if self._extreme is None or w < self._extreme[0]:
                self._extreme = (w, t)
            elif w > 0:  # the swing pulse has started
                value, t_to = self._extreme
                self._enter(SWING, t_to)
                if value < 0:  # no negative peak = the push-off was missed: no TO
                    return events + [GaitEvent(self.foot, "TO", t_to, value)]

        elif self.state == SWING:  # mid-swing = top of the positive pulse
            if self._extreme is None or w > self._extreme[0]:
                self._extreme = (w, t)
            elif w < 0:  # the pulse is over
                value, t_ms = self._extreme
                self._enter(LANDING, t_ms)
                if value > 0:
                    return [GaitEvent(self.foot, "MS", t_ms, value)]

        elif self.state == LANDING:  # heel strike = second clockwise peak of the cycle
            if self._extreme is None or w < self._extreme[0]:
                self._extreme = (w, t)
            elif abs(w) < FF_DPS:  # Eq. 3: foot-flat
                value, t_hs = self._extreme
                self._enter(STANCE, t)
                if value < 0:
                    return [GaitEvent(self.foot, "HS", t_hs, value)]
        return events

    def _check_impacts(self, t: float):
        # The IMU data of a strike arrives a little after the FSR message; wait for it
        while self._pending_strikes and t >= self._pending_strikes[0] + 0.02:
            fsr_t = self._pending_strikes.pop(0)
            prev = None
            for ts, dev in self._acc_dev:
                if (prev is not None and fsr_t - 0.08 <= ts <= fsr_t + 0.01 and abs(dev) > IMPACT_MS2
                        and abs(dev - prev) > IMPACT_JUMP_MS2):
                    self._fsr_delays.append(fsr_t - (ts - IMPACT_SAMPLE_LAG_S))
                    break
                prev = dev
