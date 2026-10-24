"""
Online gait event detection for one foot (Integration Plan §15.2, §15.4).

Works on the calibrated pitch rate (positive = mid-swing direction), lightly filtered:
  STANCE   --(push-off dip, then rising again)-->  toe-off (TO) at the bottom of the dip
  SWING    --(big peak, then falling again)----->  mid-swing (MS) at the top of the peak
  TERMINAL --(FSR strike of this foot)---------->  STANCE
Events are timestamped with the sender time of the extreme, not the time they were
noticed, so detection delay does not shift them. Timeouts return to STANCE if a strike
never comes (stopping, shuffling).

Also measures how late the FSR fires after the real contact: the heel-strike impact is
the first sample whose |acceleration| leaves gravity sharply just before the FSR strike.
"""
from collections import deque
from dataclasses import dataclass
from statistics import median

import numpy as np

from .calibration import FootCalibration, G

FILTER_ALPHA = 0.5  # one-pole low-pass on the pitch rate (~11 Hz at 100 Hz)
TO_DPS = 60.0  # push-off dip must go below -TO_DPS
MS_DPS = 150.0  # mid-swing peak must go above MS_DPS
SLAP_GUARD_S = 0.15  # ignore the foot slapping flat right after a strike
SWING_TIMEOUT_S = 1.0
IMPACT_MS2 = 4.0  # |acceleration| this far from gravity = impact
IMPACT_SAMPLE_LAG_S = 0.005  # first sample after contact is on average half a sample late

STANCE, SWING, TERMINAL = "stance", "swing", "terminal"


@dataclass
class GaitEvent:
    foot: int
    kind: str  # "TO" or "MS"
    sender_s: float
    value_dps: float


class FootTracker:
    def __init__(self, foot: int):
        self.foot = foot
        self.calib = FootCalibration()
        self.state = STANCE
        self._state_since = 0.0
        self._filtered = None
        self._extreme = None  # (value, time) of the dip / peak being tracked
        self._last_strike = None
        self._raw_pitch = deque(maxlen=80)  # (t, raw pitch rate), for the sign votes
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
        if self.calib.axis is None:
            return []
        raw = self.calib.raw_pitch_rate(gyr)
        self._raw_pitch.append((t, raw))
        if not self.calib.sign:
            return []

        w = raw * self.calib.sign
        self._filtered = w if self._filtered is None else self._filtered + FILTER_ALPHA * (w - self._filtered)
        return self._detect(t, self._filtered)

    def on_strike(self, t: float):
        """FSR strike of this foot (sender time)."""
        self.calib.vote_sign([v for (ts, v) in self._raw_pitch if t - 0.45 <= ts <= t])
        self._pending_strikes.append(t)
        self._last_strike = t
        self._enter(STANCE, t)

    def _enter(self, state: str, t: float):
        self.state = state
        self._state_since = t
        self._extreme = None

    def _detect(self, t: float, w: float) -> list:
        if self.state != STANCE and t - self._state_since > SWING_TIMEOUT_S:
            self._enter(STANCE, t)

        if self.state == STANCE:
            if self._last_strike is not None and t - self._last_strike < SLAP_GUARD_S:
                return []
            if w < -TO_DPS:
                if self._extreme is None or w < self._extreme[0]:
                    self._extreme = (w, t)
            elif self._extreme is not None and w > 0.5 * self._extreme[0]:
                value, t_to = self._extreme
                self._enter(SWING, t_to)
                return [GaitEvent(self.foot, "TO", t_to, value)]
            elif w > MS_DPS:  # missed the push-off; the swing is already on
                self._enter(SWING, t)

        if self.state == SWING:
            if w > MS_DPS and (self._extreme is None or w > self._extreme[0]):
                self._extreme = (w, t)
            elif self._extreme is not None and w < 0.8 * self._extreme[0]:
                value, t_ms = self._extreme
                self._enter(TERMINAL, t_ms)
                return [GaitEvent(self.foot, "MS", t_ms, value)]
        return []

    def _check_impacts(self, t: float):
        # The IMU data of a strike arrives a little after the FSR message; wait for it
        while self._pending_strikes and t >= self._pending_strikes[0] + 0.02:
            fsr_t = self._pending_strikes.pop(0)
            for ts, dev in self._acc_dev:
                if fsr_t - 0.08 <= ts <= fsr_t + 0.01 and abs(dev) > IMPACT_MS2:
                    self._fsr_delays.append(fsr_t - (ts - IMPACT_SAMPLE_LAG_S))
                    break
