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
Each peak must have the expected sign (TO negative, MS positive, HS negative).
Events are timestamped with the sender time of the extreme, not the time they were
noticed, so detection delay does not shift them. A timeout returns to STANCE (stopping).

IMU only (the FSR is not an input):
  LAND  the landing impact: the acceleration vector jumps sharply from one sample to the next
        (the impact rings, so its size alone can read close to rest on the first sample), and the
        foot then settles: a landing is followed by the foot flat on the ground (Sabatini's foot-flat),
        while the push of a toe-off, which can jump as much when walking, is followed by the swing.
        So a jump counts once the rotation has stayed low over SETTLE_S after it; the event keeps
        the time of the jump and is reported SETTLE_S[1] later. Sooner when the foot is known to
        have lifted since its last landing (RISE) and is nearly flat at the jump (the tilt carried
        by calibration.py): a toe-off is steep (simulated walking: 50-70°), a landing is not
        (recordings: at most 30°): reported IMPACT_MOVE_S later.
  RISE  after the foot has been flat for a moment, its vertical speed (vertical acceleration from
        calibration.py, integrated from rest) first goes up: the foot leaves the ground.
  FALL  the upward speed has fallen back to FALL_FRAC of its peak: the rise is ending. It comes
        30-40 ms before TOP and is almost as steady a time before the landing (recordings: ±15-20 ms
        against ±13-18 ms for TOP), so it is often known in time when TOP is not yet.
  TOP   the vertical speed turns from up to down: the foot's highest point.
These work with or without a forward swing. All are reported live.

For analysis only, on_strike() measures how late the FSR fires after the IMU landing.
"""
from collections import deque
from dataclasses import dataclass
from statistics import median

import numpy as np

from .calibration import FootCalibration, G

FILTER_ALPHA = 0.5  # one-pole low-pass on the pitch rate (~11 Hz at 100 Hz)
HO_DPS = 30.0  # heel-off when |pitch rate| >= 30 dps (Sabatini 2005, Eq. 2, TH_HO)
FF_DPS = 30.0  # foot-flat when |pitch rate| < 30 dps (Sabatini 2005, Eq. 3, TH_FF)
SWING_TIMEOUT_S = 1.0
# Landing: the acceleration vector changes this much from one sample to the next. An impact is a
# jump, while the braking just before landing (up to ~3 g on real feet) builds up over several samples
IMPACT_JUMP_MS2 = 9.0
IMPACT_SAMPLE_LAG_S = 0.005  # first sample after contact is on average half a sample late
# After a landing the foot settles (recordings: median rotation 0.1-0.2 s after the impact at most
# 67 °/s on the right foot, 123 °/s on the left, which rocks on its mount); after a toe-off it swings
# (simulated walking: at least 180 °/s; real swings peak at 300-450 °/s)
SETTLE_S = (0.10, 0.20)
SETTLE_DPS = 150.0
IMPACT_MOVE_S = 0.03  # a jump twice as big this soon after is the impact (the first was a jolt before it)
LAND_TILT_DEG = 40.0  # flatter than this at the jump, after a lift: a landing without waiting to settle
LAND_GAP_S = 0.4  # one landing per foot within this
FLAT_S = 0.30  # after a landing the foot is flat and still until at least this (calibration UP_WINDOW_S)
TOP_MIN_VZ = 0.05  # the foot must rise at least this fast (m/s) to count as leaving the ground
FALL_FRAC = 0.5  # FALL: the upward speed is back down to this share of its peak
TOP_MAX_S = 1.0  # no TOP this long after the flat moment: a stop, not a step
# Mid-swing is reported once the pulse has fallen to this share
# of its peak, so the predictor gets it during the swing. Waiting for the pulse to end (w < 0) reported
# it only at the landing. The timestamp is still the peak.
MS_REPORT_FRAC = 0.8
SAMPLE_S = 0.01  # 100 Hz

STANCE, PUSH_OFF, SWING, LANDING = "stance", "push_off", "swing", "landing"


@dataclass
class GaitEvent:
    foot: int
    kind: str  # "HO", "TO", "MS", "HS" (Sabatini), "LAND", "RISE", "FALL" or "TOP"
    sender_s: float
    value_dps: float  # °/s for Sabatini's events, m/s² jump for LAND, m/s for RISE / TOP


class FootTracker:
    def __init__(self, foot: int, axis=None, sign: int | None = None, up=None):
        """axis / sign / up: from the IMU calibration file (calibration.py)."""
        self.foot = foot
        self.calib = FootCalibration(axis, sign, up)
        self.state = STANCE
        self._state_since = 0.0
        self._filtered = None
        self._extreme = None  # (value, time) of the peak being tracked
        self._ms_sent = False  # mid-swing already reported for this swing
        self._pending_strikes = []
        self._fsr_delays = deque(maxlen=20)
        self.last_t = None
        self._prev_acc = None
        self.last_land = None
        self._candidate = None  # (time, jump) of an impact waiting to see the foot settle
        self._settle = []  # rotation (°/s) seen in its SETTLE_S window
        self._held = []  # (t, vertical acceleration) held back from the climb meanwhile
        self._quick = False  # the candidate can count without waiting for the foot to settle
        self._climb_from = None  # time from which the vertical speed is followed (foot flat)
        self._falling = False
        self._vz = self._vz_max = 0.0
        self._rising = False

    @property
    def fsr_delay_s(self):
        """How long the FSR fires after the IMU landing (median), or None if unknown yet."""
        return median(self._fsr_delays) if len(self._fsr_delays) >= 3 else None

    def add_sample(self, t: float, acc: np.ndarray, gyr: np.ndarray) -> list:
        self.last_t = t
        self._check_impacts(t)
        events = self._land(t, np.asarray(acc, float), np.asarray(gyr, float))
        av = self.calib.vertical_acc(acc)
        if av is not None:
            if self._candidate is not None:  # maybe on the ground: wait (the impact is not a climb)
                self._held.append((t, av))
            else:
                events += self._climb(t, av)
        self.calib.add(acc, gyr, t)
        if not self.calib.ready or self.calib.axis is None:  # Sabatini's events need the bias and the axis
            return events
        w = self.calib.pitch_rate(gyr)
        self._filtered = w if self._filtered is None else self._filtered + FILTER_ALPHA * (w - self._filtered)
        return events + self._detect(t, self._filtered)

    def on_strike(self, t: float):
        """FSR strike of this foot (sender time). Analysis only: measures how late the FSR fires."""
        self._pending_strikes.append(t)

    def _land(self, t: float, acc: np.ndarray, gyr: np.ndarray) -> list:
        prev, self._prev_acc = self._prev_acc, acc
        events = []
        if self._candidate is not None:
            t_land, jump = self._candidate
            if t - t_land >= SETTLE_S[0]:
                self._settle.append(float(np.linalg.norm(gyr - self.calib.gyro_bias)))
            if self._quick and t - t_land >= IMPACT_MOVE_S:  # lifted, and flat at the jump: a landing
                events += self._accept(t_land, jump)
            elif t - t_land >= SETTLE_S[1]:
                if self._settle and median(self._settle) < SETTLE_DPS:  # the foot settled: a landing
                    events += self._accept(t_land, jump)
                else:  # not a landing: the climb goes on as if it had not waited
                    self._candidate = None
                    held, self._held = self._held, []
                    for th, av in held:
                        events += self._climb(th, av)
        if prev is None:
            return events
        jump = float(np.linalg.norm(acc - prev))
        if self._candidate is not None:
            t_land, first = self._candidate
            if t - t_land <= IMPACT_MOVE_S and jump > 2 * first:  # that was a jolt before the contact
                self._candidate = (t - IMPACT_SAMPLE_LAG_S, jump)
            return events
        if jump <= IMPACT_JUMP_MS2:
            return events
        if self.last_land is not None and t - self.last_land < LAND_GAP_S:
            return events
        self._candidate, self._settle = (t - IMPACT_SAMPLE_LAG_S, jump), []
        tilt = self.calib.tilt_now_deg()
        self._quick = self._rising and tilt is not None and tilt < LAND_TILT_DEG
        return events

    def _accept(self, t_land: float, jump: float) -> list:
        self._candidate, self._held = None, []
        self.last_land = t_land
        self.calib.on_land(t_land)
        self._climb_from, self._vz, self._vz_max, self._rising = t_land + FLAT_S, 0.0, 0.0, False
        self._falling = False
        return [GaitEvent(self.foot, "LAND", t_land, jump)]

    def _climb(self, t: float, av: float) -> list:
        """Vertical speed from the flat moment after a landing (speed 0 there): RISE, FALL, then TOP."""
        if self._climb_from is None or t < self._climb_from:
            return []
        if t - self._climb_from > TOP_MAX_S:  # no step came: a stop
            self._climb_from = None
            return []
        self._vz += av * SAMPLE_S
        self._vz_max = max(self._vz_max, self._vz)
        if not self._rising and self._vz >= TOP_MIN_VZ:
            self._rising = True
            return [GaitEvent(self.foot, "RISE", t, self._vz)]
        if self._rising and not self._falling and self._vz <= FALL_FRAC * self._vz_max:
            self._falling = True
            return [GaitEvent(self.foot, "FALL", t, self._vz)]
        if self._rising and self._vz <= 0.0:  # rising has ended: the highest point
            self._climb_from = None
            return [GaitEvent(self.foot, "TOP", t, self._vz_max)]
        return []

    def _enter(self, state: str, t: float):
        self.state = state
        self._state_since = t
        self._extreme = None
        self._ms_sent = False

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
            if not self._ms_sent and (self._extreme is None or w > self._extreme[0]):
                self._extreme = (w, t)
            elif w < 0:  # the pulse is over
                value, t_ms = self._extreme
                sent = self._ms_sent
                self._enter(LANDING, t_ms)
                if value > 0 and not sent:
                    return [GaitEvent(self.foot, "MS", t_ms, value)]
            elif not self._ms_sent and self._extreme[0] > 0 and w < MS_REPORT_FRAC * self._extreme[0]:
                self._ms_sent = True  # past the peak: report now, keep the state until the pulse ends
                return [GaitEvent(self.foot, "MS", self._extreme[1], self._extreme[0])]

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
        # The IMU data of a strike arrives a little after the FSR message, and its landing is
        # confirmed SETTLE_S[1] later; wait for it
        while self._pending_strikes and t >= self._pending_strikes[0] + 0.02 + SETTLE_S[1]:
            fsr_t = self._pending_strikes.pop(0)
            if self.last_land is not None and 0 <= fsr_t - self.last_land <= 0.08:
                self._fsr_delays.append(fsr_t - self.last_land)
