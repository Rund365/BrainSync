"""
Predicts when the next foot strike will happen (Integration Plan §16).

Three estimates for the next strike, all in sender time:
  B0 rhythm     last strike + usual step time onto this foot      (no IMU, ~1 step ahead)
  B1 toe-off    toe-off of the swinging foot + its usual TO->strike delay   (~400 ms ahead)
  B2 mid-swing  mid-swing of the swinging foot + its usual MS->strike delay (~200 ms ahead),
                scaled by how fast this swing is (TO -> MS against the foot's usual)
Fused by inverse variance: each estimate weighs 1 / (its recent error)², so the newest,
most precise one dominates as the strike approaches. Estimates far from the most precise
one are left out.

Everything is learned per person and per foot from the FSR strikes: on each strike the
delays (strike - TO, strike - MS) and the step time are added to rolling medians, and
each method's error is recorded to update its weight.
"""
from collections import deque
from dataclasses import dataclass, field
from statistics import median

RIGHT, LEFT = 1, 2
HISTORY = 8  # strides kept for the rolling medians
MIN_HISTORY = 3
PRIOR_SIGMA_S = {"B0": 0.040, "B1": 0.025, "B2": 0.012}
MIN_SIGMA_S = 0.004
STEP_RANGE_S = (0.25, 1.5)  # step times outside this are not walking (stopped, started)
# Swing speed: a faster swing (shorter TO -> MS than this foot's usual) also ends sooner, so
# B2's MS -> strike delay is scaled by (TO -> MS now / usual) ** SCALE_POWER. Below 1 because
# the swing timing jitters, and a full scale passes that jitter on to steady walking
SCALE_B2 = True
SCALE_RANGE = (0.7, 1.3)
SCALE_POWER = 0.75
# Fusion leaves out an estimate that is more than this many of its own sigmas away from the
# most precise one (during a pace change B0 / B1 lag, and averaging them in pulls the guess late)
FUSE_GATE = 3.0

@dataclass
class StrikePrediction:
    foot: int
    sender_s: float  # predicted FSR strike time
    sigma_s: float
    method: str  # which estimate weighs most
    estimates: dict = field(default_factory=dict)  # method -> (sender_s, sigma_s)
    strike_number: int = 0  # index of the strike this prediction is for


def other(foot: int) -> int:
    return LEFT if foot == RIGHT else RIGHT


class StrikePredictor:
    def __init__(self):
        self.step_times = {f: deque(maxlen=HISTORY) for f in (RIGHT, LEFT)}  # step ending on foot f
        self.to_delay = {f: deque(maxlen=HISTORY) for f in (RIGHT, LEFT)}
        self.ms_delay = {f: deque(maxlen=HISTORY) for f in (RIGHT, LEFT)}
        self.to_ms = {f: deque(maxlen=HISTORY) for f in (RIGHT, LEFT)}  # TO -> MS of each swing
        self.errors = {m: deque(maxlen=HISTORY) for m in PRIOR_SIGMA_S}
        self.last_strike = None  # (foot, sender_s)
        self.strike_count = 0
        self.recent_steps = deque(maxlen=4 * HISTORY)  # step times of both feet, in order
        self._swing = {f: {} for f in (RIGHT, LEFT)}  # foot -> {"TO": t, "MS": t} of its current swing
        self._made = {}  # method -> latest estimate for the coming strike, for scoring

    # ----------- Inputs -----------
    def on_event(self, foot: int, kind: str, sender_s: float):
        if kind == "TO":
            self._swing[foot] = {"TO": sender_s}
        elif kind == "MS":
            self._swing[foot]["MS"] = sender_s

    def on_strike(self, foot: int, sender_s: float) -> dict:
        """Learn from a real strike. Returns each method's error (actual - predicted) for it."""
        errors = {}
        if self.last_strike and self._made.get("foot") == foot:
            for method in PRIOR_SIGMA_S:
                if method in self._made:
                    err = sender_s - self._made[method]
                    if abs(err) < 0.3:  # a wild miss is a stop/start, not the method's precision
                        self.errors[method].append(err)
                    errors[method] = err

        swing = self._swing[foot]
        if "TO" in swing and 0.15 < sender_s - swing["TO"] < 0.8:
            self.to_delay[foot].append(sender_s - swing["TO"])
        if "MS" in swing and 0.03 < sender_s - swing["MS"] < 0.5:
            self.ms_delay[foot].append(sender_s - swing["MS"])
            if "TO" in swing and 0.03 < swing["MS"] - swing["TO"] < 0.5:
                self.to_ms[foot].append(swing["MS"] - swing["TO"])
        if self.last_strike and self.last_strike[0] != foot:
            step = sender_s - self.last_strike[1]
            if STEP_RANGE_S[0] < step < STEP_RANGE_S[1]:
                self.step_times[foot].append(step)
                self.recent_steps.append(step)
            else:
                self.recent_steps.clear()  # stopped: the old pace no longer applies

        self.last_strike = (foot, sender_s)
        self.strike_count += 1
        self._swing[foot] = {}
        self._made = {}
        return errors

    # ----------- Output -----------
    def cadence_bpm(self, steps: int = 4, ahead: float | None = None, min_steps: int | None = None) -> float | None:
        """Steps per minute over the last `steps` steps, from exact strike times.
        An even number has as many left as right steps, so a left/right difference cancels.
        With `ahead`, a straight line is fitted through the step times and read `ahead` steps
        after the last one, so a speed change is followed without the lag of an average.
        After a stop, `min_steps` or more steps are enough (default: all `steps`)."""
        available = len(self.recent_steps)
        if available < (steps if min_steps is None else min_steps):
            return None
        steps = min(steps, available)
        if steps >= 2:
            steps = steps // 2 * 2
        if ahead is None or steps < 4:
            recent = list(self.recent_steps)[-steps:]
            return 60.0 * len(recent) / sum(recent)
        return 60.0 / self._trend_step(steps, ahead)

    def _trend_step(self, steps: int, ahead: float) -> float:
        """Step time (s) from a straight line through the last `steps` step times, read `ahead`
        steps after the last one. Fitted on left + right pairs, so the foot difference cancels."""
        recent = list(self.recent_steps)[-steps:]
        pairs = [recent[i - 1] + recent[i] for i in range(1, len(recent))]  # pair i is centred on step i - 0.5
        xs = [i - 0.5 for i in range(1, len(recent))]
        x_mean, p_mean = sum(xs) / len(xs), sum(pairs) / len(pairs)
        slope = (sum((x - x_mean) * (p - p_mean) for x, p in zip(xs, pairs))
                 / sum((x - x_mean) ** 2 for x in xs))
        step = (p_mean + slope * (len(recent) - 1 + ahead - x_mean)) / 2.0
        return max(min(recent) * 0.85, min(max(recent) * 1.15, step))

    @staticmethod
    def _scale(now_s: float, usual) -> float:
        """This swing's duration relative to the foot's usual one (1.0 without enough history)."""
        if len(usual) < MIN_HISTORY or now_s <= 0:
            return 1.0
        return max(SCALE_RANGE[0], min(SCALE_RANGE[1], now_s / median(usual))) ** SCALE_POWER

    def sigma(self, method: str) -> float:
        errs = self.errors[method]
        if len(errs) < MIN_HISTORY:
            return PRIOR_SIGMA_S[method]
        return max((sum(e * e for e in errs) / len(errs)) ** 0.5, MIN_SIGMA_S)

    def predict(self, now_sender_s: float) -> StrikePrediction | None:
        """Best guess for the next strike, or None if there is nothing to go on (standing)."""
        if not self.last_strike:
            return None
        last_foot, last_t = self.last_strike
        foot = other(last_foot)
        swing = self._swing[foot]
        estimates = {}

        if len(self.step_times[foot]) >= MIN_HISTORY:
            estimates["B0"] = last_t + median(self.step_times[foot])
        if "TO" in swing and len(self.to_delay[foot]) >= MIN_HISTORY:
            estimates["B1"] = swing["TO"] + median(self.to_delay[foot])
        if "MS" in swing and len(self.ms_delay[foot]) >= MIN_HISTORY:
            scale = 1.0
            if SCALE_B2 and "TO" in swing:
                scale = self._scale(swing["MS"] - swing["TO"], self.to_ms[foot])
            estimates["B2"] = swing["MS"] + median(self.ms_delay[foot]) * scale

        # A rhythm guess that is long past (no swing seen) means the person stopped
        if "B0" in estimates and not swing and now_sender_s > estimates["B0"] + 0.15:
            del estimates["B0"]
        if not estimates:
            return None

        self._made.update(estimates)
        self._made["foot"] = foot
        weights = {m: 1.0 / self.sigma(m) ** 2 for m in estimates}
        if FUSE_GATE:
            best = max(weights, key=lambda m: (weights[m], m))  # a tie goes to the later event
            weights = {m: w for m, w in weights.items()
                       if m == best or abs(estimates[m] - estimates[best]) < FUSE_GATE * self.sigma(m)}
        total = sum(weights.values())
        t = sum(estimates[m] * w for m, w in weights.items()) / total
        return StrikePrediction(
            foot=foot, sender_s=t, sigma_s=total ** -0.5,
            method=max(weights, key=weights.get),
            estimates={m: (estimates[m], self.sigma(m)) for m in estimates},
            strike_number=self.strike_count,
        )

    def state(self) -> dict:
        def med(d):
            return round(median(d) * 1000, 1) if d else None
        return {
            "step_ms": {"right": med(self.step_times[RIGHT]), "left": med(self.step_times[LEFT])},
            "to_delay_ms": {"right": med(self.to_delay[RIGHT]), "left": med(self.to_delay[LEFT])},
            "ms_delay_ms": {"right": med(self.ms_delay[RIGHT]), "left": med(self.ms_delay[LEFT])},
            "sigma_ms": {m: round(self.sigma(m) * 1000, 1) for m in PRIOR_SIGMA_S},
        }
