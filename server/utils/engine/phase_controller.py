"""
Phase alignment: nudge the tempo so beats land on foot strikes (Integration Plan §17.3).

The mode decides the base tempo (how fast). The controller adds a small correction δ
(when): applied tempo = base × (1 + δ), |δ| ≤ δ_max.

On every update (each main-loop pass, so each time the prediction sharpens):
  1. the note must be sent at  t_send = predicted strike - output latency L - bias
  2. pair the strike with a beat k (the nearest one, then consecutive beats)
  3. phase error e = when beat k comes at base tempo - t_send   (> 0: the beat would be late)
  4. errors up to the dead-band are ignored (soft: the dead-band is subtracted), so the
     person's natural step-to-step jitter is not chased into the tempo
  5. δ moves toward  Kp × e / beat period, limited in size and in rate
Applied over one beat, that shifts the beat by Kp × e: a lasting error shrinks by Kp per
beat (a phase-locked loop). Larger errors are caught up within the beat, and very large
ones (restart after standing) reset the phase: the coming beat alone is stretched or
squeezed, beyond δ_max, to land on the strike.

The integral part is a slow bias: after each strike, the real error (beat heard - strike)
shifts the target, removing constant offsets (wrong L, FSR offset, clock offset).
With low confidence (no prediction, large uncertainty) δ fades back to 0, i.e. pure
tempo following.
"""
import math
import time
from dataclasses import dataclass, asdict


@dataclass
class PhaseSettings:
    latency_s: float = 0.030  # MIDI send -> sound
    # Firm: the beat-to-beat tempo change this causes (~2-3 BPM) is below what listeners notice,
    # and a gentler loop leaves the beat 50-100 ms off for seconds after a pace change
    kp: float = 1.0  # fraction of the phase error corrected per beat
    ki: float = 0.1  # fraction of each strike's error added to the bias
    delta_max: float = 0.15
    delta_rate_per_s: float = 2.0  # how fast δ may change
    deadband_s: float = 0.0  # phase errors are reduced by this
    catchup_s: float = 0.030  # larger errors (pace change, restart) are caught up within the beat
    # Phase reset: from this error on (restart after standing, first steps) the coming beat is
    # stretched or squeezed to land right on the strike, with up to reset_delta_max and no rate limit
    reset_s: float | None = 0.100
    reset_delta_max: float = 0.5
    t_min_s: float = 0.050  # do not re-plan a beat closer than this
    bias_max_s: float = 0.050
    # Only small errors update the bias: large ones are transients (pace change, restart), and
    # learning them made the controller believe it was aligned while it was far off
    bias_window_s: float = 0.060
    max_sigma_s: float = 0.080  # less certain predictions are ignored
    beats_per_step: int = 1
    # Base tempo from exact strike times instead of the receiver's 3-step average, which lags
    # and zig-zags between the feet: a line fitted through the last N steps (even = left/right
    # balanced), so a pace change is followed without the lag of an average
    base_from_strikes: bool = True
    base_steps: int = 8
    base_ahead: float | None = 0.0  # read the fitted line this many steps after the last one (None: plain mean)
    # After a stop, start from the first step (None: wait for base_steps; the receiver average
    # used meanwhile includes the pause)
    base_min_steps: int | None = 1
    base_uses_model: bool = False  # LGBM blend on top (it was trained on the receiver average)
    # The mode's "no step yet: slow down" safety starts only when the next step is this many
    # step periods late (1.0 = today, which fires on every slightly late step and saw-tooths the tempo)
    decay_grace: float = 1.5
    # Standing still: keep the last walking tempo instead of slowing down toward 0, so the
    # music is ready to restart on the first step (the phase reset puts the beat on it)
    hold_when_standing: bool = True
    # Smooth tempo: each beat may be at most this much faster or slower than the beat before
    # (base tempo and correction together), so pace changes and restarts become ramps, not
    # jumps. None: no limit
    max_change_per_beat: float | None = 0.05


class PhaseController:
    def __init__(self, settings: PhaseSettings | None = None):
        self.s = settings or PhaseSettings()
        self.delta = 0.0
        self.bias_s = 0.0
        self.pairs = {}  # strike number -> beat index
        self._last_t = None

    def update(self, now: float, base_bpm: float, beat_now: float, strike) -> float:
        dt = 0.0 if self._last_t is None else now - self._last_t
        self._last_t = now
        target, reset = self._target(now, base_bpm, beat_now, strike)
        if reset or abs(self.delta) > self.s.delta_max:  # a reset beat, and the beat after it
            self.delta = target
            return self.delta
        step = self.s.delta_rate_per_s * dt
        self.delta += max(-step, min(step, target - self.delta))
        return self.delta

    def _target(self, now, base_bpm, beat_now, strike) -> tuple[float, bool]:
        """(target δ, whether to jump to it at once)."""
        if strike is None or strike.sigma_s > self.s.max_sigma_s or base_bpm <= 0:
            return 0.0, False
        t_send = strike.pc_t - self.s.latency_s - self.bias_s
        remaining = t_send - now
        k = self._pair(strike.number, beat_now, remaining, base_bpm * (1.0 + self.delta))
        if k is None or remaining < self.s.t_min_s:
            return self.delta, False  # too late to re-plan this beat: hold
        beat_period = 60.0 / base_bpm
        phase_error = (k - beat_now) * beat_period - remaining  # > 0: at base tempo the beat is late
        if abs(phase_error) > self.s.catchup_s:
            # Far off: the tempo that lands beat k exactly on the strike
            target = 60.0 * (k - beat_now) / remaining / base_bpm - 1.0
            if self.s.reset_s is not None and abs(phase_error) > self.s.reset_s:
                return max(-self.s.reset_delta_max, min(self.s.reset_delta_max, target)), True
        else:
            error = math.copysign(max(abs(phase_error) - self.s.deadband_s, 0.0), phase_error)
            target = self.s.kp * error / beat_period
        return max(-self.s.delta_max, min(self.s.delta_max, target)), False

    def _pair(self, number, beat_now, remaining, bpm):
        """Beat index for strike `number`, or None if its beat has already played."""
        at_send = beat_now + remaining * bpm / 60.0  # beat position at t_send, at the applied tempo
        k = self.pairs.get(number)
        if k is None and number - 1 in self.pairs:
            k = self.pairs[number - 1] + self.s.beats_per_step
        if k is not None and k <= beat_now:
            return None if number in self.pairs else self._lock(number, at_send, beat_now)
        if k is None or abs(k - at_send) > 0.5:  # more than half a beat off: re-lock
            return self._lock(number, at_send, beat_now)
        self.pairs[number] = k
        return k

    def _lock(self, number, at_send, beat_now):
        k = max(round(at_send), math.floor(beat_now) + 1)
        self.pairs[number] = k
        for old in [n for n in self.pairs if n < number - 8]:
            del self.pairs[old]
        return k

    def on_result(self, error_s: float):
        """Real error of a played beat: (beat heard) - (strike). Updates the bias."""
        if abs(error_s) < self.s.deadband_s or abs(error_s) > self.s.bias_window_s:
            return  # large errors are transients (pace change, restart), not a constant offset
        if abs(self.delta) >= self.s.delta_max:
            return  # anti-windup: already correcting as hard as allowed
        self.bias_s = max(-self.s.bias_max_s, min(self.s.bias_max_s, self.bias_s + self.s.ki * error_s))


class PhaseAligner:
    """Connects the controller to the player and the strike tracker, and logs beats."""

    def __init__(self, player, tracker, settings: PhaseSettings | None = None, enabled: bool = True,
                 sensor_logger=None):
        self.player = player
        self.tracker = tracker
        self.ctrl = PhaseController(settings)
        self.enabled = enabled
        self.log = sensor_logger
        self.last_strike = None
        self._pending = []  # (strike number, foot, contact pc time) waiting for their beat to play
        self._logged_beat = -1
        self._base_bpm = 0.0

    @property
    def settings(self) -> PhaseSettings:
        return self.ctrl.s

    def update(self, base_bpm: float) -> float:
        now = time.perf_counter()
        self._base_bpm = base_bpm
        self._log_beats()
        self._resolve()
        strike = self.tracker.next_strike(now)
        self.last_strike = strike
        if not self.enabled:
            return 0.0
        return self.ctrl.update(now, base_bpm, self.player.get_beat_now(), strike)

    def step_bpm(self, receiver_bpm: float) -> tuple[float, bool]:
        """Walking BPM for this step, and whether the LGBM blend should be applied to it.
        Call after tracker.on_step(), so the step itself is included."""
        if self.enabled and self.settings.base_from_strikes:
            s = self.settings
            cadence = self.tracker.predictor.cadence_bpm(s.base_steps, s.base_ahead, s.base_min_steps)
            if cadence:
                return cadence, self.settings.base_uses_model
        return receiver_bpm, True

    def on_strike(self, number: int, foot: int, contact_pc: float | None):
        if contact_pc is not None:
            self._pending.append((number, foot, contact_pc))

    def _resolve(self):
        beat_times = self.player.beat_times
        keep = []
        for number, foot, contact_pc in self._pending:
            k = self.ctrl.pairs.get(number)
            if k is None:
                continue  # no beat was planned for it (alignment off or not confident yet)
            if k not in beat_times:
                keep.append((number, foot, contact_pc))
                continue
            error = beat_times[k] + self.settings.latency_s - contact_pc
            self.ctrl.on_result(error)
            if self.log:
                self.log.row("phase_errors", [number, foot, f"{contact_pc:.6f}", k, f"{beat_times[k]:.6f}",
                                              f"{error * 1000:.1f}", f"{self.ctrl.bias_s * 1000:.1f}"])
        self._pending = keep

    def _log_beats(self):
        if not self.log:
            return
        beat_times = self.player.beat_times
        while self._logged_beat + 1 in beat_times:
            self._logged_beat += 1
            self.log.row("beats", [self._logged_beat, f"{beat_times[self._logged_beat]:.6f}",
                                   f"{self.player.walkingBPM:.3f}", f"{self._base_bpm:.3f}",
                                   f"{self.ctrl.delta:.4f}", int(self.enabled)])

    def state(self) -> dict:
        return {"enabled": self.enabled, "settings": asdict(self.settings), "bias_ms": round(self.ctrl.bias_s * 1000, 1)}
