"""
Fast closed-loop simulation (Integration Plan §17.4): the real tempo engine, strike
predictor and phase controller on a fake clock, with a beat-clock stand-in for the MIDI
player. A whole walk takes seconds, so settings can be tuned before a real-time run.

Usage (from the repo root):
  .venv\\Scripts\\python.exe server\\utils\\simulation\\fast_loop.py mixed --seed 1
"""
import argparse
import sys
import types
from dataclasses import dataclass
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # server/, so "utils." imports work

import utils.engine.BPM_estimation as bpm_estimation_module
import utils.engine.phase_controller as phase_module
from utils.engine.strike_tracking import StrikeTracker
from utils.hardware.comms import handle_step
from utils.hardware.sensor_stream import SensorStream, ImuMsg, StepMsg
from utils.simulation.generate import ManualClock
from utils.simulation.simulated_receiver import SimulatedReceiver
from utils.simulation.walker import Walker, parse_scenario

TICK_S = 0.001


class BeatClock:
    """Beat-clock part of BeatClockPlayer, on the fake clock, without MIDI."""

    def __init__(self, clock, bpm: float):
        self.clock = clock
        self.walkingBPM = self.songBPM = bpm
        self.TempoFactor = 1.0
        self._anchor_beat, self._anchor_time = 0.0, clock()
        self.beat_times = {0: clock()}
        self._last = 0

    def _beat_at(self, t):
        return self._anchor_beat + (t - self._anchor_time) * self.walkingBPM / 60.0

    def set_BPM(self, bpm):
        t = self.clock()
        self._anchor_beat, self._anchor_time = self._beat_at(t), t
        self.walkingBPM = bpm

    def get_beat_now(self):
        return self._beat_at(self.clock())

    def tick(self):
        now = self.clock()
        beat = self._beat_at(now)
        while self._last + 1 <= beat:
            self._last += 1
            self.beat_times[self._last] = now - (beat - self._last) * 60.0 / self.walkingBPM


class _SilentLogger:
    def log(self, *args, **kwargs):
        pass

    def log_data(self, *args, **kwargs):
        pass


@dataclass
class LoopResult:
    strike_t: np.ndarray  # true contacts, seconds since the walk started
    feet: np.ndarray
    asynchrony_ms: np.ndarray  # heard beat - true contact, nearest beat
    beat_t: np.ndarray  # heard beats, seconds since the walk started
    tempo_t: np.ndarray
    tempo_bpm: np.ndarray
    base_bpm: np.ndarray  # the mode's base tempo, before the phase nudge
    cadence_bpm: np.ndarray  # the walker's cadence at tempo_t


def run(scenario: str, seed: int | None, enabled: bool, start_bpm: float = 110.0,
        settings: phase_module.PhaseSettings | None = None,
        alpha_up: float | None = None, alpha_down: float | None = None,
        realistic: bool = True, window: int = 3, use_model: bool = True) -> LoopResult:
    clock = ManualClock()
    fake_time = types.SimpleNamespace(perf_counter=clock, time=clock)
    saved = phase_module.time, bpm_estimation_module.time
    phase_module.time = bpm_estimation_module.time = fake_time
    try:
        walker = Walker(parse_scenario(scenario), realistic=realistic, seed=seed)
        rx = SimulatedReceiver(walker, seed=seed, time_source=clock)
        rx.timeout = 0
        rx.write(f"RESET\nSET_WINDOW,{window}\nSTART\n".encode())
        rx.reset_input_buffer()
        player = BeatClock(clock, start_bpm)
        model = None
        if use_model:  # the LGBM BPM model, as main.py uses by default
            from utils.prediction_model.LGBM_predictor import LGBMPredictor
            model = LGBMPredictor(model_path=None, run_type="dynamic")
        engine = bpm_estimation_module.BPM_estimation(player, _SilentLogger(), smoothing_window=window,
                                                      prediction_model=model)
        if model:
            engine._warmup_done.wait(timeout=30)
        if alpha_up is not None:
            engine.set_smoothing_alpha_up(alpha_up)
        if alpha_down is not None:
            engine.set_smoothing_alpha_down(alpha_down)
        stream = SensorStream()
        tracker = StrikeTracker(stream)
        aligner = phase_module.PhaseAligner(player, tracker, settings, enabled=enabled)
        engine.phase_aligner = aligner

        tempo = []
        while not rx.finished:
            clock.t += TICK_S
            while raw := rx.readline():
                msg = stream.feed(raw.decode().strip(), clock.t)
                if isinstance(msg, ImuMsg):
                    tracker.on_imu(msg)
                elif isinstance(msg, StepMsg):
                    bpm, instant_bpm, _, _ = handle_step(raw, player.walkingBPM)
                    number, contact_pc = tracker.on_step(msg)
                    aligner.on_strike(number, msg.foot, contact_pc)
                    bpm, use_model = aligner.step_bpm(bpm)
                    engine.register_step(bpm, instant_bpm, use_model=use_model)
            engine.update_bpm()
            player.tick()
            if int(round(clock.t / TICK_S)) % 20 == 0:
                tempo.append((clock.t, player.walkingBPM, engine.base_bpm))
    finally:
        phase_module.time, bpm_estimation_module.time = saved

    start = rx._boot + rx._walk_start
    beats = np.array(sorted(player.beat_times.values())) + aligner.settings.latency_s - start
    strikes = np.array([s.hs for s in rx.legs.strikes])
    nearest = beats[np.abs(beats[None, :] - strikes[:, None]).argmin(axis=1)]
    tempo = np.array(tempo)
    tempo_t = tempo[:, 0] - start
    return LoopResult(
        strike_t=strikes, feet=np.array([s.foot for s in rx.legs.strikes]),
        asynchrony_ms=(nearest - strikes) * 1000, beat_t=beats,
        tempo_t=tempo_t, tempo_bpm=tempo[:, 1], base_bpm=tempo[:, 2],
        cadence_bpm=np.array([walker.cadence_at(t) for t in tempo_t]),
    )


def summary(result: LoopResult) -> str:
    a = np.abs(result.asynchrony_ms)
    return (f"median |beat - strike| {np.median(a):5.1f} ms, mean {a.mean():5.1f} ms, "
            f"within 30 ms {np.mean(a < 30) * 100:3.0f}%, within 50 ms {np.mean(a < 50) * 100:3.0f}%")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("scenario")
    parser.add_argument("--seed", type=int, default=None)
    args = parser.parse_args()
    for name, enabled in (("alignment off", False), ("alignment on ", True)):
        print(f"{name}: {summary(run(args.scenario, args.seed, enabled))}")


if __name__ == "__main__":
    main()
