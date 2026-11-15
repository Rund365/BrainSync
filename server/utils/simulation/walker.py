"""
Simulated walker: turns a walking scenario into a list of foot strikes.

A scenario is a list of segments "cadence:seconds[:motion]", e.g. "110:20,130:15,0:5,110:20:place":
walk at 110 steps/min for 20 s, then 130 for 15 s, stand still for 5 s, then step in place at 110
for 20 s. Cadence 0 means standing still. Motion (legs.py): walk (forward, the default), place
(stepping in place, as in the recordings) or turn (walking in a circle).

Realistic walking (default):
  - pace changes are gradual, an S-curve over `ramp_s` seconds
  - before standing still (and at the end) the walker slows down over `stop_s` seconds
  - after standing still (and at the start) the first steps are slower, speeding up over `start_s`
  - the pace drifts slowly by a percent or two, like a real person's
Sharp walking (realistic=False) is the stress test: linear 3 s ramps between walking
segments, instant stops and starts, no drift.
"""
import math
import random
from dataclasses import dataclass

RIGHT = 1  # Foot IDs as sent by the sender ESP32
LEFT = 2

PRESETS = {
    "steady": "110:60",
    "speedup": "100:20,130:30,100:20",
    "stop_start": "110:20,0:5,110:20,0:5,110:20",
    "mixed": "105:20,125:20,0:4,110:20,95:20",
    "in_place": "115:60:place",  # like the recordings (session_2026-11-01)
    "around": "110:20,110:20:turn,0:4,115:20:place,110:20",
    "calibration": "0:5,100:12,0:5",  # the IMU calibration walk (utils/analysis/calibrate_imus.py)
}
MOTIONS = ("walk", "place", "turn")


@dataclass
class Segment:
    cadence: float  # steps per minute, 0 = standing still
    duration: float  # seconds
    motion: str = "walk"


@dataclass
class Step:
    time: float  # seconds since the walk started
    foot: int  # RIGHT or LEFT
    interval: float  # seconds since the previous step (of either foot); 0 for the first step
    motion: str = "walk"


def parse_scenario(text: str) -> list[Segment]:
    """Parse a preset name or a "cadence:seconds[:motion],..." string."""
    text = PRESETS.get(text.strip(), text.strip())
    segments = []
    for part in text.split(","):
        cadence, _, rest = part.partition(":")
        duration, _, motion = rest.partition(":")
        try:
            segment = Segment(float(cadence), float(duration), motion.strip() or "walk")
        except ValueError:
            raise ValueError(f"Bad scenario segment {part!r}; expected 'cadence:seconds[:motion]' "
                             f"or one of {list(PRESETS)}")
        if segment.cadence < 0 or segment.duration <= 0:
            raise ValueError(f"Bad scenario segment {part!r}; cadence must be >= 0 and seconds > 0")
        if segment.motion not in MOTIONS:
            raise ValueError(f"Bad scenario segment {part!r}; motion must be one of {MOTIONS}")
        segments.append(segment)
    return segments


def _smoothstep(x: float) -> float:
    x = min(max(x, 0.0), 1.0)
    return x * x * (3 - 2 * x)


class Walker:
    def __init__(self, segments: list[Segment], realistic: bool = True, ramp_s: float | None = None,
                 start_s: float = 3.0, stop_s: float = 2.0, drift: float = 0.015, step_cv: float = 0.035,
                 asymmetry: float = 0.02, first_step_s: float = 0.5, seed: int | None = None):
        """
        Args:
            segments: the scenario.
            realistic: gradual changes, slowing into stops, slow first steps, drift (see module doc).
            ramp_s: seconds to change between two walking cadences (default 5 realistic, 3 sharp).
            start_s: seconds to reach full pace after standing (realistic only).
            stop_s: seconds of slowing down before standing (realistic only).
            drift: size of the slow pace drift, as a fraction of the cadence (realistic only).
            step_cv: random step-to-step variation, as a fraction of the step time (the recordings:
                     stride-to-stride differences of 40-80 ms, about 3.5% per step).
            asymmetry: left steps this fraction longer, right steps this fraction shorter.
            first_step_s: time of the first step after the walk starts.
            seed: random seed, for repeatable walks.
        """
        self.segments = segments
        self.realistic = realistic
        self.ramp_s = ramp_s if ramp_s is not None else (5.0 if realistic else 3.0)
        self.start_s = start_s
        self.stop_s = stop_s
        self.step_cv = step_cv
        self.asymmetry = asymmetry
        self.duration = sum(s.duration for s in segments)
        self._rng = random.Random(seed)
        # Slow drift: a few slow waves with random periods and phases
        self._drift = [(drift / 2 * self._rng.uniform(0.6, 1.0), self._rng.uniform(8, 30), self._rng.uniform(0, 6.3))
                       for _ in range(3)] if realistic else []
        self.steps = self._generate(first_step_s)

    def cadence_at(self, t: float) -> float:
        start = 0.0
        for i, segment in enumerate(self.segments):
            end = start + segment.duration
            if t < end:
                previous = self.segments[i - 1].cadence if i > 0 else 0.0
                following = self.segments[i + 1].cadence if i + 1 < len(self.segments) else 0.0
                if self.realistic:
                    return self._realistic(t, start, end, segment.cadence, previous, following)
                return self._sharp(t, start, segment.cadence, previous)
            start = end
        return 0.0

    def _sharp(self, t, start, cadence, previous):
        if previous > 0 and cadence > 0 and t - start < self.ramp_s:
            return previous + (cadence - previous) * (t - start) / self.ramp_s
        return cadence

    def _realistic(self, t, start, end, cadence, previous, following):
        if cadence == 0:
            return 0.0
        if previous == 0:  # starting from standing: first steps slower
            c = cadence * (0.75 + 0.25 * _smoothstep((t - start) / self.start_s))
        else:
            c = previous + (cadence - previous) * _smoothstep((t - start) / self.ramp_s)
        if following == 0 and t > end - self.stop_s:  # slowing down to stand still
            c *= 1 - 0.25 * _smoothstep((t - (end - self.stop_s)) / self.stop_s)
        return c * (1 + sum(a * math.sin(2 * math.pi * t / period + phase) for a, period, phase in self._drift))

    def motion_at(self, t: float) -> str:
        end = 0.0
        for segment in self.segments:
            end += segment.duration
            if t < end:
                return segment.motion
        return self.segments[-1].motion

    def _segment_end(self, t: float) -> float:
        end = 0.0
        for segment in self.segments:
            end += segment.duration
            if t < end:
                return end
        return end

    def _generate(self, first_step_s: float) -> list[Step]:
        steps = []
        t = first_step_s
        foot = RIGHT
        while t < self.duration:
            cadence = self.cadence_at(t)
            if cadence == 0:
                # Standing still: the next step is the first one after the pause
                t = self._segment_end(t) + (0.4 if self.realistic else 0.0)
                continue
            interval = t - steps[-1].time if steps else 0.0
            steps.append(Step(time=t, foot=foot, interval=interval, motion=self.motion_at(t)))
            foot = LEFT if foot == RIGHT else RIGHT

            side = 1 + self.asymmetry if foot == LEFT else 1 - self.asymmetry
            step_s = 60.0 / cadence * side * (1 + self._rng.gauss(0, self.step_cv))
            t += max(step_s, 0.25)
        return steps
