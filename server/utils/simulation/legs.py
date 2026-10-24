"""
Simulated legs: what the two foot IMUs and FSRs measure while the walker walks.

For every heel strike (HS) of the walker, the foot that lands goes through:
  toe-off (TO)  ->  mid-swing (MS)  ->  heel strike (HS)  ->  flat on the ground
and its IMU sees:
  gyro (pitch rate):  a push-off dip at TO, a big positive peak at MS, a small dip
                      as the foot slaps flat after HS, ~0 while the foot is on the ground
  accelerometer:      gravity while still, push-off and braking bumps during the swing,
                      a sharp ringing spike at HS
The FSR crosses its threshold a little after the true contact and releases a little
before toe-off.

Each person has their own timing and signal sizes, slightly different per foot, so a
predictor has to learn them. Each sensor is mounted a bit crooked (and the left one
mirrored), has a gyro bias and noise, like the real BNO055s.

Signals are in the foot frame (x toward the toes, y to the left, z up) and in the
sensor frame (what the BNO055 actually reports).
"""
import math
from dataclasses import dataclass, field, asdict

import numpy as np

from .walker import Walker, RIGHT, LEFT

G = 9.81
SAMPLE_PERIOD_S = 0.01  # 100 Hz, same for both feet

# BNO055 raw mode, ±16 g and ±2000 dps
ACC_LSB_PER_MS2 = 100  # 1 LSB = 0.01 m/s²
GYR_LSB_PER_DPS = 16  # 1 LSB = 1/16 dps
ACC_RANGE_MS2 = 16 * G
GYR_RANGE_DPS = 2000.0


@dataclass
class FootProfile:
    """How one foot of one person moves. Timings are what the predictor has to learn."""
    swing_frac: float = 0.38  # swing time as a fraction of the stride (TO -> HS)
    ms_frac: float = 0.50  # where the mid-swing peak sits in the swing (0 = TO, 1 = HS)
    to_dps: float = -220.0  # push-off rotation at toe-off
    ms_dps: float = 380.0  # mid-swing peak
    slap_dps: float = -110.0  # foot slapping flat right after HS
    impact_g: float = 3.5  # extra acceleration at HS, in g
    brake_ms2: float = 20.0  # forward deceleration just before landing, ~2 g (real feet: up to ~3 g)
    fsr_delay_s: float = 0.020  # FSR crosses its threshold this long after true contact
    lift_lead_s: float = 0.030  # FSR releases this long before toe-off (heel rises first)
    timing_cv: float = 0.03  # stride-to-stride variation of the swing timing
    size_cv: float = 0.06  # stride-to-stride variation of the signal sizes


@dataclass
class Person:
    right: FootProfile = field(default_factory=FootProfile)
    left: FootProfile = field(default_factory=FootProfile)

    def foot(self, foot_id: int) -> FootProfile:
        return self.right if foot_id == RIGHT else self.left

    @classmethod
    def random(cls, rng: np.random.Generator) -> "Person":
        """A random person: own timing and signal sizes, with small left/right differences."""
        base = dict(
            swing_frac=rng.uniform(0.35, 0.41),
            ms_frac=rng.uniform(0.42, 0.58),
            to_dps=-rng.uniform(160, 280),
            ms_dps=rng.uniform(300, 450),
            slap_dps=-rng.uniform(70, 150),
            impact_g=rng.uniform(2.5, 5.0),
            fsr_delay_s=rng.uniform(0.010, 0.035),
            lift_lead_s=rng.uniform(0.010, 0.050),
        )

        def one_foot():
            return FootProfile(**{k: v * rng.normal(1.0, 0.04) for k, v in base.items()})

        return cls(right=one_foot(), left=one_foot())


@dataclass
class SensorProfile:
    """How one BNO055 is mounted in the sole and how imperfect it is."""
    mount: list  # 3x3 rotation, foot = mount @ sensor
    gyro_bias_dps: list
    gyro_noise_dps: float = 0.8
    acc_noise_ms2: float = 0.08
    i2c_error_rate: float = 0.0005  # chance a sample read fails

    @classmethod
    def random(cls, rng: np.random.Generator, mirrored: bool) -> "SensorProfile":
        tilt = np.radians(rng.normal(0, 4, size=3))  # a few degrees crooked
        mount = _rotation(*tilt)
        if mirrored:  # left sensor turned 180° around the vertical axis
            mount = mount @ np.diag([-1.0, -1.0, 1.0])
        return cls(mount=mount.round(6).tolist(), gyro_bias_dps=rng.normal(0, 1.5, size=3).round(3).tolist())


@dataclass
class Strike:
    """The true events of one step, in seconds since the walk started."""
    index: int
    foot: int
    hs: float  # true heel strike (initial contact)
    fsr: float  # FSR threshold crossing (what the STEP message reports)
    to: float  # true toe-off of this swing
    ms: float  # mid-swing peak of this swing
    lift: float  # FSR release before this swing
    fsr_value: int


@dataclass
class LegRecording:
    t: np.ndarray  # sample times, seconds since the walk started
    acc_foot: dict  # foot_id -> (N, 3) m/s², foot frame
    gyr_foot: dict  # foot_id -> (N, 3) dps, foot frame
    acc_raw: dict  # foot_id -> (N, 3) int LSB, sensor frame (what the BNO055 sends)
    gyr_raw: dict
    valid: dict  # foot_id -> (N,) bool, False where the I2C read failed
    strikes: list
    person: Person
    sensors: dict  # foot_id -> SensorProfile

    def meta(self) -> dict:
        return {
            "sample_period_s": SAMPLE_PERIOD_S,
            "acc_lsb_per_ms2": ACC_LSB_PER_MS2,
            "gyr_lsb_per_dps": GYR_LSB_PER_DPS,
            "foot_frame": "x toward toes, y left, z up; pitch rate is gyro y",
            "person": {"right": asdict(self.person.right), "left": asdict(self.person.left)},
            "sensors": {"right": asdict(self.sensors[RIGHT]), "left": asdict(self.sensors[LEFT])},
        }


def simulate_legs(walker: Walker, seed: int | None = None, person: Person | None = None) -> LegRecording:
    rng = np.random.default_rng(seed)
    person = person or Person.random(rng)
    sensors = {RIGHT: SensorProfile.random(rng, mirrored=False), LEFT: SensorProfile.random(rng, mirrored=True)}

    t = np.arange(0.0, walker.duration + 1.0, SAMPLE_PERIOD_S)
    acc = {f: np.zeros((len(t), 3)) for f in (RIGHT, LEFT)}
    gyr = {f: np.zeros((len(t), 3)) for f in (RIGHT, LEFT)}
    for f in (RIGHT, LEFT):
        acc[f][:, 2] = G

    strikes = []
    for i, step in enumerate(walker.steps):
        p = person.foot(step.foot)
        cadence = max(walker.cadence_at(step.time), 40.0)
        stride = 120.0 / cadence
        speed = cadence / 110.0  # faster walking = bigger swings
        size = lambda: speed * rng.normal(1.0, p.size_cv)

        swing = p.swing_frac * stride * rng.normal(1.0, p.timing_cv)
        hs = step.time
        to = max(hs - swing, 0.0)
        ms = to + (hs - to) * p.ms_frac * rng.normal(1.0, p.timing_cv)
        width = stride / 1.1

        g, a = gyr[step.foot], acc[step.foot]
        _bump(t, g[:, 1], to, 0.040 * width, p.to_dps * size())
        _bump(t, g[:, 1], ms, 0.060 * width, p.ms_dps * size())
        _bump(t, g[:, 1], hs + 0.045, 0.018, p.slap_dps * size())
        _bump(t, g[:, 0], ms, 0.070 * width, 0.12 * p.ms_dps * size())  # a little roll
        _bump(t, g[:, 2], ms, 0.080 * width, -0.08 * p.ms_dps * size())  # and yaw

        _bump(t, a[:, 0], to + 0.03, 0.035 * width, 9.0 * size())  # push-off forward
        _bump(t, a[:, 2], to + 0.05, 0.040 * width, 5.0 * size())  # foot lifts
        _bump(t, a[:, 0], hs - 0.06, 0.035 * width, -p.brake_ms2 * size())  # braking before landing
        _impact(t, a, hs, p.impact_g * G * size())

        strikes.append(Strike(
            index=i, foot=step.foot, hs=hs,
            fsr=hs + p.fsr_delay_s * rng.normal(1.0, 0.15),
            to=to, ms=ms,
            lift=max(to - p.lift_lead_s * rng.normal(1.0, 0.2), 0.0),
            fsr_value=int(rng.uniform(1050, 1450)),
        ))

    acc_raw, gyr_raw, valid = {}, {}, {}
    for f in (RIGHT, LEFT):
        s = sensors[f]
        to_sensor = np.asarray(s.mount).T  # foot = mount @ sensor  ->  sensor = mount.T @ foot
        gyr_s = gyr[f] @ to_sensor.T + np.asarray(s.gyro_bias_dps) + rng.normal(0, s.gyro_noise_dps, gyr[f].shape)
        acc_s = acc[f] @ to_sensor.T + rng.normal(0, s.acc_noise_ms2, acc[f].shape)
        gyr_raw[f] = _to_lsb(gyr_s, GYR_RANGE_DPS, GYR_LSB_PER_DPS)
        acc_raw[f] = _to_lsb(acc_s, ACC_RANGE_MS2, ACC_LSB_PER_MS2)
        valid[f] = rng.random(len(t)) >= s.i2c_error_rate
        gyr_raw[f][~valid[f]] = 0
        acc_raw[f][~valid[f]] = 0

    return LegRecording(t=t, acc_foot=acc, gyr_foot=gyr, acc_raw=acc_raw, gyr_raw=gyr_raw,
                        valid=valid, strikes=strikes, person=person, sensors=sensors)


def _bump(t, signal, center, sigma, amplitude):
    """Add a Gaussian bump to `signal` (in place), only touching samples near `center`."""
    lo, hi = np.searchsorted(t, [center - 4 * sigma, center + 4 * sigma])
    signal[lo:hi] += amplitude * np.exp(-0.5 * ((t[lo:hi] - center) / sigma) ** 2)


def _impact(t, acc, hs, peak):
    """Heel-strike impact: a sharp, quickly decaying ring, mostly vertical."""
    lo, hi = np.searchsorted(t, [hs, hs + 0.08])
    dt = t[lo:hi] - hs
    ring = peak * np.exp(-dt / 0.012) * np.cos(2 * math.pi * 35 * dt)
    acc[lo:hi, 2] += ring
    acc[lo:hi, 0] -= 0.4 * ring


def _to_lsb(values, limit, lsb_per_unit):
    return (np.clip(values, -limit, limit) * lsb_per_unit).round().astype(np.int32)


def _rotation(rx, ry, rz):
    cx, sx, cy, sy, cz, sz = math.cos(rx), math.sin(rx), math.cos(ry), math.sin(ry), math.cos(rz), math.sin(rz)
    x = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    y = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    z = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return z @ y @ x
