"""
Simulated legs: what the two foot IMUs and FSRs measure while the walker walks or steps in place.

Each foot is a rigid body (heel at the origin, toes 22 cm ahead, the IMU on top of the mid-foot)
moved through the phases of every stride:
  loading   contact -> foot flat, turning about the point that touched: the heel when walking,
            the toes when stepping in place (as in the recordings)
  flat      on the ground, with a small wobble (the recorded feet are never still: 5-18 °/s)
  push-off  heel-off -> toe-off, turning about the toes; the heel FSR releases at heel-off
  swing     in the air to the next contact: forward when walking, back to the same spot in place,
            over a highest point, turning back for the landing; a turn changes the heading
The IMU signals are computed from that motion (gyro = rotation rate, accelerometer = acceleration
plus gravity, both in the sensor frame), so they agree with each other as real ones do. At contact
the foot's speed is stopped by a short ringing impact. The heel FSR fires once the heel is down and
loaded: right after contact when walking, when the heel comes down after a toes-first landing.

Fitted to the right foot of the recordings (session_2026-11-01, stepping in place, IMU landing
aligned): the foot leaves the ground ~0.3 s before landing and rises ~0.2 m/s, tilts ~10°, lands
toes first, the FSR fires 20-40 ms after the impact (SD 9-17 ms), the impact jumps 18-31 m/s²
between samples, the accelerometer reads ~10.4 m/s² at rest and the IMU is mounted at a large,
unknown angle. Walking forward follows the usual foot kinematics (Sabatini et al. 2005, Fig. 4:
toes down ~60° at toe-off, up ~20° at heel strike; swing ~38% of the stride).
"""
import math
from dataclasses import dataclass, field, asdict, replace

import numpy as np

from .walker import Walker, RIGHT, LEFT

G = 9.81
SAMPLE_PERIOD_S = 0.01  # 100 Hz, same for both feet
FINE_S = 0.001  # the motion is built at 1 kHz and sampled at 100 Hz
SENSOR_AVG_S = 0.008  # the sensor's own low-pass, as a moving average (62.5 Hz bandwidth ~ 8 ms)

# BNO055 raw mode, ±16 g and ±2000 dps
ACC_LSB_PER_MS2 = 100  # 1 LSB = 0.01 m/s²
GYR_LSB_PER_DPS = 16  # 1 LSB = 1/16 dps
ACC_RANGE_MS2 = 16 * G
GYR_RANGE_DPS = 2000.0

# Points of the foot, foot frame (x toward the toes, y to the left, z up), metres from the heel
TOE = np.array([0.22, 0.0, 0.0])
IMU_AT = np.array([0.10, 0.0, 0.06])
HEEL_DOWN_M = 0.009  # the heel FSR starts to load this close to the ground (sole and insole give)


@dataclass
class Motion:
    """How one foot moves in one kind of stride. Angles: + = toes down."""
    stride_m: float  # stride length at 110 steps/min (one landing to the next of the same foot); 0 = in place
    swing_frac: float  # time in the air, as a fraction of the stride
    push_frac: float  # heel-off -> toe-off (heel rising, toes on the ground), fraction of the stride
    load_s: float  # contact -> foot flat (toes first: at land_deg; a steeper landing takes longer)
    toe_off_deg: float  # tilt at toe-off
    land_deg: float  # tilt at contact: - = toes up (heel first), + = toes down (toes first)
    apex_frac: float  # where in the swing the heel is highest (0 = toe-off, 1 = contact): it rises on the
                      # speed it had at toe-off, slowing down evenly, then falls back
    land_speed: float  # downward speed at contact (m/s), stopped by the impact
    turn_deg: float = 0.0  # change of heading per stride (+ = left)


WALK = Motion(stride_m=1.3, swing_frac=0.38, push_frac=0.22, load_s=0.15, toe_off_deg=60, land_deg=-20,
              apex_frac=0.30, land_speed=0.30)
PLACE = Motion(stride_m=0.0, swing_frac=0.21, push_frac=0.15, load_s=0.09, toe_off_deg=8, land_deg=3.0,
               apex_frac=0.25, land_speed=0.30)
TURN = replace(WALK, stride_m=1.1, turn_deg=20.0)


@dataclass
class FootProfile:
    """How one foot of one person moves. Timings are what the predictor has to learn."""
    walk: Motion = field(default_factory=lambda: replace(WALK))
    place: Motion = field(default_factory=lambda: replace(PLACE))
    turn: Motion = field(default_factory=lambda: replace(TURN))
    fsr_build_s: float = 0.012  # heel down -> FSR above its threshold
    lift_lag_s: float = 0.010  # heel-off -> FSR below its threshold
    timing_cv: float = 0.03  # stride-to-stride variation of the swing timing
    size_cv: float = 0.06  # stride-to-stride variation of angles, heights and speeds
    land_deg_sd: float = 1.0  # stride-to-stride variation of the tilt at contact (moves the FSR time)
    wobble_dps: float = 8.0  # rotation of the foot on the ground between steps (recordings: 5-18 °/s);
                             # a tenth of it while standing still
    impact_tau_s: float = 0.006  # decay of the impact (recordings: |a| jumps 18-31 m/s² from one sample to the next)

    def motion(self, name: str) -> Motion:
        return getattr(self, name)


@dataclass
class Person:
    right: FootProfile = field(default_factory=FootProfile)
    left: FootProfile = field(default_factory=FootProfile)

    def foot(self, foot_id: int) -> FootProfile:
        return self.right if foot_id == RIGHT else self.left

    @classmethod
    def random(cls, rng: np.random.Generator) -> "Person":
        """A random person: own timing, angles and heights, with small left/right differences."""
        def vary(m: Motion, cv: float) -> Motion:
            return Motion(**{k: v * rng.normal(1.0, cv) for k, v in asdict(m).items()})

        base = {name: vary(m, 0.08) for name, m in (("walk", WALK), ("place", PLACE), ("turn", TURN))}
        fsr_build, wobble = rng.uniform(0.006, 0.018), rng.uniform(5, 12)

        def one_foot():
            return FootProfile(**{name: vary(m, 0.03) for name, m in base.items()},
                               fsr_build_s=fsr_build * rng.normal(1.0, 0.1), wobble_dps=wobble * rng.normal(1.0, 0.1))

        return cls(right=one_foot(), left=one_foot())


@dataclass
class SensorProfile:
    """How one BNO055 sits on the shoe and how imperfect it is."""
    mount: list  # 3x3 rotation, foot = mount @ sensor: any angle (the recorded IMUs sit at 40-70°)
    gyro_bias_dps: list
    acc_gain: list  # per axis (the recorded IMUs read ~10.4 m/s² at rest)
    gyro_noise_dps: float = 0.3
    acc_noise_ms2: float = 0.05
    i2c_error_rate: float = 0.0005  # chance a sample read fails

    @classmethod
    def random(cls, rng: np.random.Generator) -> "SensorProfile":
        q, r = np.linalg.qr(rng.normal(size=(3, 3)))
        mount = q * np.sign(np.diag(r))
        if np.linalg.det(mount) < 0:
            mount[:, 0] = -mount[:, 0]
        return cls(mount=mount.round(6).tolist(), gyro_bias_dps=rng.normal(0, 1.5, size=3).round(3).tolist(),
                   acc_gain=rng.uniform(1.03, 1.07, size=3).round(4).tolist())


@dataclass
class Strike:
    """The true events of one step, in seconds since the walk started."""
    index: int
    foot: int
    hs: float  # true initial contact (heel when walking, toes in place): the impact
    fsr: float  # FSR threshold crossing (what the STEP message reports)
    to: float  # true toe-off of this swing
    ms: float  # fastest rotation of this swing (mid-swing)
    lift: float  # FSR release before this swing
    fsr_value: int
    motion: str = "walk"


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
            "foot_frame": "x toward toes, y left, z up; pitch rate is gyro y (+ = toes down)",
            "person": {"right": asdict(self.person.right), "left": asdict(self.person.left)},
            "sensors": {"right": asdict(self.sensors[RIGHT]), "left": asdict(self.sensors[LEFT])},
        }


def simulate_legs(walker: Walker, seed: int | None = None, person: Person | None = None) -> LegRecording:
    rng = np.random.default_rng(seed)
    person = person or Person.random(rng)
    sensors = {RIGHT: SensorProfile.random(rng), LEFT: SensorProfile.random(rng)}

    fine = np.arange(0.0, walker.duration + 1.0, FINE_S)
    every = round(SAMPLE_PERIOD_S / FINE_S)
    t = fine[::every]
    acc, gyr, acc_raw, gyr_raw, valid, strikes = {}, {}, {}, {}, {}, []
    for f in (RIGHT, LEFT):
        a, w, foot_strikes = _foot(fine, walker, f, person.foot(f), rng)
        acc[f], gyr[f] = _sensor_filter(a)[::every], _sensor_filter(w)[::every]
        strikes += foot_strikes

        s = sensors[f]
        to_sensor = np.asarray(s.mount).T  # foot = mount @ sensor  ->  sensor = mount.T @ foot
        gyr_s = gyr[f] @ to_sensor.T + np.asarray(s.gyro_bias_dps) + rng.normal(0, s.gyro_noise_dps, gyr[f].shape)
        acc_s = (acc[f] @ to_sensor.T) * np.asarray(s.acc_gain) + rng.normal(0, s.acc_noise_ms2, acc[f].shape)
        gyr_raw[f] = _to_lsb(gyr_s, GYR_RANGE_DPS, GYR_LSB_PER_DPS)
        acc_raw[f] = _to_lsb(acc_s, ACC_RANGE_MS2, ACC_LSB_PER_MS2)
        valid[f] = rng.random(len(t)) >= s.i2c_error_rate
        gyr_raw[f][~valid[f]] = 0
        acc_raw[f][~valid[f]] = 0

    strikes.sort(key=lambda s: s.hs)
    for i, s in enumerate(strikes):
        s.index = i
    return LegRecording(t=t, acc_foot=acc, gyr_foot=gyr, acc_raw=acc_raw, gyr_raw=gyr_raw,
                        valid=valid, strikes=strikes, person=person, sensors=sensors)


def _foot(fine, walker, foot, p: FootProfile, rng):
    """One foot's motion on the fine grid -> (acceleration, rotation rate) in the foot frame, strikes."""
    n = len(fine)
    tilt, heading, heel = np.zeros(n), np.zeros(n), np.zeros((n, 3))
    contacts, strikes = [], []
    ground = np.zeros(3)  # heel point while the foot is flat
    yaw = 0.0
    flat_from, prev_hs = 0, None
    idx = lambda x: int(np.clip(round(x / FINE_S), 0, n))

    for step in (s for s in walker.steps if s.foot == foot):
        m = p.motion(step.motion)
        size = lambda: rng.normal(1.0, p.size_cv)
        hs = step.time
        cadence = max(walker.cadence_at(hs), 40.0)
        stride = 120.0 / cadence
        first = prev_hs is None or hs - prev_hs > 1.6 * stride
        swing = m.swing_frac * stride * rng.normal(1.0, p.timing_cv)
        push = m.push_frac * stride * rng.normal(1.0, p.timing_cv)
        earliest = flat_from * FINE_S + 0.05
        swing = min(swing, hs - earliest - 0.04)
        push = max(0.04, min(push, hs - swing - earliest))
        t_to, t_ho = hs - swing, hs - swing - push
        a_to = math.radians(m.toe_off_deg * size())
        a_land = math.radians(m.land_deg * size() + (rng.normal(0, p.land_deg_sd) if m.land_deg > 0 else 0.0))
        length = m.stride_m * (cadence / 110.0) ** 0.6 * size() * (0.5 if first else 1.0)
        yaw_new = yaw + math.radians(m.turn_deg * size())
        direction = np.array([math.cos((yaw + yaw_new) / 2), math.sin((yaw + yaw_new) / 2), 0.0])

        # flat until heel-off
        i_ho, i_to, i_hs = idx(t_ho), idx(t_to), idx(hs)
        tilt[flat_from:i_ho], heading[flat_from:i_ho], heel[flat_from:i_ho] = 0.0, yaw, ground

        # push-off: about the toes
        rate_to = 0.8 * a_to / push
        toe = ground + _rz(yaw) @ TOE
        u = fine[i_ho:i_to]
        tilt[i_ho:i_to] = _quintic(u, t_ho, t_to, 0.0, 0.0, a_to, rate_to)
        heading[i_ho:i_to] = yaw
        heel[i_ho:i_to] = toe - _rot(heading[i_ho:i_to], tilt[i_ho:i_to]) @ TOE
        heel_to = toe - _rot(yaw, a_to) @ TOE
        spin = _rz(yaw) @ np.array([0.0, rate_to, 0.0])
        v_to = -np.cross(spin, _rot(yaw, a_to) @ TOE)

        # landing pose: heel first (toes up) or toes first (toes down, the heel still coming down)
        load = max(m.load_s * (a_land / math.radians(m.land_deg) if a_land > 0 else 1.0), 0.02)
        rate_land = -1.5 * a_land / load if a_land > 0 else 0.0
        if a_land <= 0:
            pivot_local = np.zeros(3)
            ground_new = ground + length * direction
            heel_hs = ground_new.copy()
        else:
            pivot_local = TOE
            toe_new = toe + length * direction
            ground_new = toe_new - _rz(yaw_new) @ TOE
            heel_hs = toe_new - _rot(yaw_new, a_land) @ TOE
        pivot = heel_hs + _rot(yaw_new, a_land) @ pivot_local

        # swing: tilt and heading turn back for the landing; the heel goes over its highest point
        u = fine[i_to:i_hs]
        tilt[i_to:i_hs] = _quintic(u, t_to, hs, a_to, rate_to, a_land, rate_land)
        heading[i_to:i_hs] = _quintic(u, t_to, hs, yaw, 0.0, yaw_new, 0.0)
        frac = m.apex_frac
        t_ap = t_to + frac * swing
        travel = heel_hs - heel_to
        apex = heel_to + frac * travel
        rise = max(v_to[2], 0.0)
        apex[2] = max(heel_to[2] + rise * frac * swing / 2, heel_hs[2] + 0.01)
        v_ap = 1.4 * travel / swing
        v_ap[2] = 0.0
        v_pivot = 0.35 * length / swing * direction
        v_pivot[2] = -m.land_speed * size()
        spin = _rz(yaw_new) @ np.array([0.0, rate_land, 0.0])
        v_hs = v_pivot - np.cross(spin, _rot(yaw_new, a_land) @ pivot_local)
        i_ap = idx(t_ap)
        drop_s = (1 - frac) * swing
        top = np.array([0.0, 0.0, -rise / (frac * swing)])
        fall = np.array([0.0, 0.0, -2 * (apex[2] - heel_hs[2]) / drop_s**2])  # about like a thrown object
        heel[i_to:i_ap] = _quintic(fine[i_to:i_ap], t_to, t_ap, heel_to, v_to, apex, v_ap, a1=top)
        heel[i_ap:i_hs] = _quintic(fine[i_ap:i_hs], t_ap, hs, apex, v_ap, heel_hs, v_hs, a0=top,
                                   a1=fall if a_land > 0 else 0.0)

        # loading: about the point that touched, down to flat
        i_flat = idx(hs + load)
        u = fine[i_hs:i_flat]
        tilt[i_hs:i_flat] = _quintic(u, hs, hs + load, a_land, rate_land, 0.0, 0.0)
        heading[i_hs:i_flat] = yaw_new
        heel[i_hs:i_flat] = pivot - _rot(heading[i_hs:i_flat], tilt[i_hs:i_flat]) @ pivot_local
        if a_land <= 0:
            heel_down = hs
        else:
            down = np.nonzero(heel[i_hs:i_flat, 2] <= HEEL_DOWN_M)[0]
            heel_down = fine[i_hs + down[0]] if len(down) else hs + load

        sw = np.gradient(tilt[i_to:i_hs]) * np.sign(a_land - a_to)
        strikes.append(Strike(
            index=0, foot=foot, hs=hs,
            fsr=heel_down + p.fsr_build_s * rng.normal(1.0, 0.2),
            to=t_to, ms=float(fine[i_to + int(np.argmax(sw))]) if len(sw) else t_to,
            lift=t_ho + p.lift_lag_s * rng.normal(1.0, 0.2),
            fsr_value=int(rng.uniform(1050, 1450)), motion=step.motion,
        ))
        contacts.append(i_hs)
        ground, yaw, flat_from, prev_hs = ground_new, yaw_new, i_flat, hs
    tilt[flat_from:], heading[flat_from:], heel[flat_from:] = 0.0, yaw, ground

    # the foot is never quite still: a slow wobble in tilt and roll
    stepping = np.array([walker.cadence_at(x) > 0 for x in fine[::100]], float)
    level = np.convolve(np.repeat(stepping, 100)[:n], np.ones(500) / 500, mode="same")  # eases over 0.5 s
    level = 0.1 + 0.9 * np.pad(level, (0, n - len(level)), mode="edge")
    wobble_tilt, wobble_roll = (level * _wobble(n, p.wobble_dps, rng) for _ in range(2))
    rot = _rot(heading, tilt + wobble_tilt, wobble_roll)
    imu = heel + rot @ IMU_AT
    vel = np.gradient(imu, FINE_S, axis=0)
    acc = np.gradient(vel, FINE_S, axis=0)
    for c in contacts:  # the speed at contact is stopped by a short ringing impact
        if 4 <= c < n - 100:
            dv = vel[c + 3] - vel[c - 3]
            acc[c - 3:c + 4] = np.linspace(acc[c - 4], acc[c + 4], 7)
            u = np.arange(100) * FINE_S
            pulse = np.exp(-u / p.impact_tau_s) * (1.0 + 0.3 * np.cos(2 * math.pi * 35 * u))
            acc[c:c + 100] += np.outer(pulse / (pulse.sum() * FINE_S), dv)
    spin = np.einsum("nji,njk->nik", rot, np.gradient(rot, FINE_S, axis=0))  # R^T dR/dt
    gyr = np.degrees(np.stack([spin[:, 2, 1], spin[:, 0, 2], spin[:, 1, 0]], axis=1))
    acc_foot = np.einsum("nji,nj->ni", rot, acc + np.array([0.0, 0.0, G]))
    return acc_foot, gyr, strikes


def _sensor_filter(x):
    """The BNO055 low-passes before it samples (accelerometer bandwidth 62.5 Hz in its default
    setting): about an average over the last SENSOR_AVG_S, so a short impact cannot fall between samples."""
    k = round(SENSOR_AVG_S / FINE_S)
    c = np.cumsum(np.vstack([np.zeros((1, x.shape[1])), x]), axis=0)
    out = x.copy()
    out[k:] = (c[k + 1:] - c[1:-k]) / k
    return out


def _wobble(n, rms_dps, rng):
    """Random tilt (rad) whose rate has `rms_dps` RMS, from 1-6 Hz."""
    spectrum = np.fft.rfft(rng.normal(size=n))
    freq = np.fft.rfftfreq(n, FINE_S)
    spectrum[(freq < 1.0) | (freq > 6.0)] = 0.0
    angle = np.fft.irfft(spectrum, n)
    rate = np.gradient(angle, FINE_S)
    return angle * math.radians(rms_dps) / max(rate.std(), 1e-12)


def _quintic(t, t0, t1, x0, v0, x1, v1, a0=0.0, a1=0.0):
    """Smooth move from (x0, v0, a0) at t0 to (x1, v1, a1) at t1 (numbers or vectors): position,
    speed and acceleration given at both ends, so joined moves have no jumps in acceleration."""
    T = t1 - t0
    s = (np.asarray(t) - t0) / T
    if np.ndim(x0):
        s = s[:, None]
    return ((1 - 10 * s**3 + 15 * s**4 - 6 * s**5) * x0 + (s - 6 * s**3 + 8 * s**4 - 3 * s**5) * T * v0
            + (0.5 * s**2 - 1.5 * s**3 + 1.5 * s**4 - 0.5 * s**5) * T**2 * a0
            + (0.5 * s**3 - s**4 + 0.5 * s**5) * T**2 * a1
            + (-4 * s**3 + 7 * s**4 - 3 * s**5) * T * v1 + (10 * s**3 - 15 * s**4 + 6 * s**5) * x1)


def _rz(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _rot(yaw, tilt, roll=0.0):
    """Foot -> room rotation(s): heading about z, then tilt about the foot's y (+ = toes down), then roll."""
    yaw, tilt, roll = np.broadcast_arrays(np.asarray(yaw, float), np.asarray(tilt, float), np.asarray(roll, float))
    cz, sz, cy, sy, cx, sx = np.cos(yaw), np.sin(yaw), np.cos(tilt), np.sin(tilt), np.cos(roll), np.sin(roll)
    one, zero = np.ones_like(yaw), np.zeros_like(yaw)
    z = np.stack([np.stack([cz, -sz, zero], -1), np.stack([sz, cz, zero], -1), np.stack([zero, zero, one], -1)], -2)
    y = np.stack([np.stack([cy, zero, sy], -1), np.stack([zero, one, zero], -1), np.stack([-sy, zero, cy], -1)], -2)
    x = np.stack([np.stack([one, zero, zero], -1), np.stack([zero, cx, -sx], -1), np.stack([zero, sx, cx], -1)], -2)
    return z @ y @ x


def _to_lsb(values, limit, lsb_per_unit):
    return (np.clip(values, -limit, limit) * lsb_per_unit).round().astype(np.int32)
