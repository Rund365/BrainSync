"""
Simulated sender ESP32 + radio + receiver ESP32 + USB: turns a LegRecording into the
messages that reach the PC, and when each one arrives.

Sender side (Integration Plan §8, §9):
  - one microsecond clock for everything (32-bit, wraps), a little fast or slow (drift)
  - IMU sampled at 100 Hz, both feet per sample, sent in batches of 5 (every 50 ms)
  - STEP at the FSR threshold crossing, LIFT at the FSR release, STATUS once per second
  - every message type has its own sequence number; lost packets leave gaps
Transport: a minimum delay, random jitter and occasional hiccups, first in first out.

The receiver turns each message into text lines (§11.2); that part lives in
SimulatedReceiver because the STEP line carries the receiver's own BPM calculation.
"""
from dataclasses import dataclass, asdict

import numpy as np

from ..gait.calibration import IMU_OF_FOOT
from .legs import LegRecording
from .walker import RIGHT, LEFT

T_US_WRAP = 2 ** 32
FIRMWARE_VERSION = 2


@dataclass
class LinkProfile:
    boot_offset_s: float  # how long the sender has been on when the receiver's clock starts
    drift_ppm: float  # sender crystal error: + runs fast
    min_delay_s: float = 0.004
    jitter_s: float = 0.0015  # mean of the random extra delay
    hiccup_rate: float = 0.01  # chance a message is held up (USB / OS)
    hiccup_s: float = 0.03
    packet_loss: float = 0.003
    batch_n: int = 5
    imu_extra_s: float = 0.017  # IMU batches leave this much later than a STEP would (recordings
                                # session_2026-11-01: IMU samples reach the PC 22-59 ms late, median 41)

    @classmethod
    def random(cls, rng: np.random.Generator) -> "LinkProfile":
        return cls(boot_offset_s=float(rng.uniform(5, 600)), drift_ppm=float(rng.uniform(-40, 40)))


class SenderClock:
    def __init__(self, link: LinkProfile):
        self.link = link

    def t_us(self, t_s: float) -> int:
        """Sender timestamp for a true time (seconds on the simulation's clock)."""
        sender_s = (self.link.boot_offset_s + t_s) * (1 + self.link.drift_ppm * 1e-6)
        return int(sender_s * 1e6) % T_US_WRAP


@dataclass
class Message:
    arrive_s: float  # when the receiver prints it (simulation clock)
    kind: str  # IMU, STEP, LIFT, STATUS
    seq: int
    lines: list | None = None  # ready-made text lines (IMU, LIFT, STATUS)
    step: dict | None = None  # STEP fields; the receiver adds its BPM and formats the line
    lost: bool = False


def build_messages(rec: LegRecording, link: LinkProfile, walk_start_s: float,
                   rng: np.random.Generator) -> list[Message]:
    """All messages of the walk, sorted by arrival. Times are on the simulation clock,
    where the walk starts at `walk_start_s`."""
    clock = SenderClock(link)
    sent = []  # (send_s, Message)

    # IMU batches, each IMU in the slot it is wired to, as on the real shoes
    on = {label: foot for foot, label in IMU_OF_FOOT.items()}  # IMU slot -> foot it sits on
    n = link.batch_n
    jitter_s = rng.normal(0, 30e-6, len(rec.t))
    for seq, k in enumerate(range(0, len(rec.t) - n + 1, n)):
        lines = []
        for i in range(k, k + n):
            t_us = clock.t_us(walk_start_s + rec.t[i] + jitter_s[i])
            valid = int(rec.valid[on["left"]][i]) | (int(rec.valid[on["right"]][i]) << 1)
            values = [*rec.acc_raw[on["left"]][i], *rec.gyr_raw[on["left"]][i],
                      *rec.acc_raw[on["right"]][i], *rec.gyr_raw[on["right"]][i]]
            lines.append(f"IMU,{seq},{t_us},{valid}," + ",".join(str(v) for v in values))
        sent.append((walk_start_s + rec.t[k + n - 1] + link.imu_extra_s, Message(0.0, "IMU", seq, lines=lines)))

    # FSR strikes and releases
    previous_fsr = None
    for seq, s in enumerate(rec.strikes):
        interval_ms = 0 if previous_fsr is None else round((s.fsr - previous_fsr) * 1000)
        previous_fsr = s.fsr
        step = dict(t_us=clock.t_us(walk_start_s + s.fsr), foot=s.foot, interval_ms=interval_ms,
                    fsr_value=s.fsr_value, strike=s)
        sent.append((walk_start_s + s.fsr, Message(0.0, "STEP", seq, step=step)))
    lifts = sorted(rec.strikes, key=lambda s: s.lift)
    for seq, s in enumerate(lifts):
        line = f"LIFT,{seq},{clock.t_us(walk_start_s + s.lift)},{s.foot},{int(rng.uniform(600, 900))}"
        sent.append((walk_start_s + s.lift, Message(0.0, "LIFT", seq, lines=[line])))

    # Health, once per second
    for seq, t in enumerate(np.arange(1.0, rec.t[-1], 1.0)):
        upto = rec.t <= t
        err_l = int((~rec.valid[LEFT][upto]).sum())
        err_r = int((~rec.valid[RIGHT][upto]).sum())
        line = f"STATUS,{clock.t_us(walk_start_s + t)},{err_l},{err_r},6,6,{FIRMWARE_VERSION}"
        sent.append((walk_start_s + t, Message(0.0, "STATUS", seq, lines=[line])))

    # Radio + USB: losses, delays, first in first out
    sent.sort(key=lambda item: item[0])
    messages = []
    last_arrival = 0.0
    for send_s, msg in sent:
        msg.lost = rng.random() < link.packet_loss
        delay = link.min_delay_s + rng.exponential(link.jitter_s)
        if rng.random() < link.hiccup_rate:
            delay += rng.uniform(0.5, 1.0) * link.hiccup_s
        msg.arrive_s = max(send_s + delay, last_arrival)
        if not msg.lost:
            last_arrival = msg.arrive_s
        messages.append(msg)
    return messages


def link_meta(link: LinkProfile) -> dict:
    return {"link": asdict(link), "firmware_version": FIRMWARE_VERSION, "t_us_wrap": T_US_WRAP}
