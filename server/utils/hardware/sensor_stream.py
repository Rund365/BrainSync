"""
Input layer for the receiver's serial lines (Integration Plan §11.2, §12, §13).

SerialReader  own thread: reads lines, stamps each with its PC arrival time
parse_line    one line -> one message (STEP, IMU, LIFT, STATUS, RXSTAT, PONG, BTN, other)
ClockMapper   sender clock -> PC clock (lower envelope of arrival - sender, with drift)
SensorStream  ties them together: unwraps sender timestamps, converts IMU units,
              feeds the clock mapper and counts lost messages
"""
import queue
import threading
import time
from collections import deque
from dataclasses import dataclass

import numpy as np

T_US_WRAP = 2 ** 32
ACC_LSB_PER_MS2 = 100.0  # BNO055 raw mode
GYR_LSB_PER_DPS = 16.0


# ----------- Messages -----------
@dataclass
class StepMsg:
    seq: int
    t_us: int
    foot: int
    interval_ms: int
    fsr_value: int
    instant_bpm: float
    avg_bpm: float
    sender_s: float = 0.0


@dataclass
class ImuMsg:
    seq: int
    t_us: int
    valid_left: bool
    valid_right: bool
    left: np.ndarray  # raw LSB: ax, ay, az, gx, gy, gz
    right: np.ndarray
    sender_s: float = 0.0


@dataclass
class LiftMsg:
    seq: int
    t_us: int
    foot: int
    fsr_value: int
    sender_s: float = 0.0


@dataclass
class StatusMsg:
    t_us: int
    i2c_err_left: int
    i2c_err_right: int
    sys_left: int
    sys_right: int
    firmware: str
    sender_s: float = 0.0


@dataclass
class OtherMsg:
    kind: str  # RXSTAT, PONG, BTN, ACK, ERR, LEGACY_STEP, UNKNOWN
    fields: list


def parse_line(line: str):
    """One protocol-2 line -> message. Raises ValueError on a malformed known line."""
    parts = line.strip().split(",")
    kind = parts[0]
    if kind == "IMU":
        if len(parts) != 16:
            raise ValueError(f"IMU line has {len(parts)} fields")
        valid = int(parts[3])
        values = np.array(parts[4:], dtype=np.int32)
        return ImuMsg(int(parts[1]), int(parts[2]), bool(valid & 1), bool(valid & 2), values[:6], values[6:])
    if kind == "STEP":
        if len(parts) != 8:
            raise ValueError(f"STEP line has {len(parts)} fields")
        return StepMsg(int(parts[1]), int(parts[2]), int(parts[3]), int(parts[4]), int(parts[5]),
                       float(parts[6]), float(parts[7]))
    if kind == "LIFT":
        return LiftMsg(int(parts[1]), int(parts[2]), int(parts[3]), int(parts[4]))
    if kind == "STATUS":
        return StatusMsg(int(parts[1]), int(parts[2]), int(parts[3]), int(parts[4]), int(parts[5]), parts[6])
    if kind in ("RXSTAT", "PONG", "BTN", "ACK", "ERR"):
        return OtherMsg(kind, parts[1:])
    if len(parts) == 4:  # protocol 1: "<millis>, <foot>, <instant_bpm>, <avg_bpm>"
        return OtherMsg("LEGACY_STEP", parts)
    return OtherMsg("UNKNOWN", parts)


# ----------- Serial reader -----------
class SerialReader:
    """Reads the serial port in its own thread so the main loop never waits on it."""

    def __init__(self, ser):
        self.ser = ser
        self.lines = queue.SimpleQueue()  # (arrival perf_counter, text)
        self.error = None  # exception that stopped the reader, for the main loop to report
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=1.0)

    def drain(self, limit: int = 500):
        out = []
        while len(out) < limit:
            try:
                out.append(self.lines.get_nowait())
            except queue.Empty:
                break
        return out

    def _run(self):
        if self.ser.timeout is None or self.ser.timeout == 0:
            self.ser.timeout = 0.2  # blocking with a timeout, so stop() is noticed
        while not self._stop.is_set():
            try:
                raw = self.ser.readline()
            except Exception as e:
                import traceback
                self.error = f"{e!r}\n{traceback.format_exc()}"
                break
            if raw:
                self.lines.put((time.perf_counter(), raw.decode("utf-8", errors="ignore").strip()))


# ----------- Clocks -----------
class TimeUnwrapper:
    """32-bit microsecond sender timestamps -> continuous seconds."""

    def __init__(self):
        self._epoch = 0
        self._last = None

    def __call__(self, t_us: int) -> float:
        epoch = self._epoch
        if self._last is not None:
            if t_us < self._last - T_US_WRAP // 2:  # wrapped forward
                self._epoch += T_US_WRAP
                epoch = self._epoch
            elif t_us > self._last + T_US_WRAP // 2:  # a late message from before the wrap
                return (t_us + epoch - T_US_WRAP) / 1e6
        self._last = t_us
        return (t_us + epoch) / 1e6


class ClockMapper:
    """
    pc_time = sender_time + offset(sender_time).

    Every message gives arrival - sender = offset + transport delay. The delay is never
    negative and often near its minimum, so the smallest value per bin (lower envelope)
    tracks the offset. A line fitted through recent bin minima also follows the drift
    of the sender's crystal. The constant minimum delay stays in the offset; it is part
    of the overall latency.
    """

    def __init__(self, bin_s: float = 1.0, fit_window_s: float = 60.0):
        self.bin_s = bin_s
        self.fit_window_s = fit_window_s
        self._bins = deque()  # [bin start sender_s, min offset, sender_s at min]
        self._slope = 0.0
        self._intercept = None
        self._ref = 0.0

    @property
    def ready(self) -> bool:
        return self._intercept is not None

    def add(self, sender_s: float, pc_arrival: float):
        offset = pc_arrival - sender_s
        if self._bins and sender_s < self._bins[-1][0] + self.bin_s:
            b = self._bins[-1]
            if offset < b[1]:
                b[1], b[2] = offset, sender_s
                if len(self._bins) < 3:
                    self._fit()
            return
        self._bins.append([sender_s, offset, sender_s])
        while self._bins and self._bins[0][0] < sender_s - self.fit_window_s:
            self._bins.popleft()
        self._fit()

    def _fit(self):
        done = list(self._bins)[:-1] if len(self._bins) >= 3 else list(self._bins)
        xs = np.array([b[2] for b in done])
        ys = np.array([b[1] for b in done])
        self._ref = float(xs[-1])
        if len(done) >= 5:
            slope, intercept = np.polyfit(xs - self._ref, ys, 1)
            self._slope, self._intercept = float(slope), float(intercept)
        else:
            self._slope, self._intercept = 0.0, float(ys.min())

    def to_pc(self, sender_s: float) -> float:
        return sender_s + self._intercept + self._slope * (sender_s - self._ref)

    def to_sender(self, pc_t: float) -> float:
        return (pc_t - self._intercept + self._slope * self._ref) / (1 + self._slope)

    def state(self) -> dict:
        return {"offset_s": self._intercept, "drift_ppm": self._slope * 1e6, "ref_sender_s": self._ref}


# ----------- Stream -----------
class LinkStats:
    def __init__(self):
        self.received = {}
        self.lost = {}
        self._last_seq = {}
        self.parse_errors = 0
        self.unknown = 0
        self.last_status = None
        self.last_rxstat = None

    def seq(self, kind: str, seq: int):
        self.received[kind] = self.received.get(kind, 0) + 1
        last = self._last_seq.get(kind)
        if last is not None and seq != last:
            gap = (seq - last) % 65536 - 1  # sequence numbers wrap at 16 bits
            if 0 < gap < 1000:
                self.lost[kind] = self.lost.get(kind, 0) + gap
        self._last_seq[kind] = seq

    def summary(self) -> str:
        parts = [f"{k}: {self.received.get(k, 0)} received, {self.lost.get(k, 0)} lost"
                 for k in sorted(self.received)]
        return "; ".join(parts) + f"; parse errors {self.parse_errors}, unknown lines {self.unknown}"


class SensorStream:
    """Parses lines, keeps the sender clock continuous and mapped to the PC clock."""

    def __init__(self):
        self.unwrap = TimeUnwrapper()
        self.clock = ClockMapper()
        self.stats = LinkStats()

    def feed(self, line: str, arrival_pc: float):
        """Returns the parsed message (with sender_s filled in), or None if unparsable."""
        try:
            msg = parse_line(line)
        except (ValueError, IndexError):
            self.stats.parse_errors += 1
            return None

        if isinstance(msg, ImuMsg):
            msg.sender_s = self.unwrap(msg.t_us)
            # IMU lines of a batch share one sequence number; count each batch once
            if self.stats._last_seq.get("IMU") != msg.seq:
                self.stats.seq("IMU", msg.seq)
            self.clock.add(msg.sender_s, arrival_pc)
        elif isinstance(msg, (StepMsg, LiftMsg)):
            msg.sender_s = self.unwrap(msg.t_us)
            self.stats.seq("STEP" if isinstance(msg, StepMsg) else "LIFT", msg.seq)
            self.clock.add(msg.sender_s, arrival_pc)
        elif isinstance(msg, StatusMsg):
            msg.sender_s = self.unwrap(msg.t_us)
            self.stats.last_status = msg
            self.clock.add(msg.sender_s, arrival_pc)
        elif msg.kind == "RXSTAT":
            self.stats.last_rxstat = msg.fields
        elif msg.kind == "UNKNOWN":
            self.stats.unknown += 1
        return msg


def imu_to_units(raw: np.ndarray):
    """Raw LSB (ax, ay, az, gx, gy, gz) -> (acc m/s², gyro dps), sensor frame."""
    return raw[:3] / ACC_LSB_PER_MS2, raw[3:] / GYR_LSB_PER_DPS
