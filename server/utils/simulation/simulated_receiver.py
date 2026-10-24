"""
Stand-in for the serial port of the receiver ESP32.

Behaves like ESP32/step_receiver_sketch: answers the PC's commands with the same ACK
lines, averages step intervals the same way, and prints each message when it would
reach the PC. Two protocols:
  1 = today's firmware: only step lines "<millis>, <foot>, <instant_bpm>, <avg_bpm>"
  2 = planned IMU firmware (Integration Plan §11.2): STEP, IMU (100 Hz, both feet),
      LIFT, STATUS, RXSTAT lines, plus the VERSION, PING and IMU_STREAM commands
Only the parts of pyserial that the engine uses are implemented: write, readline,
reset_input_buffer, close, timeout.
"""
import csv
import json
import os
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np

from .legs import simulate_legs
from .link import LinkProfile, SenderClock, build_messages, link_meta, FIRMWARE_VERSION
from .walker import Walker

MAX_WINDOW_SIZE = 20  # Same limit as the receiver firmware
PLAN_ENV = "BRAINSYNC_SIM_PLAN"


class SimulatedReceiver:
    def __init__(self, walker: Walker, protocol: int = 2, seed: int | None = None, time_source=time.perf_counter):
        """`time_source` is the PC clock; pass a fake one to run faster than real time."""
        if protocol not in (1, 2):
            raise ValueError(f"Unknown protocol {protocol}; expected 1 or 2")
        self.walker = walker
        self.protocol = protocol
        self.port = "SIMULATED"
        self.timeout = None
        self.is_open = True

        self._rng = np.random.default_rng(seed)
        self.legs = simulate_legs(walker, seed=seed)
        self.link = LinkProfile.random(self._rng)
        self._clock = SenderClock(self.link)

        self._time = time_source
        self._lock = threading.Lock()
        self._emit_lock = threading.Lock()  # readline (reader thread) and finished (main loop) both emit
        self._boot = time_source()  # zero of the simulation clock
        self._lines = deque()

        # Receiver firmware state
        self.smoothing_window = 3
        self.update_stride = 1
        self.imu_streaming = True
        self._session_starting = True
        self._session_started = False
        self._intervals_ms = deque(maxlen=MAX_WINDOW_SIZE)
        self._step_counter = 0
        self._last_bpm = 0.0
        self._rx_count = 0
        self._next_rxstat = None

        # Walk playback
        self._walk_start = None  # simulation clock, seconds
        self._messages = []
        self._next_message = 0
        self.emitted_steps = []  # (strike, perf_counter time the STEP line was printed)
        self.emitted_imu = []  # (perf_counter, IMU line)

    # ----------- pyserial-like API -----------
    @property
    def in_waiting(self) -> int:
        self._emit_due()
        with self._lock:
            return sum(len(line) for line in self._lines)

    def write(self, data: bytes) -> int:
        for command in data.decode("utf-8", errors="ignore").splitlines():
            self._handle_command(command.strip())
        return len(data)

    def readline(self) -> bytes:
        deadline = None if self.timeout is None else self._time() + self.timeout
        while True:
            self._emit_due()
            with self._lock:
                if self._lines:
                    return self._lines.popleft()
            if deadline is not None and self._time() >= deadline:
                return b""
            time.sleep(0.001)

    def reset_input_buffer(self):
        with self._lock:
            self._lines.clear()

    def close(self):
        self.is_open = False

    # ----------- Simulation status -----------
    @property
    def finished(self) -> bool:
        """True once the whole walk has been delivered."""
        if self._walk_start is None:
            return False
        self._emit_due()
        return self._next_message >= len(self._messages) and self._now() - self._walk_start > self.walker.duration

    def write_truth(self, session_dir: Path):
        """Save what really happened, next to the engine's own logs:
        sim_steps.csv     true events of every step (heel strike, FSR, toe-off, mid-swing, lift)
        sim_imu_data.csv  every IMU sample the PC received, in the planned imu_data.csv format
        sim_meta.json     the simulated person, sensors and link, to check calibration against
        """
        session_dir = Path(session_dir)
        start = self._walk_start or 0.0
        printed = {id(strike): at for strike, at in self.emitted_steps}

        with open(session_dir / "sim_steps.csv", "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(["step_index", "foot", "hs_s", "fsr_s", "to_s", "ms_s", "lift_s",
                             "hs_t_us", "fsr_t_us", "to_t_us", "ms_t_us", "lift_t_us",
                             "fsr_value", "delivered", "printed_pc_t", "hs_pc_t", "fsr_pc_t"])
            for s in self.legs.strikes:
                t_us = [self._clock.t_us(start + x) for x in (s.hs, s.fsr, s.to, s.ms, s.lift)]
                at = printed.get(id(s))
                writer.writerow([s.index, s.foot, *(f"{x:.4f}" for x in (s.hs, s.fsr, s.to, s.ms, s.lift)),
                                 *t_us, s.fsr_value, int(at is not None), "" if at is None else f"{at:.6f}",
                                 f"{self._boot + start + s.hs:.6f}", f"{self._boot + start + s.fsr:.6f}"])

        with open(session_dir / "sim_imu_data.csv", "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(["sender_t_us", "pc_t", "seq", "valid_l", "valid_r",
                             "l_ax", "l_ay", "l_az", "l_gx", "l_gy", "l_gz",
                             "r_ax", "r_ay", "r_az", "r_gx", "r_gy", "r_gz"])
            for at, line in self.emitted_imu:
                _, seq, t_us, valid, *values = line.split(",")
                valid = int(valid)
                writer.writerow([t_us, f"{at:.6f}", seq, valid & 1, (valid >> 1) & 1, *values])

        meta = {"protocol": self.protocol, "walk_start_s": start, **self.legs.meta(), **link_meta(self.link)}
        with open(session_dir / "sim_meta.json", "w", encoding="utf-8") as f:
            json.dump(meta, f, indent=2)

    # ----------- Receiver firmware behavior -----------
    def _now(self) -> float:
        return self._time() - self._boot

    def _send(self, text: str):
        with self._lock:
            self._lines.append((text + "\r\n").encode("utf-8"))

    def _handle_command(self, command: str):
        if command == "RESET":
            self._session_starting = True
            self._send("ACK,RESET")
        elif command == "START":
            if not self._session_starting:
                return
            self._intervals_ms.clear()
            self._session_starting = False
            self._session_started = True
            self._step_counter = 0
            self._last_bpm = 0.0
            self._walk_start = self._now()
            self._messages = build_messages(self.legs, self.link, self._walk_start, self._rng)
            if self.protocol == 1:
                self._messages = [m for m in self._messages if m.kind == "STEP"]
            self._next_message = 0
            self._next_rxstat = self._walk_start + 1.0
            self._write_plan()
            self._send("ACK,START")
        elif command.startswith("SET_WINDOW,"):
            value = self._int_arg(command)
            if value is not None and 0 < value <= MAX_WINDOW_SIZE:
                self.smoothing_window = value
                self.update_stride = min(self.update_stride, value)
                self._send(f"ACK,WINDOW,{value}")
        elif command.startswith("SET_STRIDE,"):
            value = self._int_arg(command)
            if value is not None and 0 < value <= self.smoothing_window:
                self.update_stride = value
                self._send(f"ACK,STRIDE,{value}")
        elif command.startswith("CAL_WEIGHT"):
            self._send("ACK,CAL_WEIGHT")
            self._send("ACK,CAL_WEIGHT,DISPATCHED")
            self._send("ACK,CAL_WEIGHT,1000")  # threshold reported back by the sender
        elif self.protocol == 2:
            self._handle_v2_command(command)

    def _handle_v2_command(self, command: str):
        if command == "VERSION":
            self._send(f"ACK,VERSION,{FIRMWARE_VERSION}")
        elif command.startswith("PING,"):
            ping_id = self._int_arg(command)
            if ping_id is not None:
                self._send(f"PONG,{ping_id},{self._clock.t_us(self._now())}")
        elif command.startswith("IMU_STREAM,"):
            value = self._int_arg(command)
            if value in (0, 1):
                self.imu_streaming = bool(value)
                self._send(f"ACK,IMU_STREAM,{value}")

    def _write_plan(self):
        """For utils/analysis/live_view.py: every future heel strike in PC time, written at START
        to the file named by BRAINSYNC_SIM_PLAN so the viewer can flash the feet exactly on time."""
        path = os.environ.get(PLAN_ENV)
        if not path:
            return
        tmp = Path(path).with_suffix(".tmp")
        with open(tmp, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(["step_index", "foot", "hs_pc_t"])
            for s in self.legs.strikes:
                writer.writerow([s.index, s.foot, f"{self._boot + self._walk_start + s.hs:.6f}"])
        os.replace(tmp, path)

    @staticmethod
    def _int_arg(command: str):
        try:
            return int(command.split(",", 1)[1])
        except (IndexError, ValueError):
            return None

    def _emit_due(self):
        if self._walk_start is None:
            return
        with self._emit_lock:
            self._emit_due_locked()

    def _emit_due_locked(self):
        now = self._now()
        while self._next_message < len(self._messages):
            msg = self._messages[self._next_message]
            if msg.arrive_s > now:
                break
            self._next_message += 1
            if msg.lost:
                continue
            self._rx_count += 1
            if msg.kind == "STEP":
                self._send(self._step_line(msg))
                self.emitted_steps.append((msg.step["strike"], self._time()))
            elif msg.kind == "IMU":
                if self.imu_streaming:
                    for line in msg.lines:
                        self._send(line)
                        self.emitted_imu.append((self._time(), line))
            else:
                for line in msg.lines:
                    self._send(line)
        if self.protocol == 2 and now >= self._next_rxstat:
            self._next_rxstat += 1.0
            self._send(f"RXSTAT,{self._rx_count},0,0,0")

    def _step_line(self, msg) -> str:
        step = msg.step
        if self._session_started:
            # First step after START: the receiver ignores its interval
            self._session_started = False
            interval_ms, bpm = 0, 0.0
        else:
            interval_ms = step["interval_ms"]
            self._intervals_ms.append(interval_ms)
            self._step_counter += 1
            if self._step_counter % self.update_stride == 0:
                recent = list(self._intervals_ms)[-self.smoothing_window:]
                avg = sum(recent) / len(recent)
                self._last_bpm = 60000.0 / avg if avg else 0.0
            bpm = self._last_bpm
        instant_bpm = 60000.0 / interval_ms if interval_ms > 0 else 0.0
        if self.protocol == 1:
            return f"{int(self._now() * 1000)}, {step['foot']}, {instant_bpm:.2f}, {bpm:.2f}"
        return (f"STEP,{msg.seq},{step['t_us']},{step['foot']},{interval_ms},{step['fsr_value']},"
                f"{instant_bpm:.2f},{bpm:.2f}")
