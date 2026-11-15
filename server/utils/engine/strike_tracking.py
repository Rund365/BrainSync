"""
Feet -> next strike, in PC time, from the IMU alone.

IMU samples go to one FootTracker per foot (calibration and gait events, events.py). Its
landings (LAND) teach the StrikePredictor and its other events feed the estimates; the
prediction is mapped from the sender clock to the PC clock. The IMU landing is the real
contact, so that is what is predicted.

FSR steps are not an input. on_step() only pairs each FSR step with its IMU landing for the
phase alignment (number and contact time) and measures how late the FSR fires, for analysis.
"""
from collections import deque
from dataclasses import dataclass

from ..gait.calibration import imu_of_foot
from ..gait.events import FootTracker
from ..hardware.sensor_stream import SensorStream, ImuMsg, StepMsg, imu_to_units
from ..prediction_model.strike_predictor import StrikePredictor, RIGHT, LEFT

MATCH_S = (-0.15, 0.05)  # an IMU landing this far from an FSR step (landing - FSR) is the same step


@dataclass
class NextStrike:
    pc_t: float  # predicted real contact, PC clock
    sigma_s: float
    foot: int
    number: int  # index of the strike (0 = first of the session)
    method: str


class StrikeTracker:
    def __init__(self, stream: SensorStream, sensor_logger=None, mounting: dict | None = None,
                 up: dict | None = None):
        """mounting: foot -> (axis, sign), up: foot -> "up", both from the IMU calibration file
        (gait.calibration.mounting_from / up_from). Without them there are no IMU gait events.
        up is also used for the "IMU moved" check (engine/sensor_health.py)."""
        self.stream = stream
        self.up = up or {}
        mounting = mounting or {}
        self.feet = {foot: FootTracker(foot, *mounting.get(foot, (None, None)), up=self.up.get(foot))
                     for foot in (RIGHT, LEFT)}
        self.predictor = StrikePredictor()
        self.log = sensor_logger
        self._last_logged = None
        self._landings = deque(maxlen=8)  # (number, foot, sender_s) of the latest IMU landings

    @property
    def calibrated(self) -> bool:
        return all(f.calib.ready for f in self.feet.values())

    def fsr_delay_s(self, foot: int) -> float:
        return self.feet[foot].fsr_delay_s or 0.0

    def on_imu(self, msg: ImuMsg):
        for foot in (LEFT, RIGHT):
            valid, raw = imu_of_foot(msg, foot)
            if not valid:
                continue
            acc, gyr = imu_to_units(raw)
            for event in self.feet[foot].add_sample(msg.sender_s, acc, gyr):
                if event.kind == "LAND":
                    self._landings.append((self.predictor.strike_count, event.foot, event.sender_s))
                    self.predictor.on_strike(event.foot, event.sender_s)
                else:
                    self.predictor.on_event(event.foot, event.kind, event.sender_s)
                if self.log:
                    self.log.row("gait_events", [f"{event.sender_s:.4f}", self._pc(event.sender_s), event.foot,
                                                 event.kind, f"{event.value_dps:.1f}"])

    def on_step(self, msg: StepMsg) -> tuple[int, float | None]:
        """FSR step -> (strike number, real contact in PC time or None if the clock is not ready).
        The number and contact come from the matching IMU landing. The FSR message usually arrives
        before the IMU samples of the impact, so without a match yet the step gets the number the
        coming landing will have and the FSR time minus its measured delay."""
        self.feet[msg.foot].on_strike(msg.sender_s)
        match = [(n, t) for n, f, t in self._landings
                 if f == msg.foot and MATCH_S[0] <= t - msg.sender_s <= MATCH_S[1]]
        if match:
            number, contact = match[-1]
        else:
            number, contact = self.predictor.strike_count, msg.sender_s - self.fsr_delay_s(msg.foot)
        contact_pc = self.stream.clock.to_pc(contact) if self.stream.clock.ready else None
        return number, contact_pc

    def next_strike(self, now_pc: float) -> NextStrike | None:
        clock = self.stream.clock
        if not clock.ready:
            return None
        now_sender = clock.to_sender(now_pc)
        pred = self.predictor.predict(now_sender)
        if pred is None:
            return None
        pc_t = clock.to_pc(pred.sender_s)
        self._log_prediction(now_pc, now_sender, pred, pc_t)
        return NextStrike(pc_t, pred.sigma_s, pred.foot, pred.strike_number, pred.method)

    def _pc(self, sender_s):
        return f"{self.stream.clock.to_pc(sender_s):.6f}" if self.stream.clock.ready else ""

    def _log_prediction(self, now_pc, now_sender, pred, pc_t):
        key = (pred.strike_number, round(pred.sender_s, 3), pred.method)
        if not self.log or key == self._last_logged:
            return
        self._last_logged = key
        est = {m: f"{v[0]:.4f}" for m, v in pred.estimates.items()}
        self.log.row("predictions", [f"{now_pc:.6f}", f"{now_sender:.4f}", pred.foot, pred.strike_number,
                                     f"{pred.sender_s:.4f}", f"{pc_t:.6f}", f"{pred.sigma_s * 1000:.1f}",
                                     pred.method, est.get("B0", ""), est.get("B1", ""), est.get("B2", "")])

    def state(self) -> dict:
        return {
            "calibration": {"right": self.feet[RIGHT].calib.state(), "left": self.feet[LEFT].calib.state()},
            "fsr_delay_ms": {"right": round(self.fsr_delay_s(RIGHT) * 1000, 1),
                             "left": round(self.fsr_delay_s(LEFT) * 1000, 1)},
            "predictor": self.predictor.state(),
            "clock": self.stream.clock.state(),
        }
