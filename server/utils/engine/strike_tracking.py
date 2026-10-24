"""
Feet -> next strike, in PC time.

IMU samples go to one FootTracker per foot (calibration + toe-off / mid-swing events),
FSR strikes teach the StrikePredictor, and the prediction is mapped from the sender
clock to the PC clock. The FSR fires a little after the real contact; that delay is
measured from the IMU impact and subtracted, so the target is the real contact.
"""
from dataclasses import dataclass

from ..gait.events import FootTracker
from ..hardware.sensor_stream import SensorStream, ImuMsg, StepMsg, imu_to_units
from ..prediction_model.strike_predictor import StrikePredictor, RIGHT, LEFT


@dataclass
class NextStrike:
    pc_t: float  # predicted real contact, PC clock
    sigma_s: float
    foot: int
    number: int  # index of the strike (0 = first of the session)
    method: str


class StrikeTracker:
    def __init__(self, stream: SensorStream, sensor_logger=None):
        self.stream = stream
        self.feet = {RIGHT: FootTracker(RIGHT), LEFT: FootTracker(LEFT)}
        self.predictor = StrikePredictor()
        self.log = sensor_logger
        self._last_logged = None

    @property
    def calibrated(self) -> bool:
        return all(f.calib.ready for f in self.feet.values())

    def fsr_delay_s(self, foot: int) -> float:
        return self.feet[foot].fsr_delay_s or 0.0

    def on_imu(self, msg: ImuMsg):
        for foot, raw, valid in ((LEFT, msg.left, msg.valid_left), (RIGHT, msg.right, msg.valid_right)):
            if not valid:
                continue
            acc, gyr = imu_to_units(raw)
            for event in self.feet[foot].add_sample(msg.sender_s, acc, gyr):
                self.predictor.on_event(event.foot, event.kind, event.sender_s)
                if self.log:
                    self.log.row("gait_events", [f"{event.sender_s:.4f}", self._pc(event.sender_s), event.foot,
                                                 event.kind, f"{event.value_dps:.1f}"])

    def on_step(self, msg: StepMsg) -> tuple[int, float | None]:
        """Returns (strike number, real contact in PC time or None if the clock is not ready)."""
        number = self.predictor.strike_count
        self.feet[msg.foot].on_strike(msg.sender_s)
        self.predictor.on_strike(msg.foot, msg.sender_s)
        contact_pc = None
        if self.stream.clock.ready:
            contact_pc = self.stream.clock.to_pc(msg.sender_s - self.fsr_delay_s(msg.foot))
        return number, contact_pc

    def next_strike(self, now_pc: float) -> NextStrike | None:
        clock = self.stream.clock
        if not clock.ready:
            return None
        now_sender = clock.to_sender(now_pc)
        pred = self.predictor.predict(now_sender)
        if pred is None:
            return None
        pc_t = clock.to_pc(pred.sender_s - self.fsr_delay_s(pred.foot))
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
