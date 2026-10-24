"""
Watches the sensor stream during a session and logs plain warnings for problems that are
easy to miss on a first hardware test: an IMU that sends nothing, a foot that does not
calibrate, IMU bus errors, lost messages, errors reported by the receiver.

Each problem is logged once when it starts and once when it is gone.
"""
from collections import deque

from ..hardware.sensor_stream import ImuMsg, StatusMsg, OtherMsg
from ..prediction_model.strike_predictor import RIGHT, LEFT

FOOT_NAMES = {RIGHT: "right", LEFT: "left"}
IMU_SILENT_S = 1.0  # walking without IMU samples from a foot for this long = problem
WALKING_S = 3.0  # a step within this = walking
CALIBRATION_STEPS = 20  # a foot should be calibrated after this many steps
LOSS_WINDOW_S = 10.0
LOSS_WARN = 0.02  # more than 2 % of the messages lost in a window
BUS_ERROR_WINDOW_S = 10.0
BUS_ERRORS_WARN = 3  # a few bus errors happen; more than this in the window = problem


class SensorHealth:
    def __init__(self, logger, tracker, stream, protocol: int):
        self.logger = logger
        self.tracker = tracker
        self.stream = stream
        self.protocol = protocol
        self.problems = set()
        self.history = []  # (pc time, message), for meta.json
        self._last_imu = {RIGHT: None, LEFT: None}
        self._last_step = None
        self._steps = 0
        self._calibrated = {RIGHT: False, LEFT: False}
        self._bus_errors = None
        self._bus_error_times = {RIGHT: deque(), LEFT: deque()}
        self._errors_seen = set()
        self._next_check = 0.0
        self._loss_mark = None  # (pc time, received, lost) at the start of the loss window

    # ----------- Inputs -----------
    def on_message(self, msg, arrival: float):
        if isinstance(msg, ImuMsg):
            if msg.valid_right:
                self._last_imu[RIGHT] = arrival
            if msg.valid_left:
                self._last_imu[LEFT] = arrival
        elif isinstance(msg, StatusMsg):
            self._on_status(msg, arrival)
        elif isinstance(msg, OtherMsg) and msg.kind == "ERR":
            text = ",".join(msg.fields)
            if text not in self._errors_seen:
                self._errors_seen.add(text)
                self._note(arrival, f"Receiver reported an error: ERR,{text}")

    def on_step(self, arrival: float):
        self._steps += 1
        self._last_step = arrival

    def _on_status(self, msg: StatusMsg, now: float):
        errors = {RIGHT: msg.i2c_err_right, LEFT: msg.i2c_err_left}
        if self._bus_errors is not None:
            for foot, name in FOOT_NAMES.items():
                recent = self._bus_error_times[foot]
                recent.extend([now] * max(errors[foot] - self._bus_errors[foot], 0))
                while recent and recent[0] < now - BUS_ERROR_WINDOW_S:
                    recent.popleft()
                self._set(now, f"bus_{foot}", len(recent) > BUS_ERRORS_WARN,
                          f"Frequent IMU bus errors on the {name} foot ({errors[foot]} so far): check its wiring",
                          f"IMU bus errors on the {name} foot have stopped")
        self._bus_errors = errors

    # ----------- Periodic check (main loop) -----------
    def check(self, now: float):
        if now < self._next_check or self.protocol < 2:
            return
        self._next_check = now + 1.0
        walking = self._last_step is not None and now - self._last_step < WALKING_S
        for foot, name in FOOT_NAMES.items():
            last = self._last_imu[foot]
            silent = walking and (last is None or now - last > IMU_SILENT_S)
            self._set(now, f"imu_{foot}", silent, f"No data from the {name} IMU: check its wiring and power",
                      f"The {name} IMU sends data again")
            calib = self.tracker.feet[foot].calib
            if calib.ready and not self._calibrated[foot]:
                self._calibrated[foot] = True
                self._note(now, f"The {name} IMU is calibrated (after {self._steps} steps)")
            self._set(now, f"calib_{foot}", self._steps >= CALIBRATION_STEPS and not calib.ready,
                      f"The {name} IMU is not calibrated after {CALIBRATION_STEPS} steps: "
                      "check that it is fixed firmly to the foot")
        self._check_loss(now)

    def _check_loss(self, now: float):
        stats = self.stream.stats
        received, lost = sum(stats.received.values()), sum(stats.lost.values())
        if self._loss_mark is None:
            self._loss_mark = (now, received, lost)
            return
        t0, r0, l0 = self._loss_mark
        if now - t0 < LOSS_WINDOW_S:
            return
        new_lost, new_total = lost - l0, (received - r0) + (lost - l0)
        share = new_lost / new_total if new_total else 0.0
        self._set(now, "loss", share > LOSS_WARN,
                  f"{share:.0%} of the sensor messages lost in the last {LOSS_WINDOW_S:.0f} s: "
                  "move the receiver closer or check for interference", "Message loss is back to normal")
        self._loss_mark = (now, received, lost)

    # ----------- Helpers -----------
    def _set(self, now, key, bad, message, ok_message=None):
        if bad and key not in self.problems:
            self.problems.add(key)
            self._note(now, f"WARNING: {message}")
        elif not bad and key in self.problems:
            self.problems.discard(key)
            if ok_message:
                self._note(now, ok_message)

    def _note(self, now, message):
        self.history.append((round(now, 3), message))
        self.logger.log(f"SENSORS: {message}")

    def state(self) -> dict:
        return {"open_problems": sorted(self.problems), "log": self.history}
