"""
Predicting the next landing: our three clues next to the method of Zank et al., on recorded sessions.

    python utils/analysis/prediction_comparison.py logs/<folder>/<session> [more sessions] [--out DIR]

Needs sessions recorded after an IMU calibration (it uses each IMU's forward and up directions).

For every landing (pressure-pad strike) and every method it measures
    lead time   how long before the landing the prediction became available
    error       predicted landing - real landing

Ours, exactly as the engine runs it (utils/prediction_model/strike_predictor.py):
    B0 rhythm, B1 toe-off, B2 mid-swing.

Zank, Nescher, Kunz, "Robust Prediction of Auditory Step Feedback for Forward Walking", VRST 2013.
Four events per step, all zero crossings (their Fig. 3; their rotation sign is the opposite of ours):
    1  the foot's rotation crosses zero, the swing starts  (the moment our detector reports TO)
    2  the forward acceleration crosses zero, going negative
    3  the upward acceleration crosses zero, going negative
    4  the foot's rotation crosses zero, the swing ends    (the moment our detector reports MS)
The time left to the landing is a polynomial in the time between two events (their Eq. 1, Table 2):
    Z-I   events 2 and 4, quadratic        Z-III  events 1 and 2, quadratic
    Z-II  events 1 and 3, linear           Z-IV   events 1 and 4, quadratic (gyro only)
Their coefficients do not carry over (their target is the step sound, ours the pressure pad), so
they are fitted here by least squares (their Eq. 2), per foot, in two ways:
    live  on the earlier steps of the session only: what a running system could do
    loo   leave-one-out over the session: comparable to their cross-validation

OURS, not from the paper:
- The accelerations are taken along the calibrated forward and up directions of the foot, with
  gravity subtracted from up, and smoothed like the pitch rate. The paper does not say in which
  frame its accelerations are.
- Event 2 is the first such crossing after event 1, event 3 the first after event 2, while the
  detector is in the swing. The paper does not say how it picks the crossing.
- MIN_FIT, the number of earlier steps the live fit waits for.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # server/
from utils.gait.calibration import G, IMU_OF_FOOT, load_calibration
from utils.gait.events import FILTER_ALPHA, LANDING, PUSH_OFF, SWING, FootTracker
from utils.hardware.sensor_stream import ACC_LSB_PER_MS2, GYR_LSB_PER_DPS
from utils.prediction_model.strike_predictor import LEFT, RIGHT, StrikePredictor

FOOT_NAME = {RIGHT: "right", LEFT: "left"}
PREFIX = {"right": "r", "left": "l"}  # IMU label -> column prefix in imu_data.csv
ZANK = {"Z-I": (2, 4, 2), "Z-II": (1, 3, 1), "Z-III": (1, 2, 2), "Z-IV": (1, 4, 2)}  # first event, last event, degree
MIN_FIT = 10  # OURS: earlier steps of the same foot before the live fit is used
MAX_LEAD_S = 1.5  # a "prediction" made longer before the landing than this belongs to a stop, not a step


class ZankEvents:
    """Events 1-4 of one foot, found sample by sample (causal)."""

    def __init__(self, tracker: FootTracker, up: np.ndarray, forward: np.ndarray):
        self.tracker, self.up, self.forward = tracker, up, forward
        self.times = {}  # event number -> time, of the current swing
        self._fwd = self._up = None

    def add_sample(self, t: float, acc: np.ndarray, state_before: str):
        fwd, up = float(acc @ self.forward), float(acc @ self.up) - G
        if self._fwd is not None:  # same one-pole smoothing as the pitch rate
            fwd = self._fwd + FILTER_ALPHA * (fwd - self._fwd)
            up = self._up + FILTER_ALPHA * (up - self._up)
        state = self.tracker.state
        if state_before == PUSH_OFF and state == SWING:
            self.times = {1: t}
        elif state_before == SWING and state == LANDING:
            self.times.setdefault(4, t)
        elif state == SWING and 1 in self.times and self._fwd is not None:
            if 2 not in self.times and self._fwd > 0 >= fwd:
                self.times[2] = t
            if 2 in self.times and 3 not in self.times and self._up > 0 >= up:
                self.times[3] = t
        self._fwd, self._up = fwd, up

    def take(self) -> dict:
        times, self.times = self.times, {}
        return times


def directions(imu: dict) -> tuple:
    """(pitch axis, up, forward) in the sensor's axes. pitch = forward x up, so forward = up x pitch."""
    pitch, up = np.asarray(imu["pitch_axis"], float), np.asarray(imu["up"], float)
    forward = np.cross(up, pitch)
    return pitch, up, forward / np.linalg.norm(forward)


def run_session(folder: Path) -> tuple[pd.DataFrame, dict]:
    """Replays the session. Returns one row per (landing, method) and, per foot, the Zank event times."""
    meta_file = folder / "meta.json"
    calibration = (json.loads(meta_file.read_text(encoding="utf-8")).get("imu_calibration")
                   if meta_file.exists() else None) or load_calibration()
    imus = (calibration or {}).get("imus", {})
    if any(IMU_OF_FOOT[foot] not in imus or "up" not in imus[IMU_OF_FOOT[foot]] for foot in FOOT_NAME):
        sys.exit(f"{folder.name}: no IMU calibration with 'up' for both IMUs. Record a calibration walk first.")

    imu = pd.read_csv(folder / "imu_data.csv")
    strikes = pd.read_csv(folder / "strikes.csv").sort_values("sender_t_us")
    t_imu = imu["sender_t_us"].to_numpy() / 1e6
    feet, zank, data = {}, {}, {}
    for foot, label in IMU_OF_FOOT.items():
        pitch, up, forward = directions(imus[label])
        feet[foot] = FootTracker(foot, axis=pitch, sign=+1)
        zank[foot] = ZankEvents(feet[foot], up, forward)
        p = PREFIX[label]
        data[foot] = (imu[f"valid_{p}"].to_numpy() == 1,
                      imu[[f"{p}_ax", f"{p}_ay", f"{p}_az"]].to_numpy(float) / ACC_LSB_PER_MS2,
                      imu[[f"{p}_gx", f"{p}_gy", f"{p}_gz"]].to_numpy(float) / GYR_LSB_PER_DPS)

    predictor = StrikePredictor()
    pending = {}  # (foot, strike number) -> {method: [time first available, latest estimate]}
    rows, zank_steps = [], {foot: [] for foot in FOOT_NAME}
    strike_list = list(zip(strikes["sender_t_us"].to_numpy() / 1e6, strikes["foot"].to_numpy()))
    next_strike = 0
    for i, t in enumerate(t_imu):
        while next_strike < len(strike_list) and strike_list[next_strike][0] <= t:
            ts, foot = strike_list[next_strike]
            next_strike += 1
            foot = int(foot)
            for method, (first_t, estimate) in pending.pop((foot, predictor.strike_count), {}).items():
                rows.append((folder.name, FOOT_NAME[foot], ts, method, (ts - first_t) * 1000, (estimate - ts) * 1000))
            zank_steps[foot].append((zank[foot].take(), ts))
            feet[foot].on_strike(ts)
            predictor.on_strike(foot, ts)
        for foot, (valid, acc, gyr) in data.items():
            if not valid[i]:
                continue
            before = feet[foot].state
            for event in feet[foot].add_sample(t, acc[i], gyr[i]):
                predictor.on_event(event.foot, event.kind, event.sender_s)
            if feet[foot].calib.ready:
                zank[foot].add_sample(t, acc[i], before)
        prediction = predictor.predict(t)
        if prediction is not None:
            made = pending.setdefault((prediction.foot, prediction.strike_number), {})
            for method, (estimate, _) in prediction.estimates.items():
                made.setdefault(method, [t, estimate])[1] = estimate
    for foot, steps in zank_steps.items():
        rows += zank_rows(folder.name, FOOT_NAME[foot], steps)
    table = pd.DataFrame(rows, columns=["session", "foot", "strike_s", "method", "lead_ms", "error_ms"])
    return table[(table.lead_ms > 0) & (table.lead_ms < MAX_LEAD_S * 1000)], zank_steps


def zank_rows(session: str, foot: str, steps: list) -> list:
    rows = []
    for name, (a, b, degree) in ZANK.items():
        ok = [(times[b] - times[a], ts - times[b], ts) for times, ts in steps
              if a in times and b in times and times[b] > times[a] and 0 < ts - times[b] < MAX_LEAD_S]
        if len(ok) <= degree + 1:
            continue
        dt, left, ts = (np.array(x) for x in zip(*ok))
        for k in range(len(ok)):
            for mode, train in (("live", np.arange(len(ok)) < k), ("loo", np.arange(len(ok)) != k)):
                if train.sum() < max(MIN_FIT, degree + 2):
                    continue
                guess = np.polyval(np.polyfit(dt[train], left[train], degree), dt[k])
                rows.append((session, foot, ts[k], f"{name} {mode}", left[k] * 1000, (guess - left[k]) * 1000))
    return rows


def summary(table: pd.DataFrame) -> pd.DataFrame:
    out = table.groupby("method").agg(steps=("error_ms", "size"), lead_ms=("lead_ms", "median"),
                                      error_mean_ms=("error_ms", "mean"), error_sd_ms=("error_ms", "std"))
    return out.sort_values("lead_ms", ascending=False).round(1)


def plot(result: pd.DataFrame, path: Path, title: str):
    fig, ax = plt.subplots(figsize=(7.5, 4.5))
    for method, row in result.iterrows():
        ours = method.startswith("B")
        ax.scatter(row.lead_ms, row.error_sd_ms, s=45, color="#2a78d6" if ours else "#eb6834",
                   marker="o" if ours else "s")
        ax.annotate(method, (row.lead_ms, row.error_sd_ms), textcoords="offset points", xytext=(6, 4), fontsize=8)
    ax.set_xlabel("lead time: how long before the landing the prediction is available (ms, median)")
    ax.set_ylabel("spread of the error (ms, standard deviation)")
    ax.set_title(title, fontsize=10)
    ax.grid(True, color="#e4e3df")
    ax.set_xlim(0, result.lead_ms.max() * 1.12)  # room for the labels
    ax.set_ylim(0, result.error_sd_ms.max() * 1.15)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("sessions", nargs="+", type=Path)
    parser.add_argument("--out", type=Path, default=None, help="folder for the csv and the graph (default: the session)")
    args = parser.parse_args()

    tables = []
    for folder in args.sessions:
        table, zank_steps = run_session(folder)
        tables.append(table)
        print(f"\n== {folder.name}")
        for foot, steps in zank_steps.items():
            found = {n: sum(n in times for times, _ in steps) for n in (1, 2, 3, 4)}
            print(f"   {FOOT_NAME[foot]} foot: {len(steps)} landings, Zank events found: "
                  + ", ".join(f"{n}: {found[n]}" for n in found))
        print(summary(table).to_string())
    everything = pd.concat(tables)
    out = args.out or args.sessions[0]
    out.mkdir(parents=True, exist_ok=True)
    everything.round(2).to_csv(out / "prediction_comparison.csv", index=False)
    result = summary(everything)
    if len(args.sessions) > 1:
        print(f"\n== all {len(args.sessions)} sessions")
        print(result.to_string())
    plot(result, out / "prediction_comparison.png",
         f"Predicting the landing: ours (blue) and Zank et al. 2013 (orange), {len(args.sessions)} session(s)")
    print(f"\nWritten: {out / 'prediction_comparison.csv'}, {out / 'prediction_comparison.png'}")


if __name__ == "__main__":
    main()
