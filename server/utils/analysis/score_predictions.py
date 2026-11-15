"""
Score the strike predictor on recorded sessions. The algorithm gets the IMU only; the FSR strikes
are the ground truth.

Replays imu_data.csv through FootTracker + StrikePredictor (its landings teach the predictor, as in
StrikeTracker), asks for the next strike at every IMU sample (as the engine loop does), and for each
FSR strike of the chosen foot takes the prediction that was current `h` ms before it.
Error = predicted - FSR strike (ms, + = predicted late).

A prediction counts from when its IMU sample reached the PC, not from when it was measured: the
IMU samples come in batches over the radio, 20-40 ms late in the recordings. The PC arrival times
(pc_t) are put on the sender clock by the lower edge of (pc_t - sender time) over the session, which
the fastest messages reach; MIN_LINK_S is added for the radio and USB.

The IMU predicts the real contact, and the FSR fires a little after it, so the error has a steady
offset; its median is reported, and "within" counts the strikes within ±GOOD_MS of that offset.
The beat can still be moved until about 80 ms before the strike (phase_controller: latency_s 30 ms
+ t_min_s 50 ms), so that horizon is the one that counts.

The IMU axes come from a calibration walk: --calibration, or the file saved in the session's
meta.json. Without one there are no IMU gait events, only the rhythm (gait/calibration.py).

Usage (from server/):
    python utils/analysis/score_predictions.py logs/STRAVA/session_* --foot right
    python utils/analysis/score_predictions.py logs/STRAVA/session_* --foot right --rhythm-only
    python utils/analysis/score_predictions.py logs/STRAVA/session_* --foot right --events RISE,TOP
    python utils/analysis/score_predictions.py <simulated session> --calibration <imu_calibration.json>
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # server/
from utils.gait.calibration import IMU_OF_FOOT, mounting_from, up_from
from utils.gait.events import FootTracker
from utils.hardware.sensor_stream import ACC_LSB_PER_MS2, GYR_LSB_PER_DPS
from utils.prediction_model.strike_predictor import StrikePredictor

RIGHT, LEFT = 1, 2
FOOT = {"right": RIGHT, "left": LEFT}
HORIZONS_MS = (300, 200, 150, 100, 80)
MIN_LINK_S = 0.004  # fastest sender -> PC delay (simulation/link.py LinkProfile.min_delay_s)
GOOD_MS = 30  # "on the step" for the alignment numbers (compare.py uses the same band)
ALL_EVENTS = "HO,TO,MS,HS,RISE,FALL,TOP"


def arrival(imu: pd.DataFrame, strikes: pd.DataFrame) -> np.ndarray:
    """Sender-clock time at which each IMU sample reached the PC."""
    sender = np.concatenate([imu["sender_t_us"], strikes["sender_t_us"]]) / 1e6
    gap = np.concatenate([imu["pc_t"], strikes["pc_t"]]) - sender
    edges = np.arange(sender.min(), sender.max() + 10.0, 10.0)  # lowest gap per 10 s, a line through them
    bins = np.digitize(sender, edges)
    lows = [(sender[bins == b][np.argmin(gap[bins == b])], gap[bins == b].min()) for b in np.unique(bins)]
    x, y = np.array(lows).T
    line = np.polyfit(x, y, 1) if len(x) > 1 else (0.0, y[0])
    t = imu["sender_t_us"].to_numpy() / 1e6
    late = imu["pc_t"].to_numpy() - t - np.polyval(line, t)
    return t + np.maximum(late, 0.0) + MIN_LINK_S


def session_calibration(folder: Path, path: Path | None) -> dict | None:
    """The calibration file given, else the one saved with the session (meta.json), else None."""
    if path is not None:
        return json.loads(path.read_text(encoding="utf-8"))
    meta = folder / "meta.json"
    if meta.exists():
        return json.loads(meta.read_text(encoding="utf-8")).get("imu_calibration")
    return None


def score_session(folder: Path, foot: int, use: set, latency: bool = True,
                  calibration: dict | None = None) -> pd.DataFrame:
    mounting, up = mounting_from(calibration), up_from(calibration)
    imu = pd.read_csv(folder / "imu_data.csv")
    strikes = pd.read_csv(folder / "strikes.csv")
    t = imu["sender_t_us"].to_numpy() / 1e6
    now = arrival(imu, strikes) if latency else t
    source = {f: label[0] for f, label in IMU_OF_FOOT.items()}  # foot -> its IMU's column prefix
    data, feet = {}, {}
    for f, p in source.items():
        data[f] = (imu[f"valid_{p}"].to_numpy() == 1,
                   imu[[f"{p}_ax", f"{p}_ay", f"{p}_az"]].to_numpy(float) / ACC_LSB_PER_MS2,
                   imu[[f"{p}_gx", f"{p}_gy", f"{p}_gz"]].to_numpy(float) / GYR_LSB_PER_DPS)
        feet[f] = FootTracker(f, *mounting.get(f, (None, None)), up=up.get(f))
    predictor = StrikePredictor()
    preds = []  # (now, foot, predicted_s, method)
    for i, ti in enumerate(t):
        for f in (LEFT, RIGHT):  # same order as StrikeTracker.on_imu
            valid, acc, gyr = data[f]
            if not valid[i]:
                continue
            for e in feet[f].add_sample(ti, acc[i], gyr[i]):
                if e.kind == "LAND":
                    predictor.on_strike(e.foot, e.sender_s)
                elif e.kind in use:
                    predictor.on_event(e.foot, e.kind, e.sender_s)
        p = predictor.predict(ti)
        if p is not None:
            preds.append((now[i], p.foot, p.sender_s, p.method))
    P = pd.DataFrame(preds, columns=["now", "foot", "pred", "method"])
    actual = np.sort(strikes.loc[strikes["foot"] == foot, "sender_t_us"].to_numpy() / 1e6)
    rows = []
    for s in actual:
        row = {"strike": s}
        for h in HORIZONS_MS:
            q = P[(P["now"] <= s - h / 1000) & (P["now"] > s - 1.5) & (P["foot"] == foot)]
            if len(q):
                last = q.iloc[-1]  # the latest prediction before s - h
                if abs(last["pred"] - s) < 0.5:  # a guess for this strike, not a stale one
                    row[f"err_{h}"] = (last["pred"] - s) * 1000
                    row[f"method_{h}"] = last["method"]
        rows.append(row)
    return pd.DataFrame(rows)


def summary(df: pd.DataFrame) -> dict:
    out = {"strikes": len(df)}
    for h in HORIZONS_MS:
        e = df.get(f"err_{h}", pd.Series(dtype=float)).dropna().to_numpy()
        out[f"{h} ms: predicted"] = f"{100 * len(e) / max(len(df), 1):.0f}%"
        if len(e):
            off = np.median(e)
            out[f"{h} ms: offset / spread"] = f"{off:+.0f} / {np.std(e):.0f}"
            out[f"{h} ms: within ±{GOOD_MS}"] = f"{100 * np.mean(np.abs(e - off) <= GOOD_MS):.0f}%"
    m = df.get("method_80", pd.Series(dtype=object)).dropna()
    out["80 ms: method used"] = ", ".join(f"{k} {v}" for k, v in m.value_counts().items())
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("sessions", nargs="+", type=Path)
    ap.add_argument("--foot", choices=FOOT, default="right")
    ap.add_argument("--rhythm-only", action="store_true", help="feed only the landings: rhythm only (B0)")
    ap.add_argument("--events", default=ALL_EVENTS, help=f"event kinds fed to the predictor (default: {ALL_EVENTS})")
    ap.add_argument("--calibration", type=Path, default=None,
                    help="imu_calibration.json to use (default: the one in the session's meta.json, if any)")
    ap.add_argument("--no-latency", action="store_true", help="count predictions from when the sample was "
                                                                "measured, not from when it reached the PC")
    args = ap.parse_args()
    use = set() if args.rhythm_only else set(args.events.split(","))
    rows = []
    for folder in args.sessions:
        s = summary(score_session(folder, FOOT[args.foot], use, latency=not args.no_latency,
                                  calibration=session_calibration(folder, args.calibration)))
        rows.append({"session": folder.name.replace("session_", ""), **s})
    print(pd.DataFrame(rows).set_index("session").T.to_string())


if __name__ == "__main__":
    main()
