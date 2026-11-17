"""
One graph of how the gait-event detector decides, on a recorded session: the foot's pitch rate
with the two 30 dps thresholds (Sabatini et al. 2005, Eq. 2-3) and the events they trigger, and the
accelerometer jump with the 9 m/s^2 landing threshold (events.py IMPACT_JUMP_MS2), against the FSR
strikes.

The session is replayed through the engine's FootTracker (as replay_events.py / score_predictions.py
do), with the IMU calibration saved in its meta.json.

Usage (from server/):
    python utils/analysis/decision_plot.py logs/STRAVA/session_2026-11-16_23-30-51 --foot right --at 150 --out decision_plot.png
--at: time in the session (s) where the 2.5 s window starts.
"""
import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # server/
from utils.analysis.score_predictions import session_calibration
from utils.gait.calibration import IMU_OF_FOOT, mounting_from, up_from
from utils.gait.events import FootTracker, HO_DPS, FF_DPS, IMPACT_JUMP_MS2
from utils.hardware.sensor_stream import ACC_LSB_PER_MS2, GYR_LSB_PER_DPS

FOOT = {"right": 1, "left": 2}
EVENT_STYLE = {"HO": ("#2a78d6", "heel-off"), "TO": ("#d62a2a", "toe-off"), "MS": ("#1baf7a", "mid-swing"),
               "HS": ("#6250d6", "heel strike (gyro)"), "LAND": ("#eb6834", "landing (accelerometer)")}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session", type=Path)
    ap.add_argument("--foot", choices=FOOT, default="right")
    ap.add_argument("--at", type=float, default=60.0)
    ap.add_argument("--span", type=float, default=2.5)
    ap.add_argument("--out", type=Path, default=Path("decision_plot.png"))
    args = ap.parse_args()
    foot = FOOT[args.foot]

    calibration = session_calibration(args.session, None)
    imu = pd.read_csv(args.session / "imu_data.csv")
    strikes = pd.read_csv(args.session / "strikes.csv")
    p = IMU_OF_FOOT[foot][0]
    t = imu["sender_t_us"].to_numpy() / 1e6
    valid = imu[f"valid_{p}"].to_numpy() == 1
    acc = imu[[f"{p}_ax", f"{p}_ay", f"{p}_az"]].to_numpy(float) / ACC_LSB_PER_MS2
    gyr = imu[[f"{p}_gx", f"{p}_gy", f"{p}_gz"]].to_numpy(float) / GYR_LSB_PER_DPS
    tracker = FootTracker(foot, *mounting_from(calibration).get(foot, (None, None)), up=up_from(calibration).get(foot))

    rate, jump, events = np.full(len(t), np.nan), np.full(len(t), np.nan), []
    prev = None
    for i in range(len(t)):
        if not valid[i]:
            continue
        events += tracker.add_sample(t[i], acc[i], gyr[i])
        if tracker._filtered is not None:
            rate[i] = tracker._filtered
        if prev is not None:
            jump[i] = np.linalg.norm(acc[i] - prev)
        prev = acc[i]

    t0 = t[0] + args.at
    w = (t >= t0) & (t <= t0 + args.span)
    x = t[w] - t0
    fsr = strikes.loc[strikes["foot"] == foot, "sender_t_us"].to_numpy() / 1e6 - t0
    fsr = fsr[(fsr >= 0) & (fsr <= args.span)]

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(12, 7), sharex=True, gridspec_kw={"height_ratios": [3, 2]})
    a1.plot(x, rate[w], color="#333333", lw=1.4, label="pitch rate (filtered)")
    a1.axhline(HO_DPS, color="#888888", ls="--", lw=1)
    a1.axhline(-FF_DPS, color="#888888", ls="--", lw=1)
    a1.axhspan(-FF_DPS, HO_DPS, color="#cccccc", alpha=0.25, label=f"|rate| < {HO_DPS:.0f} °/s: foot flat")
    a2.plot(x, jump[w], color="#333333", lw=1.2, label="acceleration change from the previous sample")
    a2.axhline(IMPACT_JUMP_MS2, color="#888888", ls="--", lw=1, label=f"{IMPACT_JUMP_MS2:.0f} m/s² landing threshold")
    seen = set()
    for e in events:
        if e.foot != foot or e.kind not in EVENT_STYLE or not (t0 <= e.sender_s <= t0 + args.span):
            continue
        color, name = EVENT_STYLE[e.kind]
        ax = a2 if e.kind == "LAND" else a1
        y = IMPACT_JUMP_MS2 if e.kind == "LAND" else np.interp(e.sender_s - t0, x, rate[w])  # LAND: shown on its threshold
        ax.plot(e.sender_s - t0, y, "o", ms=9, color=color, label=None if e.kind in seen else name, zorder=5)
        seen.add(e.kind)
    for ax in (a1, a2):
        for k, s in enumerate(fsr):
            ax.axvline(s, color="black", lw=2, label="FSR heel strike (truth)" if (k == 0 and ax is a1) else None)
        ax.grid(alpha=0.3)
    a1.set_ylabel("pitch rate (°/s)\n+ = toes up")
    a2.set_ylabel("m/s²")
    a2.set_xlabel(f"time (s) from {args.at:.0f} s into the session")
    a1.set_title(f"{args.session.name}, {args.foot} foot: where each threshold triggers a decision")
    a1.legend(fontsize=8, ncol=3, loc="upper left")
    a2.legend(fontsize=8, loc="upper left")
    fig.tight_layout()
    fig.savefig(args.out, dpi=150)
    print(f"{args.out}")


if __name__ == "__main__":
    main()
