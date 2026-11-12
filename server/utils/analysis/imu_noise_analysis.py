"""
IMU noise analysis: is the weak gyro signal coming from the sensor, the mounting or our code?

Test 3 (this step) - while walking, compare the total rotation |w| with the pitch rate our
calibration measures, inside each FSR swing window (lift -> next strike of the same foot).

Works on any recorded session folder (old or new recordings).

Usage (from server/):
    python utils/analysis/imu_noise_analysis.py logs/STRAVA/session_*
"""
import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd

GYR_LSB_PER_DPS = 16.0    # BNO055 gyro output in dps units
ACC_LSB_PER_MS2 = 100.0   # BNO055 accel output in m/s^2 units
IMUS = {"l": "left", "r": "right"}   # imu_data.csv column prefix -> IMU label in meta.json
FEET = {1: "right", 2: "left"}       # FSR foot id -> foot
MAX_SWING_S = 1.0  # lift -> next strike longer than this is a stop or a turn, not a swing


def load(folder: Path):
    imu = pd.read_csv(folder / "imu_data.csv")
    t = imu["sender_t_us"].to_numpy() / 1e6
    data = {}
    for p, name in IMUS.items():
        ok = imu[f"valid_{p}"].to_numpy() == 1
        gyr = imu[[f"{p}_gx", f"{p}_gy", f"{p}_gz"]].to_numpy(float) / GYR_LSB_PER_DPS
        acc = imu[[f"{p}_ax", f"{p}_ay", f"{p}_az"]].to_numpy(float) / ACC_LSB_PER_MS2
        data[name] = {"t": t[ok], "gyr": gyr[ok], "acc": acc[ok]}

    calib = json.loads((folder / "meta.json").read_text())["strike_tracking"]["calibration"]

    strikes = pd.read_csv(folder / "strikes.csv")
    events = pd.read_csv(folder / "gait_events.csv")
    lifts = events[events["event"] == "LIFT"]
    swings, dropped = {}, {}
    for fid, foot in FEET.items():
        s = np.sort(strikes.loc[strikes["foot"] == fid, "sender_t_us"].to_numpy() / 1e6)
        windows, n_drop = [], 0
        for lift in lifts.loc[lifts["foot"] == fid, "sender_s"].to_numpy():
            nxt = s[s > lift]
            if len(nxt) and nxt[0] - lift <= MAX_SWING_S:
                windows.append((lift, nxt[0]))
            else:
                n_drop += 1
        swings[foot], dropped[foot] = windows, n_drop
    return data, calib, swings, dropped


def swing_rotation(data, calib, swings) -> pd.DataFrame:
    """Per IMU, per FSR foot whose swings we look at: median peaks and the axis angle."""
    rows = []
    for imu, d in data.items():
        bias = np.array(calib[imu]["gyro_bias_dps"])
        axis = np.array(calib[imu]["pitch_axis"])  # unit vector chosen by calibration.py
        g = d["gyr"] - bias
        for foot, windows in swings.items():
            total, pitch, samples = [], [], []
            for a, b in windows:
                m = (d["t"] >= a) & (d["t"] <= b)
                if m.sum() < 5:
                    continue
                w = g[m]
                total.append(np.linalg.norm(w, axis=1).max())  # |w|: rotation about any axis
                pitch.append(np.abs(w @ axis).max())           # what our code sees
                samples.append(w)
            if not total:
                continue
            allw = np.vstack(samples)
            main_axis = np.linalg.eigh(allw.T @ allw)[1][:, -1]  # dominant axis in these swings
            angle = np.degrees(np.arccos(min(1.0, abs(float(main_axis @ axis)))))
            rows.append({
                "imu": imu, "swings_of": foot, "n": len(total),
                "peak_total_dps": np.median(total),
                "peak_pitch_dps": np.median(pitch),
                "pitch/total": np.median(np.array(pitch) / np.array(total)),
                "axis_angle_deg": angle,
            })
    return pd.DataFrame(rows)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("sessions", nargs="+", type=Path)
    args = ap.parse_args()
    for folder in args.sessions:
        data, calib, swings, dropped = load(folder)
        print(f"\n== {folder.name}  swings used: right {len(swings['right'])}, left {len(swings['left'])}"
              f"  (dropped >{MAX_SWING_S}s: right {dropped['right']}, left {dropped['left']})")
        print(swing_rotation(data, calib, swings).round(2).to_string(index=False))


if __name__ == "__main__":
    main()
