"""
Replay a recorded session through the engine's gait event detector (utils/gait/events.py).

Feeds imu_data.csv and the FSR strikes (strikes.csv) to one FootTracker per foot, in the order
they arrived on the PC, the same way StrikeTracker does live, and collects the events it detects.

--check   compares them with the session's own gait_events.csv (only meaningful with the code
          the session was recorded with; it matched 100% before the Sabatini change).
--vs-fsr  compares the IMU events with the FSR: heel strike (HS) vs. FSR strike, heel-off (HO)
          vs. FSR lift (the FSR is under the heel only), and relative stance. Reference values, Sabatini et al. 2005 (IEEE TBME
          52(3)), §III-IV: HS -2 ms (95% CI -16..+12 ms) vs. footswitches, TO about 35 ms early,
          relative stance 54-60 %.
--swap    feeds the IMU labelled 'left' to the right foot's tracker and vice versa (the mapping
          the FSR swing windows show in imu_noise_analysis.py), without changing the engine.
--mount   which gyro axis and sign each tracker uses:
            fixed: the IMU calibration the session ran with (meta.json), else the current
                   imu_calibration.json; an IMU that is not there gets no pitch axis (no IMU events)
            meta:  recordings made before that. Axis and gyro bias = the ones the old calibration found
                   (meta.json), sign = the one that makes the foot rotate forward during its FSR
                   swings: the foot pitch rises from about -70 deg at toe-off to about +20 deg at
                   heel strike (Sabatini 2005, Fig. 4b), so the integral of the pitch rate over a
                   swing is positive.

Usage (from server/):
    python utils/analysis/replay_events.py logs/STRAVA/session_* --mount meta --vs-fsr
    python utils/analysis/replay_events.py logs/STRAVA/session_* --mount meta --swap --vs-fsr
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # server/
from utils.gait.calibration import load_calibration
from utils.gait.events import FootTracker
from utils.hardware.sensor_stream import ACC_LSB_PER_MS2, GYR_LSB_PER_DPS

RIGHT, LEFT = 1, 2
FOOT_NAME = {RIGHT: "right", LEFT: "left"}
IMU_LABEL = {"r": "right", "l": "left"}  # imu_data.csv column prefix -> IMU label in meta.json
MATCH_S = 0.015  # --check: a replayed event within 1.5 samples of a logged one is the same event
NEAREST_S = 0.15  # --vs-fsr: an IMU event is paired with an FSR event of the same foot within this
                  # (well under the ~0.5 s between the two feet's events)
MAX_SWING_S = 1.0  # lift -> next strike longer than this is a stop, not a swing


def fsr_swings(folder: Path, foot: int) -> list:
    strikes = pd.read_csv(folder / "strikes.csv")
    events = pd.read_csv(folder / "gait_events.csv")
    s = np.sort(strikes.loc[strikes["foot"] == foot, "sender_t_us"].to_numpy() / 1e6)
    lifts = events.loc[(events["event"] == "LIFT") & (events["foot"] == foot), "sender_s"].to_numpy()
    out = []
    for lift in lifts:
        nxt = s[s > lift]
        if len(nxt) and nxt[0] - lift <= MAX_SWING_S:
            out.append((lift, nxt[0]))
    return out


def meta_mount(folder: Path, foot: int, prefix: str, t: np.ndarray, gyr: np.ndarray):
    """Axis from the old calibration (meta.json), sign from the swings of the foot it is fed to."""
    calib = json.loads((folder / "meta.json").read_text())["strike_tracking"]["calibration"]
    c = calib[IMU_LABEL[prefix]]
    axis = np.array(c["pitch_axis"], float)
    rate = (gyr - np.array(c["gyro_bias_dps"])) @ axis
    integrals = [rate[(t >= a) & (t <= b)].sum() * 0.01 for a, b in fsr_swings(folder, foot)]
    sign = 1 if np.median(integrals) > 0 else -1
    return axis, sign, float(np.median(integrals)) * sign, np.array(c["gyro_bias_dps"], float)


def replay(folder: Path, swap: bool = False, mount: str = "fixed"):
    imu = pd.read_csv(folder / "imu_data.csv")
    strikes = pd.read_csv(folder / "strikes.csv")
    t = imu["sender_t_us"].to_numpy() / 1e6
    # live (strike_tracking.py): columns l_* -> LEFT tracker, r_* -> RIGHT tracker
    source = {RIGHT: "l", LEFT: "r"} if swap else {RIGHT: "r", LEFT: "l"}
    data, feet, mounts = {}, {}, {}
    meta_file = folder / "meta.json"
    calibration = (json.loads(meta_file.read_text(encoding="utf-8")).get("imu_calibration")
                   if meta_file.exists() else None) or load_calibration() or {}
    for foot, p in source.items():
        gyr = imu[[f"{p}_gx", f"{p}_gy", f"{p}_gz"]].to_numpy(float) / GYR_LSB_PER_DPS
        data[foot] = (imu[f"valid_{p}"].to_numpy() == 1,
                      imu[[f"{p}_ax", f"{p}_ay", f"{p}_az"]].to_numpy(float) / ACC_LSB_PER_MS2, gyr)
        if mount == "meta":
            axis, sign, swing_deg, bias = meta_mount(folder, foot, p, t, gyr)
            feet[foot] = FootTracker(foot, axis=axis, sign=sign)
            # These recordings never stood still for 0.5 s (no window passes STILL_STD_DPS), so
            # use the bias the old calibration measured, marked as accepted.
            feet[foot].calib.gyro_bias, feet[foot].calib._bias_ok = bias, True
            mounts[foot] = (IMU_LABEL[p], sign, swing_deg)
        else:
            measured = calibration.get("imus", {}).get(IMU_LABEL[p])  # by IMU, so --swap keeps each IMU's axis
            feet[foot] = (FootTracker(foot, axis=measured["pitch_axis"], sign=+1, up=measured.get("up"))
                          if measured else FootTracker(foot))
            mounts[foot] = (IMU_LABEL[p], feet[foot].calib.sign, None)
    # Live, a strike reaches the tracker when its message arrives on the PC, usually before the IMU
    # batch holding the last few samples before it (median 3 samples). Use the PC arrival times
    # (pc_t in both files) for that order.
    arrival = imu["pc_t"].to_numpy()
    pending = sorted(zip(strikes["pc_t"].to_numpy(), strikes["sender_t_us"].to_numpy() / 1e6,
                         strikes["foot"].to_numpy()))

    events = []
    for i, ti in enumerate(t):
        while pending and pending[0][0] <= arrival[i]:  # strikes reach the trackers in arrival order
            _, ts, foot = pending.pop(0)
            feet[foot].on_strike(ts)
        for foot in (LEFT, RIGHT):  # same order as StrikeTracker.on_imu
            valid, acc, gyr = data[foot]
            if valid[i]:
                for e in feet[foot].add_sample(ti, acc[i], gyr[i]):
                    events.append((e.sender_s, e.foot, e.kind, e.value_dps))
    return pd.DataFrame(events, columns=["sender_s", "foot", "event", "value_dps"]), feet, mounts


def check(folder: Path, replayed: pd.DataFrame) -> pd.DataFrame:
    """Per foot and event type: logged vs. replayed counts, how many match, median time difference."""
    logged = pd.read_csv(folder / "gait_events.csv")
    logged = logged[logged["event"].isin(["TO", "MS"])]
    rows = []
    for (foot, kind), lg in logged.groupby(["foot", "event"]):
        rp = replayed.loc[(replayed["foot"] == foot) & (replayed["event"] == kind), "sender_s"].to_numpy()
        diffs = []
        for ts in lg["sender_s"].to_numpy():
            if len(rp):
                d = rp[np.abs(rp - ts).argmin()] - ts
                if abs(d) <= MATCH_S:
                    diffs.append(d)
        rows.append({"foot": foot, "event": kind, "logged": len(lg), "replayed": len(rp),
                     "matched": len(diffs),
                     "median_diff_ms": round(float(np.median(diffs)) * 1000, 1) if diffs else None})
    return pd.DataFrame(rows)


def _offsets_ms(imu_t: np.ndarray, ref: np.ndarray) -> np.ndarray:
    d = []
    for x in imu_t:
        if len(ref):
            k = ref[np.abs(ref - x).argmin()]
            if abs(x - k) <= NEAREST_S:
                d.append((x - k) * 1000)
    return np.array(d)


def _relative_stance(hs: np.ndarray, to: np.ndarray) -> list:
    """Sabatini Eq. 4-7: RS = stance / stride * 100, stride = HS(j+1) - HS(j), swing = HS(j+1) - TO(j)."""
    out = []
    for a, b in zip(hs[:-1], hs[1:]):
        if b - a > 2 * MAX_SWING_S:
            continue
        between = to[(to > a) & (to < b)]
        if len(between) == 1:
            out.append(100 * (1 - (b - between[0]) / (b - a)))
    return out


def vs_fsr(folder: Path, events: pd.DataFrame) -> pd.DataFrame:
    strikes = pd.read_csv(folder / "strikes.csv")
    logged = pd.read_csv(folder / "gait_events.csv")
    rows = []
    for foot in (RIGHT, LEFT):
        fsr_hs = np.sort(strikes.loc[strikes["foot"] == foot, "sender_t_us"].to_numpy() / 1e6)
        fsr_lift = np.sort(logged.loc[(logged["event"] == "LIFT") & (logged["foot"] == foot), "sender_s"].to_numpy())
        ev = events[events["foot"] == foot]
        imu_hs = np.sort(ev.loc[ev["event"] == "HS", "sender_s"].to_numpy())
        imu_to = np.sort(ev.loc[ev["event"] == "TO", "sender_s"].to_numpy())
        imu_ho = np.sort(ev.loc[ev["event"] == "HO", "sender_s"].to_numpy())
        hs = _offsets_ms(imu_hs, fsr_hs)
        ho = _offsets_ms(imu_ho, fsr_lift)  # FSR under the heel only: its lift is a heel-off
        rs = _relative_stance(imu_hs, imu_to)
        pct = lambda a, q: round(float(np.percentile(a, q)), 1) if len(a) else None
        rows.append({
            "foot": FOOT_NAME[foot], "FSR strikes": len(fsr_hs),
            "IMU HS": len(imu_hs), "HS paired": len(hs),
            "HS-FSR median ms": pct(hs, 50), "HS 2.5%": pct(hs, 2.5), "HS 97.5%": pct(hs, 97.5),
            "IMU HO": len(imu_ho), "HO paired": len(ho), "HO-lift median ms": pct(ho, 50),
            "rel. stance %": pct(np.array(rs), 50),
        })
    return pd.DataFrame(rows)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("sessions", nargs="+", type=Path)
    ap.add_argument("--swap", action="store_true", help="feed each IMU to the opposite foot")
    ap.add_argument("--mount", choices=["fixed", "meta"], default="fixed")
    ap.add_argument("--check", action="store_true", help="compare with the logged gait_events.csv")
    ap.add_argument("--vs-fsr", action="store_true", help="compare the IMU events with the FSR")
    ap.add_argument("--out", type=Path, help="folder for <session>_events.csv")
    args = ap.parse_args()
    if args.out:
        args.out.mkdir(parents=True, exist_ok=True)
    for folder in args.sessions:
        events, feet, mounts = replay(folder, swap=args.swap, mount=args.mount)
        counts = events.groupby(["foot", "event"]).size().to_dict()
        print(f"\n== {folder.name}{'  (swapped)' if args.swap else ''}  mount={args.mount}  events: {counts}")
        for foot in (RIGHT, LEFT):
            label, sign, swing_deg = mounts[foot]
            extra = f", median swing rotation {swing_deg:+.0f} deg" if swing_deg is not None else ""
            bias_std = feet[foot].calib.state()["bias_std_dps"]
            print(f"   {FOOT_NAME[foot]} foot <- IMU '{label}', sign {sign:+d}{extra}, "
                  f"bias {feet[foot].calib.gyro_bias.round(2).tolist()} dps, spread {bias_std} dps")
        if args.check:
            print(check(folder, events).to_string(index=False))
        if args.vs_fsr:
            print(vs_fsr(folder, events).to_string(index=False))
        if args.out:
            events.to_csv(args.out / f"{folder.name}_events.csv", index=False)


if __name__ == "__main__":
    main()
