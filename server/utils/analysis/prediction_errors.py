"""
Where do the strike-prediction errors come from? (Tom, 16 Nov: "show on the raw data axes exactly
where the source of the error is".)

For every FSR heel strike (ground truth) the session is replayed through the engine exactly as
score_predictions.py does (IMU only), and the prediction that was current 80 ms before the strike
is compared with it. 80 ms is the last moment the beat can still be moved (phase_controller:
latency 30 ms + t_min 50 ms). The FSR fires a little after the real contact, so the error has a
steady offset per session and foot; the median is removed. A prediction more than GOOD_MS from
that, or no prediction at all, is an error.

Each error gets ONE cause, the first of these that applies (all thresholds here are OURS, they
only sort the errors, they are not part of the algorithm):
  1 data lost        IMU samples missing (radio) or invalid (I2C bus) in the second before the strike
  2 missed landing   the IMU did not report the landing of the step before (the other foot)
  3 false landing    the IMU reported a landing of this foot, FALSE_WINDOW_S before the strike, that
                     no FSR strike confirms
  4 pace changed     the stride ending at this strike is more than PACE_CHANGE longer or shorter
                     than the stride before it
  5 other            none of the above

Two reference numbers are printed next to ours:
  - rhythm from the FSR alone: predict each strike as the strike before + the step time two steps
    back (same pair of feet). No IMU, perfect step detection. What is left is how much the walker's
    own steps vary.
  - IMU landing - FSR strike, per foot: how exactly the IMU finds the landing.

Outputs (in --out):
  errors.csv            one row per FSR strike: error, method, cause
  fig_causes.png        errors by cause, per session
  fig_over_time_*.png   error of every step over the session, with the music tempo
  fig_example_*.png     raw gyro and accelerometer (x, y, z) around example steps (a good one, each
                        cause, a false landing), with the FSR strikes, the IMU landings and the
                        predicted strike marked

Usage (from server/):
    python utils/analysis/prediction_errors.py logs/STRAVA/session_2026-11-16_23-* --out prediction_errors
"""
import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # server/
from utils.analysis.score_predictions import ALL_EVENTS, arrival, session_calibration
from utils.gait.calibration import IMU_OF_FOOT, mounting_from, up_from
from utils.gait.events import FootTracker
from utils.hardware.sensor_stream import ACC_LSB_PER_MS2, GYR_LSB_PER_DPS
from utils.prediction_model.strike_predictor import StrikePredictor

RIGHT, LEFT = 1, 2
FOOT_NAME = {RIGHT: "right", LEFT: "left"}
HORIZON_S = 0.080  # the beat can still be moved until this long before the strike
GOOD_MS = 30  # "on the step" (score_predictions.py, simulation/compare.py)
MATCH_S = (-0.15, 0.05)  # IMU landing - FSR strike of the same step (engine/strike_tracking.py)
LOOK_BACK_S = 1.0  # OURS: the second before a strike is searched for the cause
GAP_S = 0.015  # OURS: samples come every 10 ms; a longer gap means samples were lost
PACE_CHANGE = 0.05  # OURS: a stride this much longer or shorter than the one before = the pace changed
BIG_MS = 100  # OURS: errors above this are counted separately as large
FALSE_WINDOW_S = (0.7, 0.08)  # OURS: a false landing of the same foot this long before the strike
CAUSES = ["data lost", "missed landing", "false landing", "pace changed", "other"]
COLORS = {"data lost": "#6250d6", "missed landing": "#eb6834", "false landing": "#d6a12a",
          "pace changed": "#2a78d6", "other": "#8a8a8a", "ok": "#1baf7a"}


def run_session(folder: Path) -> dict:
    """Replay the session (as score_predictions.score_session) and keep everything needed."""
    calibration = session_calibration(folder, None)
    mounting, up = mounting_from(calibration), up_from(calibration)
    imu = pd.read_csv(folder / "imu_data.csv")
    strikes = pd.read_csv(folder / "strikes.csv")
    t = imu["sender_t_us"].to_numpy() / 1e6
    now = arrival(imu, strikes)
    use = set(ALL_EVENTS.split(","))
    data, feet = {}, {}
    for f, label in IMU_OF_FOOT.items():
        p = label[0]
        data[f] = (imu[f"valid_{p}"].to_numpy() == 1,
                   imu[[f"{p}_ax", f"{p}_ay", f"{p}_az"]].to_numpy(float) / ACC_LSB_PER_MS2,
                   imu[[f"{p}_gx", f"{p}_gy", f"{p}_gz"]].to_numpy(float) / GYR_LSB_PER_DPS)
        feet[f] = FootTracker(f, *mounting.get(f, (None, None)), up=up.get(f))
    predictor = StrikePredictor()
    preds, lands = [], []
    for i, ti in enumerate(t):
        for f in (LEFT, RIGHT):
            valid, acc, gyr = data[f]
            if not valid[i]:
                continue
            for e in feet[f].add_sample(ti, acc[i], gyr[i]):
                if e.kind == "LAND":
                    lands.append((e.sender_s, e.foot))
                    predictor.on_strike(e.foot, e.sender_s)
                elif e.kind in use:
                    predictor.on_event(e.foot, e.kind, e.sender_s)
        p = predictor.predict(ti)
        if p is not None:
            preds.append((now[i], p.foot, p.sender_s, p.method))
    return {"t": t, "data": data, "strikes": strikes,
            "preds": pd.DataFrame(preds, columns=["now", "foot", "pred", "method"]),
            "lands": pd.DataFrame(lands, columns=["t", "foot"])}


def classify(folder: Path, run: dict) -> pd.DataFrame:
    t, data, P, L = run["t"], run["data"], run["preds"], run["lands"]
    s_all = run["strikes"].sort_values("sender_t_us")
    st = s_all["sender_t_us"].to_numpy() / 1e6
    sf = s_all["foot"].to_numpy()
    spc = s_all["pc_t"].to_numpy()
    gap = np.diff(t) > GAP_S  # samples missing (both IMUs share a packet)
    gap_from, gap_to = t[:-1][gap], t[1:][gap]
    # IMU landings that match an FSR strike of the same foot
    matched = np.zeros(len(L), bool)
    landed = np.zeros(len(st), bool)
    for k, (s, f) in enumerate(zip(st, sf)):
        d = L["t"].to_numpy() - s
        hit = np.where((L["foot"].to_numpy() == f) & (d >= MATCH_S[0]) & (d <= MATCH_S[1]))[0]
        if len(hit):
            landed[k] = True
            matched[hit[np.abs(d[hit]).argmin()]] = True
    false_t, false_f = L["t"].to_numpy()[~matched], L["foot"].to_numpy()[~matched]

    rows = []
    for k, (s, f) in enumerate(zip(st, sf)):
        q = P[(P["now"] <= s - HORIZON_S) & (P["now"] > s - 1.5) & (P["foot"] == f)]
        pred, method = np.nan, ""
        if len(q) and abs(q.iloc[-1]["pred"] - s) < 0.5:
            pred, method = q.iloc[-1]["pred"], q.iloc[-1]["method"]
        valid = data[f][0]
        window = (t >= s - LOOK_BACK_S) & (t <= s)
        lost = bool(((gap_to >= s - LOOK_BACK_S) & (gap_from <= s - MATCH_S[0] / 3)).any() or (~valid[window]).any())
        false_land = bool(((false_f == f) & (false_t >= s - FALSE_WINDOW_S[0]) & (false_t <= s - FALSE_WINDOW_S[1])).any())
        missed_prev = bool(k >= 1 and not landed[k - 1])
        pace = False
        if k >= 3:
            stride, before = st[k] - st[k - 2], st[k - 1] - st[k - 3]
            pace = bool(before > 0 and abs(stride - before) / before > PACE_CHANGE)
        land_d = L["t"].to_numpy()[L["foot"].to_numpy() == f] - s
        land_d = land_d[(land_d >= MATCH_S[0]) & (land_d <= MATCH_S[1])]
        rhythm = (s - (st[k - 1] + (st[k - 2] - st[k - 3]))) * 1000 if k >= 3 else np.nan
        rows.append({"session": folder.name, "foot": FOOT_NAME[f], "strike_s": s, "strike_pc_t": spc[k],
                     "predicted_s": pred, "raw_error_ms": (pred - s) * 1000, "method": method,
                     "imu_landed": bool(landed[k]),
                     "landing_minus_fsr_ms": land_d[np.abs(land_d).argmin()] * 1000 if len(land_d) else np.nan,
                     "fsr_rhythm_error_ms": rhythm if abs(rhythm) < 400 else np.nan, "data_lost": lost, "false_landing": false_land,
                     "missed_landing_before": missed_prev, "pace_changed": pace})
    df = pd.DataFrame(rows)
    df["error_ms"] = np.nan
    for f in df["foot"].unique():  # remove the steady FSR offset of each foot
        m = df["foot"] == f
        df.loc[m, "error_ms"] = df.loc[m, "raw_error_ms"] - df.loc[m, "raw_error_ms"].median()
    df["is_error"] = df["error_ms"].isna() | (df["error_ms"].abs() > GOOD_MS)

    def cause(r):
        if not r["is_error"]:
            return "ok"
        if r["data_lost"]:
            return "data lost"
        if r["missed_landing_before"]:
            return "missed landing"
        if r["false_landing"]:
            return "false landing"
        if r["pace_changed"]:
            return "pace changed"
        return "other"
    df["cause"] = df.apply(cause, axis=1)
    df.attrs["false_landings"] = [(tl, int(ff)) for tl, ff in zip(L["t"].to_numpy()[~matched], L["foot"].to_numpy()[~matched])]
    out = df.iloc[5:]
    out.attrs = df.attrs
    return out  # the first steps teach the predictor (it needs MIN_HISTORY steps per foot)


# ----------- Figures -----------
def _plt():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    return plt


def fig_causes(all_df: pd.DataFrame, out: Path):
    plt = _plt()
    sessions = list(all_df["session"].unique())
    fig, ax = plt.subplots(figsize=(11, 4.8))
    bottom = np.zeros(len(sessions))
    for c in ["ok"] + CAUSES:
        share = np.array([100 * np.mean(all_df.loc[all_df["session"] == s, "cause"] == c) for s in sessions])
        ax.bar(range(len(sessions)), share, bottom=bottom, color=COLORS[c], label=c)
        for i, v in enumerate(share):
            if v >= 6:
                ax.text(i, bottom[i] + v / 2, f"{v:.0f}%", ha="center", va="center", color="white", fontsize=9)
        bottom += share
    ax.set_xticks(range(len(sessions)))
    ax.set_xticklabels([s.replace("session_2026-", "") for s in sessions], fontsize=9)
    ax.set_ylabel("share of the steps (%)")
    ax.set_title(f"Prediction 80 ms before each FSR heel strike: on the step (within ±{GOOD_MS} ms) or an error, by cause")
    ax.legend(loc="center left", bbox_to_anchor=(1.0, 0.5))
    fig.tight_layout()
    fig.savefig(out / "fig_causes.png", dpi=150)
    plt.close(fig)


def fig_over_time(folder: Path, df: pd.DataFrame, out: Path):
    plt = _plt()
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(12, 6), sharex=True, gridspec_kw={"height_ratios": [3, 1]})
    t0 = df["strike_pc_t"].min()
    for c in ["ok"] + CAUSES:
        m = (df["cause"] == c) & df["error_ms"].notna()
        a1.scatter(df.loc[m, "strike_pc_t"] - t0, df.loc[m, "error_ms"].clip(-300, 300), s=14, color=COLORS[c], label=c)
    none = df["error_ms"].isna()
    a1.scatter(df.loc[none, "strike_pc_t"] - t0, np.full(none.sum(), -300), marker="x", s=18, color="black",
               label="no prediction")
    a1.axhspan(-GOOD_MS, GOOD_MS, color=COLORS["ok"], alpha=0.12)
    a1.set_ylabel("predicted − FSR strike (ms)\nsteady offset removed, clipped at ±300")
    a1.legend(ncol=4, fontsize=8, loc="upper right")
    a1.grid(alpha=0.3)
    beats = folder / "beats.csv"
    if beats.exists():
        b = pd.read_csv(beats)
        a2.plot(b["pc_t"] - t0, b["tempo_bpm_applied"], color="#333333")
    a2.set_ylabel("music tempo (BPM)")
    a2.set_xlabel("time in the session (s)")
    a2.grid(alpha=0.3)
    a1.set_title(f"{folder.name}: prediction error of every step, 80 ms before the strike")
    fig.tight_layout()
    fig.savefig(out / f"fig_over_time_{folder.name}.png", dpi=150)
    plt.close(fig)


def fig_example(folder: Path, run: dict, row: pd.Series, out: Path, name: str, title: str = ""):
    plt = _plt()
    f = RIGHT if row["foot"] == "right" else LEFT
    t, (valid, acc, gyr) = run["t"], run["data"][f]
    s = row["strike_s"]
    w = (t >= s - 1.2) & (t <= s + 0.4)
    x = t[w] - s
    fig, axes = plt.subplots(2, 1, figsize=(12, 6.5), sharex=True)
    for ax, sig, unit, what_sig in ((axes[0], gyr[w], "°/s", "raw gyro"), (axes[1], acc[w], "m/s²", "raw accelerometer")):
        for k, c in enumerate(("#d62a2a", "#1baf7a", "#2a78d6")):
            ax.plot(x, np.where(valid[w], sig[:, k], np.nan), color=c, lw=1.1, marker=".", ms=3, label=f"{'xyz'[k]}")
        ax.set_ylabel(f"{what_sig} ({unit})")
        ax.grid(alpha=0.3)
    st = run["strikes"]["sender_t_us"].to_numpy() / 1e6
    sfoot = run["strikes"]["foot"].to_numpy()
    L = run["lands"]
    tw = t[w]
    gaps = [(a, b) for a, b in zip(tw[:-1], tw[1:]) if b - a > GAP_S]
    for ax in axes:
        for ts, ff in zip(st, sfoot):
            if s - 1.2 <= ts <= s + 0.4:
                ax.axvline(ts - s, color="black", lw=2.0 if ff == f else 1.0, ls="-" if ff == f else ":")
        for tl, ff in zip(L["t"], L["foot"]):
            if s - 1.2 <= tl <= s + 0.4 and ff == f:
                ax.axvline(tl - s, color="#eb6834", lw=1.6, ls="--")
        if not np.isnan(row["predicted_s"]):
            ax.axvline(row["predicted_s"] - s, color="#6250d6", lw=4.5, alpha=0.55, zorder=1)
        ax.axvline(-HORIZON_S, color="#888888", lw=1, ls="-.")
        for a, b in gaps:
            ax.axvspan(a - s, b - s, color="#6250d6", alpha=0.18)
    import matplotlib.lines as ml
    from matplotlib.patches import Patch
    handles = [ml.Line2D([], [], color=c, label=f"axis {a}") for a, c in zip("xyz", ("#d62a2a", "#1baf7a", "#2a78d6"))]
    handles += [ml.Line2D([], [], color="black", lw=2, label="FSR strike, this foot (truth)"),
                ml.Line2D([], [], color="black", lw=1, ls=":", label="FSR strike, other foot"),
                ml.Line2D([], [], color="#eb6834", ls="--", label="landing the IMU reported, this foot"),
                ml.Line2D([], [], color="#6250d6", lw=4.5, alpha=0.55, label="predicted strike"),
                ml.Line2D([], [], color="#888888", ls="-.", label="80 ms before: last moment to move the beat"),
                Patch(color="#6250d6", alpha=0.18, label="samples lost")]
    axes[0].legend(handles=handles, ncol=3, fontsize=8, loc="upper left")
    err = "no prediction" if np.isnan(row["error_ms"]) else f"prediction error {row['error_ms']:+.0f} ms"
    what = title or (f"{err}, cause: {row['cause']}" if row["cause"] != "ok" else f"{err} (on the step)")
    axes[0].set_title(f"{folder.name}, {row['foot']} foot: {what}")
    axes[1].set_xlabel("time relative to the FSR strike (s)")
    fig.tight_layout()
    fig.savefig(out / name, dpi=150)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("sessions", nargs="+", type=Path)
    ap.add_argument("--out", type=Path, default=Path("prediction_errors"))
    ap.add_argument("--examples", type=int, default=2, help="example graphs per cause")
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    runs, parts, false_lands = {}, [], {}
    for folder in args.sessions:
        runs[folder.name] = (folder, run_session(folder))
        parts.append(classify(folder, runs[folder.name][1]))
        false_lands[folder.name] = parts[-1].attrs["false_landings"]
    df = pd.concat(parts, ignore_index=True)
    df.to_csv(args.out / "errors.csv", index=False)

    print(f"Steps scored: {len(df)} (FSR heel strikes, both feet; the first 5 of each session left out)")
    print(f"An error = no prediction, or more than ±{GOOD_MS} ms from the strike, {HORIZON_S * 1000:.0f} ms before it\n")
    table = (df.groupby("session")["cause"].value_counts().unstack(fill_value=0)
             .reindex(columns=["ok"] + CAUSES, fill_value=0))
    table["steps"] = table.sum(axis=1)
    table["on the step %"] = (100 * table["ok"] / table["steps"]).round(0).astype(int)
    big = df["error_ms"].isna() | (df["error_ms"].abs() > BIG_MS)
    table["large errors %"] = (100 * big.groupby(df["session"]).mean()).round(0).astype(int)
    rh = df["fsr_rhythm_error_ms"]
    rh = rh - rh.groupby([df["session"], df["foot"]]).transform("median")
    table["FSR rhythm: on the step %"] = (100 * (rh.abs() <= GOOD_MS).groupby(df["session"]).sum()
                                          / rh.notna().groupby(df["session"]).sum()).round(0).astype(int)
    table["false landings"] = [len(false_lands[s]) for s in table.index]
    print(table.rename(index=lambda s: s.replace("session_", "")).to_string())

    print("\nIMU landing - FSR strike, per session and foot: found %, median ms, spread ms (1.48 x MAD)")
    for (sess, foot), g in df.groupby(["session", "foot"]):
        d = g["landing_minus_fsr_ms"].dropna()
        spread = 1.48 * np.median(np.abs(d - d.median())) if len(d) else np.nan
        print(f"  {sess.replace('session_', '')} {foot:5s}: {100 * g['imu_landed'].mean():3.0f}%  {d.median():+4.0f}  {spread:3.0f}")

    print(f"\nLarge errors (no prediction or more than {BIG_MS} ms): {big.sum()} of {len(df)} steps ({100 * big.mean():.0f}%)")
    print("How often a large error follows each condition (with it / without it), and how common it is:")
    for col in ("data_lost", "missed_landing_before", "false_landing", "pace_changed"):
        c = df[col]
        print(f"  {col:24s} {100 * big[c].mean():3.0f}% / {100 * big[~c].mean():3.0f}%   in {100 * c.mean():3.0f}% of the steps")
    none = ~(df["data_lost"] | df["missed_landing_before"] | df["false_landing"] | df["pace_changed"])
    print(f"Steps with none of these conditions: {100 * none.mean():.0f}% of the steps; of them "
          f"{100 * (df.loc[none, 'cause'] == 'ok').mean():.0f}% on the step, {100 * big[none].mean():.0f}% large errors")
    print("Method that gave the prediction 80 ms before the strike (steps, on the step %):")
    for m, g in df[df["method"] != ""].groupby("method"):
        print(f"  {m}: {len(g):4d}  {100 * (g['cause'] == 'ok').mean():3.0f}%")

    fig_causes(df, args.out)
    for name, (folder, run) in runs.items():
        fig_over_time(folder, df[df["session"] == name], args.out)
    best = df[df["session"] == df["session"].value_counts().idxmax()]  # the longest session
    good = best[(best["cause"] == "ok") & ~best["false_landing"] & ~best["data_lost"]]
    if len(good):
        row = good.iloc[len(good) // 2]
        fig_example(*runs[row["session"]], row, args.out, "fig_example_good_step.png")
    missed = df[~df["imu_landed"] & ~df["data_lost"]]  # a landing the IMU did not report (no data loss)
    for n, (_, row) in enumerate(missed.drop_duplicates("session").head(args.examples).iterrows(), 1):
        fig_example(*runs[row["session"]], row, args.out, f"fig_example_missed_landing_{n}.png",
                    title="the IMU reported no landing for this FSR strike (the next step's prediction then fails)")
    for c in CAUSES:  # the largest errors of each cause, one per session first
        if c in ("missed landing", "false landing"):
            continue
        cand = df[df["cause"] == c].copy()
        cand["abs"] = cand["error_ms"].abs().fillna(350)
        cand = cand[cand["abs"] < 400].sort_values("abs", ascending=False)
        picked = cand.drop_duplicates("session").head(args.examples)
        for n, (_, row) in enumerate(picked.iterrows(), 1):
            fig_example(*runs[row["session"]], row, args.out, f"fig_example_{c.replace(' ', '_')}_{n}.png")
    n = 0
    for name, lands in false_lands.items():  # a false landing: centre the graph on the next strike of that foot
        d = df[df["session"] == name]
        for tl, ff in lands[len(lands) // 2:]:
            nxt = d[(d["foot"] == FOOT_NAME[ff]) & (d["strike_s"] > tl) & (d["strike_s"] < tl + 0.9) & ~d["data_lost"]]
            if len(nxt) and n < args.examples:
                n += 1
                fig_example(*runs[name], nxt.iloc[0], args.out, f"fig_example_false_landing_{n}.png",
                            title=f"a landing the IMU reported {nxt.iloc[0]['strike_s'] - tl:.2f} s before the real one "
                                  "(no FSR strike there)")
                break
    print(f"\nGraphs and errors.csv written to {args.out}")


if __name__ == "__main__":
    main()
