"""
How the IMU event detector decided, and whether each decision was right.

  python utils/analysis/decision_analysis.py logs/STRAVA/session_A logs/STRAVA/session_B ... --out decision_analysis

For every session it
  1. recomputes the foot rotation speed exactly as the engine does (utils/gait: bias, PCA axis,
     filter 0.5) from imu_data.csv and the calibration saved in meta.json, and checks it against
     the values the engine logged at each event (gait_events.csv);
  2. checks every IMU detection (toe-off TO, mid-swing MS) against the FSR, the ground truth:
       - TO: is there an FSR lift of the same foot / of the other foot 100 ms before to 200 ms after it?
       - MS: does it fall inside a swing (FSR lift -> next FSR strike) of the same / the other foot?
     and sorts it into: at a heel strike (within 50 ms of any FSR strike: the landing impact),
     correct foot, other foot, or no gait event;
  3. measures the rotation speed in the 150 ms after each real toe-off (FSR lift) of each foot,
     and the high-frequency content (> 15 Hz) of the gyro around every detection.

Every number in "BrainSync - algorithm decisions" is printed here, every graph is written to --out,
and every detection is listed in detections.csv so each one can be checked by hand.
"""
import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

GYR_LSB_PER_DPS = 16.0          # BNO055 raw mode, as in utils/hardware/sensor_stream.py
PITCH_ALPHA = 0.5               # filter on the pitch rate, as in utils/gait/events.py
TO_THRESHOLD, MS_THRESHOLD = -60.0, 150.0
SLAP_GUARD_S = 0.15             # events.py ignores this long after an FSR strike of the same foot
AT_STRIKE_S = 0.05              # a detection this close to any FSR heel strike is the landing impact
FEET = {1: "right", 2: "left"}  # FSR foot ids
CLASSES = {
    "correct foot": "#1baf7a",
    "other foot": "#2a78d6",
    "at a heel strike": "#eb6834",
    "no gait event": "#6250d6",
}
INK, MUTED, GRID = "#0b0b0b", "#52514e", "#e4e3df"
plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10, "axes.edgecolor": MUTED, "xtick.color": MUTED,
                     "ytick.color": MUTED, "axes.grid": True, "grid.color": GRID, "axes.spines.top": False,
                     "axes.spines.right": False, "axes.titlesize": 10.5, "axes.titleweight": "bold"})


def highpass(x: np.ndarray, cutoff_hz: float, fs_hz: float) -> np.ndarray:
    """Zero-phase high-pass: remove everything below cutoff_hz in the frequency domain (numpy only)."""
    spec = np.fft.rfft(x - x.mean())
    spec[np.fft.rfftfreq(len(x), 1 / fs_hz) < cutoff_hz] = 0
    return np.fft.irfft(spec, n=len(x))


# ---------------------------------------------------------------- loading and recomputing
def load_session(folder: Path) -> dict:
    meta = json.loads((folder / "meta.json").read_text(encoding="utf-8"))
    cal = meta["strike_tracking"]["calibration"]
    imu = pd.read_csv(folder / "imu_data.csv")
    events = pd.read_csv(folder / "gait_events.csv")
    strikes = pd.read_csv(folder / "strikes.csv").sort_values("contact_pc_t")
    t = imu.sender_t_us.values / 1e6
    # time axis of the Excel tables: seconds from the first FSR step, on the shoe clock
    t0 = strikes.sender_t_us.iloc[0] / 1e6 - (strikes.pc_t.iloc[0] - strikes.contact_pc_t.iloc[0])
    s = {"name": folder.name, "t": t, "t0": t0, "events": events,
         "strike": {f: np.sort(strikes[strikes.foot == f].sender_t_us.values / 1e6) for f in FEET},
         "lift": {f: np.sort(events[(events.event == "LIFT") & (events.foot == f)].sender_s.values) for f in FEET},
         "imu": {}}
    for side, foot in (("r", 1), ("l", 2)):
        gyro = imu[[f"{side}_g{ax}" for ax in "xyz"]].values / GYR_LSB_PER_DPS
        c = cal[FEET[foot]]
        rate = (gyro - np.array(c["gyro_bias_dps"])) @ np.array(c["pitch_axis"])
        filt = np.empty_like(rate)
        acc = 0.0
        for i, v in enumerate(rate):
            acc += PITCH_ALPHA * (v - acc)
            filt[i] = acc
        ev = events[(events.foot == foot) & events.event.isin(["TO", "MS"])]
        # the saved axis has no sign; take the sign that matches the values the engine logged
        sign = np.sign(np.median(np.interp(ev.sender_s.values, t, filt) * np.sign(ev.value_dps.values))) or 1.0
        filt *= sign
        s["imu"][foot] = {"rate": filt, "events": ev,
                          "check": np.abs(np.interp(ev.sender_s.values, t, filt) - ev.value_dps.values),
                          "hf": np.abs(highpass(np.linalg.norm(gyro, axis=1), 15.0, 100.0))}
    return s


def classify(s: dict) -> pd.DataFrame:
    rows = []
    all_strikes = np.sort(np.concatenate(list(s["strike"].values())))
    for foot, imu in s["imu"].items():
        other = 3 - foot
        for _, e in imu["events"].iterrows():
            t = e.sender_s
            if e.event == "TO":
                hit = {f: bool(np.any((t - s["lift"][f] > -0.10) & (t - s["lift"][f] < 0.20))) for f in FEET}
            else:
                hit = {}
                for f in FEET:
                    before = s["lift"][f][s["lift"][f] < t]
                    nxt = s["strike"][f][s["strike"][f] > before[-1]] if len(before) else []
                    hit[f] = bool(len(before) and len(nxt) and t < nxt[0] + 0.02 and t - before[-1] < 0.8)
            if np.min(np.abs(all_strikes - t)) < AT_STRIKE_S:
                cls = "at a heel strike"   # impact of a foot landing, not a toe-off or mid-swing
            elif hit[foot]:
                cls = "correct foot"
            elif hit[other]:
                cls = "other foot"
            else:
                cls = "no gait event"
            win = (s["t"] > t - 0.15) & (s["t"] < t + 0.15)
            rows.append({"session": s["name"], "imu_label": FEET[foot], "event": e.event,
                         "excel_time_s": round(t - s["t0"], 3), "rate_dps": e.value_dps, "class": cls,
                         "gyro_above_15hz_dps": round(float(s["imu"][foot]["hf"][win].mean()), 2)})
    return pd.DataFrame(rows)


def toe_off_response(s: dict) -> dict:
    """Rotation speed of each IMU in the 150 ms after the real toe-off (FSR lift) of each foot."""
    out = {}
    for foot, imu in s["imu"].items():
        for f in FEET:
            mins = [imu["rate"][(s["t"] >= a) & (s["t"] < a + 0.15)].min() for a in s["lift"][f]
                    if np.any((s["t"] >= a) & (s["t"] < a + 0.15))]
            out[(FEET[foot], FEET[f])] = float(np.median(mins))
    return out


def to_timing(s: dict) -> dict:
    """Time of each IMU toe-off relative to the nearest FSR lift of each foot."""
    out = {}
    for foot, imu in s["imu"].items():
        to = imu["events"][imu["events"].event == "TO"].sender_s.values
        for f in FEET:
            out[(FEET[foot], FEET[f])] = np.array([(t - s["lift"][f])[np.argmin(np.abs(t - s["lift"][f]))] for t in to])
    return out


def missed_toe_offs(s: dict, det: pd.DataFrame, imu_foot: int, real_foot: int) -> dict:
    """Real toe-offs (dip below the threshold after an FSR lift of real_foot) on one IMU: detected or not,
    and whether the dip fell inside the ignore window after an FSR strike of the foot the IMU is labelled."""
    imu = s["imu"][imu_foot]
    detected_t = det[(det.imu_label == FEET[imu_foot]) & (det.event == "TO")].excel_time_s.values + s["t0"]
    n = {"lifts": 0, "dips": 0, "detected": 0, "missed_in_window": 0, "missed_other": 0}
    for a in s["lift"][real_foot]:
        m = (s["t"] >= a) & (s["t"] < a + 0.2)
        if not m.any():
            continue
        n["lifts"] += 1
        if imu["rate"][m].min() >= TO_THRESHOLD:
            continue
        n["dips"] += 1
        tmin = s["t"][m][np.argmin(imu["rate"][m])]
        if np.any(np.abs(detected_t - tmin) < 0.05):
            n["detected"] += 1
            continue
        prev = s["strike"][imu_foot][s["strike"][imu_foot] < tmin]
        n["missed_in_window" if len(prev) and tmin - prev[-1] < SLAP_GUARD_S else "missed_other"] += 1
    return n


def swing_peaks(s: dict) -> dict:
    """Largest rotation speed of each IMU inside each real swing of each foot (FSR lift + 50 ms
    to the next FSR strike - 50 ms, so the lift and landing themselves are left out)."""
    out = {}
    for foot, imu in s["imu"].items():
        for f in FEET:
            peaks = []
            for a in s["lift"][f]:
                nxt = s["strike"][f][s["strike"][f] > a]
                if not len(nxt) or nxt[0] - a > 0.8:
                    continue
                m = (s["t"] > a + 0.05) & (s["t"] < nxt[0] - 0.05)
                if m.any():
                    peaks.append(imu["rate"][m].max())
            out[(FEET[foot], FEET[f])] = (float(np.median(peaks)), float(np.mean(np.array(peaks) > MS_THRESHOLD) * 100), len(peaks))
    return out


# ---------------------------------------------------------------- graphs
def fig_timeline(s, det, start_excel_s, length_s, path, title):
    a = s["t0"] + start_excel_s
    b = a + length_s
    fig, axs = plt.subplots(2, 1, figsize=(12, 6.6), sharex=True)
    for ax, foot in zip(axs, (1, 2)):
        m = (s["t"] >= a) & (s["t"] <= b)
        ax.plot(s["t"][m] - s["t0"], s["imu"][foot]["rate"][m], color=INK, lw=1.4)
        for y, txt in ((MS_THRESHOLD, "mid-swing threshold +150"), (TO_THRESHOLD, "toe-off threshold −60")):
            ax.axhline(y, color=MUTED, ls="--", lw=1)
            ax.text(b - s["t0"] + 0.02 * length_s, y, txt, fontsize=8, color=MUTED, va="center")
        for f, ls in ((1, "-"), (2, (0, (4, 3)))):
            for x in s["strike"][f][(s["strike"][f] >= a) & (s["strike"][f] <= b)]:
                ax.axvline(x - s["t0"], color="#9a9890", ls=ls, lw=1.1, zorder=0)
            lx = s["lift"][f][(s["lift"][f] >= a) & (s["lift"][f] <= b)]
            ax.scatter(lx - s["t0"], np.full(len(lx), -180), marker="^", s=40,
                       color="#9a9890" if f == 1 else "white", edgecolor="#6b6a64", zorder=3)
        ev = det[(det.imu_label == FEET[foot]) & (det.excel_time_s >= start_excel_s) & (det.excel_time_s <= start_excel_s + length_s)]
        for cls, c in CLASSES.items():
            e = ev[ev["class"] == cls]
            ax.scatter(e.excel_time_s, e.rate_dps, s=70, color=c, edgecolor="white", linewidth=1.2, zorder=5)
        ax.set_ylim(-200, 280)
        ax.set_ylabel("°/s")
        ax.set_title(f"IMU labelled '{FEET[foot]}' (imu{'R' if foot == 1 else 'L'}_gyro_* in the Excel)", loc="left")
    axs[1].set_xlabel("time (s), same axis as the Excel tables")
    handles = [Line2D([], [], color=INK, lw=1.4, label="foot rotation speed (filtered), as the algorithm saw it"),
               Line2D([], [], color="#9a9890", lw=1.1, label="FSR heel strike – right foot"),
               Line2D([], [], color="#9a9890", lw=1.1, ls=(0, (4, 3)), label="FSR heel strike – left foot"),
               Line2D([], [], marker="^", ls="", color="#9a9890", markeredgecolor="#6b6a64", label="FSR lift – right (filled) / left (hollow)")]
    handles += [Line2D([], [], marker="o", ls="", markersize=8, color=c, label="detection: " + k) for k, c in CLASSES.items()]
    fig.legend(handles=handles, loc="lower center", ncol=3, frameon=False, fontsize=8.5)
    fig.suptitle(title, x=0.02, ha="left", fontsize=12, fontweight="bold")
    fig.tight_layout(rect=(0, 0.12, 0.96, 1))
    fig.savefig(path, dpi=150)
    plt.close(fig)


def fig_summary(det, path):
    tab = pd.crosstab(det.session, det["class"]).reindex(columns=list(CLASSES), fill_value=0)
    pct = tab.div(tab.sum(axis=1), axis=0) * 100
    fig, ax = plt.subplots(figsize=(10, 4.2))
    y = np.arange(len(pct))[::-1]
    left = np.zeros(len(pct))
    for c in CLASSES:
        ax.barh(y, pct[c], left=left, color=CLASSES[c], edgecolor="white", linewidth=2, label=c)
        for yi, l, v in zip(y, left, pct[c]):
            if v >= 7:
                ax.text(l + v / 2, yi, f"{v:.0f}%", ha="center", va="center", color="white", fontsize=9, fontweight="bold")
        left += pct[c].values
    for yi, tot in zip(y, tab.sum(axis=1)):
        ax.text(101, yi, str(tot), va="center", fontsize=9, color=MUTED)
    ax.text(101, len(pct) - 0.35, "detections", fontsize=8, color=MUTED)
    ax.set_yticks(y, [f"Session {i}" for i in range(1, len(pct) + 1)])
    ax.set_xlim(0, 112)
    ax.set_xlabel("share of the IMU detections (%)")
    ax.grid(axis="y", visible=False)
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.2), ncol=4, frameon=False, fontsize=9)
    ax.set_title(f"Every IMU detection checked against the FSR ({len(det)} detections)", loc="left")
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def fig_timing(sessions, path):
    fig, axs = plt.subplots(1, len(sessions), figsize=(3.0 * len(sessions), 3.6), sharey=True)
    bins = np.arange(-0.6, 0.61, 0.04)
    for i, (ax, s) in enumerate(zip(axs, sessions), 1):
        tim = to_timing(s)
        ax.hist(tim[("left", "right")], bins=bins, color="#2a78d6", alpha=0.85, label="vs RIGHT-foot FSR lift")
        ax.hist(tim[("left", "left")], bins=bins, histtype="step", color="#eb6834", lw=2, label="vs LEFT-foot FSR lift")
        ax.axvline(0, color=MUTED, lw=1)
        ax.set_title(f"Session {i}", loc="left")
        ax.set_xlabel("TO time − nearest FSR lift (s)")
    axs[0].set_ylabel("toe-offs of the IMU labelled 'left'")
    axs[0].legend(frameon=False, fontsize=8, loc="upper left")
    fig.suptitle("Toe-offs detected by the IMU labelled 'left': which foot's FSR lift do they follow?", x=0.02, ha="left",
                 fontsize=12, fontweight="bold")
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def fig_response(sessions, path):
    lags = np.arange(-0.3, 0.5, 0.01)
    fig, axs = plt.subplots(1, 2, figsize=(12, 4), sharey=True)
    colors = ["#2a78d6", "#eb6834", "#1baf7a", "#6250d6", "#9a9890"]
    for ax, (foot, real) in zip(axs, ((2, 1), (1, 2))):
        for i, s in enumerate(sessions):
            r = s["imu"][foot]["rate"]
            avg = [np.mean(np.interp(s["lift"][real] + L, s["t"], r)) for L in lags]
            ax.plot(lags * 1000, avg, color=colors[i], lw=2, label=f"Session {i + 1}")
        ax.axvline(0, color=MUTED, lw=1)
        ax.axhline(TO_THRESHOLD, color=MUTED, ls="--", lw=1)
        ax.text(480, TO_THRESHOLD - 6, "toe-off threshold −60", fontsize=8, color=MUTED, ha="right", va="top")
        ax.set_xlabel(f"time from the {FEET[real]}-foot FSR lift (ms)")
        ax.set_title(f"IMU labelled '{FEET[foot]}', around the {FEET[real]} foot's lift", loc="left")
    axs[0].set_ylabel("rotation speed, average of all lifts (°/s)")
    axs[0].legend(frameon=False, fontsize=9)
    fig.suptitle("What each IMU measures at a real toe-off: a dip below −60 = correct direction; a rise = direction backwards",
                 x=0.02, ha="left", fontsize=11.5, fontweight="bold")
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def fig_noise(det, path):
    fig, ax = plt.subplots(figsize=(9, 4))
    for i, (cls, c) in enumerate(CLASSES.items()):
        v = det[det["class"] == cls].gyro_above_15hz_dps.values
        x = i + (np.random.default_rng(0).random(len(v)) - 0.5) * 0.5
        ax.scatter(x, v, s=10, color=c, alpha=0.5)
        if len(v):
            ax.plot([i - 0.3, i + 0.3], [np.median(v)] * 2, color=INK, lw=2)
            ax.text(i + 0.33, np.median(v), f"median {np.median(v):.1f}", va="center", fontsize=9)
    ax.set_xticks(range(len(CLASSES)), [f"{k}\n({(det['class'] == k).sum()})" for k in CLASSES])
    ax.set_ylabel("gyro signal above 15 Hz,\n±150 ms around the detection (°/s)")
    ax.set_yscale("log")
    ax.grid(axis="x", visible=False)
    ax.set_title("High-frequency signal around each detection, by what the detection really was", loc="left")
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def fig_missed(counts, path):
    labels = [f"Session {i}" for i in range(1, len(counts) + 1)]
    det = np.array([c["detected"] for c in counts])
    win = np.array([c["missed_in_window"] for c in counts])
    oth = np.array([c["missed_other"] for c in counts])
    fig, ax = plt.subplots(figsize=(9, 3.8))
    y = np.arange(len(counts))[::-1]
    for vals, left, c, lab in ((det, 0, "#1baf7a", "detected"), (win, det, "#eb6834", "missed: inside the 150 ms window after a LEFT heel strike"),
                               (oth, det + win, "#9a9890", "missed: other reason")):
        ax.barh(y, vals, left=left, color=c, edgecolor="white", linewidth=2, label=lab)
        for yi, l, v in zip(y, np.broadcast_to(left, vals.shape), vals):
            if v >= 6:
                ax.text(l + v / 2, yi, str(v), ha="center", va="center", color="white", fontsize=9, fontweight="bold")
    ax.set_yticks(y, labels)
    ax.set_xlabel("real right-foot toe-offs that the IMU labelled 'left' shows as a dip below −60 °/s")
    ax.grid(axis="y", visible=False)
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.25), ncol=2, frameon=False, fontsize=9)
    ax.set_title("Real toe-offs on the IMU labelled 'left': detected or missed", loc="left")
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def fig_swing_peaks(sessions, path):
    fig, ax = plt.subplots(figsize=(10, 4))
    pos = 0
    ticks, labels = [], []
    for i, s in enumerate(sessions, 1):
        for foot, real, c in ((2, 1, "#2a78d6"), (1, 2, "#eb6834")):
            peaks = []
            for a in s["lift"][real]:
                nxt = s["strike"][real][s["strike"][real] > a]
                if len(nxt) and nxt[0] - a <= 0.8:
                    m = (s["t"] > a + 0.05) & (s["t"] < nxt[0] - 0.05)
                    if m.any():
                        peaks.append(s["imu"][foot]["rate"][m].max())
            x = pos + (np.random.default_rng(i).random(len(peaks)) - 0.5) * 0.6
            ax.scatter(x, peaks, s=8, color=c, alpha=0.5)
            ax.plot([pos - 0.35, pos + 0.35], [np.median(peaks)] * 2, color=INK, lw=2)
            pos += 1
        ticks.append(pos - 1.5)
        labels.append(f"Session {i}")
        pos += 0.8
    ax.axhline(MS_THRESHOLD, color=MUTED, ls="--", lw=1)
    ax.text(pos - 0.6, MS_THRESHOLD + 5, "mid-swing threshold +150", fontsize=8, color=MUTED, ha="right")
    ax.set_xticks(ticks, labels)
    ax.set_ylabel("largest rotation speed in the swing (°/s)")
    ax.grid(axis="x", visible=False)
    ax.legend(handles=[Line2D([], [], marker="o", ls="", color="#2a78d6", label="IMU labelled 'left', right-foot swings"),
                       Line2D([], [], marker="o", ls="", color="#eb6834", label="IMU labelled 'right', left-foot swings"),
                       Line2D([], [], color=INK, lw=2, label="median")], frameon=False, fontsize=9, loc="upper left")
    ax.set_title("Mid-swing: the rotation speed in real swings (FSR lift to FSR strike) almost never reaches +150", loc="left")
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("sessions", nargs="+", type=Path)
    ap.add_argument("--out", type=Path, default=Path("decision_analysis"))
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    sessions = [load_session(f) for f in sorted(args.sessions)]
    det = pd.concat([classify(s) for s in sessions], ignore_index=True)
    det.to_csv(args.out / "detections.csv", index=False)

    print("1. Recomputed rotation speed vs. the value the engine logged at each event (°/s)")
    for i, s in enumerate(sessions, 1):
        d = np.concatenate([s["imu"][f]["check"] for f in FEET])
        print(f"   session {i}: {len(d)} events, median difference {np.median(d):.1f}")

    print("\n2. Every IMU detection checked against the FSR")
    tab = det["class"].value_counts().reindex(list(CLASSES), fill_value=0)
    for k, v in tab.items():
        print(f"   {k:18s} {v:4d}  {v / len(det) * 100:4.0f}%")
    print(f"   total              {len(det):4d}")

    print("\n3. Toe-offs of the IMU labelled 'left': time from the nearest FSR lift")
    for i, s in enumerate(sessions, 1):
        tim = to_timing(s)
        r, l = tim[("left", "right")], tim[("left", "left")]
        print(f"   session {i}: right-foot lift median {np.median(r) * 1000:+5.0f} ms, within 100 ms {np.mean(np.abs(r) < 0.1) * 100:3.0f}%"
              f" | left-foot lift median {np.median(l) * 1000:+5.0f} ms, within 100 ms {np.mean(np.abs(l) < 0.1) * 100:3.0f}%")

    print("\n4. Median rotation speed (°/s) in the 150 ms after a real toe-off (FSR lift)")
    for i, s in enumerate(sessions, 1):
        r = toe_off_response(s)
        print(f"   session {i}: IMU 'left' at RIGHT lift {r[('left', 'right')]:6.0f} | IMU 'right' at LEFT lift {r[('right', 'left')]:6.0f}"
              f" | IMU 'left' at LEFT lift {r[('left', 'left')]:6.0f} | IMU 'right' at RIGHT lift {r[('right', 'right')]:6.0f}")

    print("\n4b. Largest rotation speed in the middle of each real swing (median °/s, % of swings above +150)")
    for i, s in enumerate(sessions, 1):
        p = swing_peaks(s)
        a, b = p[("left", "right")], p[("right", "left")]
        print(f"   session {i}: IMU 'left' in RIGHT swings {a[0]:5.0f} ({a[1]:3.0f}% > 150, n={a[2]})"
              f" | IMU 'right' in LEFT swings {b[0]:5.0f} ({b[1]:3.0f}% > 150, n={b[2]})")

    print("\n5. Real right-foot toe-offs on the IMU labelled 'left' (dip below -60 °/s after a right FSR lift)")
    counts = [missed_toe_offs(s, det[det.session == s["name"]], imu_foot=2, real_foot=1) for s in sessions]
    for i, c in enumerate(counts, 1):
        print(f"   session {i}: {c}")

    print("\n6. Gyro signal above 15 Hz around each detection (median, °/s)")
    print(det.groupby("class").gyro_above_15hz_dps.median().round(1).to_string())

    fig_summary(det, args.out / "fig_summary.png")
    fig_timing(sessions, args.out / "fig_toe_off_timing.png")
    fig_response(sessions, args.out / "fig_toe_off_response.png")
    fig_missed(counts, args.out / "fig_missed_toe_offs.png")
    fig_noise(det, args.out / "fig_noise.png")
    fig_swing_peaks(sessions, args.out / "fig_swing_peaks.png")
    for i, s in enumerate(sessions, 1):
        start = (s["strike"][1].min() - s["t0"]) + 40
        fig_timeline(s, det[det.session == s["name"]], start, 5, args.out / f"fig_session{i}.png",
                     f"Session {i}: when the algorithm decided toe-off / mid-swing, and whether it was right")
    first = det[(det.session == sessions[0]["name"]) & (det.imu_label == "left") & (det.event == "TO") & (det["class"] == "other foot")]
    ex = float(first.excel_time_s.iloc[1])
    fig_timeline(sessions[0], det[det.session == sessions[0]["name"]], ex - 1.0, 2.0, args.out / "fig_example.png",
                 f"Session 1, around {ex:.2f} s: one toe-off decision, step by step")
    print(f"\nExample used in the document: session 1, IMU 'left', TO at {ex:.2f} s, {first.rate_dps.iloc[1]:.1f} °/s")
    print(f"\nGraphs and detections.csv written to {args.out}")


if __name__ == "__main__":
    main()
