"""
Report for one recorded session, real or simulated: how early and how well the steps were
predicted, how close the beats were to the steps, the tempo, the sensor link, and the
strides of both feet. Uses only the session's log files, so it works on any session
recorded with the IMU firmware (protocol 2). Simulated sessions also get the error
against the true heel strikes.

Writes report.png and report.json into the session folder and prints a summary.

Usage (from the repo root):
  .venv\\Scripts\\python.exe server\\utils\\analysis\\session_report.py [session folder]
Without a folder: the newest session in server/logs (with --sim: in server/sim_logs).
"""
import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np

SERVER = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SERVER))

from utils.paths import get_logs_dir, get_sim_logs_dir
from utils.gait.events import FootTracker
from utils.hardware.sensor_stream import TimeUnwrapper, ACC_LSB_PER_MS2, GYR_LSB_PER_DPS
from utils.prediction_model.strike_predictor import RIGHT, LEFT

LEADS_S = (0.4, 0.2, 0.1)  # how long before the strike a prediction is judged
MISS_S = 0.3  # predictions further off than this were a stop or a start, counted apart
METHODS = (("fused", "predicted_sender_s"), ("B0", "b0_sender_s"), ("B1", "b1_sender_s"), ("B2", "b2_sender_s"))
FEET = ((RIGHT, "right"), (LEFT, "left"))
AXES = ("ax", "ay", "az", "gx", "gy", "gz")
STRIDE_WINDOW_S = (-0.9, 0.35)


def read_csv(path: Path) -> list[dict]:
    if not path.exists():
        return []
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def column(rows: list[dict], name: str) -> np.ndarray:
    return np.array([float(r[name]) if r.get(name) not in (None, "") else np.nan for r in rows])


def newest_session(root: Path) -> Path:
    sessions = sorted((p.parent for p in root.glob("**/meta.json")), key=lambda p: p.stat().st_mtime)
    if not sessions:
        raise SystemExit(f"No session with meta.json under {root}")
    return sessions[-1]


def stats_ms(errors) -> dict:
    e = np.asarray(errors, dtype=float)
    if len(e) == 0:
        return {"n": 0}
    a = np.abs(e)
    return {"n": int(len(e)), "median_abs": float(np.median(a)), "mean": float(e.mean()), "sd": float(e.std()),
            "within30": float(np.mean(a < 30) * 100), "within50": float(np.mean(a < 50) * 100)}


# ----------- Prediction -----------
def prediction_errors(strikes: list[dict], strike_sender: np.ndarray, predictions: list[dict]) -> dict:
    """For each lead time: (predicted - actual FSR strike) of the prediction in effect then, per method."""
    actual = {int(r["strike_number"]): s for r, s in zip(strikes, strike_sender) if r.get("strike_number")}
    by_strike = {}
    for r in predictions:
        by_strike.setdefault(int(r["strike_number"]), []).append(r)
    out = {lead: {m: [] for m, _ in METHODS} for lead in LEADS_S}
    misses = {lead: 0 for lead in LEADS_S}
    for number, rows in by_strike.items():
        if number not in actual:
            continue
        s = actual[number]
        for lead in LEADS_S:
            made = [r for r in rows if float(r["sender_s_made"]) <= s - lead]
            if not made:
                continue
            for method, col in METHODS:
                values = [r[col] for r in made if r[col]]
                if not values:
                    continue
                error = float(values[-1]) - s
                if abs(error) < MISS_S:
                    out[lead][method].append(error * 1000)
                elif method == "fused":
                    misses[lead] += 1
    return {"errors_ms": out, "misses": misses, "strikes": len(actual)}


# ----------- Alignment -----------
def nearest_beat_errors(heard: np.ndarray, targets: np.ndarray) -> np.ndarray:
    """(nearest heard beat - target) in ms, for every target time."""
    targets = targets[~np.isnan(targets)]
    if len(heard) == 0 or len(targets) == 0:
        return np.array([])
    heard = np.sort(heard)
    i = np.clip(np.searchsorted(heard, targets), 1, len(heard) - 1)
    before, after = heard[i - 1], heard[i]
    nearest = np.where(targets - before < after - targets, before, after)
    return (nearest - targets) * 1000


# ----------- Strides (replay of the IMU data through the live gait code) -----------
def replay_feet(imu: list[dict], strikes: list[dict], strike_sender: np.ndarray):
    unwrap = TimeUnwrapper()
    t = np.array([unwrap(int(r["sender_t_us"])) for r in imu])
    raw = {RIGHT: np.array([[float(r[f"r_{a}"]) for a in AXES] for r in imu]),
           LEFT: np.array([[float(r[f"l_{a}"]) for a in AXES] for r in imu])}
    valid = {RIGHT: column(imu, "valid_r") > 0, LEFT: column(imu, "valid_l") > 0}
    trackers = {RIGHT: FootTracker(RIGHT), LEFT: FootTracker(LEFT)}
    order = np.argsort(strike_sender)
    feet = [int(strikes[i]["foot"]) for i in order]
    times = strike_sender[order]
    k = 0
    for i, ti in enumerate(t):
        while k < len(times) and times[k] <= ti:
            trackers[feet[k]].on_strike(times[k])
            k += 1
        for foot in (RIGHT, LEFT):
            if valid[foot][i]:
                trackers[foot].add_sample(ti, raw[foot][i, :3] / ACC_LSB_PER_MS2, raw[foot][i, 3:] / GYR_LSB_PER_DPS)
    pitch, acc = {}, {}
    for foot in (RIGHT, LEFT):
        calib = trackers[foot].calib
        acc[foot] = np.linalg.norm(raw[foot][:, :3] / ACC_LSB_PER_MS2, axis=1)
        if calib.ready:
            pitch[foot] = calib.sign * ((raw[foot][:, 3:] / GYR_LSB_PER_DPS - calib.gyro_bias) @ calib.axis)
    return t, pitch, acc, trackers


def event_leads(gait_events: list[dict], strikes: list[dict], strike_sender: np.ndarray) -> dict:
    """Median time from toe-off / mid-swing (as detected live) to the foot's next FSR strike."""
    out = {}
    for foot, name in FEET:
        for kind in ("TO", "MS"):
            ev = np.sort([float(r["sender_s"]) for r in gait_events if r["event"] == kind and int(r["foot"]) == foot])
            leads = []
            for r, s in zip(strikes, strike_sender):
                if int(r["foot"]) != foot or len(ev) == 0:
                    continue
                j = np.searchsorted(ev, s) - 1
                if j >= 0 and 0 < s - ev[j] < 1.0:
                    leads.append(s - ev[j])
            out[f"{name}_{kind}"] = float(np.median(leads)) if leads else None
    return out


# ----------- Report -----------
def analyze(session: Path) -> dict:
    meta = json.loads((session / "meta.json").read_text(encoding="utf-8"))
    strikes = read_csv(session / "strikes.csv")
    predictions = read_csv(session / "predictions.csv")
    beats = read_csv(session / "beats.csv")
    imu = read_csv(session / "imu_data.csv")
    gait_events = read_csv(session / "gait_events.csv")
    truth = read_csv(session / "sim_steps.csv")
    latency = meta.get("phase_alignment", {}).get("settings", {}).get("latency_s", 0.030)

    unwrap = TimeUnwrapper()
    strike_sender = np.array([unwrap(int(r["sender_t_us"])) for r in strikes])
    contact = column(strikes, "contact_pc_t")
    heard = column(beats, "pc_t") + latency
    t0 = np.nanmin(np.r_[column(strikes, "pc_t"), column(beats, "pc_t")]) if (strikes or beats) else 0.0

    result = {
        "session": str(session), "latency_ms": latency * 1000,
        "alignment_enabled": meta.get("phase_alignment", {}).get("enabled"),
        "receiver_protocol": meta.get("receiver_protocol"),
        "steps": len(strikes), "beats": len(beats),
        "duration_s": float(np.nanmax(column(beats, "pc_t")) - t0) if beats else 0.0,
        "link": meta.get("link"), "sensor_health": meta.get("sensor_health"),
        "tracking": meta.get("strike_tracking"),
        "beat_vs_estimated_contact_ms": stats_ms(nearest_beat_errors(heard, contact)),
    }
    plot_data = {"t0": t0, "heard": heard, "contact": contact, "beats": beats, "strikes": strikes,
                 "strike_sender": strike_sender}
    if truth:
        true_hs = column(truth, "hs_pc_t")
        result["beat_vs_true_heel_strike_ms"] = stats_ms(nearest_beat_errors(heard, true_hs))
        plot_data["true_hs"] = true_hs

    if predictions:
        pred = prediction_errors(strikes, strike_sender, predictions)
        result["prediction"] = {
            f"{int(lead * 1000)}ms_ahead": {m: stats_ms(e) for m, e in pred["errors_ms"][lead].items()}
            | {"missed": pred["misses"][lead]} for lead in LEADS_S}
        plot_data["pred"] = pred

    if imu:
        valid_r, valid_l = column(imu, "valid_r"), column(imu, "valid_l")
        span = np.nanmax(column(imu, "pc_t")) - np.nanmin(column(imu, "pc_t"))
        result["imu"] = {"lines": len(imu), "rate_hz": float(len(imu) / span) if span > 0 else None,
                         "valid_right_pct": float(np.mean(valid_r > 0) * 100),
                         "valid_left_pct": float(np.mean(valid_l > 0) * 100)}
        t, pitch, acc, trackers = replay_feet(imu, strikes, strike_sender)
        result["event_lead_s"] = event_leads(gait_events, strikes, strike_sender)
        result["calibrated"] = {name: trackers[foot].calib.ready for foot, name in FEET}
        plot_data.update(imu_t=t, pitch=pitch, acc=acc)
    return result, plot_data


def plot(result: dict, d: dict, path: Path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(3, 2, figsize=(15, 13))
    t0 = d["t0"]

    # Beat vs step over time
    ax = axes[0, 0]
    contact = d["contact"][~np.isnan(d["contact"])]
    ax.plot(contact - t0, nearest_beat_errors(d["heard"], contact), "o", ms=3, color="tab:blue",
            label="vs estimated contact (FSR - learned delay)")
    if "true_hs" in d:
        ax.plot(d["true_hs"] - t0, nearest_beat_errors(d["heard"], d["true_hs"]), "o", ms=3, color="k",
                alpha=0.4, label="vs true heel strike (simulation)")
    ax.axhspan(-30, 30, color="tab:green", alpha=0.12, label="±30 ms")
    ax.set_ylim(-300, 300)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Beat heard - step (ms)")
    ax.set_title("How far each step is from the nearest beat")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    ax = axes[0, 1]
    errors = nearest_beat_errors(d["heard"], contact)
    ax.hist(np.clip(errors, -300, 300), bins=np.arange(-300, 310, 10), color="tab:blue", alpha=0.7)
    ax.axvspan(-30, 30, color="tab:green", alpha=0.12)
    s = result["beat_vs_estimated_contact_ms"]
    if s["n"]:
        ax.set_title(f"Distribution: median {s['median_abs']:.0f} ms, {s['within30']:.0f}% within ±30 ms, "
                     f"mean {s['mean']:+.0f} ms")
    ax.set_xlabel("Beat heard - estimated contact (ms)")
    ax.grid(alpha=0.3)

    # Tempo
    ax = axes[1, 0]
    beats = d["beats"]
    if beats:
        bt = column(beats, "pc_t") - t0
        ax.plot(bt, column(beats, "tempo_bpm_applied"), color="tab:blue", lw=1, label="Music tempo")
        ax.plot(bt, column(beats, "tempo_bpm_base"), color="tab:cyan", lw=1, ls="--", label="Base tempo (before nudge)")
    ss, pc = d["strike_sender"], column(d["strikes"], "pc_t")
    if len(ss) > 1:
        dt = np.diff(ss)
        ok = (dt > 0.25) & (dt < 1.5)
        ax.plot(pc[1:][ok] - t0, 60.0 / dt[ok], ".", color="k", ms=3, alpha=0.5, label="Each step (60 / step time)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("BPM")
    ax.set_title("Tempo")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    # Prediction accuracy by lead time
    ax = axes[1, 1]
    if "pred" in d:
        width = 0.2
        for i, (method, _) in enumerate(METHODS):
            values = [np.median(np.abs(d["pred"]["errors_ms"][lead][method])) if d["pred"]["errors_ms"][lead][method]
                      else 0 for lead in LEADS_S]
            ax.bar(np.arange(len(LEADS_S)) + (i - 1.5) * width, values, width, label=method)
        ax.set_xticks(range(len(LEADS_S)), [f"{int(lead * 1000)} ms before" for lead in LEADS_S])
        ax.set_ylabel("Median |predicted - actual| (ms)")
        ax.set_title("Step prediction error, by how early it was made\n"
                     "B0 = rhythm, B1 = toe-off, B2 = mid-swing, fused = what the controller uses")
        ax.legend(fontsize=8)
    else:
        ax.text(0.5, 0.5, "No predictions logged (no IMU data)", ha="center", va="center", transform=ax.transAxes)
    ax.grid(alpha=0.3, axis="y")

    # Strides
    window = np.arange(*STRIDE_WINDOW_S, 0.01)
    leads = result.get("event_lead_s", {})
    for col, (foot, name) in enumerate(FEET):
        ax = axes[2, col]
        if "pitch" not in d or foot not in d["pitch"]:
            ax.text(0.5, 0.5, f"{name.capitalize()} IMU not calibrated / no data", ha="center", va="center",
                    transform=ax.transAxes)
            continue
        first, last = d["imu_t"][0] - window[0], d["imu_t"][-1] - window[-1]
        times = [s for r, s in zip(d["strikes"], d["strike_sender"]) if int(r["foot"]) == foot and first <= s <= last]
        for s in times:
            ax.plot(window, np.interp(s + window, d["imu_t"], d["pitch"][foot]), color="tab:blue", alpha=0.15)
        ax.axvline(0, color="k", lw=1)
        for kind, color in (("TO", "tab:green"), ("MS", "tab:purple")):
            if leads.get(f"{name}_{kind}"):
                ax.axvline(-leads[f"{name}_{kind}"], color=color, ls="--", lw=1)
        ax.set_title(f"{name.capitalize()} foot, {len(times)} strides, aligned on the FSR strike\n"
                     f"green = toe-off, purple = mid-swing (median, as detected live)")
        ax.set_xlabel("Time from FSR strike (s)")
        ax.set_ylabel("Pitch rate (dps)")
        ax.grid(alpha=0.3)

    fig.suptitle(f"Session report: {Path(result['session']).name}  (alignment "
                 f"{'on' if result['alignment_enabled'] else 'off'}, latency {result['latency_ms']:.0f} ms)")
    fig.tight_layout()
    fig.savefig(path, dpi=100)
    plt.close(fig)


def summary(result: dict) -> str:
    lines = [f"Session {result['session']}",
             f"  {result['steps']} steps, {result['beats']} beats, {result['duration_s']:.0f} s, "
             f"alignment {'on' if result['alignment_enabled'] else 'off'}, receiver protocol "
             f"{result['receiver_protocol']}"]
    if result.get("link"):
        lines.append(f"  Link: {result['link']}")
    if "imu" in result:
        i = result["imu"]
        lines.append(f"  IMU: {i['rate_hz']:.0f} lines/s, valid right {i['valid_right_pct']:.0f}%, "
                     f"left {i['valid_left_pct']:.0f}%, calibrated {result['calibrated']}")
    for key, label in (("beat_vs_estimated_contact_ms", "Beat vs estimated contact"),
                       ("beat_vs_true_heel_strike_ms", "Beat vs true heel strike")):
        s = result.get(key)
        if s and s["n"]:
            lines.append(f"  {label}: median {s['median_abs']:.0f} ms, {s['within30']:.0f}% within ±30 ms, "
                         f"{s['within50']:.0f}% within ±50 ms, mean {s['mean']:+.0f} ms")
    for lead, methods in result.get("prediction", {}).items():
        parts = [f"{m} {s['median_abs']:.0f}" for m, s in methods.items() if isinstance(s, dict) and s["n"]]
        lines.append(f"  Prediction {lead}: median error (ms) " + ", ".join(parts)
                     + f"; missed {methods['missed']}")
    health = (result.get("sensor_health") or {}).get("log") or []
    for t, message in health:
        lines.append(f"  Sensors: {message}")
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("session", nargs="?", type=Path, help="Session folder (default: the newest)")
    parser.add_argument("--sim", action="store_true", help="Newest session in server/sim_logs instead of server/logs")
    args = parser.parse_args()
    session = args.session or newest_session(get_sim_logs_dir() if args.sim else get_logs_dir())
    result, plot_data = analyze(session)
    plot(result, plot_data, session / "report.png")
    (session / "report.json").write_text(json.dumps(result, indent=2, default=str), encoding="utf-8")
    print(summary(result))
    print(f"Report: {session / 'report.png'}")


if __name__ == "__main__":
    main()
