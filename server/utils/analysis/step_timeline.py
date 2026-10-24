"""
Step-by-step view of one simulated session: for every step, when the heel really hit the
ground, when the pressure sensor fired, when the PC got the message, what was predicted,
and when the beat was heard.

Writes into the session folder:
  steps_table.csv           one row per step
  timeline/page_NN.png      the walk in pages of a few seconds: both legs, the swing,
                            the leg rotation from the IMU and the heard beats
and prints the table.

Usage (from the repo root):
  .venv\\Scripts\\python.exe server\\utils\\analysis\\step_timeline.py [session folder] [--page 8] [--from 0 --to 30]
Without a folder: the newest simulated session.
"""
import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np

SERVER = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SERVER))

from utils.paths import get_sim_logs_dir
from utils.hardware.sensor_stream import GYR_LSB_PER_DPS
from utils.simulation.compare import music_spans
from utils.simulation.fast_loop import with_music

FOOT_NAME = {1: "right", 2: "left"}
LANE = {1: 1.0, 2: 0.0}
COLOR = {1: "tab:orange", 2: "tab:green"}
MATCH_S = 0.05
PREDICT_LEAD_S = 0.1


def read_csv(path: Path) -> list[dict]:
    if not path.exists():
        return []
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def num(value) -> float:
    return float(value) if value not in (None, "") else np.nan


def newest_sim_session(root: Path) -> Path:
    sessions = sorted((p.parent for p in root.glob("**/sim_steps.csv")), key=lambda p: p.stat().st_mtime)
    if not sessions:
        raise SystemExit(f"No simulated session under {root}")
    return sessions[-1]


def build_table(session: Path) -> tuple[list[dict], dict]:
    meta = json.loads((session / "meta.json").read_text(encoding="utf-8"))
    latency = float(meta.get("phase_alignment", {}).get("settings", {}).get("latency_s", 0.0))
    truth = read_csv(session / "sim_steps.csv")
    strikes = read_csv(session / "strikes.csv")
    predictions = read_csv(session / "predictions.csv")
    beats = read_csv(session / "beats.csv")
    if not truth:
        raise SystemExit(f"{session} has no sim_steps.csv rows")

    start = num(truth[0]["hs_pc_t"]) - num(truth[0]["hs_s"])
    heard = np.array([num(b["pc_t"]) for b in beats]) + latency
    tempo = np.array([num(b["tempo_bpm_applied"]) for b in beats])
    strike_pc = np.array([num(s["pc_t"]) for s in strikes])

    # Steps before the music started (it waits for the pace) or while it paused have no beat
    hs_all = np.array([num(t["hs_pc_t"]) for t in truth])
    music = with_music(hs_all, music_spans(read_csv(session / "transport.csv"), heard, latency))

    by_strike = {}
    for p in predictions:
        by_strike.setdefault(int(p["strike_number"]), []).append(p)

    rows, prev_hs = [], {}
    for t in truth:
        foot = int(t["foot"])
        hs = num(t["hs_pc_t"])
        offset = hs - num(t["hs_s"])
        row = {
            "step": int(t["step_index"]),
            "foot": FOOT_NAME.get(foot, str(foot)),
            "t_s": hs - start,
            "heel_strike_s": hs - start,
            "toe_off_s": num(t["to_s"]) + offset - start,
            "mid_swing_s": num(t["ms_s"]) + offset - start,
            "step_time_ms": (hs - prev_hs[foot]) * 500 if foot in prev_hs else np.nan,
            "fsr_after_hs_ms": (num(t["fsr_pc_t"]) - hs) * 1000,
            "pc_got_it_after_hs_ms": (num(t["printed_pc_t"]) - hs) * 1000 if t["delivered"] == "1" else np.nan,
            "lost_on_link": t["delivered"] != "1",
            "estimated_contact_err_ms": np.nan,
            "predicted_err_ms": np.nan,
            "predicted_method": "",
            "beat_s": np.nan,
            "beat_gap_ms": np.nan,
            "beat_tempo_bpm": np.nan,
        }
        prev_hs[foot] = hs

        if t["delivered"] == "1" and len(strike_pc):
            i = int(np.abs(strike_pc - num(t["printed_pc_t"])).argmin())
            if abs(strike_pc[i] - num(t["printed_pc_t"])) < MATCH_S:
                s = strikes[i]
                row["estimated_contact_err_ms"] = (num(s["contact_pc_t"]) - hs) * 1000
                if s.get("strike_number") not in (None, ""):
                    made_before = [p for p in by_strike.get(int(s["strike_number"]), [])
                                   if num(p["pc_t_made"]) <= hs - PREDICT_LEAD_S]
                    if made_before:
                        p = made_before[-1]
                        row["predicted_err_ms"] = (num(p["predicted_contact_pc_t"]) - hs) * 1000
                        row["predicted_method"] = p["method"]

        if len(heard) and music[len(rows)]:
            j = int(np.abs(heard - hs).argmin())
            row["beat_s"] = heard[j] - start
            row["beat_gap_ms"] = (heard[j] - hs) * 1000
            row["beat_tempo_bpm"] = tempo[j]
        rows.append(row)

    imu = read_csv(session / "sim_imu_data.csv")
    context = {"start": start, "latency": latency, "heard": heard - start, "imu": imu, "session": session,
               "aligned": bool(meta.get("phase_alignment", {}).get("enabled", False))}
    return rows, context


def save_table(rows: list[dict], path: Path) -> None:
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        for r in rows:
            writer.writerow({k: (f"{v:.1f}" if isinstance(v, float) and k.endswith("_ms") else
                                 f"{v:.3f}" if isinstance(v, float) else v) for k, v in r.items()})


def print_table(rows: list[dict]) -> None:
    def f(v, w=7):
        return f"{v:{w}.0f}" if np.isfinite(v) else " " * (w - 1) + "-"
    print(f"{'step':>4} {'foot':>5} {'time s':>7} {'step ms':>7} {'FSR +ms':>7} {'PC +ms':>7} "
          f"{'pred ms':>7} {'beat ms':>7} {'BPM':>6}")
    for r in rows:
        lost = "  lost on link" if r["lost_on_link"] else ""
        if not np.isfinite(r["beat_gap_ms"]):
            lost += "  no music (waiting for the pace / paused)"
        print(f"{r['step']:>4} {r['foot']:>5} {r['t_s']:7.2f} {f(r['step_time_ms'])} {f(r['fsr_after_hs_ms'])} "
              f"{f(r['pc_got_it_after_hs_ms'])} {f(r['predicted_err_ms'])} {f(r['beat_gap_ms'])} "
              f"{r['beat_tempo_bpm']:6.1f}{lost}")


def pitch_traces(imu: list[dict], start: float) -> dict:
    """Leg rotation (dps) on each leg's main swing axis, against seconds since the walk started."""
    if not imu:
        return {}
    t = np.array([num(r["pc_t"]) for r in imu]) - start
    traces = {}
    for foot, side in ((1, "r"), (2, "l")):
        g = np.array([[num(r[f"{side}_g{a}"]) for a in "xyz"] for r in imu]) / GYR_LSB_PER_DPS
        axis = int(np.nanargmax(np.nanvar(g, axis=0)))
        trace = g[:, axis]
        if np.nanpercentile(trace, 1) < -np.nanpercentile(trace, 99):
            trace = -trace  # swing forward drawn upwards on both legs
        traces[foot] = (t, trace)
    return traces


def plot_pages(rows: list[dict], ctx: dict, out_dir: Path, page_s: float, t_from: float, t_to: float) -> list[Path]:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    out_dir.mkdir(exist_ok=True)
    for old in out_dir.glob("page_*.png"):
        old.unlink()
    traces = pitch_traces(ctx["imu"], ctx["start"])
    end = min(t_to, max(r["t_s"] for r in rows) + 1.0)
    paths, page = [], 0
    t0 = t_from
    while t0 < end:
        t1 = t0 + page_s
        page_rows = [r for r in rows if t0 - 1.0 <= r["t_s"] <= t1 + 1.0]
        if not any(t0 <= r["t_s"] <= t1 for r in rows):
            t0 = t1
            continue
        page += 1
        fig, (ax_legs, ax_imu) = plt.subplots(2, 1, figsize=(16, 7), sharex=True,
                                              gridspec_kw={"height_ratios": [2.2, 1]})

        for b in ctx["heard"]:
            if t0 <= b <= t1:
                for ax in (ax_legs, ax_imu):
                    ax.axvline(b, color="tab:blue", lw=1.6, alpha=0.8, zorder=1)

        for r in page_rows:
            foot = 1 if r["foot"] == "right" else 2
            y = LANE[foot]
            ax_legs.plot([r["toe_off_s"], r["heel_strike_s"]], [y, y], color=COLOR[foot], lw=6, alpha=0.35,
                         solid_capstyle="butt")
            ax_legs.plot(r["toe_off_s"], y, "o", color=COLOR[foot], ms=6)
            ax_legs.plot(r["mid_swing_s"], y, "x", color=COLOR[foot], ms=8, mew=2)
            ax_legs.plot(r["heel_strike_s"], y, "v", color="black", ms=11, zorder=4)
            if np.isfinite(r["pc_got_it_after_hs_ms"]):
                ax_legs.plot(r["heel_strike_s"] + r["pc_got_it_after_hs_ms"] / 1000, y - 0.12, "s",
                             color="gray", ms=5)
            else:
                ax_legs.text(r["heel_strike_s"], y - 0.2, "lost", color="red", ha="center", fontsize=8)
            if not np.isfinite(r["beat_gap_ms"]) and t0 <= r["heel_strike_s"] <= t1:
                ax_legs.text(r["heel_strike_s"], y + 0.3, "no music", color="gray", ha="center", fontsize=8)
            if np.isfinite(r["beat_gap_ms"]) and t0 <= r["heel_strike_s"] <= t1:
                gap = r["beat_gap_ms"]
                color = "green" if abs(gap) < 30 else "darkorange" if abs(gap) < 60 else "red"
                ax_legs.annotate("", xy=(r["beat_s"], y + 0.22), xytext=(r["heel_strike_s"], y + 0.22),
                                 arrowprops=dict(arrowstyle="->", color=color, lw=1.5))
                ax_legs.text(r["heel_strike_s"], y + 0.3, f"{gap:+.0f} ms", color=color, ha="center",
                             fontsize=9, fontweight="bold")
                ax_legs.text(r["heel_strike_s"], y - 0.33, f"#{r['step']}", color="gray", ha="center", fontsize=7)

        ax_legs.set_yticks([0, 1], ["left foot", "right foot"])
        ax_legs.set_ylim(-0.6, 1.6)
        ax_legs.set_xlim(t0, t1)
        ax_legs.set_title(f"{ctx['session'].name}   {'alignment ON' if ctx['aligned'] else 'alignment OFF'}   "
                          f"{t0:.0f}-{t1:.0f} s\n"
                          "o toe-off   x mid-swing   ▼ true heel strike   ■ PC got the step   "
                          "| blue = beat heard   label = beat minus heel strike (green < 30 ms, orange < 60, red)",
                          fontsize=10)
        ax_legs.grid(axis="x", alpha=0.3)

        for foot, (t, g) in traces.items():
            m = (t >= t0) & (t <= t1)
            ax_imu.plot(t[m], g[m], color=COLOR[foot], lw=1, label=f"{FOOT_NAME[foot]} leg")
        ax_imu.set_ylabel("leg rotation (dps)")
        ax_imu.set_xlabel("seconds since the walk started")
        ax_imu.legend(loc="upper right", fontsize=8)
        ax_imu.grid(alpha=0.3)
        ax_imu.set_xlim(t0, t1)

        fig.tight_layout()
        path = out_dir / f"page_{page:02d}.png"
        fig.savefig(path, dpi=110)
        plt.close(fig)
        paths.append(path)
        t0 = t1
    return paths


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("session", nargs="?", help="session folder (default: newest simulated session)")
    parser.add_argument("--page", type=float, default=8.0, help="seconds per picture")
    parser.add_argument("--from", dest="t_from", type=float, default=0.0)
    parser.add_argument("--to", dest="t_to", type=float, default=float("inf"))
    parser.add_argument("--no-print", action="store_true")
    args = parser.parse_args()

    session = Path(args.session) if args.session else newest_sim_session(get_sim_logs_dir())
    rows, ctx = build_table(session)
    save_table(rows, session / "steps_table.csv")
    if not args.no_print:
        print_table(rows)
    pages = plot_pages(rows, ctx, session / "timeline", args.page, args.t_from, args.t_to)
    print(f"\nTable: {session / 'steps_table.csv'}")
    print(f"Pictures: {session / 'timeline'} ({len(pages)} pages)")


if __name__ == "__main__":
    main()
