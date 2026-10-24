"""
Before / after comparison on the real engine, in real time and silent.

Runs main.py twice on the same simulated person and walk:
  before  phase alignment off: today's system, the music only follows the tempo
  after   phase alignment on:  IMU strike prediction + phase controller
then measures, for every true foot strike, how far the nearest heard beat was from it,
and saves a comparison plot.

Usage (from the repo root):
  .venv\\Scripts\\python.exe server\\utils\\simulation\\compare.py mixed --seed 1
"""
import argparse
import csv
import json
import subprocess
import sys
from datetime import datetime
from pathlib import Path

import numpy as np

SERVER = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SERVER))

from utils.paths import get_sim_logs_dir
from utils.simulation.fast_loop import with_music
from utils.simulation.walker import Walker, parse_scenario

RUNS = (("before", "off"), ("after", "on"))
COLORS = {"before": "tab:red", "after": "tab:blue"}
LABELS = {"before": "Before: tempo following only (today)", "after": "After: IMU prediction + phase alignment"}


def run_sessions(scenario: str, seed: int, stamp: str, out_dir: Path, sharp: bool = False) -> dict:
    # Output goes to files: a pipe that is not read while the other run is going would fill up
    # and freeze that run's main loop
    procs = {}
    for name, align in RUNS:
        cmd = [sys.executable, str(SERVER / "main.py"), "--simulate", scenario, "--sim-seed", str(seed),
               "--no-audio", "--phase-align", align, "--session-name", f"compare_{stamp}_{name}"]
        if sharp:
            cmd.append("--sim-sharp")
        console = open(out_dir / f"{name}_console.txt", "w", encoding="utf-8")
        procs[name] = (subprocess.Popen(cmd, cwd=SERVER.parent, stdin=subprocess.DEVNULL, stdout=console,
                                        stderr=subprocess.STDOUT), console)
    dirs = {}
    for name, (proc, console) in procs.items():
        proc.wait()
        console.close()
        out = (out_dir / f"{name}_console.txt").read_text(encoding="utf-8", errors="replace")
        session = next((line.split(":", 1)[1].strip() for line in out.splitlines()
                        if line.startswith("SESSION_DIR:")), None)
        if proc.returncode != 0 or session is None or "EXIT_CLEAN" not in out:
            raise RuntimeError(f"{name} run failed:\n{out[-3000:]}")
        dirs[name] = Path(session)
    return dirs


def read_csv(path: Path) -> list[dict]:
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def analyze(session: Path) -> dict:
    meta = json.loads((session / "meta.json").read_text(encoding="utf-8"))
    latency = meta["phase_alignment"]["settings"]["latency_s"]
    truth = read_csv(session / "sim_steps.csv")
    beats = read_csv(session / "beats.csv")
    sim_meta = json.loads((session / "sim_meta.json").read_text(encoding="utf-8"))

    hs_pc = np.array([float(r["hs_pc_t"]) for r in truth])
    start = hs_pc[0] - float(truth[0]["hs_s"])  # PC time of the walk start
    heard = np.array([float(b["pc_t"]) for b in beats]) + latency
    nearest = heard[np.abs(heard[None, :] - hs_pc[:, None]).argmin(axis=1)]
    music = with_music(hs_pc, music_spans(read_csv(session / "transport.csv"), heard, latency))
    index = np.array([int(b["beat_index"]) for b in beats])
    return {
        "strike_t": (hs_pc - start)[music],
        "async_ms": ((nearest - hs_pc) * 1000)[music],
        "silent_steps": int((~music).sum()),
        "beat_t": heard - start,
        "beat_tempo": np.array([float(b["tempo_bpm_applied"]) for b in beats]),
        "beat_consecutive": np.diff(index) == 1,  # False across a pause
        "latency_s": latency,
        "tracking": meta["strike_tracking"],
        "person": sim_meta["person"],
    }


def music_spans(transport: list[dict], heard: np.ndarray, latency: float) -> list[tuple[float, float]]:
    """(first, last) heard time of each stretch the music played. Without transport events
    (today's system) the music played from the first beat to the last."""
    if len(heard) == 0:
        return []
    if not transport:
        return [(heard[0], heard[-1])]
    spans, begin = [], None
    for event in transport:
        if event["event"] == "start":
            begin = float(event["beat_pc_t"]) + latency
        elif event["event"] == "pause" and begin is not None:
            spans.append((begin, float(event["pc_t"]) + latency))
            begin = None
    if begin is not None:
        spans.append((begin, heard[-1]))
    return spans


def stats(a: np.ndarray) -> dict:
    a = np.abs(a)
    return {"median": float(np.median(a)), "mean": float(a.mean()),
            "within30": float(np.mean(a < 30) * 100), "within50": float(np.mean(a < 50) * 100)}


def first_seconds(r: dict, seconds: float = 12.0) -> dict:
    """Error over the first seconds of the walk (the start-up), strikes with music only."""
    inside = r["strike_t"] < seconds
    return stats(r["async_ms"][inside]) if inside.any() else {}


def tempo_jumps(r: dict) -> float:
    """Smoothness: average change of the music tempo from one beat to the next (BPM)."""
    return float(np.mean(np.abs(np.diff(r["beat_tempo"]))[r["beat_consecutive"]]))


def steady_jumps(r: dict, walker) -> float:
    """Tempo jumps while the walker keeps an even pace (cadence changing < 0.3 BPM/s), i.e.
    wobble only, without the changes that follow a real pace change."""
    t = r["beat_t"]
    cadence = np.array([walker.cadence_at(x) for x in t])
    steady = (cadence > 0) & (np.abs(np.gradient(cadence, t)) < 0.3)
    both = steady[1:] & steady[:-1] & r["beat_consecutive"]
    return float(np.mean(np.abs(np.diff(r["beat_tempo"]))[both])) if both.any() else float("nan")


def plot(results: dict, scenario: str, seed: int, path: Path, sharp: bool = False):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    walker = Walker(parse_scenario(scenario), realistic=not sharp, seed=seed)
    fig = plt.figure(figsize=(15, 11))
    grid = fig.add_gridspec(3, 2, height_ratios=[1.2, 1, 1])

    # 1. Beat - strike over the walk
    ax = fig.add_subplot(grid[0, :])
    ax.axhspan(-30, 30, color="tab:green", alpha=0.12, label="±30 ms (feels together)")
    for name, r in results.items():
        s = stats(r["async_ms"])
        silent = f", {r['silent_steps']} steps before the music started / while paused" if r["silent_steps"] else ""
        ax.plot(r["strike_t"], r["async_ms"], "o", ms=3.5, color=COLORS[name],
                label=f"{LABELS[name]}: median {s['median']:.0f} ms, {s['within30']:.0f}% within ±30 ms{silent}")
    ax.axhline(0, color="k", lw=0.8)
    ax.set_ylabel("Beat heard − foot strike (ms)")
    ax.set_xlabel("Time (s)")
    ax.set_title("How far each foot strike is from the nearest beat (0 = beat exactly on the strike)")
    ax.legend(loc="lower left", fontsize=9)
    ax.grid(alpha=0.3)
    _shade_scenario(ax, walker)

    # 2. Histogram
    ax = fig.add_subplot(grid[1, 0])
    bins = np.arange(-300, 301, 10)
    for name, r in results.items():
        ax.hist(np.clip(r["async_ms"], -300, 300), bins=bins, color=COLORS[name], alpha=0.6, label=name)
    ax.axvspan(-30, 30, color="tab:green", alpha=0.12)
    ax.set_xlabel("Beat heard − foot strike (ms)")
    ax.set_ylabel("Strikes")
    ax.set_title("Distribution")
    ax.legend()
    ax.grid(alpha=0.3)

    # 3. Tempo
    ax = fig.add_subplot(grid[1, 1])
    t = np.linspace(0, walker.duration, 800)
    cadence = np.array([walker.cadence_at(x) for x in t])
    cadence[cadence == 0] = np.nan  # standing: no line, instead of a drop to 0
    ax.plot(t, cadence, color="k", lw=2, alpha=0.5, label="Walker cadence")
    for name, r in results.items():
        tempo = r["beat_tempo"].copy()
        gaps = np.flatnonzero(~r["beat_consecutive"]) + 1
        t_beats = np.insert(r["beat_t"], gaps, np.nan)  # no line across a pause
        tempo = np.insert(tempo, gaps, np.nan)
        ax.plot(t_beats, tempo, color=COLORS[name], lw=1,
                label=f"Music tempo, {name}: jumps {tempo_jumps(r):.1f} BPM per beat")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("BPM")
    ax.set_title("Tempo: does the music follow the walker's pace?")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    # 4. Zoom: strikes vs beats
    window = _typical_window(walker, results["before"])
    for col, (name, r) in enumerate(results.items()):
        ax = fig.add_subplot(grid[2, col])
        strikes = r["strike_t"][(r["strike_t"] > window[0]) & (r["strike_t"] < window[1])]
        beats = r["beat_t"][(r["beat_t"] > window[0]) & (r["beat_t"] < window[1])]
        ax.vlines(strikes, 0.5, 1.0, color="black", lw=2, label="Foot strike")
        ax.vlines(beats, 0.0, 0.5, color=COLORS[name], lw=2, label="Beat heard")
        ax.set_xlim(*window)
        ax.set_yticks([])
        ax.set_xlabel("Time (s)")
        ax.set_title(f"Zoom, {name}: strikes (top) vs beats (bottom)")
        ax.legend(loc="upper right", fontsize=8)

    fig.suptitle(f"BrainSync simulation '{scenario}' (seed {seed}): beat-to-step alignment, before vs after IMU",
                 fontsize=13)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def _shade_scenario(ax, walker):
    start = 0.0
    for seg in walker.segments:
        ax.text(start + seg.duration / 2, 0.98, "standing" if seg.cadence == 0 else f"{seg.cadence:g} steps/min",
                transform=ax.get_xaxis_transform(), ha="center", va="top", fontsize=8, color="0.3")
        if start > 0:
            ax.axvline(start, color="0.6", ls=":", lw=1)
        start += seg.duration


def _typical_window(walker, before, length=6.0):
    """A steady-walking window where 'before' is as far off as it typically is (not hand-picked)."""
    candidates = []
    start = 0.0
    for seg in walker.segments:
        t = start + 4.0  # skip the ramp at the segment start
        while seg.cadence > 0 and t + length <= start + seg.duration - 0.5:
            candidates.append((t, t + length))
            t += 1.0
        start += seg.duration
    if not candidates:
        return 2.0, 2.0 + length
    typical = np.median(np.abs(before["async_ms"]))

    def window_error(w):
        inside = (before["strike_t"] > w[0]) & (before["strike_t"] < w[1])
        return abs(np.median(np.abs(before["async_ms"][inside])) - typical) if inside.any() else np.inf

    return min(candidates, key=window_error)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("scenario", nargs="?", default="mixed")
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--sharp", action="store_true", help="Stress-test walker: instant stops and starts, no drift")
    args = parser.parse_args()

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    walker = Walker(parse_scenario(args.scenario), realistic=not args.sharp, seed=args.seed)
    name = "".join(c if c.isalnum() else "_" for c in args.scenario)
    out_dir = get_sim_logs_dir() / "compare" / f"{stamp}_{name}"
    out_dir.mkdir(parents=True, exist_ok=True)
    print(f"Running before and after in parallel, about {walker.duration + 15:.0f} s ...", flush=True)
    dirs = run_sessions(args.scenario, args.seed, stamp, out_dir, sharp=args.sharp)
    results = {name: analyze(d) for name, d in dirs.items()}

    plot(results, args.scenario, args.seed, out_dir / "comparison.png", sharp=args.sharp)
    summary = {name: {**stats(r["async_ms"]), "tempo_jumps_bpm": tempo_jumps(r),
                      "steady_jumps_bpm": steady_jumps(r, walker), "first_12s": first_seconds(r),
                      "steps_without_music": r["silent_steps"]} for name, r in results.items()}
    (out_dir / "summary.json").write_text(json.dumps(
        {"scenario": args.scenario, "seed": args.seed, "sessions": {k: str(v) for k, v in dirs.items()},
         "beat_minus_strike_ms": summary, "learned": results["after"]["tracking"]}, indent=2), encoding="utf-8")

    for name, s in summary.items():
        print(f"{name:6s}: median {s['median']:5.1f} ms, mean {s['mean']:5.1f} ms, "
              f"within ±30 ms {s['within30']:3.0f}%, within ±50 ms {s['within50']:3.0f}%, "
              f"tempo jumps {s['tempo_jumps_bpm']:.2f} BPM per beat ({s['steady_jumps_bpm']:.2f} at an even pace), "
              f"first 12 s median {s['first_12s'].get('median', float('nan')):.0f} ms, "
              f"{s['steps_without_music']} steps without music")
    print(f"Plot: {out_dir / 'comparison.png'}")


if __name__ == "__main__":
    main()
