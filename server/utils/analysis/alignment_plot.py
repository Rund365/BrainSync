"""
One simple figure per session: how far each foot strike is from the nearest beat heard.

  python utils/analysis/alignment_plot.py logs/STRAVA/session_2026-11-01_20-16-12 [more sessions...]

Writes "<session folder name> - alignment.png" into each session folder.
Beat heard = beat sent (beats.csv) + output latency; foot strike = estimated contact time (strikes.csv).
"""
import csv
import json
import re
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

BAND_MS = 30
DEFAULT_LATENCY_MS = 30.0


def read_csv(path):
    if not path.exists():
        return []
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def session_info(folder: Path):
    """(alignment on?, latency in ms) from meta.json, else from the session log."""
    meta_path = folder / "meta.json"
    meta = json.loads(meta_path.read_text(encoding="utf-8")) if meta_path.exists() else {}
    log = (folder / "session_log.txt").read_text(encoding="utf-8", errors="ignore") if (folder / "session_log.txt").exists() else ""
    on = re.search(r"Phase alignment (on|off)", log)
    latency = re.search(r"output latency (\d+(?:\.\d+)?) ms", log)
    aligned = meta.get("alignment_enabled", on.group(1) == "on" if on else None)
    latency_ms = meta.get("latency_ms", float(latency.group(1)) if latency else DEFAULT_LATENCY_MS)
    return aligned, float(latency_ms)


def strike_errors(folder: Path, latency_s: float):
    """Times (s from the first beat) and errors (ms) of the strikes taken while the music played."""
    beats = np.array(sorted(float(r["pc_t"]) for r in read_csv(folder / "beats.csv"))) + latency_s
    contacts = np.array([float(r["contact_pc_t"] or r["pc_t"]) for r in read_csv(folder / "strikes.csv")])
    if len(beats) < 2 or len(contacts) == 0:
        return np.array([]), np.array([]), contacts, beats
    period = np.median(np.diff(beats))
    playing = (contacts >= beats[0] - period / 2) & (contacts <= beats[-1] + period / 2)
    c = contacts[playing]
    idx = np.clip(np.searchsorted(beats, c), 1, len(beats) - 1)
    nearest = np.where(np.abs(beats[idx] - c) < np.abs(beats[idx - 1] - c), beats[idx], beats[idx - 1])
    return c - beats[0], (nearest - c) * 1000, contacts, beats


def plot(folder: Path) -> str:
    aligned, latency_ms = session_info(folder)
    t, err, contacts, beats = strike_errors(folder, latency_ms / 1000)
    if len(err) == 0:
        return f"{folder.name}: no music or no steps, skipped"
    a = np.abs(err)
    median, within = np.median(a), np.mean(a <= BAND_MS) * 100
    cadence = 60 / np.median(np.diff(contacts)) if len(contacts) > 2 else 0
    before_music = int(np.sum(contacts < beats[0] - np.median(np.diff(beats)) / 2))

    mode = "IMU prediction + phase alignment" if aligned else "tempo following only (no alignment)"
    color = "tab:blue" if aligned else "tab:red"
    fig, ax = plt.subplots(figsize=(14, 4.2))
    ax.axhspan(-BAND_MS, BAND_MS, color="tab:green", alpha=0.12, label=f"±{BAND_MS} ms (feels together)")
    ax.axhline(0, color="black", lw=0.8)
    ax.scatter(t, err, s=12, color=color,
               label=f"{mode}: median {median:.0f} ms, {within:.0f}% within ±{BAND_MS} ms")
    ax.set_xlabel("Time since the music started (s)")
    ax.set_ylabel("Beat heard − foot strike (ms)")
    ax.set_ylim(-300, 300)
    ax.grid(alpha=0.3)
    ax.legend(loc="lower left", fontsize=9)
    stamp = folder.name.replace("session_", "")
    fig.suptitle(f"BrainSync session {stamp}: beat-to-step alignment ({mode})", fontsize=12)
    ax.set_title(f"How far each foot strike is from the nearest beat (0 = beat exactly on the strike) · "
                 f"{len(err)} steps with music, ~{cadence:.0f} steps/min"
                 + (f", {before_music} steps before the music started" if before_music else ""), fontsize=9.5)
    out_of_range = int(np.sum(a > 300))
    if out_of_range:
        ax.text(0.99, 0.02, f"{out_of_range} points beyond ±300 ms not shown", transform=ax.transAxes,
                ha="right", fontsize=8, color="gray")
    fig.tight_layout()
    fig.savefig(folder / f"{folder.name} - alignment.png", dpi=150)
    plt.close(fig)
    return (f"{folder.name}: alignment {'on' if aligned else 'off'}, {len(err)} steps, "
            f"median {median:.0f} ms, {within:.0f}% within ±{BAND_MS} ms, "
            f"{np.mean(a <= 50) * 100:.0f}% within ±50 ms, mean {np.mean(err):+.0f} ms")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    for arg in sys.argv[1:]:
        print(plot(Path(arg)))
