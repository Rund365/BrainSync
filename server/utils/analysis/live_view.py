"""
Live check by eye and ear: runs a simulated walk with sound and shows two foot pads that
light up at the exact moment each heel hits the ground. The music is a click on every
beat, so when the system works, each flash and each click happen together.

Under the pads: how far the last beat was from the step (green = in sync, under 30 ms).

Usage (from the repo root):
  .venv\\Scripts\\python.exe server\\utils\\analysis\\live_view.py [scenario] [--seed 1] [--align off] [--song FILE]
      [--latency-ms 150] [--max-tempo-change 5]
scenario: steady, speedup, stop_start, mixed (default steady), or 'cadence:seconds,...'.
--align off plays today's system (tempo following only), to hear the difference.
Esc or closing the window stops the run.
"""
import argparse
import csv
import os
import subprocess
import sys
import time
import tkinter as tk
from datetime import datetime
from pathlib import Path

import mido

SERVER = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SERVER))

from utils.paths import get_midi_dir, get_sim_logs_dir
from utils.simulation.simulated_receiver import PLAN_ENV

CLICK_BPM = 100
CLICK_NOTE = 76  # General MIDI hi wood block
FLASH_S = 0.15
GAP_WAIT_S = 0.35  # the beat log is written every 0.1 s; wait for the beats around the step
IN_SYNC_MS = 30
CLOSE_MS = 60
RECENT = 20

BG, PAD_OFF, PAD_ON, TEXT = "#15171c", "#2a2e37", "#f5f5f5", "#c8ccd4"
GREEN, ORANGE, RED = "#3ecf6e", "#f0a030", "#ef4b4b"


def click_track() -> Path:
    """A song that is one wood-block click per beat."""
    path = get_midi_dir() / "click_track.mid"
    if path.exists():
        return path
    mid = mido.MidiFile(ticks_per_beat=480)
    track = mido.MidiTrack()
    mid.tracks.append(track)
    track.append(mido.MetaMessage("set_tempo", tempo=mido.bpm2tempo(CLICK_BPM), time=0))
    track.append(mido.Message("program_change", channel=9, program=0, time=0))
    for _ in range(64):
        track.append(mido.Message("note_on", channel=9, note=CLICK_NOTE, velocity=120, time=0))
        track.append(mido.Message("note_off", channel=9, note=CLICK_NOTE, velocity=0, time=60))
        track.append(mido.Message("note_on", channel=9, note=CLICK_NOTE, velocity=0, time=420))
    mid.save(path)
    return path


class Tail:
    """Reads the rows added to a CSV file that another process is writing."""

    def __init__(self, path: Path):
        self.path = path
        self._pos = 0
        self._header = None
        self._partial = ""

    def new_rows(self) -> list[dict]:
        if not self.path.exists():
            return []
        with open(self.path, encoding="utf-8", newline="") as f:
            f.seek(self._pos)
            chunk = f.read()
            self._pos = f.tell()
        text = self._partial + chunk
        lines = text.split("\n")
        self._partial = lines.pop()
        rows = []
        for line in lines:
            line = line.strip()
            if not line:
                continue
            values = next(csv.reader([line]))
            if self._header is None:
                self._header = values
            else:
                rows.append(dict(zip(self._header, values)))
        return rows


class LiveView:
    def __init__(self, args):
        self.args = args
        out = get_sim_logs_dir() / "live"
        out.mkdir(exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.plan_path = out / f"plan_{stamp}.csv"
        self.console_path = out / f"console_{stamp}.txt"
        self.latency = args.latency_ms / 1000.0

        env = dict(os.environ, **{PLAN_ENV: str(self.plan_path)})
        cmd = [sys.executable, str(SERVER / "main.py"), str(args.song), "--simulate", args.scenario,
               "--phase-align", args.align, "--latency-ms", str(args.latency_ms),
               "--session-name", f"live_{stamp}"]
        if args.seed is not None:
            cmd += ["--sim-seed", str(args.seed)]
        if args.max_tempo_change is not None:
            cmd += ["--max-tempo-change", str(args.max_tempo_change)]
        self._console = open(self.console_path, "w", encoding="utf-8")
        self.proc = subprocess.Popen(cmd, cwd=SERVER.parent, env=env, stdin=subprocess.DEVNULL,
                                     stdout=self._console, stderr=subprocess.STDOUT)

        self.steps = []  # (pc time, foot), in order
        self.next_step = 0
        self.flash_until = {1: 0.0, 2: 0.0}
        self.pending = []  # steps waiting for their beat
        self.beats = []
        self.beat_tail = None
        self.gaps = []
        self.session = None

        self.root = tk.Tk()
        self.root.title("BrainSync live check")
        self.root.configure(bg=BG)
        self.root.geometry("820x520")
        self.root.bind("<Escape>", lambda e: self.close())
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        mode = "IMU + phase alignment" if args.align != "off" else "today's system (tempo only)"
        tk.Label(self.root, text=f"{args.scenario}   |   {mode}", bg=BG, fg=TEXT,
                 font=("Segoe UI", 12)).pack(pady=(14, 0))
        self.canvas = tk.Canvas(self.root, bg=BG, highlightthickness=0, width=800, height=300)
        self.canvas.pack()
        self.pads = {
            2: self.canvas.create_rectangle(110, 30, 370, 280, fill=PAD_OFF, outline="", width=0),
            1: self.canvas.create_rectangle(430, 30, 690, 280, fill=PAD_OFF, outline="", width=0),
        }
        self.canvas.create_text(240, 155, text="LEFT", fill=TEXT, font=("Segoe UI", 26, "bold"), tags="label2")
        self.canvas.create_text(560, 155, text="RIGHT", fill=TEXT, font=("Segoe UI", 26, "bold"), tags="label1")
        self.gap_label = tk.Label(self.root, text="starting ...", bg=BG, fg=TEXT, font=("Segoe UI", 22, "bold"))
        self.gap_label.pack(pady=(6, 0))
        self.score_label = tk.Label(self.root, text="", bg=BG, fg=TEXT, font=("Segoe UI", 13))
        self.score_label.pack(pady=(4, 0))
        tk.Label(self.root, text="Flash = heel hits the ground.  In sync = you hear the click at the same moment.",
                 bg=BG, fg="#80858f", font=("Segoe UI", 10)).pack(side="bottom", pady=10)

        self.root.after(5, self.tick)
        self.root.after(100, self.slow_tick)

    # ----------- Fast loop: flashes -----------
    def tick(self):
        now = time.perf_counter()
        while self.next_step < len(self.steps) and self.steps[self.next_step][0] <= now:
            t, foot = self.steps[self.next_step]
            self.next_step += 1
            if now - t < 0.05:  # a step missed by a stall of the window is not flashed late
                self.flash_until[foot] = t + FLASH_S
            self.pending.append((t, foot))
        for foot, pad in self.pads.items():
            on = now < self.flash_until[foot]
            self.canvas.itemconfigure(pad, fill=PAD_ON if on else PAD_OFF)
            self.canvas.itemconfigure(f"label{foot}", fill=BG if on else TEXT)
        self.root.after(2, self.tick)

    # ----------- Slow loop: files, gaps, end of run -----------
    def slow_tick(self):
        if not self.steps and self.plan_path.exists():
            with open(self.plan_path, encoding="utf-8", newline="") as f:
                self.steps = sorted((float(r["hs_pc_t"]), int(r["foot"])) for r in csv.DictReader(f))
            self.gap_label.configure(text="walking ...", fg=TEXT)
        if self.session is None:
            self.session = self._find_session()
            if self.session is not None:
                self.beat_tail = Tail(self.session / "beats.csv")
        if self.beat_tail is not None:
            self.beats += [float(r["pc_t"]) + self.latency for r in self.beat_tail.new_rows()]
        self._score_steps()

        if self.proc.poll() is not None:
            done = "walk finished" if self.proc.returncode == 0 else f"main.py stopped (see {self.console_path})"
            self.score_label.configure(text=f"{self.score_label.cget('text')}   |   {done}")
            return
        self.root.after(100, self.slow_tick)

    def _find_session(self):
        try:
            text = self.console_path.read_text(encoding="utf-8", errors="replace")
        except OSError:
            return None
        for line in text.splitlines():
            if line.startswith("SESSION_DIR:"):
                return Path(line.split(":", 1)[1].strip())
        return None

    def _score_steps(self):
        now = time.perf_counter()
        while self.pending and now - self.pending[0][0] > GAP_WAIT_S:
            t, foot = self.pending.pop(0)
            near = [b for b in self.beats if abs(b - t) < 0.6]
            if not near:
                self.gap_label.configure(text="no beat near this step", fg=RED)
                continue
            gap = (min(near, key=lambda b: abs(b - t)) - t) * 1000
            self.gaps.append(gap)
            color = GREEN if abs(gap) < IN_SYNC_MS else ORANGE if abs(gap) < CLOSE_MS else RED
            when = "late" if gap > 0 else "early"
            side = "left" if foot == 2 else "right"
            self.gap_label.configure(text=f"{side} step: beat {abs(gap):.0f} ms {when}", fg=color)
            recent = self.gaps[-RECENT:]
            in_sync = sum(abs(g) < IN_SYNC_MS for g in recent)
            self.score_label.configure(text=f"last {len(recent)} steps: {in_sync} in sync (under {IN_SYNC_MS} ms)")
            print(f"{side:>5}  beat {gap:+5.0f} ms", flush=True)

    def close(self):
        if self.proc.poll() is None:
            self.proc.terminate()
        self.root.destroy()

    def run(self):
        self.root.mainloop()
        if self.proc.poll() is None:
            self.proc.terminate()
        self._console.close()
        if self.gaps:
            inside = sum(abs(g) < IN_SYNC_MS for g in self.gaps)
            print(f"\n{inside} of {len(self.gaps)} steps in sync (under {IN_SYNC_MS} ms)")
        if self.session:
            print(f"Session: {self.session}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("scenario", nargs="?", default="steady")
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--align", choices=("on", "off"), default="on")
    parser.add_argument("--song", type=Path, default=None, help="MIDI file (default: a click on every beat)")
    parser.add_argument("--latency-ms", type=float, default=30.0, help="sound output delay, as in main.py")
    parser.add_argument("--max-tempo-change", type=float, default=None,
                        help="largest tempo change from one beat to the next, in percent, as in main.py")
    args = parser.parse_args()
    args.song = args.song or click_track()
    LiveView(args).run()


if __name__ == "__main__":
    main()
