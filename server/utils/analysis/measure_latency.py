"""
Measures the output latency L: the time from sending a MIDI note to hearing it
(Integration Plan §18.2, method A). Plays short clicks through the same MIDI output the
engine uses and records them with a microphone right next to the speaker (or a cable from
the audio output to an audio input). The median delay is the value for
main.py --latency-ms.

The microphone and the clicks are timed on the same PC clock; the sound card's own input
delay is taken from what the audio driver reports. A microphone 1 m from the speaker
adds about 3 ms (sound travel), so keep it close.

Needs the sounddevice package (in requirements.txt).

Usage (from the repo root):
  .venv\\Scripts\\python.exe server\\utils\\analysis\\measure_latency.py            measure
  .venv\\Scripts\\python.exe server\\utils\\analysis\\measure_latency.py --list     show devices
Results go to server/latency_logs/<time>/ (latency.json, latency.png); outside server/logs so the
GUI's session lists and the model training never pick them up.
"""
import argparse
import json
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # server/, so "utils." imports work

from utils.paths import get_latency_logs_dir

RATE = 48000
NOTE, CHANNEL = 76, 9  # high wood block on the General MIDI drum channel: short and sharp
SEARCH_S = 0.5  # look for the click this long after sending it
NOISE_FACTOR = 6.0  # a click must be this much louder than the background


def record_clicks(midi_name: str | None, device, clicks: int, gap_s: float):
    """Plays the clicks and records. Returns (blocks [(PC time of first sample, samples)], send times)."""
    import mido
    import sounddevice as sd
    mido.set_backend("mido.backends.rtmidi")

    blocks = []
    stream_latency = [0.0]

    def callback(indata, frames, time_info, status):
        now = time.perf_counter()
        if time_info.inputBufferAdcTime > 0:
            age = time_info.currentTime - time_info.inputBufferAdcTime
        else:  # the driver gives no capture time: assume the block ended one input latency ago
            age = frames / RATE + stream_latency[0]
        blocks.append((now - age, indata[:, 0].copy()))

    out = mido.open_output(midi_name) if midi_name else mido.open_output()
    rng = np.random.default_rng()
    sends = []
    try:
        with sd.InputStream(samplerate=RATE, channels=1, device=device, callback=callback, latency="low") as stream:
            stream_latency[0] = stream.latency
            time.sleep(1.0)  # background noise only, for the threshold
            for _ in range(clicks):
                sends.append(time.perf_counter())
                out.send(mido.Message("note_on", channel=CHANNEL, note=NOTE, velocity=127))
                time.sleep(0.05)
                out.send(mido.Message("note_off", channel=CHANNEL, note=NOTE))
                time.sleep(gap_s - 0.05 + rng.uniform(0, 0.2))  # irregular, so echoes cannot line up
            time.sleep(SEARCH_S)
    finally:
        out.close()
    return blocks, np.array(sends)


def find_delays(blocks, sends: np.ndarray) -> tuple[np.ndarray, list]:
    """Delay from each send time to the start of its click (NaN if no click was found)."""
    t = np.concatenate([start + np.arange(len(x)) / RATE for start, x in blocks])
    x = np.concatenate([x for _, x in blocks])
    env = np.abs(np.diff(x, prepend=x[0]))  # the difference removes hum and offset, keeps the sharp click
    background = env[t < sends[0] - 0.05]
    floor = np.percentile(background, 99) if len(background) else np.percentile(env, 50)
    delays, segments = [], []
    for s in sends:
        m = (t >= s) & (t < s + SEARCH_S)
        seg = env[m]
        if len(seg) == 0 or seg.max() < NOISE_FACTOR * floor:
            delays.append(np.nan)
            continue
        threshold = max(NOISE_FACTOR * floor, 0.3 * seg.max())
        i = int(np.argmax(seg > threshold))
        delays.append(t[m][i] - s)
        segments.append((t[m] - s, x[m]))
    return np.array(delays), segments


def save(delays: np.ndarray, segments: list, out_dir: Path) -> dict:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    ok = delays[~np.isnan(delays)] * 1000
    result = {"clicks": int(len(delays)), "found": int(len(ok))}
    if len(ok):
        result.update(median_ms=float(np.median(ok)), p25_ms=float(np.percentile(ok, 25)),
                      p75_ms=float(np.percentile(ok, 75)), min_ms=float(ok.min()), max_ms=float(ok.max()))
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "latency.json").write_text(json.dumps(result, indent=2), encoding="utf-8")

    fig, (a, b) = plt.subplots(1, 2, figsize=(13, 4.5))
    for st, sx in segments:
        a.plot(st * 1000, sx, lw=0.5, alpha=0.3)
    if len(ok):
        a.axvline(result["median_ms"], color="k", ls="--", label=f"median {result['median_ms']:.1f} ms")
        a.legend()
        b.hist(ok, bins=30, color="tab:blue")
    a.set_xlabel("Time after sending the note (ms)")
    a.set_title("Recorded clicks, aligned on the send time")
    b.set_xlabel("Latency (ms)")
    b.set_title(f"{len(ok)} of {len(delays)} clicks found")
    fig.tight_layout()
    fig.savefig(out_dir / "latency.png", dpi=100)
    plt.close(fig)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--list", action="store_true", help="Show audio inputs and MIDI outputs, then exit")
    parser.add_argument("--input", default=None, help="Audio input device number or name (default: system default)")
    parser.add_argument("--midi", default=None, help="MIDI output name (default: the system default, like main.py)")
    parser.add_argument("--clicks", type=int, default=40)
    parser.add_argument("--gap", type=float, default=0.6, help="Seconds between clicks (default 0.6)")
    args = parser.parse_args()
    try:
        import sounddevice as sd
    except ImportError:
        raise SystemExit("Needs the sounddevice package: .venv\\Scripts\\pip install sounddevice")
    import mido
    mido.set_backend("mido.backends.rtmidi")

    if args.list:
        print("Audio devices:\n", sd.query_devices())
        print("MIDI outputs:", mido.get_output_names())
        return
    device = int(args.input) if args.input and args.input.isdigit() else args.input
    print(f"Playing {args.clicks} clicks. Keep the microphone next to the speaker and the room quiet ...")
    blocks, sends = record_clicks(args.midi, device, args.clicks, args.gap)
    delays, segments = find_delays(blocks, sends)
    out_dir = get_latency_logs_dir() / datetime.now().strftime("%Y%m%d_%H%M%S")
    result = save(delays, segments, out_dir)
    if result["found"] < max(5, args.clicks // 2):
        print(f"Only {result['found']} of {result['clicks']} clicks heard: turn the volume up, move the "
              "microphone closer, or pick the right input with --input (see --list)")
    if "median_ms" in result:
        print(f"Latency: median {result['median_ms']:.1f} ms (middle half {result['p25_ms']:.1f}-"
              f"{result['p75_ms']:.1f} ms), from {result['found']} clicks")
        print(f"Use it with:  main.py --latency-ms {result['median_ms']:.0f}")
    print(f"Saved to {out_dir}")


if __name__ == "__main__":
    main()
