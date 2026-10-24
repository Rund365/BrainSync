"""
Generate a simulated walking recording offline (faster than real time), without the engine.

Writes to server/sim_logs/generated/<time>_<scenario>/:
  serial_capture.txt   every line the receiver printed, with the PC arrival time
  sim_imu_data.csv     IMU samples (planned imu_data.csv format, raw LSB, sensor frame)
  sim_steps.csv        true events of every step (heel strike, FSR, toe-off, mid-swing, lift)
  sim_meta.json        the simulated person, sensors and link
  stride_overlay.png   strides of each foot overlaid, aligned on the true heel strike

Usage (from the repo root):
  .venv\\Scripts\\python.exe server\\utils\\simulation\\generate.py mixed --seed 1
"""
import argparse
import sys
from datetime import datetime
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # server/, so "utils." imports work

from utils.paths import get_sim_logs_dir
from utils.simulation.legs import SAMPLE_PERIOD_S
from utils.simulation.simulated_receiver import SimulatedReceiver
from utils.simulation.walker import Walker, parse_scenario, RIGHT, LEFT

TICK_S = 0.001


class ManualClock:
    def __init__(self):
        self.t = 0.0

    def __call__(self) -> float:
        return self.t


def generate(scenario: str, seed: int | None, out_dir: Path, realistic: bool = True) -> SimulatedReceiver:
    clock = ManualClock()
    receiver = SimulatedReceiver(Walker(parse_scenario(scenario), realistic=realistic, seed=seed), protocol=2, seed=seed,
                                 time_source=clock)
    receiver.timeout = 0
    receiver.write(b"RESET\nSTART\n")

    out_dir.mkdir(parents=True, exist_ok=True)
    with open(out_dir / "serial_capture.txt", "w", encoding="utf-8") as capture:
        while not receiver.finished:
            clock.t += TICK_S
            while line := receiver.readline():
                capture.write(f"{clock.t:.4f} {line.decode().strip()}\n")
    receiver.write_truth(out_dir)
    return receiver


def plot_stride_overlay(receiver: SimulatedReceiver, path: Path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    legs = receiver.legs
    window = np.arange(-0.9, 0.35, SAMPLE_PERIOD_S)
    fig, axes = plt.subplots(2, 2, figsize=(12, 7), sharex=True)
    for col, (foot, name) in enumerate([(RIGHT, "Right"), (LEFT, "Left")]):
        strikes = [s for s in legs.strikes if s.foot == foot]
        for s in strikes:
            t = s.hs + window
            axes[0, col].plot(window, np.interp(t, legs.t, legs.gyr_foot[foot][:, 1]), color="tab:blue", alpha=0.25)
            axes[1, col].plot(window, np.interp(t, legs.t, np.linalg.norm(legs.acc_foot[foot], axis=1)),
                              color="tab:red", alpha=0.25)
        to_lead = np.mean([s.to - s.hs for s in strikes])
        ms_lead = np.mean([s.ms - s.hs for s in strikes])
        for ax in axes[:, col]:
            ax.axvline(0, color="k", lw=1)
            ax.axvline(to_lead, color="tab:green", ls="--", lw=1)
            ax.axvline(ms_lead, color="tab:purple", ls="--", lw=1)
            ax.grid(alpha=0.3)
        axes[0, col].set_title(f"{name} foot ({len(strikes)} strikes): "
                               f"TO {to_lead * 1000:.0f} ms, MS {ms_lead * 1000:.0f} ms before HS")
    axes[0, 0].set_ylabel("Pitch rate (dps)")
    axes[1, 0].set_ylabel("|acceleration| (m/s²)")
    for ax in axes[1]:
        ax.set_xlabel("Time from true heel strike (s)   green = toe-off, purple = mid-swing")
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def summary(receiver: SimulatedReceiver) -> str:
    legs = receiver.legs
    lines = [f"steps {len(legs.strikes)}, IMU samples received {len(receiver.emitted_imu)} "
             f"of {len(legs.t) // receiver.link.batch_n * receiver.link.batch_n}, "
             f"STEP lines received {len(receiver.emitted_steps)}"]
    for foot, name in [(RIGHT, "right"), (LEFT, "left")]:
        s = [x for x in legs.strikes if x.foot == foot]
        lines.append(f"{name}: TO {np.mean([x.hs - x.to for x in s]) * 1000:.0f} ms and "
                     f"MS {np.mean([x.hs - x.ms for x in s]) * 1000:.0f} ms before HS, "
                     f"FSR {np.mean([x.fsr - x.hs for x in s]) * 1000:.0f} ms after HS")
    lines.append(f"sender clock drift {receiver.link.drift_ppm:+.1f} ppm")
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("scenario", help="Preset (steady, speedup, stop_start, mixed) or 'cadence:seconds,...'")
    parser.add_argument("--seed", type=int, default=None, help="Same seed = same person, sensors and walk")
    parser.add_argument("--out", type=Path, default=None, help="Output folder")
    parser.add_argument("--sharp", action="store_true", help="Stress-test walker: instant stops and starts, no drift")
    args = parser.parse_args()

    name = "".join(c if c.isalnum() else "_" for c in args.scenario)
    out_dir = args.out or get_sim_logs_dir() / "generated" / f"{datetime.now():%Y%m%d_%H%M%S}_{name}"
    receiver = generate(args.scenario, args.seed, out_dir, realistic=not args.sharp)
    plot_stride_overlay(receiver, out_dir / "stride_overlay.png")
    print(summary(receiver))
    print(f"Saved to {out_dir}")


if __name__ == "__main__":
    main()
