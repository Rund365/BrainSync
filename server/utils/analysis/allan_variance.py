"""
Gyro noise of each IMU from a recording where the IMUs lie still (test 1: on a table, ~15 min).

Allan variance as described in O. J. Woodman, "An introduction to inertial navigation",
Univ. of Cambridge Tech. Report UCAM-CL-TR-696, 2007, §5.1:
  - split the still recording into n bins of length tau and average each bin: a_1 .. a_n
  - AVAR(tau) = 1 / (2 (n - 1)) * sum_i (a_(i+1) - a_i)^2      (at least 9 bins per tau)
  - plot the Allan deviation sqrt(AVAR) against tau on log-log axes
  - angle random walk (white noise) = the slope -0.5 part of the curve, read at tau = 1 s
  - bias instability = the minimum of the curve (Woodman §5.1 reads it there)

Usage (from server/):
    python utils/analysis/allan_variance.py <session folder> [--start S] [--end S] [--out allan.png]
--start / --end cut the still part, in seconds from the start of the recording.
"""
import argparse
from pathlib import Path

import numpy as np
import pandas as pd

GYR_LSB_PER_DPS = 16.0  # BNO055 gyro output in dps units
IMUS = {"l": "left", "r": "right"}  # imu_data.csv column prefix -> IMU label
AXES = "xyz"
MIN_BINS = 9


def allan_deviation(x: np.ndarray, fs: float):
    """Non-overlapping Allan deviation of x (one axis), for log-spaced tau."""
    n = len(x)
    ms = np.unique(np.logspace(0, np.log10(n // MIN_BINS), 60).astype(int))
    taus, adev = [], []
    for m in ms:
        bins = n // m
        if bins < MIN_BINS:
            break
        a = x[: bins * m].reshape(bins, m).mean(axis=1)
        avar = np.sum(np.diff(a) ** 2) / (2 * (bins - 1))
        taus.append(m / fs)
        adev.append(np.sqrt(avar))
    return np.array(taus), np.array(adev)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session", type=Path)
    ap.add_argument("--start", type=float, default=0.0)
    ap.add_argument("--end", type=float, default=None)
    ap.add_argument("--out", type=Path, default=None, help="graph file (png)")
    args = ap.parse_args()

    imu = pd.read_csv(args.session / "imu_data.csv")
    t = imu["sender_t_us"].to_numpy() / 1e6
    t = t - t[0]
    keep = (t >= args.start) & (t <= (args.end if args.end is not None else t[-1]))
    fs = 1.0 / np.median(np.diff(t))
    print(f"{args.session.name}: {keep.sum()} samples, {keep.sum() / fs:.0f} s, {fs:.1f} Hz")

    results = {}
    for p, label in IMUS.items():
        ok = keep & (imu[f"valid_{p}"].to_numpy() == 1)
        gyr = imu.loc[ok, [f"{p}_gx", f"{p}_gy", f"{p}_gz"]].to_numpy(float) / GYR_LSB_PER_DPS
        for k, ax in enumerate(AXES):
            taus, adev = allan_deviation(gyr[:, k], fs)
            results[(label, ax)] = (taus, adev)
            at1 = np.interp(0.0, np.log10(taus), np.log10(adev)) if taus[0] <= 1 <= taus[-1] else np.nan
            i = int(np.argmin(adev))
            print(f"  IMU '{label}' gyro {ax}: mean {gyr[:, k].mean():+.3f} dps, std {gyr[:, k].std():.3f} dps, "
                  f"Allan dev at 1 s {10 ** at1:.4f} dps, minimum {adev[i]:.4f} dps at {taus[i]:.1f} s")

    if args.out:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(1, 2, figsize=(12, 4.5), sharey=True)
        for axp, label in zip(axes, IMUS.values()):
            for ax in AXES:
                taus, adev = results[(label, ax)]
                axp.loglog(taus, adev, label=f"gyro {ax}")
            axp.set_title(f"IMU labelled '{label}'")
            axp.set_xlabel("tau (s)")
            axp.grid(True, which="both", alpha=0.3)
        axes[0].set_ylabel("Allan deviation (dps)")
        axes[0].legend()
        fig.tight_layout()
        fig.savefig(args.out, dpi=150)
        print(f"graph: {args.out}")


if __name__ == "__main__":
    main()
