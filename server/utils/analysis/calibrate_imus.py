"""
Automatic IMU-to-foot alignment from one calibration recording, with gaitmap.

    python utils/analysis/calibrate_imus.py logs/<folder>/<session> [--start S] [--end S] [--out FILE]
    python utils/analysis/calibrate_imus.py logs/<folder>/<session> --check

The recording, once per mounting: stand still 5 s, walk straight ahead about 20 steps, stand still
5 s. Walking in place does not work: step 3 needs real forward movement.

Method: gaitmap (Kuderle et al., "Gaitmap - An Open Ecosystem for IMU-Based Human Gait Analysis and
Algorithm Benchmarking", IEEE Open J. Eng. Med. Biol., 2024), the three steps of its "Automatic
sensor alignment" example, with that example's parameters:
  1. align_dataset_to_gravity        standing, accelerometer: which way is up
  2. PcaAlignment                    walking, gyro: the axis across the foot (largest rotation)
  3. ForwardDirectionSignAlignment   walking: which way is forward (a 0 or 180 degree flip)
Each step is a fixed rotation, so together they are one rotation R from the sensor's axes to the
foot's axes (x forward, z up). R is recovered by fitting the gyro before to the gyro after (OURS:
gaitmap returns the rotated data, not one combined rotation). Then, in the sensor's axes,
    pitch axis = forward x up.
Turning "forward" towards "up" lifts the toes, so the rotation speed about this axis is positive
in the swing, the sign events.py expects (Sabatini et al. 2005, Fig. 2 and 4b).

Writes imu_calibration.json (default: server/imu_calibration.json), read by utils/gait/calibration.py,
and a copy into the session folder, so the calibration can be traced back to its recording.
"up" (gravity while standing, in the sensor's axes) is stored to check later that the IMU has not moved.
The keys "left" / "right" are the IMU labels of imu_data.csv (I2C address), not checked feet.

--check writes nothing: it measures the axes of any walking session the same way and prints how far
they are from the calibration in use. A growing angle means the IMU has moved on the shoe.
"""
import argparse
import json
import sys
from datetime import datetime
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # server/
from utils.gait.calibration import SAMPLE_RATE_HZ
from utils.hardware.sensor_stream import ACC_LSB_PER_MS2, GYR_LSB_PER_DPS

ACC, GYR = ["acc_x", "acc_y", "acc_z"], ["gyr_x", "gyr_y", "gyr_z"]
IMUS = {"left": "l", "right": "r"}  # IMU label -> column prefix in imu_data.csv
T_US_WRAP = 2 ** 32
# align_dataset_to_gravity, values of the gaitmap example: a 0.1 s window counts as standing
# when the gyro stays below 15 dps
STATIC_WINDOW_S = 0.1
STATIC_GYRO_DPS = 15


def load_imu(folder: Path, start: float | None, end: float | None) -> tuple[dict, dict]:
    """imu_data.csv -> {IMU label: acc (m/s2) and gyro (dps) on an even 100 Hz grid}, largest gap (s).
    gaitmap assumes evenly spaced samples, so lost packets are filled by linear interpolation."""
    raw = pd.read_csv(folder / "imu_data.csv")
    t = np.unwrap(raw.sender_t_us.values.astype(float), period=T_US_WRAP) / 1e6
    t -= t[0]
    data, gaps = {}, {}
    for name, p in IMUS.items():
        ok = raw[f"valid_{p}"].values == 1
        if start is not None:
            ok &= t >= start
        if end is not None:
            ok &= t <= end
        if ok.sum() < 5 * SAMPLE_RATE_HZ:
            continue  # this IMU sent (almost) nothing
        ts = t[ok]
        grid = np.arange(ts[0], ts[-1], 1.0 / SAMPLE_RATE_HZ)
        cols = {}
        for axis in "xyz":
            cols[f"acc_{axis}"] = np.interp(grid, ts, raw[f"{p}_a{axis}"].values[ok] / ACC_LSB_PER_MS2)
            cols[f"gyr_{axis}"] = np.interp(grid, ts, raw[f"{p}_g{axis}"].values[ok] / GYR_LSB_PER_DPS)
        data[name] = pd.DataFrame(cols)[ACC + GYR]
        gaps[name] = float(np.diff(ts).max())
    return data, gaps


def align(data: dict) -> dict:
    """The three gaitmap steps. Returns the data in the foot's axes (x forward, z up)."""
    from gaitmap.preprocessing.sensor_alignment import (ForwardDirectionSignAlignment, PcaAlignment,
                                                        align_dataset_to_gravity)
    upright = align_dataset_to_gravity(data, sampling_rate_hz=SAMPLE_RATE_HZ, window_length_s=STATIC_WINDOW_S,
                                       static_signal_th=STATIC_GYRO_DPS)
    across = PcaAlignment().align(upright).aligned_data_
    return ForwardDirectionSignAlignment().align(across, sampling_rate_hz=SAMPLE_RATE_HZ).aligned_data_


def pitch_axis(before: pd.DataFrame, after: pd.DataFrame) -> tuple[np.ndarray, np.ndarray, float]:
    """(pitch axis, up; both in the sensor's axes, rms error of the rotation fit in dps)."""
    n = min(len(before), len(after))
    b, a = before[GYR].values[:n], after[GYR].values[:n]
    to_foot, rssd = Rotation.align_vectors(a, b)  # a = R b
    forward, up = to_foot.inv().apply([[1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    axis = np.cross(forward, up)
    return axis / np.linalg.norm(axis), up, float(rssd / np.sqrt(n))


def describe(axis: np.ndarray, gyr: np.ndarray) -> dict:
    """Numbers to judge the result by: the closest board axis, and how much of the rotation is pitch."""
    i = int(np.abs(axis).argmax())
    board = ("-" if axis[i] < 0 else "+") + "xyz"[i]
    return {
        "pitch_axis": axis.round(4).tolist(),
        "nearest_board_axis": board,
        "angle_to_board_axis_deg": round(float(np.degrees(np.arccos(min(1.0, abs(axis[i]))))), 1),
        # share of the gyro's energy that is about the pitch axis, over the whole recording
        "pitch_share": round(float(((gyr @ axis) ** 2).sum() / (gyr ** 2).sum()), 3),
    }


def angle_deg(a, b) -> float:
    a, b = np.asarray(a, float), np.asarray(b, float)
    return float(np.degrees(np.arccos(np.clip(a @ b / (np.linalg.norm(a) * np.linalg.norm(b)), -1.0, 1.0))))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("session", type=Path, help="session folder with imu_data.csv")
    parser.add_argument("--start", type=float, default=None, help="use the recording from this second on")
    parser.add_argument("--end", type=float, default=None, help="use the recording up to this second")
    parser.add_argument("--out", type=Path, default=Path(__file__).resolve().parents[2] / "imu_calibration.json")
    parser.add_argument("--check", action="store_true",
                        help="write nothing: print the angle between this session's axes and the calibration in use")
    args = parser.parse_args()

    try:
        import gaitmap
    except ImportError:
        sys.exit("gaitmap is not installed: run the setup script (setup_env), it installs requirements.txt")

    data, gaps = load_imu(args.session, args.start, args.end)
    if not data:
        sys.exit("No valid IMU samples in this recording.")
    try:
        aligned = align(data)
    except Exception as e:
        sys.exit(f"gaitmap could not align this recording: {e}\n"
                 "It needs standing still at the start and straight forward walking.")

    result = {"created": datetime.now().isoformat(timespec="seconds"), "session": args.session.name,
              "method": f"gaitmap {gaitmap.__version__}: gravity, PCA, forward direction sign", "imus": {}}
    for name in data:
        axis, up, fit_error = pitch_axis(data[name], aligned[name])
        info = describe(axis, data[name][GYR].values)
        info["up"] = up.round(4).tolist()
        info["rotation_fit_error_dps"] = round(fit_error, 2)
        info["seconds_used"] = round(len(data[name]) / SAMPLE_RATE_HZ, 1)
        info["largest_gap_s"] = round(gaps[name], 3)
        result["imus"][name] = info
        print(f"IMU '{name}': pitch axis {info['pitch_axis']} (closest board axis {info['nearest_board_axis']}, "
              f"{info['angle_to_board_axis_deg']} deg away), pitch share {info['pitch_share']}, "
              f"fit error {info['rotation_fit_error_dps']} dps, largest gap {info['largest_gap_s']} s")
    if args.check:
        if not args.out.exists():
            sys.exit(f"No calibration to compare with: {args.out} does not exist.")
        current = json.loads(args.out.read_text(encoding="utf-8"))
        print(f"Compared with {args.out.name} (session {current.get('session')}, made {current.get('created')}):")
        for name, info in result["imus"].items():
            old = current.get("imus", {}).get(name)
            if not old:
                print(f"  IMU '{name}': not in the calibration")
                continue
            print(f"  IMU '{name}': pitch axis {angle_deg(info['pitch_axis'], old['pitch_axis']):.1f} deg away"
                  + (f", up {angle_deg(info['up'], old['up']):.1f} deg away" if "up" in old else ""))
        return
    text = json.dumps(result, indent=2)
    args.out.write_text(text, encoding="utf-8")
    (args.session / "imu_calibration.json").write_text(text, encoding="utf-8")
    print(f"Written to {args.out} (copy in {args.session})")


if __name__ == "__main__":
    main()
