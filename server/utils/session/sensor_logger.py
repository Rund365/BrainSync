"""
Session files for the IMU pipeline (Integration Plan §14), next to session_data.csv.

Files stay open and rows are buffered, written every 0.1 s and at the end, because IMU data
arrives 100 times per second. The 0.1 s keeps utils/analysis/live_view.py close to real time.
"""
import csv
import json
import time
from pathlib import Path

COLUMNS = {
    "imu_data": ["sender_t_us", "pc_t", "seq", "valid_l", "valid_r",
                 "l_ax", "l_ay", "l_az", "l_gx", "l_gy", "l_gz",
                 "r_ax", "r_ay", "r_az", "r_gx", "r_gy", "r_gz"],
    "strikes": ["sender_t_us", "pc_t", "seq", "foot", "interval_ms", "fsr_value", "instant_bpm", "avg_bpm",
                "strike_number", "contact_pc_t"],
    "gait_events": ["sender_s", "pc_t", "foot", "event", "value_dps"],
    "predictions": ["pc_t_made", "sender_s_made", "foot", "strike_number", "predicted_sender_s",
                    "predicted_contact_pc_t", "sigma_ms", "method", "b0_sender_s", "b1_sender_s", "b2_sender_s"],
    "beats": ["beat_index", "pc_t", "tempo_bpm_applied", "tempo_bpm_base", "delta", "alignment_enabled"],
    "phase_errors": ["strike_number", "foot", "contact_pc_t", "beat_index", "beat_pc_t", "error_ms", "bias_ms"],
}
FLUSH_EVERY_S = 0.1


class SensorLogger:
    def __init__(self, session_dir: Path):
        self.dir = Path(session_dir)
        self._files = {}
        self._writers = {}
        self._rows = {name: [] for name in COLUMNS}
        for name, header in COLUMNS.items():
            f = open(self.dir / f"{name}.csv", "w", newline="", encoding="utf-8")
            self._files[name] = f
            self._writers[name] = csv.writer(f)
            self._writers[name].writerow(header)
        self._last_flush = time.perf_counter()

    def row(self, name: str, values: list):
        self._rows[name].append(values)

    def maybe_flush(self):
        if time.perf_counter() - self._last_flush >= FLUSH_EVERY_S:
            self.flush()

    def flush(self):
        for name, rows in self._rows.items():
            if rows:
                self._writers[name].writerows(rows)
                self._files[name].flush()
                rows.clear()
        self._last_flush = time.perf_counter()

    def close(self, meta: dict | None = None):
        self.flush()
        for f in self._files.values():
            f.close()
        if meta is not None:
            with open(self.dir / "meta.json", "w", encoding="utf-8") as f:
                json.dump(meta, f, indent=2, default=str)
