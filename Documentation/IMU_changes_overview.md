# What changed in the IMU code (branch `imu-algorithm-improvements`)

## Why
Tom asked where our IMU thresholds come from. The answer was nowhere: they were made up. So we
replaced them with methods from published papers and built tools to check every change against
the FSR.

## What we found in the old recordings
- **The IMUs are swapped.** The IMU called "left" in the code moves with the right foot, and the
  other way round.
- **One IMU is weak.** The IMU on the left foot measures about half the rotation of the other one.
  This is the sensor or its mounting, not the code.
- **The old sign detection was random.** It flipped between sessions depending on a few
  milliseconds of data timing.

## What we changed

### Gait events (`server/utils/gait/events.py`)
- **Now:** the rules from Sabatini et al. 2005 (IEEE TBME), a paper with an IMU on the foot like ours.
  - Heel-off: the foot starts rotating faster than 30 °/s.
  - Toe-off: the biggest backward rotation.
  - Mid-swing: the biggest forward rotation.
  - Heel strike: the next backward peak. The IMU now detects the heel strike itself.
  - Foot flat: rotation drops below 30 °/s.
- **Removed:** our invented thresholds (−60, +150 °/s, 150 ms window).

### Calibration (`server/utils/gait/calibration.py`)
- **Before:** the code tried to guess the axis and the sign while walking, and the guess was unreliable.
- **Now:**
  - The axis and sign come from how the IMU is mounted, set by hand in `MOUNTING`.
  - The gyro offset (bias) is measured from 0.5 s of standing still at the start.
- **This means:** every session must start with standing still for a moment. The app warns if you don't.
- **⚠️ `MOUNTING` still has placeholder values.** Set it before running with the real shoes.

### New tools (`server/utils/analysis/`)
- **`replay_events.py`:** runs an old recording through the code again. Lets us test a change
  without walking again.
- **`imu_noise_analysis.py`:** checks whether a weak signal comes from the sensor or from the code.
- **`allan_variance.py`:** measures each IMU's noise from a recording where the IMUs lie still on
  a table.

## Still to do
- Set `MOUNTING`. Record new sessions that start standing still, and check the new rules against the FSR.
- BPM: ignore missed or double steps.
- Compare our tempo with a simple published baseline (D-Jogger).

Details: `Documentation/IMU_work_log_2026-11-12.md`.
