# BrainSync 2.0: notes for Claude

Technion IoT project. The music beat should land on the heel strike. FSR under each heel = ground
truth. 2 BNO055 IMUs (one per shoe) detect gait events and predict the next strike. ESP32 sender
-> ESP-NOW -> receiver ESP32 -> PC (Python, `server/`).

Branch: `imu-algorithm-improvements`.

## How to work with me
- Show the code before changing anything. One step at a time, and wait for my OK.
- I commit myself. Never push. Don't run git commands that write to the repo.
- Don't install anything or create venvs on my machine without asking.
- Every threshold or method needs a published source, cited in the code. If a value is ours,
  mark it `# OURS` and say so.
- Keep my edits in files. Never regenerate a file over them.
- Short answers. No slop.

## Done
- `utils/analysis/replay_events.py`: runs recorded sessions through `FootTracker`
  (reproduces all logged events). `--swap`, `--mount fixed|meta`, `--vs-fsr`.
- `utils/analysis/imu_noise_analysis.py`: total rotation vs. what the code sees, per FSR swing.
- `utils/gait/events.py`: Sabatini 2005 rules (heel-off/foot-flat at 30 dps, TO, MS, HS, HO).
- `utils/gait/calibration.py`: axis and sign from `MOUNTING`, bias from 0.5 s standing still,
  `STILL_STD_DPS = 2` (OURS, placeholder).
- `utils/engine/sensor_health.py`: "stand still" warning.
- `utils/analysis/allan_variance.py`: gyro noise (Woodman 2007 §5.1).
- Docs: `Documentation/IMU_work_log_2026-11-12.md` (full detail),
  `Documentation/IMU_sources_what_we_use.md`, `Documentation/Sabatini_2005_what_we_use.md`.
- Not committed yet: HO event, standing-still check, sensor_health warning, allan_variance.py,
  the replay meta-bias change.

## Next task: automatic IMU calibration with gaitmap (approved by me)
Sabatini has no automatic calibration (he mounts the IMU by hand). Plan:
1. **Calibration recording**, once per mounting: stand still 5 s, walk straight ~20 steps,
   stand still 5 s.
2. **New script `server/utils/analysis/calibrate_imus.py`:** runs gaitmap on that recording
   and writes `imu_calibration.json` (axis + sign per IMU).
   - gaitmap: Kuderle et al. 2024, IEEE Open J. Eng. Med. Biol.
   - Steps: `align_dataset_to_gravity` -> `PcaAlignment` -> `ForwardDirectionSignAlignment`.
   - Docs: https://gaitmap.readthedocs.io/en/stable/auto_examples/preprocessing/automatic_sensor_alignment_details.html
3. **`calibration.py`:** read axis + sign from `imu_calibration.json`; fall back to `MOUNTING`
   if there is no file. The bias is still measured at every session start (0.5 s standing still).
4. **Recalibrate** only when the IMUs are remounted or moved.

First:
- Check that gaitmap installs (`pip install gaitmap`, I said OK) and works at our 100 Hz (their
  examples are at 204.8 Hz).
- Check its expected units and axis convention against ours (BNO055: gyro 16 LSB/dps,
  acc 100 LSB/m/s^2).
- Test on a simulated recording before touching the project.

## Missing (software only)
1. **BPM: reject missed or double steps.** An interval ~2x or ~0.5x the recent ones (idea from
   D-Jogger, Moens et al. 2014, PLoS ONE). Planned place: the predictor's `cadence_bpm`
   (`utils/prediction_model/strike_predictor.py`, the line through 8 FSR steps). Not the receiver
   firmware (needs flashing). Test with the replay on the existing recordings.
2. **D-Jogger baseline:** tempo = average of the last 5 steps. An analysis script to compare
   against our method.
3. **Butterworth 15 Hz filter** (Sabatini §II-C), causal, as an option next to `FILTER_ALPHA = 0.5`
   in `events.py`. Compare in the replay and measure the added delay.

## Missing (needs hardware, later)
- Set `MOUNTING` in `calibration.py` to the real mounting. Until then it holds simulator values,
  so don't run live.
- Validate Sabatini against the FSR on new recordings (IMU HS vs. FSR: Sabatini −2 ms, 95% −16..+12).
- IMU/FSR labels are swapped (fix after a lift-one-shoe test).
- One IMU is weak (sensor or mounting: answer with the table recording + `allan_variance.py`).

## Run (from `server/`)
```
python utils/analysis/replay_events.py logs/STRAVA/session_* --mount meta --swap --vs-fsr
python utils/analysis/imu_noise_analysis.py logs/STRAVA/session_*
python -m unittest tests.test_imu_pipeline
```
