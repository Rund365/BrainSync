# IMU work log, 12 Nov 2026

Branch: `imu-algorithm-improvements`. Everything here was run on the 5 STRAVA recordings from 1 Nov
(`server/logs/STRAVA/session_2026-11-01_20-*`, numbered 1-5 in time order).

**Goal of the day:** replace the IMU thresholds we invented with methods from published, tested
work, and build tools that check every change against the FSR.

---

## 1. What is committed and what is not

You already committed:
1. `analysis: replay sessions through the gait event detector`
2. `imu: events and calibration following Sabatini et al. 2005`

Still uncommitted (made after those commits, see sections 6-9):

| File | Change |
|---|---|
| `server/utils/gait/events.py` | New heel-off event `HO` |
| `server/utils/gait/calibration.py` | The bias is accepted only from a 0.5 s window where the foot really stands still |
| `server/utils/engine/sensor_health.py` | New warning "stand still for a second" (replaces "not calibrated after 20 steps") |
| `server/utils/analysis/replay_events.py` | Compares IMU heel-off with the FSR lift; `--mount meta` uses the old recordings' bias |
| `server/utils/analysis/allan_variance.py` (new) | Gyro noise (Allan variance) for the table recording |

Suggested commit message:
```
imu: heel-off event, bias only while standing still, Allan variance script
```

Cleanup: one of my git commands left two stray files. Delete them:
`.git/stale-index-lock-claude` and `.git/objects/e4/tmp_obj_LykD0s`.

---

## 2. Test 3: total rotation vs. what the code sees (`imu_noise_analysis.py`)

**Question:** is the IMU signal weak because of our code, or because of the sensor or its mounting?

**Method:**
- The FSR gives each foot's swing: from its lift to its next strike.
- In every swing, compare two numbers:
  - the total rotation |ω|, the length of the gyro vector minus the bias. It doesn't depend on any axis the code chose;
  - the rotation about the axis the code chose (the pitch rate).
- Also compute the angle between the code's axis and the real main axis of those swings (PCA).

**Run:** `python utils/analysis/imu_noise_analysis.py logs/STRAVA/session_*`

**Results** (median peak per swing, °/s):

| IMU label | During swings of | Total \|ω\| | What the code sees | Axis angle |
|---|---|---|---|---|
| left | right foot | 225-291 | 219-272 | 5-10° |
| left | left foot | 20-34 | 18-30 | 18-37° |
| right | left foot | 116-148 | 85-126 | 11-88° |
| right | right foot | 67-124 | 14-121 | 9-88° |

**Findings:**
1. **Each IMU moves with the opposite foot to its label.** The IMU labels and the FSR labels disagree.
   The data can't say which side is wrong, the IMU addresses (0x28/0x29) or the FSR feet. A
   lift-one-shoe test settles it.
2. **The IMU on the right foot is healthy.** 225-291 °/s in swing, close to the ~300 °/s Sabatini
   reports at mid-swing. The axis the code found is within 5-10° of the real one.
3. **The IMU on the left foot is weak.**
   - Only 116-148 °/s in swing, and 67-124 °/s while its foot stands, so the swing barely stands out.
   - Total rotation doesn't depend on the code, so the cause is the sensor or its mounting. Test 1
     (table) and test 2 (standing on the shoe) will tell which.
4. With the weak IMU, the old calibration chose the wrong axis in sessions 2 and 5.

---

## 3. Baseline (`server/analysis_baseline/`)

The outputs of `decision_analysis.py` and `imu_noise_analysis.py` before any code change, kept as
the "before" for comparisons.

---

## 4. Replay tool (`server/utils/analysis/replay_events.py`)

**Why:** to test a change to the detection code on recordings we already have, without walking
again. Same data, old code vs. new code, so any difference comes from the code.

**How:** it feeds `imu_data.csv` and the FSR strikes to the engine's `FootTracker`, in the order
they arrived on the PC (`pc_t`), exactly as live.

**Check that it is trustworthy (`--check`):** with the code the sessions were recorded with, it
reproduced **all 839 logged events, 0.0 ms difference**.

**Fix found on the way:**
- The first version fed the strikes in sender-time order and matched only sessions 1 and 4.
- Live, an FSR strike reaches the PC before the last ~3 IMU samples before it (median 3, max 5).
  Feeding in PC arrival order fixed it.

**Finding about the old sign vote:**
- In session 3, the peak that decided each vote was 6-15 ms before the strike. That's the landing,
  not the swing.
- Three samples arriving or not was enough to flip the whole foot's sign. That explains why the
  sign changed between sessions.
- The old vote is now removed (section 5).

**Options:**
- `--swap`: feed each IMU to the opposite foot (what finding 1 says is the real mapping), without
  changing the engine.
- `--vs-fsr`: compare the IMU events with the FSR (section 7).
- `--mount fixed | meta`: see section 7.

---

## 5. Sabatini's calibration and event rules (committed)

**Source:** A. M. Sabatini, C. Martelloni, S. Scapellato, F. Cavallo, "Assessment of Walking
Features From Foot Inertial Sensing", IEEE TBME 52(3):486-494, 2005. The PDF you downloaded.
Summary in `Documentation/Sabatini_2005_what_we_use.md`.

### `calibration.py`

| | Before (ours, no source) | Now |
|---|---|---|
| Axis | PCA of gyro samples > 120 °/s while walking | Fixed by how the IMU is mounted (Sabatini §II-A), set in `MOUNTING` |
| Sign | 3 FSR votes before strikes | Fixed by mounting, opposite on the two feet |
| Bias | Running average whenever \|ω\| < 20 and \|a\| ≈ g | Average of 0.5 s standing still at the start (Sabatini §II-C.2) |

**`MOUNTING` still holds the simulator's values** (`{1: ("y", +1), 2: ("y", -1)}`). Set it to the
real mounting before running live: which board axis points across the shoe.

### `events.py`

Sabatini's cycle on the pitch rate w (positive = swing):

| Event | Rule | Source |
|---|---|---|
| Heel-off (HO) | \|w\| ≥ 30 °/s | Eq. 2 |
| Toe-off (TO) | Largest negative peak after heel-off | §II-C.1 |
| Mid-swing (MS) | Largest positive peak of the swing | §II-C.1 (no threshold) |
| Heel strike (HS) | Second negative peak, closed when \|w\| < 30 °/s (foot-flat) | §II-C.1, Eq. 3 |

- **Removed (no source):** `TO_DPS = 60`, `MS_DPS = 150`, `SLAP_GUARD_S = 0.15`, the FSR sign votes.
- **Ours, marked in the code:** each peak must have the right sign (a TO negative, an MS positive,
  an HS negative). The paper describes clean cycles only.
- **New:** the IMU finds the heel strike itself. Sabatini: −2 ms from the footswitch (95%: −16 to
  +12 ms).
- **Unchanged:** the filter (α = 0.5) and the FSR-delay measurement from the impact.
- **The predictor** reacts only to `TO`/`MS`, so `HO` and `HS` don't change it. They are logged to
  `gait_events.csv`.

---

## 6. Heel-off event (uncommitted)

**Why:** our FSR is under the heel only, so its "lift" is a heel-off, not a toe-off. My first
comparison checked the FSR lift against the IMU toe-off, two different events, so that number was
meaningless.

**Change:** `events.py` emits `HO` when heel-off happens (Sabatini T_HO), and `--vs-fsr` compares
`HO` with the FSR lift.

---

## 7. Standing-still check for the bias (uncommitted)

**Why:** Sabatini's bias assumes the user stands still. If they move, the bias comes out wrong by
tens of °/s, and the 30 °/s thresholds stop working.

**`calibration.py`:**
- The bias is taken from a 0.5 s window only if the gyro spread on every axis is below
  `STILL_STD_DPS = 2 °/s`. Otherwise it tries the next 0.5 s.
- **2 °/s is ours, a placeholder.** The table and standing recordings will set it (section 8).
- New fields in `meta.json`: `bias_std_dps` (spread of the accepted window), `rejected_windows`.
- In the simulator, both feet get a bias (spread ~0.7 °/s).

**`sensor_health.py`:** the warning "not calibrated after 20 steps" becomes "cannot measure its
gyro bias: stand still for a second".

### What this showed about the old recordings

In none of the 5 recordings is there **any** 0.5 s window where the gyro is quieter than 2 °/s:

| Session | IMU 'left': quietest 0.5 s (worst axis) | IMU 'right': quietest 0.5 s |
|---|---|---|
| 1 | 9.0 °/s | 16.5 °/s |
| 2 | 6.1 | 15.3 |
| 3 | 6.3 | 10.1 |
| 4 | 3.6 | 8.7 |
| 5 | 4.2 | 12.0 |

- **Either nobody stood still in these recordings, or the sensors or mounting are that noisy.**
  The table recording answers which.
- The weak IMU ('right') is the noisier one every time.
- The old calibration counted samples up to 20 °/s as "still", so its bias was measured partly
  while moving.

### Replay of the old recordings with the new code (`--mount meta --swap --vs-fsr`)

The new code needs a bias and a known mounting, and the old recordings have neither. So:
- `--mount meta` uses the bias and axis the old calibration found.
- The sign comes from the foot's net rotation in each swing (Sabatini Fig. 4b: the foot pitch
  rises during the swing).

Healthy IMU (right foot), IMU heel strike minus FSR strike:

| Session | Median | 95% range | Sign used |
|---|---|---|---|
| 1 | +54 ms | −141 to +131 | −1 |
| 2 | +34 | −10 to +76 | +1 |
| 3 | +70 | −21 to +136 | +1 |
| 4 | +71 | −52 to +116 | −1 |
| 5 | +102 | −19 to +126 | −1 |

**This does not pass** (Sabatini: −2 ms, within ±16), **and it doesn't test the method yet:**
- The sign estimate is weak: the net rotation per swing came out 0-4°, so the sign flips between
  sessions for the same IMU.
- The bias is the old, partly-moving one.
- The mounting axis is a guess from the old PCA.

The real test needs new recordings: a known mounting, and standing still at the start.

---

## 8. Allan variance (`server/utils/analysis/allan_variance.py`, uncommitted)

**Source:** O. J. Woodman, "An introduction to inertial navigation", Cambridge Tech. Report
UCAM-CL-TR-696, 2007, §5.1.

**What it does:** gyro noise of each IMU from a still recording.
1. Bins of length τ, at least 9 bins.
2. AVAR(τ) = 1/(2(n−1))·Σ(aᵢ₊₁ − aᵢ)².
3. Plot √AVAR against τ on log-log axes.
4. Read the white noise at τ = 1 s, and the bias instability at the minimum of the curve.

**Checked on synthetic white noise:** σ = 0.2 °/s at 100 Hz should give 0.020 °/s at τ = 1 s; the
script gives 0.0206.

**Run** (on the table recording):
```
python utils/analysis/allan_variance.py logs/<folder>/<session> --start 10 --end 900 --out allan.png
```

**What it answers:**
- Is the weak IMU noisier than the healthy one on the table?
  - If yes, the sensor is the problem.
  - If no, but it's noisier on the shoe, the mounting is the problem.
- It also gives a real number for `STILL_STD_DPS`.

---

## 9. Tests

`python -m unittest tests.test_imu_pipeline`:
- 32 pass.
- 6 error only because `mido` isn't installed in my environment (MIDI player tests). Run them on
  your machine.

---

## 10. Not done yet

| Item | Why it waits |
|---|---|
| **BPM: rejecting missed or double steps** | The tempo comes from three places: the receiver ESP32 averages the last 3 step intervals (`step_receiver_sketch.ino`); the PC blends that with the LGBM model (65/35, `BPM_estimation.py`); phase alignment uses a line through 8 steps (`phase_controller.py`). We need to decide where the rejection goes before writing it. Evaluating it needs the tempo-change recordings |
| **gaitmap mounting check** (offline) | Needs `pip install gaitmap`. I didn't install anything on your machine without asking |
| **Filter** (Butterworth 15 Hz vs. α = 0.5) | Compare on new recordings, with the replay |
| **Stride analysis** (Sabatini Eq. 8-16) | Last; a nice-to-have |
| **Fixing the swap** | After the lift-one-shoe test |

---

## 11. Before the next recording

1. Mount both IMUs the same way, tight, on the instep (as in Sabatini). Note which board axis
   points outward from each shoe, and give me that so I can set `MOUNTING`.
2. Lift-one-shoe test: lift only the right shoe and see which IMU and which FSR react.
3. Record:

| Recording | Length | For |
|---|---|---|
| Both IMUs still on a table | 15 min | Allan variance, `STILL_STD_DPS` |
| Standing still with the IMUs on the shoes | 1 min | Mounting noise vs. table |
| Every walking session: start with 5 s standing still | — | Bias |
| Constant tempo (`--manual --bpm 110 --phase-align off`) | 3 × 3 min | Sabatini vs. FSR |
| Tempo change every 15-20 s (random mode, phase off) | 3 × 3 min | BPM |

4. Run:
```
python utils/analysis/imu_noise_analysis.py <new sessions>
python utils/analysis/replay_events.py <new sessions> --vs-fsr
python utils/analysis/allan_variance.py <table session> --out allan.png
```
