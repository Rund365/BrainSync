# Sabatini 2005: what we use, and where

**Paper:** A. M. Sabatini, C. Martelloni, S. Scapellato, F. Cavallo, "Assessment of Walking Features From Foot Inertial Sensing", *IEEE Trans. Biomed. Eng.* 52(3):486–494, 2005. DOI 10.1109/TBME.2004.840727

Equation and section numbers below are the paper's.

## Their setup vs. ours

| | Sabatini | Us |
|---|---|---|
| Sensor | 1 gyro (Murata ENC-03J, max ±300 °/s) + 2-axis accelerometer | BNO055: 3-axis gyro ±2000 °/s, accelerometer ±16 g |
| Placement | Instep of the right foot, Velcro strap. Gyro axis set **by mounting** (mediolateral) | On each shoe. Axis found by PCA (`calibration.py`) |
| Sampling | 200 Hz | 100 Hz |
| Filter | 2nd-order Butterworth, forward-backward (offline): 15 Hz gyro, 17 Hz accelerometer | One-pole IIR, α = 0.5 (~11 Hz), online |
| Reference | Footswitches (FSR) under the heel and big toe | FSR under each heel |
| Test | 5 men, treadmill, 3–6 km/h, slope −5% to +15% | 1 person, overground |
| Sign | Swing = positive pulse, toe-off and heel strike = negative peaks | Same (mid-swing positive) |

## What we take, and what it replaces

| # | From the paper | Where in our code | Replaces |
|---|---|---|---|
| 1 | **Heel-off:** \|θ̇\| ≥ 30 °/s (Eq. 2, TH_HO) | `events.py`, start of the push-off search | Entry threshold `TO_DPS = 60` (no source) |
| 2 | **Toe-off:** the largest negative peak after heel-off (§II-C.1) | `events.py` | Same idea as ours (minimum of the dip); keep, but gate it with #1 |
| 3 | **Heel strike from the gyro:** the second negative peak in the cycle (§II-C.1) | `events.py`, **new event** | Nothing today: heel strike comes only from the FSR |
| 4 | **Foot-flat:** \|θ̇\| < 30 °/s (Eq. 3, TH_FF). The swing is over | `events.py` | `SLAP_GUARD_S = 0.15` (no source) |
| 5 | **Mid-swing** is described only as the peak of the positive pulse. No threshold is given | `events.py` | `MS_DPS = 150` (no source): take the maximum between toe-off and heel strike instead |
| 6 | **Gyro bias:** mean of N = 100 samples (0.5 s) standing still before walking (§II-C.2) | `calibration.py` | Running mean with our own stillness rule (\|ω\| < 20, \|\|a\|−g\| < 0.6, no source) |
| 7 | **Filter:** 2nd-order Butterworth, 15 Hz gyro (§II-C) | Offline: analysis scripts (exactly theirs). Online: `events.py` (causal version, adds delay that we must measure) | `FILTER_ALPHA = 0.5` |
| 8 | **Pitch angle:** integrate θ̇ from heel-off to foot-flat (Eq. 8). Start angle from gravity, θ_init = mean atan2(a_x, a_z) over M = 25 stance samples (Eq. 9). Linear drift reset over each cycle (Eq. 10) | New, e.g. `utils/gait/stride.py` (offline first) | Nothing today |
| 9 | **Velocity, stride length, speed, incline:** integrate acceleration minus gravity with v = 0 at heel-off, linear reset per cycle (Eq. 11–16) | Same new module | Nothing today. Gives speed per stride as a possible input for prediction (the paper does not test this use) |
| 10 | **Stride time, swing, stance, relative stance** (Eq. 4–7). Relative stance is 54–60%, falling with speed: RS = 0.18V² − 3.49V + 67.98 (V in km/h, treadmill) | Analysis scripts | Sanity check of our toe-off and heel-strike times |
| 11 | **Mounting check:** before each trial, \|a\| = 1 g and the pitch angle while standing. Standing pitch varied by only 1.2° SD over about 2 hours (§II-D, §IV) | New check at the start and end of each recording | Nothing today. A change in standing pitch means the IMU moved on the shoe |
| 12 | **Validation:** gyro events vs. footswitch over 300 cycles. Toe-off 35 ms early on average. Heel strike −2 ms on average (95% CI −16 to +12 ms) (§III; §IV says about 10 ms) | Analysis scripts, against our FSR | Our own classification rules (`decision_analysis.py`) |
| 13 | **Reference magnitudes:** foot peaks of about 650 °/s at toe-off and 300 °/s at mid-swing (§IV). Fig. 4 at 4 km/h: about +320 / −330 °/s | Doc for Tom, `imu_noise_analysis.py` | Our unsourced "a foot turns faster" |

## What the paper does not cover

- **Finding the axis** (they fix it by mounting). For our PCA: Mihy et al., medRxiv preprint (first principal component of the gyro while walking).
- **The sign of the axis.** Our FSR votes are our own.
- **Real time.** The paper is offline (Matlab, forward-backward filter). The 30 °/s thresholds were tuned at 200 Hz with their filter, so we must validate them on our data against the FSR.
- **Predicting the next heel strike.** Not in the paper.
- **Two feet, overground walking, one person.** They used one foot on a treadmill.
- **The sensor moving relative to the foot.** The authors say they cannot rule it out, especially at impact (§IV). This is the same suspicion we have about our weak IMU.
