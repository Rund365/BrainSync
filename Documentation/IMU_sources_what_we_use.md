# IMU work: what we use from each source, and where

**How well each source has been checked:**
- **Read** = read in full or in the relevant section.
- **Partly** = abstract, results page or README only.
- **Not read** = title only. Nothing from it goes into code or into Tom's doc until it is read.

Details for Sabatini are in `Sabatini_2005_what_we_use.md`.

## By part of the project

| Part of the project | Our code now (unsourced values) | Source to use | What we take | Status |
|---|---|---|---|---|
| Sensor noise (test 1, IMU on a table) | No test yet | Woodman 2007 §5.1 | Allan variance: average bins of length τ (at least 9 bins), AVAR(τ) = 1/(2(n−1))·Σ(aᵢ₊₁ − aᵢ)², plot √AVAR on log-log axes. Noise = the slope −0.5 line read at τ = 1 s. Bias instability = the curve's minimum | Read |
| Why rotation speed, not angle | Statement only | Woodman 2007 §6; Sabatini §II-C.2 | Integrating the gyro drifts. Sabatini integrates only within one stride and resets it every stance | Read |
| Sensor settings and units | `step_detection_sketch.ino`: ±2000 °/s at 116 Hz, ±16 g at 125 Hz, 100 Hz sampling | BNO055 datasheet | Confirm 16 LSB = 1 °/s, 100 LSB = 1 m/s², and the bandwidth table. Note: a 116 Hz filter with 100 Hz sampling lets aliasing in | Not read in this session |
| Gyro bias | `calibration.py`: running mean while \|ω\| < 20 and \|\|a\| − g\| < 0.6 | Sabatini §II-C.2 | Mean of 0.5 s standing still before walking | Read |
| Detecting when the foot is still | Same rule as above | Skog et al. 2010 (SHOE detector), code in pyshoe | A tested stillness detector instead of our rule | Not read (the code exists) |
| Foot rotation axis | `calibration.py`: PCA of gyro samples above 120 °/s | Mihy et al. (medRxiv preprint) | Medial-lateral axis = first principal component of the raw gyro while walking; vertical axis = gravity while standing. Same method as ours. The 120 °/s cut-off is still ours | Read |
| Axis sign | `calibration.py`: 3 FSR votes | None | Ours. Sabatini sets it by mounting; Mihy doesn't cover it | — |
| Signal filter | `events.py`: one-pole, α = 0.5 | Sabatini §II-C | 2nd-order Butterworth, 15 Hz (gyro). Forward-backward is offline only; online we need a causal version and must measure its delay | Read |
| Heel-off / toe-off | `events.py`: `TO_DPS = 60` | Sabatini Eq. 2 | Heel-off when \|θ̇\| ≥ 30 °/s; toe-off = the largest negative peak after it | Read |
| Mid-swing | `events.py`: `MS_DPS = 150` | Sabatini §II-C.1 | No threshold: the peak of the positive pulse between toe-off and heel strike | Read |
| End of swing / landing | `events.py`: `SLAP_GUARD_S = 0.15` | Sabatini Eq. 3 | Foot-flat when \|θ̇\| < 30 °/s | Read |
| Heel strike from the IMU | None (FSR only) | Sabatini §II-C.1, §III | The second negative peak in the cycle. Measured −2 ms vs. the footswitch (95% CI −16 to +12 ms) | Read |
| Foot angle, stride length, speed | None | Sabatini Eq. 8–16; pyshoe (code) | Integration within each stride with a reset every stance. pyshoe does the same in 3D with a Kalman filter | Sabatini read; pyshoe partly |
| Checking the mounting | None | Sabatini §II-D, §IV; placement paper in PLOS ONE 2022 | \|a\| = 1 g and the standing foot angle at the start and end of each recording (1.2° SD in their study). Where on the foot to mount | Sabatini read; PLOS not read |
| Event detection with an IMU on the shank | — | Aminian 2002, Catalfamo 2010, Formento 2014 | Reference only. Their thresholds were set on the shank and don't carry over to the foot | Not read |
| Foot vs. shank | — | Jasiewicz 2006 | Only if we consider moving the sensor | Not read |
| Base tempo (BPM) | `BPM_estimation.py` / phase controller: line through 8 step pairs | D-Jogger (Moens 2014) | Tempo = average of the last 5 steps; ignore intervals that don't match the average or its multiples | Read |
| Phase alignment | `phase_controller.py`: PI controller with kp = 1, ki = 0.1, ±15%, etc. | D-Jogger strategy 4 | Phase φ = 360·(step − previous beat)/(next beat − previous beat). Song starts on the beat. Outside ±30°, change tempo by 3%. Result R = 0.92, 10 of 10 participants. A tested baseline to compare our PI controller against | Read |
| Predicting the next heel strike | `strike_predictor.py`: rhythm / toe-off / mid-swing, 1/σ² weights | Yan et al. 2017 (adaptive oscillators) | Continuous gait phase from 0 to 2π per stride, giving the time left to the next strike. Error ≤ 0.155 rad (treadmill), ≤ 0.278 rad (free walking) | Partly (signal used and code not checked) |

## What is still ours (no source)

- The axis sign (FSR votes).
- Clock synchronisation between the ESP32 and the PC.
- The weights and gates in `strike_predictor.py`.
- The PI controller's parameters (D-Jogger is the baseline to compare against).

Each of these needs validating against the FSR on our own recordings.

## The sources

1. **Sabatini, Martelloni, Scapellato, Cavallo**, "Assessment of Walking Features From Foot Inertial Sensing", *IEEE Trans. Biomed. Eng.* 52(3):486–494, 2005. DOI 10.1109/TBME.2004.840727
2. **O. J. Woodman**, "An introduction to inertial navigation", Univ. of Cambridge Tech. Report UCAM-CL-TR-696, 2007. https://www.cl.cam.ac.uk/techreports/UCAM-CL-TR-696.pdf
3. **Bosch Sensortec**, BNO055 datasheet.
4. **Skog, Händel, Nilsson, Rantakokko**, "Zero-Velocity Detection — An Algorithm Evaluation", *IEEE Trans. Biomed. Eng.* 57(11), 2010.
5. **Wagstaff, Peretroukhin, Kelly**, "Robust Data-Driven Zero-Velocity Detection for Foot-Mounted Inertial Navigation", *IEEE Sensors J.* 20(2):957–967, 2019. Code (pyshoe): https://github.com/acbasil/pyshoe (fork of utiasSTARS/pyshoe)
6. **Mihy, Wagatsuma, Cain, Hafer**, "A Functional Sensor-to-Segment Orientation Method to Reduce the Effects of Varied Sensor Placement: Implications for IMU Best Practices", medRxiv preprint, DOI 10.1101/2022.11.29.22282894
7. **Moens et al.**, "Encouraging Spontaneous Synchronisation with D-Jogger, an Adaptive Music Player That Aligns Movement and Music", *PLoS ONE* 9(12):e114234, 2014.
8. **Yan, Parri, Ruiz Garate, Cempini, Ronsse, Vitiello**, "An oscillator-based smooth real-time estimate of gait phase for wearable robotics", *Autonomous Robots* 41(3):759–774, 2017. DOI 10.1007/s10514-016-9566-0
9. **Aminian et al.**, "Spatio-temporal parameters of gait measured by an ambulatory system using miniature gyroscopes", *J. Biomech.* 35(5), 2002.
10. **Catalfamo, Ghoussayni, Ewins**, "Gait Event Detection on Level Ground and Incline Walking Using a Rate Gyroscope", *Sensors* 10(6):5683, 2010.
11. **Formento et al.**, "Gait Event Detection during Stair Walking Using a Rate Gyroscope", *Sensors* 14(3):5470, 2014.
12. **Jasiewicz et al.**, "Gait event detection using linear accelerometers or angular velocity transducers in able-bodied and spinal-cord injured individuals", *Gait & Posture* 24, 2006.
13. "The placement of foot-mounted IMU sensors does affect the accuracy of spatial parameters during regular walking", *PLOS ONE*, 2022. https://journals.plos.org/plosone/article?id=10.1371%2Fjournal.pone.0269567
