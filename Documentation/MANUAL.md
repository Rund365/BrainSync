# BrainSync 2.0: manual

What to do, in order. Why things work the way they do is in `HOW_IT_WORKS.md`.

Commands are run from the `server` folder, with the environment active:
```
.venv\Scripts\activate
cd server
```

## 1. Install (once per computer)
1. Run `setup\Windows\dev\setup_env.bat` (Linux/Mac: `setup/Linux_Mac/dev/setup_env.sh`).
   It deletes an existing `.venv`, creates it again and installs everything in `requirements.txt`.
2. Start the app with `setup\Windows\dev\start_app.bat`.

## 2. Before every recording day
1. Power the sender (on the walker) and plug the receiver into the PC.
2. In the app: choose the serial port.
3. Stand on both feet and press **CALIBRATE WEIGHT**. This sets the pressure-pad thresholds.

## 3. Calibrate the IMUs (once per mounting)
Do this whenever an IMU was mounted, remounted or moved, and for every new user.

1. Find a straight stretch of at least 15 m. **Walking in place does not work.**
2. Press **CALIBRATE IMUs**.
3. Stand still 5 s, walk straight ahead about 20 steps, stand still 5 s.
4. Press **STOP SESSION**. The app calibrates from the walk (up to a minute the first time).
5. Read the lines in the log, one per IMU:

| Number | Good | If not |
|---|---|---|
| pitch share | high; 0.83-0.87 on the test data | sensor loose, or the walk was not straight |
| fit error | near 0 dps | repeat the walk |
| largest gap | a few hundredths of a second | radio link poor: move the receiver closer |

The result is saved as `server/imu_calibration.json` and a copy stays in the session folder.

Without the app:
```
python main.py --calibration-walk --serial-port COM5
python utils/analysis/calibrate_imus.py logs/Default/session_<date>_<time>
```

## 4. Record a session
**Every session starts with the walker standing still for 5 s** (the gyro's zero point is measured
then). Walk forward, not in place.

At the start the log should say:
- `IMU calibration: imu_calibration.json, from session ...`
- `The right IMU sits as in its calibration (... degrees off)`, and the same for the left.

### The two experiments
The music must not react to the steps, so "Align beats" is Off in both.

| Experiment | In the app | Command line |
|---|---|---|
| Fixed tempo | Mode **Manual**, choose the BPM, Align beats **Off** | `python main.py --manual --bpm 110 --phase-align off --serial-port COM5` |
| Tempo change every 15-20 s | Mode **Random**, simple, timeout 15-20 s, steps to match set high, Align beats **Off** | `python main.py --random --random-simple-timeout 18 --random-simple-steps 9999 --phase-align off --serial-port COM5` |

Sessions are saved in `server/logs/<name>/session_<date>_<time>/`.

## 5. Hardware checks
Short recordings that separate "the sensor is bad" from "the mounting is bad" from "the code is
wrong". None of them depends on the detection code.

| Check | How | What it answers |
|---|---|---|
| Table | both IMUs lying still on a table, 15 min: `python main.py --manual --phase-align off --no-audio --serial-port COM5 --session-name table` | noise of each sensor by itself |
| Standing | IMUs on the shoes, stand still 1 min | noise added by the mounting |
| Lift one shoe | lift only the right shoe a few times, then only the left | which IMU and which pressure pad belong to which foot |
| Swap | put the weak IMU on the other shoe, walk 1 min | does the weakness follow the sensor or the shoe |

The table recording is analysed with:
```
python utils/analysis/allan_variance.py logs/table/session_<...> --start 10 --end 900 --out allan.png
```
If no IMU calibration exists yet, the app warns about it. For this recording that does not matter:
only the raw data is used.

## 6. After the sessions
| Question | Command |
|---|---|
| Are the IMU events on time, against the pressure pad? | `python utils/analysis/replay_events.py logs/<name>/session_* --vs-fsr` |
| How well is the next landing predicted, ours and Zank et al.? | `python utils/analysis/prediction_comparison.py logs/<name>/session_*` |
| Has an IMU moved since the calibration? | `python utils/analysis/calibrate_imus.py logs/<name>/session_<...> --check` |
| How much of the foot's rotation does the chosen axis catch? | `python utils/analysis/imu_noise_analysis.py logs/<name>/session_*` |
| Full report of one session | `python utils/analysis/session_report.py logs/<name>/session_<...>` |
| Unit tests | `python -m unittest tests.test_imu_pipeline` |

`replay_events.py` runs the recorded data through the current detection code again, so a change in
the code can be judged on old recordings without walking again.

## 7. What is in a session folder
| File | Contents |
|---|---|
| `imu_data.csv` | raw IMU samples of both feet, 100 per second |
| `strikes.csv` | every pressure-pad landing |
| `gait_events.csv` | pad lifts and the IMU events (HO, TO, MS, HS) |
| `predictions.csv` | every prediction of the next landing, by method |
| `beats.csv` | every beat played, with the tempo |
| `meta.json` | settings, the IMU calibration used, the mounting check, link statistics |
| `session_log.txt` | everything the app logged |

## 8. Warnings and what to do
| Warning | Meaning | Do |
|---|---|---|
| no IMU calibration | `imu_calibration.json` is missing; IMU events of this session are not valid | section 3 |
| IMU sits N degrees off its calibration | the IMU moved on the shoe, or the foot was not flat | fix the mounting, section 3 again |
| cannot measure its gyro bias: stand still | the foot was not still at the start | stand still for a second |
| No data from the ... IMU | wiring or power | check the cable |
| Frequent IMU bus errors | wiring | check the cable; shorter is better |
| % of the sensor messages lost | radio link | move the receiver closer |
