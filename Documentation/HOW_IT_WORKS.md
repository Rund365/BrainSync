# BrainSync 2.0: how it works

How to run things is in `MANUAL.md`. This file explains what the system does and why.
Sections 1-7 follow the data, from the shoe to the loudspeaker. Each one starts in plain words,
then gives the details, the constants with their source, the known limits, and a table of the
methods with the reason for each.

## 0. The whole system in plain words

**What it is for.** A person walks, music plays. The goal is that the beat of the music sounds at
the moment the heel touches the ground.

**The difficulty.** Sound takes time to produce. To make a beat coincide with a landing, the
computer must know the landing is coming before it happens.

**The two kinds of sensor, and their roles**

| Sensor | Where | What it tells | Role |
|---|---|---|---|
| Pressure pad (FSR) | under each heel | "the heel is on the ground now" | the reference: what really happened |
| Motion sensor (IMU) | on each shoe | how the foot is moving in the air | the early warning |

The pressure pad is always right but always too late. The motion sensor is early but has to be
interpreted. Everything in this project is about interpreting the motion sensor and checking the
result against the pressure pad.

**The chain**

1. **Sender** (on the walker): reads the sensors, timestamps, sends by radio. Section 1.
2. **Receiver** (at the PC): passes the data on, counts steps per minute. Section 2.
3. **PC input:** real units, one common clock. Section 3.
4. **Calibration:** finds out how each motion sensor sits on its shoe. Section 4.
5. **Gait events:** recognises the phases of each step from the foot's rotation. Section 5.
6. **Prediction:** estimates when the heel will land. Section 6.
7. **Tempo and music:** chooses the tempo and plays. Section 7.

**Where we stand (November 2026)**

TODO: remember to update when we do test it

| Part | State |
|---|---|
| Sensors, radio, logging | work; no step lost in 1275 recorded steps |
| Calibration | new, tested on a public example walk, not yet on our sensors |
| Gait events | rules from a published study; not yet validated on our data |
| Prediction | built; not evaluated; one of its three clues arrives too late |
| Beat on the step | switched off until the parts above are validated |

**Where the methods come from**

| Part | Source |
|---|---|
| Gait event rules, gyro zero point | Sabatini et al. 2005 |
| Sensor axes from a walk | gaitmap (Kuderle et al. 2024), Mihy et al. 2022 |
| Prediction from swing and lift events | ours; to be compared with Zank et al. 2013 |
| Tempo from steps | ours; D-Jogger (Moens et al. 2014) as the published baseline |

Everything that is our own choice is marked OURS in the constants tables. Full references are at
the end.

## 1. Sender ESP32 (on the walker)

**In plain words:** a small computer carried by the walker. It listens to four sensors: a pressure
pad under each heel, which tells the exact moment the heel touches the ground, and a motion sensor
on each shoe, which tells how the foot is moving in the air. It decides nothing about walking. It
only notes what it measured and exactly when, and sends that to the PC by radio.

**Job:** read the sensors, timestamp everything on one clock, send it by radio (ESP-NOW). No gait
logic here except the FSR threshold.

**File:** `ESP32/step_detection_sketch/step_detection_sketch.ino`

### 1.1 FSR: heel strike and heel lift
- One FSR under each heel. Analog pins: right 33, left 32.
- Each reading is smoothed: `new = 0.75 × old + 0.25 × raw`.
- **Threshold, per foot.** Set at power-on while the user stands on both feet for 1 s:
  `threshold = standing average − 200` (minimum 300).
- **Strike:** the smoothed value rises above the threshold. Sends `STEP` with the foot, the time,
  and the time since the previous step.
- **Lift:** the value falls below `threshold − 55 %`. Sends `LIFT`. The gap between the two levels
  stops one step from firing twice.
- A second strike within 120 ms of the last one is ignored.

### 1.2 IMU: raw motion of each foot
- Two BNO055, one per shoe, on one I2C bus. Addresses: right `0x28`, left `0x29`.
- Mode: **raw accelerometer + gyroscope only.** The sensor's built-in fusion is off. No angles are
  computed anywhere on the ESP32.
- Units: acceleration 100 counts per m/s², rotation speed 16 counts per °/s.
- Ranges: ±16 g, ±2000 °/s.
- Sampled at 100 Hz. Both IMUs are read one after the other and share one timestamp.
- Sent in packets of 5 samples (every 50 ms).

**Why these two signals**
- **Gyroscope** (rotation speed, °/s): the foot rotates in a repeating pattern every step.
  Heel-off, toe-off, mid-swing and heel strike are found as thresholds and peaks of the rotation
  speed around the axis across the foot (section 5.4).
- **Accelerometer** (m/s²): the impact spike marks the landing, and the acceleration along "up"
  shows the foot rising and falling (sections 5.8, 5.8b). The landing also tells how late the FSR
  fires after the real contact.
- **One angle only: which way is up.** An angle needs integrating the gyro, and the integral
  drifts. So "up" is carried through the foot's rotation by the gyro and reset to gravity every
  time the foot is flat after a landing; it cannot drift for longer than one step. The event rules
  work on rotation speed directly.
- **Fusion off.** Raw mode lets us choose the ranges ourselves.

### 1.3 Timestamps
- Every message carries the sender's clock in microseconds, taken at the moment of the event.
- All later timing (IMU event vs. FSR strike) uses this clock, so radio and USB delay do not
  shift it.

### 1.4 Health
- Once per second: I2C error count per IMU.
- After 10 failed reads in a row an IMU is restarted, retried every 2 s.

### 1.5 Constants
| Value | Meaning | Source |
|---|---|---|
| 0.75 / 0.25 | FSR smoothing | OURS |
| standing − 200 | FSR threshold | OURS |
| 55 % | lift level | OURS |
| 120 ms | minimum gap between steps | OURS |
| 100 Hz, ±16 g, ±2000 °/s | IMU settings | OURS, registers from the BNO055 datasheet §4 |

### 1.6 Known limits
- The FSR is under the heel only: `LIFT` means heel-off, not toe-off.
- BPM buttons: legacy, unused.
- Strikes dropped by the 120 ms gap: none in 1275 recorded strikes (5 sessions, 1 Nov 2026).
  A dropped strike would show as the same foot twice in a row.
- Scheduled checks:
  - FSR loop rate (measure once);
  - the gyro's 116 Hz bandwidth with 100 Hz sampling lets aliasing in.

### 1.7 Methods and logic
| Method | Problem it solves | Cost |
|---|---|---|
| FSR threshold relative to the standing reading | Every person and every sensor gives a different raw level. A fixed number would fit one setup only | Needs 1 s of standing at power-on |
| Two levels, strike above / lift below (hysteresis) | Near a single threshold a noisy signal crosses back and forth and one step fires many times | Lift is reported a little late |
| Smoothing each FSR reading | Single noisy ADC readings would cross the threshold by themselves | Small delay on the strike time |
| 120 ms minimum gap | Bounce at landing counted as a second step | A real step inside the gap would be lost (none seen) |
| Timestamp taken on the sender, at the event | Radio and USB delay vary. Timing the event on arrival would add that variation to every measurement | Needs a clock conversion on the PC (section 3) |
| IMU sampled by a timer, in its own task | A steady 100 Hz regardless of what the FSR loop is doing. Slow I2C reads do not block step detection | Missed ticks are dropped, not caught up |
| 5 samples per radio packet, each with its own timestamp | 100 packets per second would load the radio. 20 packets carry the same data | Data reaches the PC up to 50 ms later; timing is unaffected |

## 2. Receiver ESP32 (at the PC)

**In plain words:** a second small computer, plugged into the PC by USB. It is the other end of
the radio: it catches every message from the walker and hands it to the PC as a line of text. On
the way it does one small calculation: how many steps per minute the walker is taking.

**Job:** take the radio messages and print them as text lines on USB serial (921600 baud). It also
computes the walking tempo from the FSR steps.

**File:** `ESP32/step_receiver_sketch/step_receiver_sketch.ino`

### 2.1 Forwarding
- Each radio message is checked (length, protocol version) and printed as one line: `STEP`, `LIFT`,
  `IMU` (one line per sample), `STATUS`.
- The lines keep the sender's timestamp. The receiver adds no time of its own.
- Sensor lines are printed only during a session (between `START` and `RESET` from the PC).

### 2.2 Tempo from steps
- Each `STEP` carries the time since the previous step (either foot).
- **Instant BPM** = 60000 / that interval in ms.
- **Average BPM** = 60000 / mean of the last N intervals. This is the tempo the PC receives.
- N is a setting: "Smoothing Window" in the GUI, or `--smoothing N` on the command line.
  Default 3, range 1-20.
- The first step of a session has no interval, so it gives no BPM.

**Why BPM, and why an average**
- Music tempo is in beats per minute. One beat per step, so a step interval converts directly:
  BPM = 60000 / interval in ms.
- One interval alone is noisy: steps vary naturally, left and right differ, the FSR timing
  jitters. An average smooths that.
- The price is lag: after a pace change the average is fully updated only N steps later.
- N = 3 is the default, inherited from the first project, with no published source. D-Jogger
  (Moens et al. 2014) averages the last 5 steps.

### 2.3 Link health
- Once per second, `RXSTAT`: messages received, messages dropped because the queue was full,
  messages with a wrong length or version.

### 2.4 Commands from the PC
- Session: `RESET`, `START`, `VERSION`.
- Tempo window: `SET_WINDOW` (1-20 steps), `SET_STRIDE`.
- Passed on to the sender: `CAL_WEIGHT` (FSR threshold), `IMU_STREAM` (on/off), `PING` (clock check).

### 2.5 Constants
| Value | Meaning | Source |
|---|---|---|
| 3 steps (default) | tempo averaging window | OURS |
| 921600 | serial baud rate | OURS |

### 2.6 Known limits
- A window of 3 steps holds two steps of one foot and one of the other, so a left/right difference
  makes the tempo zig-zag.
- No rejection of missed or double steps. None occurred in 1275 recorded strikes; checked per
  session.
- This tempo drives the music only when phase alignment is off and the mode follows the walker.
  In fixed-tempo mode it is logged but not used.

### 2.7 Methods and logic
| Method | Problem it solves | Cost |
|---|---|---|
| Radio callback only stores the packet; the main loop prints it | Printing is slow. Doing it inside the callback would lose packets | Queue of 32 packets; overflow is counted |
| Text lines on serial | Readable in any serial monitor, easy to log and debug | More bytes than binary; needs 921600 baud |
| Sensor lines only between `START` and `RESET` | During setup the PC must read command answers without sensor data in between | Nothing is recorded outside a session |
| Average of the last N step intervals | One interval is noisy (section 2.2) | Lag of N steps |
| Counters once per second | A weak link is visible during the session, not after | none |

## 3. PC input layer

**In plain words:** the first piece of the PC program. Its job is tidying up. It turns the text
lines back into numbers with real units, and it solves a clock problem: the walker's computer and
the PC each have their own clock, and the two do not agree. To play a beat at the moment a foot
lands, the PC must translate "when it happened on the foot" into "when that is on my clock".

**Job:** turn the receiver's text lines into messages with real units and one continuous time, and
keep count of what was lost on the way. No gait logic here.

**File:** `server/utils/hardware/sensor_stream.py`

### 3.1 Reading
- A separate thread reads the serial port and stamps every line with the PC time it arrived.
- The main loop picks the lines up on its next pass. It never waits on the port, so the music
  timing is not held up by the sensors.

### 3.2 Parsing and units
- Each line becomes one message: `STEP`, `LIFT`, `IMU`, `STATUS`, or a control answer.
- IMU counts become physical units: acceleration = counts / 100 (m/s²), rotation speed =
  counts / 16 (°/s).
- The values stay in the sensor's own axes. Choosing the axis across the foot happens in section 4.
- A line that cannot be parsed is counted and skipped.

### 3.3 One continuous sender time
- The sender's timestamp is a 32-bit microsecond counter. It wraps to zero about every
  71.6 minutes.
- The PC unwraps it into continuous seconds, so a session can run past a wrap.

### 3.4 Sender clock to PC clock
- Two clocks exist: the sender's (when things happened on the foot) and the PC's (when the music
  plays).
- For each message: `arrival − sender time = clock offset + transport delay`. The delay is never
  negative and is often near its minimum.
- So the smallest value in each 1 s window follows the clock offset. A straight line through those
  minima over the last 60 s also follows the slow drift between the two crystals.
- **Where this matters:** only for placing a beat on a predicted strike (phase alignment).
  Comparing an IMU event with an FSR strike uses the sender clock alone, so the clock mapping
  cannot affect detection results.

### 3.5 Link health
- Every message type has its own sequence number. A jump in the numbers = messages lost on the
  radio.
- Totals per type are written to the session log and to `meta.json` at the end.

### 3.6 Constants
| Value | Meaning | Source |
|---|---|---|
| 100 counts per m/s², 16 counts per °/s | BNO055 raw units | BNO055 datasheet (to be re-checked) |
| 1 s | window for the smallest offset | OURS |
| 60 s | span of the drift line | OURS |

### 3.7 Known limits
- The clock mapping is our own method, with no published source.
- The smallest transport delay (radio + USB) is not removed; it stays inside the offset. Phase
  alignment absorbs it in its latency setting.
- In the first 5 seconds there is no drift estimate yet, only an offset.

### 3.8 Methods and logic
| Method | Problem it solves | Cost |
|---|---|---|
| Reader in its own thread | The arrival time is taken when the line arrives, not when the main loop gets to it | none |
| Unwrapping the 32-bit timestamp | The counter restarts every 71.6 min; without this, time would jump backwards | none |
| Smallest `arrival − sender time` per second | Each message is late by an unknown, changing delay. The delay cannot be negative, so the smallest value is the one closest to the true clock offset | The fixed minimum delay stays in |
| Line through the minima of the last 60 s | Two crystals never run at exactly the same speed; the offset drifts slowly | No drift estimate in the first 5 s |
| Sequence number per message type | Without it a lost message is invisible | none |

## 4. Calibration: teaching the system how each sensor sits on the shoe

**Files:** `server/utils/gait/calibration.py`, `server/utils/analysis/calibrate_imus.py`

### 4.1 The problem
Each sensor measures how fast it is turning, in three directions of its own: its x, y and z,
printed on the board. The sensor has no idea how it is attached to the shoe. Stick it on straight,
tilted, or upside down, and the same foot movement shows up on different axes.

The step detector needs one simple thing: **how fast is the foot tipping, toes up or toes down?**
That is called the pitch rate. To get it out of three raw numbers, the system must know which
direction, in the sensor's own terms, runs across the foot.

Think of a door. It swings around its hinge line. A foot does the same when walking: it tips
around a line running across it, from the inner side to the outer side. Calibration means finding
that hinge line for each sensor.

### 4.2 The idea: three questions, each answered by something the sensor can feel
The sensor cannot see. But it feels two things: gravity and rotation.

| Question | When can the sensor know? | Why |
|---|---|---|
| Which way is **up**? | When the foot stands still | A still sensor feels only gravity, like a plumb line. Gravity always points down |
| Which line runs **across** the foot? | When walking | The foot tips around one line far more than around any other. The direction with the most rotation is the hinge line |
| Which way is **forward**? | When walking forward | The hinge line alone has two ends and no front. Only actually moving forward shows which side is the front |

Once up and forward are known, the tipping direction follows: turning "forward" towards "up" is
the toes lifting. We define that as positive.

Standing still can only answer the first question. Turn a standing sensor around like a compass
needle and it feels exactly the same, so it cannot know where "across" or "forward" is. That is
why calibration needs a short walk, and why walking in place is not enough.

### 4.3 Where the idea comes from
- **Our event rules** come from Sabatini et al. 2005, a study that put a gyro on the foot and
  checked its timing against foot switches. In that study the sensor was strapped on by hand so
  that its axis already pointed across the foot. No calibration, just careful mounting.
- **We cannot rely on careful mounting.** Different shoes, different users, printed holders, and
  two sensors. A few degrees off and part of the foot's motion is lost.
- **Our first attempt** (October) guessed the axis and its sign automatically while walking, with
  our own rules. The axis was often right, but the sign flipped between sessions: it depended on a
  few milliseconds of data timing. Detections then landed on the wrong events.
- **The fix** is to use a published, tested method instead of our own:
  - "Up from gravity while standing, across from the main rotation while walking" is described by
    Mihy et al. (2022, preprint).
  - The same steps, plus the forward step that settles the sign, are implemented and tested in
    gaitmap, an open gait-analysis library from a university lab (Kuderle et al. 2024). We use
    their code as it is.
- **Ours, and marked as ours:** combining gaitmap's three steps into one axis, and the rule
  "forward × up = pitch axis".

### 4.4 What we do in practice
1. Mount the sensors.
2. Press **CALIBRATE IMUs** in the app and do the **calibration walk**: stand still 5 s, walk
   straight ahead about 20 steps, stand still 5 s. Press STOP.
3. The app then runs the calibration script on that recording. It answers the three questions
   for each sensor and saves the result in `imu_calibration.json`.
4. Every session after that uses the file. Redo the walk only when a sensor is remounted.

The calibration walk is used for nothing else. It is not a test session.

### 4.5 A second, smaller calibration: the zero point
A gyro that is not moving should read zero. In practice it reads a small number that changes from
day to day, like a kitchen scale that shows 3 g with nothing on it.

- Fix, from Sabatini 2005: at the start of every session, stand still for half a second. The
  average reading in that half second is the zero point (the bias), and it is subtracted from
  then on.
- The system checks that the foot was really still in that half second (spread under 2 °/s on
  every axis). If not, it waits for the next quiet half second.
- The same half second gives gravity as this sensor feels it: its size (the recorded IMUs read
  about 10.4 m/s², not 9.81) and, if the calibration file has none, which way is up.
- No events are detected until a zero point is accepted. So every session must start standing
  still.

So there are two calibrations, often confused:
- **How the sensor sits** (the axis): one walk, once per mounting.
- **Where its zero is** (the bias): half a second standing, every session.

### 4.6 The result
`pitch rate = (gyro reading − zero point), measured along the hinge line`

One number per foot, in degrees per second, positive when the toes go up. Everything in section 5
works on this number.

### 4.7 Traceability
- The calibration file records which session it came from and when.
- A copy stays in the calibration session's folder.
- Every later session stores the calibration it ran with in its `meta.json`, and replay uses
  that one.
- No file: the app warns that the session's IMU events are not valid. Nothing is guessed in its
  place: without the axis, the rotation events (section 5.4) are off.

### 4.7b Has the sensor moved since?
Two checks, both against the calibration:
- **At every session start,** while the walker stands still: gravity as the sensor feels it now is
  compared with the "up" stored by the calibration. More than 10° apart: warning to recalibrate.
  The angle is saved with the session (`meta.json`). Sabatini et al. check their mounting the
  same way before each trial.
- **After any session,** on request: `calibrate_imus.py <session> --check` measures the axes of
  that session again and prints how far they are from the calibration. Nothing is changed.

### 4.8 Quality numbers (printed by the script)
| Number | Meaning | Good |
|---|---|---|
| pitch share | part of all rotation that is around the pitch axis | high (test data: 0.83-0.87) |
| fit error | whether the three steps add up to one clean rotation | near 0 °/s |
| angle to board axis | how far the measured axis is from the nearest printed axis | shows how the sensor sits |
| largest gap | longest stretch of lost samples | a few hundredths of a second |

### 4.9 Validation so far
- gaitmap's example walk, turned into four fake mountings (tilted, flipped, sideways, upside
  down), with added gyro bias and lost packets.
- Axis found within 1-3° of the main rotation axis every time, sign correct every time.
- On that data the hand-set axis of its authors captures 70 % of the rotation, the measured axis
  83-87 %.
- Not yet run on our own sensors.

### 4.10 Constants
| Value | Meaning | Source |
|---|---|---|
| 0.5 s | bias window | Sabatini 2005 §II-C.2 |
| 2 °/s | "standing still" limit for the bias | OURS, placeholder until the table recording |
| 0.1 s, 15 °/s | "standing" for the gravity step | gaitmap example |
| 10° | "the sensor has moved" warning | OURS, placeholder |
| 0.2-0.3 s after a landing, 20 % per sample | "up" pulled back to gravity while the foot is flat | OURS |

### 4.11 Known limits
- Needs real forward walking. Walking in place gives no forward direction.
- "left" / "right" are IMU labels (I2C address). Which foot each is on is set once, in
  `calibration.py` (`IMU_OF_FOOT`): crossed, as the shoes are wired (section 1.2). Rewiring the
  shoes means changing that line.
- The axis is fixed for the session. A shift of the sensor is checked at the session start only
  (section 4.7b), not during the walk. A turn of the sensor around the vertical is not seen by
  that check; the `--check` run sees it.
- None of the 1 Nov recordings contains 0.5 s quiet enough for the bias.

### 4.12 Methods and logic
| Method | Problem it solves | Cost |
|---|---|---|
| Measure the axis from a walk, not from how the sensor is mounted | Hand mounting is never exact, and differs per shoe and per user | One short walk per mounting |
| Axis fixed during a session | An axis that changes mid-session changes the detector mid-session; decisions could not be explained | A shifted sensor must be caught separately |
| Sign from forward movement | Our earlier sign vote from FSR timing flipped between sessions | Needs forward walking |
| Bias from standing still, with a quietness test | A bias measured while moving is wrong by tens of °/s and breaks the 30 °/s thresholds | Session must start still |
| Calibration stored with every session | Any result can be traced to the mounting it was recorded with | none |

## 5. Gait events: reading the step from the foot's rotation

**File:** `server/utils/gait/events.py`

### 5.1 The problem
The pressure pad tells us the heel has landed, but only at the moment it lands. To put a beat *on*
the landing, the music must know a little earlier that the landing is coming. So we need to follow
the foot while it is still in the air.

### 5.2 The idea: every step draws the same shape
Section 4 gave one number per foot: how fast it is tipping, toes up (positive) or toes down
(negative). Over one step that number always draws the same shape:

| Phase | What the foot does | Pitch rate |
|---|---|---|
| Standing on it | flat on the ground | about 0 |
| Push-off | heel rises, toes still down | goes negative; deepest point = **toe-off** |
| Swing | foot travels forward, toes come up | one broad positive bump; its top = **mid-swing** |
| Landing | heel touches, foot slaps down flat | negative again; deepest point = **heel strike** |
| Standing on it | flat | back to about 0 |

Typical sizes: toe-off around −500 °/s, mid-swing around +350 °/s, heel strike around −200 °/s
(gaitmap example walk; Sabatini reports similar).

Finding the events means finding these dips and bumps, in order.

### 5.3 Where the idea comes from
Sabatini et al. 2005 strapped a gyro on the foot of five people on a treadmill and compared its
timing with foot switches under the heel and toe. Their rules, which we use:
- the foot counts as "moving" above 30 °/s and "flat" below 30 °/s;
- toe-off and heel strike are the two negative peaks, mid-swing the positive peak between them.

Their result: heel strike found 2 ms from the foot switch on average (95 % between −16 and
+12 ms).

### 5.4 The rules, step by step
The detector is always in one of four phases and waits for one thing:

| Phase | Waits for | Then reports | Rule from |
|---|---|---|---|
| Stance | rotation reaches 30 °/s | **heel-off (HO)** | Sabatini Eq. 2 |
| Push-off | follows the dip down; rotation turns positive | **toe-off (TO)**, timed at the deepest point | Sabatini §II-C.1 |
| Swing | follows the bump up; rotation turns negative | **mid-swing (MS)**, timed at the top | Sabatini §II-C.1 |
| Landing | follows the dip down; rotation falls under 30 °/s | **heel strike (HS)**, timed at the deepest point | Sabatini §II-C.1, Eq. 3 |

Then back to stance.

### 5.5 Where exactly a decision is made
- **Only two thresholds exist**, both 30 °/s: one starts a step (heel-off), one ends it (foot
  flat).
- **The unit is degrees per second**, a rotation *speed*. These rules use no angle in degrees.
- Everything between the two thresholds is decided by shape, not by a number: the lowest point of
  a dip, the highest point of a bump, and the moment the signal changes sign.
- **An event is noticed later than it happened.** A peak is only known to be the peak once the
  signal has turned back. The event keeps the time of the peak itself, so this delay does not shift
  its timestamp.
- **Mid-swing is reported early.** The top of the swing bump is only certain when the bump is over,
  just before the landing: too late to use (section 6.2). So mid-swing is reported as soon as the
  rotation has fallen to 80 % of its highest value, still timed at the top. The swing bump is flat
  on top, so noise decides where exactly "the top" is.

### 5.6 How a wrong decision happens
| Cause | Effect |
|---|---|
| Zero point (bias) wrong by tens of °/s | "standing" reads above 30 °/s: false heel-off, or the foot never counts as flat |
| Weak signal (sensor loose or badly placed) | the step never reaches 30 °/s, or the bump has no clear top: events missing |
| Vibration at landing | extra sign changes: the wrong dip is taken as heel strike |
| Wrong sign of the axis | dips and bumps swap: toe-off and heel strike are looked for in the swing |

The last one is what happened in the 1 Nov recordings (section 4.3).

### 5.7 Smoothing
The pitch rate is lightly smoothed before the rules run: `new = old + 0.5 × (raw − old)`. This
removes fast jitter above roughly 11 Hz. Sabatini used a 15 Hz filter that needs the whole
recording; ours works live.

### 5.8 The landing, and how late the pressure pad is
The landing shows as a sharp jolt in the accelerometer: from one sample to the next, its reading
jumps by more than 9 m/s². A jolt counts as a **landing (LAND)** in one of two ways:
- the foot has lifted since its last landing (section 5.8b) and is nearly flat at the jolt (tilted
  less than 40°): reported 30 ms later;
- otherwise, once the foot settles: its rotation stays under 150 °/s from 0.1 to 0.2 s after the
  jolt. Reported 0.2 s later.

A toe-off can jolt as hard when walking, but the foot is steep then and swings on afterwards, so it
is not taken for a landing. A jolt twice as big within 30 ms replaces the first one (that was a
knock just before the contact). The landing keeps the time of the jolt.

The pad needs some pressure before it fires, so it reports the landing slightly late: 20-40 ms in
the 1 Nov recordings, where the toes land first and the heel after. For every pad strike the gap to
the landing before it is measured; the median of the last 20 is the pad's lateness. This
measurement is our own, with no published source.

### 5.8b The lift: rise, fall and top
The accelerometer feels gravity plus the foot's own acceleration. With "up" known (section 1.2),
the part along "up" minus gravity is the foot's vertical acceleration. Added up from a moment the
foot is flat (0.3 s after a landing, speed 0), it gives the foot's vertical speed:

| Event | When |
|---|---|
| **Rise (RISE)** | the upward speed passes 0.05 m/s: the foot leaves the ground |
| **Fall (FALL)** | the upward speed is back to half its peak: the rise is ending |
| **Top (TOP)** | the speed reaches 0: the foot's highest point |

These need no forward swing, so they also work when stepping in place, where the rotation events
of section 5.4 have little to go on. No top within 1 s means the foot did not step.

### 5.9 Who uses the events
- **LAND** teaches the strike prediction (section 6.3) and gives the pad's lateness (section 5.8).
- **TO, MS, rise, fall and top** feed the strike prediction (section 6).
- **HO and HS** are logged, to compare with the pressure pad (pad lift ↔ HO, pad strike ↔ HS).

### 5.10 Validation so far
- The detector reproduces every event of the recorded sessions when they are replayed (same code,
  same data).
- Against the pressure pad, on the 1 Nov recordings: heel strike 34 to 102 ms late (median per
  session). **Not a valid test:** those recordings had no zero point, an unknown axis and a
  guessed sign.
- The real test needs recordings made after a calibration walk.

### 5.11 Constants
| Value | Meaning | Source |
|---|---|---|
| 30 °/s | heel-off threshold | Sabatini 2005 Eq. 2 |
| 30 °/s | foot-flat threshold | Sabatini 2005 Eq. 3 |
| 0.5 | smoothing factor | OURS |
| 1.0 s | a step that takes longer is dropped (walker stopped) | OURS |
| each peak must have the right sign | reject malformed steps | OURS (the paper shows clean steps only) |
| 80 % | mid-swing is reported once the bump falls to this share of its top | OURS |
| 9 m/s² | landing: jump of the acceleration from one sample to the next | OURS |
| 30 ms, twice as big | a later jolt that replaces the first | OURS |
| 40°, 30 ms | nearly flat after a lift: a landing, reported this soon | OURS |
| 150 °/s over 0.1-0.2 s | otherwise: the foot settles after a landing | OURS |
| 0.4 s | one landing per foot within this | OURS |
| 0.3 s | after a landing the foot is flat: the lift is followed from here | OURS |
| 0.05 m/s, 50 % | rise, fall | OURS |
| 1.0 s | no top this long after the flat moment: no step | OURS |

### 5.12 Known limits
- Sabatini's thresholds were tuned at 200 Hz, on a treadmill, with their filter. We run at 100 Hz,
  overground, with ours. They must be checked on our data.
- The paper detects events after the walk. We do it live, so each event is known only a moment
  after it happened.
- The pressure pad is under the heel only, so toe-off has no direct ground truth.
- The landing and lift rules are ours. They are not yet checked on recordings made after a
  calibration walk.

### 5.13 Methods and logic
| Method | Problem it solves | Cost |
|---|---|---|
| Published thresholds (30 °/s) | Our own (−60, +150 °/s) had no basis and missed or mislabelled steps | Tuned on another setup; need checking |
| Peaks and sign changes, not fixed levels | People walk at different speeds; peak height changes, shape does not | Event known only after the peak has passed |
| Fixed order of phases | A dip in the swing cannot be mistaken for toe-off | One missed event can cost the whole step |
| Event timed at the peak | Detection delay does not shift the timestamp | none |
| Light live smoothing | Jitter would create false sign changes | Small delay; differs from the paper's filter |
| Mid-swing reported at 80 % of its top | Known early enough to be used (section 6.2) | Reported a moment after the top |
| Landing from the accelerometer jolt | The real contact, known 30 ms later, before the pad fires | A hard toe-off jolts too: needs the tilt or settle check |
| Lift from the vertical speed | Works with or without a forward swing | Needs "up", the zero point and a flat moment |

## 6. Strike prediction: knowing the landing before it happens

**Files:** `server/utils/prediction_model/strike_predictor.py`, `server/utils/engine/strike_tracking.py`

**Status: built, not evaluated.** It depends on the events of section 5, and those are not
validated on our data yet. Detection is checked first.

### 6.1 The problem
A beat that should sound when the heel lands must be sent before the heel lands. So the PC needs
the landing time in advance.

### 6.2 The idea: six clues
Like waiting for a train:

| Clue | Train | Foot | Known ahead |
|---|---|---|---|
| Rhythm | the timetable | last landing + this person's usual step time | a whole step |
| Toe-off | "it left the previous station" | the foot left the ground; the landing follows after a nearly fixed time | roughly 300-450 ms |
| Mid-swing | "you can see it coming" | the foot is halfway through the air | about 200 ms, minus the wait to 80 % (section 5.5) |
| Rise | "it left the previous station" | the foot left the ground (section 5.8b) | roughly 250-350 ms |
| Fall | "it is braking" | the foot's rise is ending | roughly 150-250 ms |
| Top | "it is at the platform signal" | the foot's highest point | roughly 100-200 ms |

Each clue's guess is its event's time plus this person's usual delay from that event to the
landing. Toe-off and mid-swing need the foot to swing forward. Rise, fall and top only need it to
go up and down, so they also work when stepping in place.

In the published measurements of Zank et al. the end of the swing is 24 ms before the landing on
average. A beat has to be sent about 30 ms early, so mid-swing is reported before the swing ends
(section 5.5).

### 6.3 Learning from every real landing
- Each time a landing is detected (section 5.8), the real delays are measured: from each clue's
  event to the landing, and the step time.
- The last 8 are kept per foot, and their median is "this person's usual".
- Each clue is also scored: how far off its last guess before the landing was.
- The pressure pad is not used to learn. It stays the referee: each pad strike is paired with its
  landing, and the gap gives the pad's lateness (section 5.8).

So the system adapts to the person. Nothing is preset per user.

### 6.4 Combining the clues
- The guesses are averaged, each weighted by how accurate it has been lately (weight = 1 / error²).
  This is a standard rule in statistics (inverse-variance weighting).
- A guess that disagrees strongly with the best one is left out (pace change).
- A guess more than 1 s overdue is dropped: the person stopped.

### 6.5 Before use
- The target is the landing itself, the real contact, not the pad (section 5.8).
- The time is converted from the sender's clock to the PC's (section 3.4).

### 6.6 Where the idea comes from, and the published work closest to it
- **The design is ours** (October). It was not taken from a paper.
- **Closest published work:** Zank, Nescher, Kunz (ETH Zurich, 2013). They predict the moment of
  the step sound from an IMU on the shoe, to play a synthetic sound on time. Same task as ours.

What they do differently:
- **Their events are zero crossings**, the moments a signal changes sign. Their reason: a threshold
  either depends on the person or is so low that noise triggers it. A sign change needs no number.
- **They use the accelerometer too**: forward and upward acceleration give events in the middle of
  the swing, where the gyro alone has none.
- **The time left is a formula of the time between two events**, fitted on data. A faster step is
  then predicted shorter automatically.

Their results, 10 people, 154 steps, tested on people the predictor had not seen:

| Events used | Sensors | Predicts ahead | Error (standard deviation) |
|---|---|---|---|
| start of swing + forward acceleration | gyro + accelerometer | 219 ms | 34 ms |
| start of swing + upward acceleration | gyro + accelerometer | 88 ms | 24 ms |
| forward acceleration + end of swing | gyro + accelerometer | 24 ms | 17 ms |
| start and end of swing | gyro only | 24 ms | 18 ms |

Two of their four events already exist in our detector: the start of the swing is the moment we
report toe-off, the end of the swing is the moment the swing bump ends. Their upward acceleration
is close to our lift events (section 5.8b).

### 6.7 How it is evaluated
`server/utils/analysis/score_predictions.py` replays a recorded session through the detector and
the prediction, as the app runs them. For every pad strike it takes the guess that was current
80-300 ms before it, counted from when the IMU data reached the PC (the radio delay included), and
compares it with the strike. It uses the calibration saved with the session, or one given with
`--calibration`.

`server/utils/analysis/prediction_comparison.py` compares our clues with Zank et al. It replays a
recorded session and, for every landing,
measures for our clues and for the four predictors of Zank et al.:
- **lead time:** how long before the landing the prediction was available;
- **error:** predicted landing minus real landing.

It prints one table and draws one graph (lead time against error). Their formulas are refitted on
our data, because their target was the step sound and ours is the pressure pad.

**First run, on a public example walk (57 landings), to test the tool only:**

| Method | Predicts ahead | Error (standard deviation) |
|---|---|---|
| Ours, rhythm | 540 ms | 17 ms |
| Ours, toe-off | 321 ms | 10 ms |
| Zank, start of swing + forward acceleration | 131 ms | 15-19 ms |
| Ours, mid-swing | never before the landing | - |
| Zank, the two predictors that end at the end of the swing | never before the landing | - |

These numbers are **not a result**: that walk has no pressure pad. Its "landings" were computed
from the same motion sensor, so the errors are too optimistic. The upward-acceleration event did
not appear before the landing in that data; whether it does on our shoes is open. Valid numbers
need our own recordings, made after a calibration walk.

### 6.8 Constants (all OURS)
| Value | Meaning |
|---|---|
| 8, minimum 3 | landings kept per foot; needed before a clue is used |
| 40 / 25 / 12 ms | assumed error of rhythm / toe-off / mid-swing before any score exists |
| 30 / 25 / 25 ms | the same for rise / fall / top |
| 0.1-0.8 s, 0.02-0.5 s | plausible delay from rise, and from fall or top, to the landing |
| 0.25-1.5 s | step times outside this are not walking |
| 0.7-1.3, power 0.75 | limits and strength of the swing-speed correction |
| 3 | a clue more than 3 of its own errors from the best one is left out |
| 1 s | an overdue guess is dropped |

The assumed errors (12 and 25 ms) are more optimistic than the published 17-34 ms.

### 6.9 Known limits
- No part is validated on our data.
- Wrong events give confidently wrong predictions; nothing here can detect that.
- It learns from the IMU landing, so a missed or false landing is learned too.
- `prediction_comparison.py` still learns from the pad (the earlier design), so its numbers are
  for that design.

### 6.10 Methods and logic
| Method | Problem it solves | Cost |
|---|---|---|
| Several clues, not one | Early clues give time, late clues give precision | More to validate |
| Learn delays per person and per foot | People and legs differ | Needs a few steps before it works |
| Median of the last 8 | One odd step should not move the estimate | Slow to follow a real change |
| Weight by recent error | The system finds out by itself which clue to trust | Needs a landing as referee |
| Learn from the IMU landing, not the pad | The landing is the real contact; the pad fires 20-40 ms after it, by a varying gap | A missed landing is a step without learning |
| Lift clues next to swing clues | The prediction works whether the person walks or steps in place | Three more clues to validate |
| Drop overdue guesses | A weight shift while standing looks like a toe-off | none |

## 7. Tempo and music

**Files:** `server/utils/engine/BPM_estimation.py`, `server/utils/Modes/`,
`server/utils/engine/phase_controller.py`, `server/utils/hardware/midi_player.py`

**In plain words:** the music is a MIDI file, a list of notes placed on beats. The player can play
it at any tempo and change the tempo at any moment. Two questions remain: how fast (the tempo),
and when exactly each beat falls (the alignment). They are answered by two separate parts.

### 7.1 The player
- Keeps a beat clock: its position in the song, in beats, advances at the current tempo.
- The tempo can be set at any time without a jump in the music.
- It records the time every beat was played (`beats.csv`), so beats can be compared with landings
  afterwards.

### 7.2 How fast: the modes
A mode decides the target tempo. The music then glides towards the target; it never jumps.

| Mode | Target tempo | Used for |
|---|---|---|
| Manual | a fixed number | the fixed-tempo recordings |
| Random | a random tempo within ±20 % of the song's; a new one after 20 matching steps or 30 s | recordings with tempo changes |
| Dynamic | the walker's measured steps per minute | music that follows the walker |
| Hybrid | follows the walker, then holds the tempo while the walking is steady | the same, calmer |

For the supervisor's two experiments (fixed tempo; tempo change every 15-20 s) Manual and Random
are used. In both, the music does not react to the steps at all.

### 7.3 The walker's tempo, when the music follows
Two paths exist, depending on whether beat alignment is on:

| | Alignment off | Alignment on |
|---|---|---|
| Steps per minute from | the receiver's average of the last N steps (section 2.2) | a line fitted through the last 8 landing times |
| Then | blended with a learned model: 65 % model, 35 % measured | used as it is |

The learned model (LightGBM) comes from the first project. It predicts the next tempo from the
recent ones.

### 7.4 When exactly: beat alignment (switched off for now)
- With alignment on, the tempo gets a small extra push or brake so that the next beat falls on the
  predicted landing (section 6). Applied tempo = mode tempo × (1 + correction), correction at most
  ±15 %.
- It also starts the music on a step and pauses it when the walker stands still.
- **It is off in the current experiments**, at the supervisor's request: first detection and
  prediction are validated, with music that does not react.

### 7.5 Sound delay
From sending a note to hearing it takes time (default setting 30 ms). It can be measured with a
microphone: "Measure latency" in the GUI, or `server/utils/analysis/measure_latency.py`.

### 7.6 Constants (all OURS unless noted)
| Value | Meaning |
|---|---|
| 0.1 up, 0.05 down | how fast the music glides to a higher / lower target |
| 65 % / 35 % | model / measurement blend |
| 8 steps | line fit for the tempo with alignment on |
| ±20 %, 20 steps, 30 s | random mode: range, steps to match, time limit |
| ±15 %, 30 ms | alignment: largest correction, assumed sound delay |

Published baseline to compare with: D-Jogger (Moens et al. 2014) takes the tempo from the average
of the last 5 steps.

### 7.7 Known limits
- The walker's tempo is computed in three places (receiver, model blend, line fit). One path would
  be easier to explain and to test.
- The learned model is inherited and has not been re-examined in this project.
- Beat alignment is not validated.
- Random mode has no plain "change every N seconds" setting. Closest: time limit 15-20 s and a
  step count that is never reached.

### 7.8 Methods and logic
| Method | Problem it solves | Cost |
|---|---|---|
| Beat clock in the player | The tempo can change mid-song and every beat time is known | none |
| Modes separate from alignment | "How fast" and "when" can be tested one at a time | Two parts to understand |
| Gliding to the target tempo | Tempo jumps sound wrong | The music lags a tempo change |
| Fixed and random modes that ignore the steps | Detection can be tested without the music reacting | none |

## References

1. A. M. Sabatini, C. Martelloni, S. Scapellato, F. Cavallo, "Assessment of Walking Features From
   Foot Inertial Sensing", *IEEE Trans. Biomed. Eng.* 52(3):486-494, 2005.
   DOI 10.1109/TBME.2004.840727. Read (summary: `Sabatini_2005_what_we_use.md`).
2. A. Kuderle et al., "Gaitmap - An Open Ecosystem for IMU-Based Human Gait Analysis and Algorithm
   Benchmarking", *IEEE Open J. Eng. Med. Biol.*, 2024. Code used as it is (version 2.6.0);
   documentation read, paper not read.
3. Mihy, Wagatsuma, Cain, Hafer, "A Functional Sensor-to-Segment Orientation Method to Reduce the
   Effects of Varied Sensor Placement", medRxiv preprint, 2022. DOI 10.1101/2022.11.29.22282894.
   Read. Not peer reviewed.
4. M. Zank, T. Nescher, A. Kunz, "Robust Prediction of Auditory Step Feedback for Forward Walking",
   *VRST 2013*. Read (PDF in this folder). The longer journal version (*Computer Animation and
   Virtual Worlds* 26(6), 2015) is not read.
5. B. Moens et al., "Encouraging Spontaneous Synchronisation with D-Jogger, an Adaptive Music
   Player That Aligns Movement and Music", *PLoS ONE* 9(12):e114234, 2014. Read.
6. O. J. Woodman, "An introduction to inertial navigation", Univ. of Cambridge Tech. Report
   UCAM-CL-TR-696, 2007. Read (sections 5.1 and 6).
7. Bosch Sensortec, BNO055 datasheet. Unit factors and filter bandwidths still to be re-checked.

Found, abstract only, not used yet: Figueiredo et al. 2018 (*IEEE TNSRE* 26(10), live gait events
from a gyro on the foot); Mannini, Genovese, Sabatini 2014 (*IEEE JBHI* 18(4), online detection
from a foot gyro); Yan et al. 2017 (*Autonomous Robots* 41(3), gait phase from adaptive
oscillators).
