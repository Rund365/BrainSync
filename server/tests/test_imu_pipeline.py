"""
Unit tests for the IMU pipeline (Integration Plan §23.1). No hardware, no extra packages.

Run from the repo root:
  .venv\\Scripts\\python.exe -m unittest discover -s server\\tests -v
"""
import shutil
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # server/

from utils.engine.phase_controller import PhaseController, PhaseSettings
from utils.hardware.comms import handle_step
from utils.hardware.sensor_stream import (ClockMapper, ImuMsg, LiftMsg, OtherMsg, StatusMsg, StepMsg, TimeUnwrapper,
                                          T_US_WRAP, parse_line)
import utils.prediction_model.strike_predictor as sp
from utils.prediction_model.strike_predictor import StrikePredictor, RIGHT, LEFT, other as other_foot


class ParserTests(unittest.TestCase):
    def test_step_line(self):
        msg = parse_line("STEP,812,123456789,1,512,2210,117.19,116.40")
        self.assertIsInstance(msg, StepMsg)
        self.assertEqual((msg.seq, msg.t_us, msg.foot, msg.interval_ms), (812, 123456789, 1, 512))
        self.assertAlmostEqual(msg.avg_bpm, 116.40)

    def test_imu_line(self):
        msg = parse_line("IMU,4051,123460000,2," + ",".join(str(i) for i in range(12)))
        self.assertIsInstance(msg, ImuMsg)
        self.assertFalse(msg.valid_left)
        self.assertTrue(msg.valid_right)
        self.assertEqual(msg.right.tolist(), list(range(6, 12)))

    def test_legacy_and_unknown(self):
        self.assertEqual(parse_line("1234, 1, 110.00, 108.50").kind, "LEGACY_STEP")
        self.assertEqual(parse_line("hello").kind, "UNKNOWN")
        self.assertIsInstance(parse_line("ACK,START"), OtherMsg)

    def test_malformed(self):
        with self.assertRaises(ValueError):
            parse_line("IMU,1,2,3")
        with self.assertRaises(ValueError):
            parse_line("STEP,1,2,x,4,5,6,7")

    def test_handle_step_both_protocols(self):
        self.assertEqual(handle_step(b"STEP,1,500,2,550,1200,109.09,108.00", 100.0)[0], 108.0)
        self.assertEqual(handle_step(b"1234, 1, 110.00, 0.00", 100.0)[0], 100.0)  # bpm 0 -> current
        with self.assertRaises(ValueError):
            handle_step(b"IMU,1,2,3", 100.0)


class FirmwareProtocolTests(unittest.TestCase):
    """The firmware's protocol header against the Python parser."""
    ESP32 = Path(__file__).resolve().parents[2] / "ESP32"

    def test_header_copies_identical(self):
        sender = (self.ESP32 / "step_detection_sketch" / "protocol_v2.h").read_bytes()
        receiver = (self.ESP32 / "step_receiver_sketch" / "protocol_v2.h").read_bytes()
        self.assertEqual(sender, receiver, "the two protocol_v2.h copies differ")

    @unittest.skipUnless(shutil.which("g++"), "needs g++")
    def test_firmware_lines_parse(self):
        with tempfile.TemporaryDirectory() as tmp:
            exe = Path(tmp) / "protocol_check"
            subprocess.run(["g++", "-std=gnu++11", "-o", str(exe), str(self.ESP32 / "host_test" / "protocol_check.cpp")],
                           check=True, capture_output=True)
            run = subprocess.run([str(exe)], capture_output=True, text=True)
        lines = run.stdout.splitlines()
        self.assertEqual(run.returncode, 0, "\n".join(l for l in lines if l.startswith("FAIL")))
        msgs = [parse_line(l) for l in lines]

        step = msgs[0]
        self.assertIsInstance(step, StepMsg)
        self.assertEqual((step.seq, step.t_us, step.foot, step.interval_ms, step.fsr_value), (812, 4000000000, 1, 512, 2210))
        self.assertAlmostEqual(step.avg_bpm, 116.40)

        imu = [m for m in msgs if isinstance(m, ImuMsg)]
        self.assertEqual(len(imu), 5)
        self.assertEqual({m.seq for m in imu}, {65535})
        self.assertEqual([m.t_us for m in imu], [123460000 + i * 10000 for i in range(5)])
        self.assertEqual([m.valid_right for m in imu], [True, True, False, True, True])
        self.assertEqual(imu[0].left.tolist(), [-1570, 12, 981, -32768, 0, 32767])
        self.assertEqual(imu[4].right.tolist(), [1, 2, 3, 4, 5, -10])

        lift, status, pong = msgs[6:]
        self.assertIsInstance(lift, LiftMsg)
        self.assertEqual((lift.seq, lift.foot, lift.fsr_value), (410, 2, 750))
        self.assertIsInstance(status, StatusMsg)
        self.assertEqual((status.i2c_err_left, status.i2c_err_right, status.sys_left, status.sys_right),
                         (3, 70000, 6, 255))
        self.assertEqual((pong.kind, pong.fields), ("PONG", ["42", "99"]))


class ClockTests(unittest.TestCase):
    def test_unwrap(self):
        unwrap = TimeUnwrapper()
        a = unwrap(T_US_WRAP - 1000)
        b = unwrap(500)  # wrapped
        late = unwrap(T_US_WRAP - 200)  # a message from just before the wrap, arriving late
        self.assertAlmostEqual(b - a, 1500e-6)
        self.assertAlmostEqual(late - a, 800e-6)

    def test_mapper_recovers_offset_and_drift(self):
        rng = np.random.default_rng(0)
        mapper = ClockMapper()
        offset, drift = 1234.5, 40e-6
        for sender in np.arange(0, 60, 0.01):
            delay = 0.004 + rng.exponential(0.002) + (0.03 if rng.random() < 0.01 else 0)
            mapper.add(sender, sender + offset + drift * sender + delay)
        for sender in (30.0, 59.0):
            true_pc = sender + offset + drift * sender + 0.004  # the minimum delay stays in the offset
            self.assertLess(abs(mapper.to_pc(sender) - true_pc), 0.001)
        self.assertAlmostEqual(mapper.to_sender(mapper.to_pc(42.0)), 42.0, places=6)


class PredictorTests(unittest.TestCase):
    def walk(self, predictor, steps, step_s=0.55, to_delay=0.42, ms_delay=0.20):
        t, foot = 0.0, RIGHT
        for _ in range(steps):
            predictor.on_event(foot, "TO", t - to_delay)
            predictor.on_event(foot, "MS", t - ms_delay)
            predictor.on_strike(foot, t)
            t += step_s
            foot = LEFT if foot == RIGHT else RIGHT
        return t, foot

    def test_learns_delays_and_predicts(self):
        p = StrikePredictor()
        t_next, foot = self.walk(p, 12)
        self.assertEqual(p.predict(t_next - 0.5).estimates.keys(), {"B0"})  # before the swing: rhythm only
        p.on_event(foot, "TO", t_next - 0.42)
        p.on_event(foot, "MS", t_next - 0.20)
        pred = p.predict(t_next - 0.19)
        self.assertEqual(pred.foot, foot)
        self.assertEqual(set(pred.estimates), {"B0", "B1", "B2"})
        self.assertAlmostEqual(pred.sender_s, t_next, places=6)

    def test_standing_gives_no_prediction(self):
        p = StrikePredictor()
        t_next, _ = self.walk(p, 12)
        self.assertIsNone(p.predict(t_next + 1.0))

    def test_weight_shift_while_standing_does_not_keep_a_prediction(self):
        p = StrikePredictor()
        t_next, foot = self.walk(p, 12)
        p.on_event(foot, "TO", t_next + 2.0)  # a toe-off without a step: shifting weight
        self.assertIsNotNone(p.predict(t_next + 2.5))
        self.assertIsNone(p.predict(t_next + 2.0 + 0.42 + sp.STALE_S + 0.01))
        self.assertIsNone(p.predict(t_next + 30.0))

    def test_step_lost_on_the_link_keeps_the_prediction_until_the_next_step(self):
        p = StrikePredictor()
        t_next, foot = self.walk(p, 12, step_s=0.8)  # slow walk: the next step comes 0.8 s late
        p.on_event(foot, "TO", t_next - 0.42)
        p.on_event(foot, "MS", t_next - 0.20)
        self.assertIsNotNone(p.predict(t_next + 0.8))  # its STEP message never arrived

    def test_cadence_trend_follows_a_slowdown_without_lag(self):
        p = StrikePredictor()
        t, foot = 0.0, RIGHT
        for i in range(10):  # each step 10 ms longer, right steps 20 ms longer than left
            p.on_strike(foot, t)
            t += 0.50 + 0.01 * i + (0.02 if foot == LEFT else 0.0)
            foot = LEFT if foot == RIGHT else RIGHT
        p.on_strike(foot, t)
        # last step (i = 9): 0.59 s plus half the foot difference = 0.60 s = 100 steps/min;
        # the plain mean of the last 8 steps (0.565 s) lags behind
        self.assertAlmostEqual(p.cadence_bpm(8, ahead=0.0), 100.0, delta=0.3)
        self.assertAlmostEqual(p.cadence_bpm(8), 60.0 / 0.565, delta=0.3)

    def test_cadence_restarts_from_the_first_step_after_a_stop(self):
        p = StrikePredictor()
        t, foot = self.walk(p, 12)
        p.on_strike(foot, t + 5.0)  # standing 5 s: the old pace is dropped
        self.assertIsNone(p.cadence_bpm(8, ahead=0.0, min_steps=1))
        p.on_strike(other_foot(foot), t + 5.6)
        self.assertAlmostEqual(p.cadence_bpm(8, ahead=0.0, min_steps=1), 100.0)
        self.assertIsNone(p.cadence_bpm(8, ahead=0.0))

    def test_faster_swing_moves_the_mid_swing_estimate_earlier(self):
        p = StrikePredictor()
        t_next, foot = self.walk(p, 12)  # usual swing: TO -> MS 220 ms, MS -> strike 200 ms
        p.on_event(foot, "TO", t_next - 0.42)
        p.on_event(foot, "MS", t_next - 0.42 + 0.176)  # 20% faster swing
        b2 = p.predict(t_next - 0.2).estimates["B2"][0]
        expected = t_next - 0.42 + 0.176 + 0.20 * 0.8 ** sp.SCALE_POWER
        self.assertAlmostEqual(b2, expected, places=6)

    def test_fusion_leaves_out_an_estimate_far_from_the_best_one(self):
        p = StrikePredictor()
        t_next, foot = self.walk(p, 12)
        p.on_event(foot, "TO", t_next - 0.42 - 0.2)  # this swing started 200 ms early (speeding up)
        p.on_event(foot, "MS", t_next - 0.20 - 0.2)
        pred = p.predict(t_next - 0.39)
        self.assertEqual(set(pred.estimates), {"B0", "B1", "B2"})
        self.assertAlmostEqual(pred.sender_s, pred.estimates["B2"][0], places=6)  # the lagging B0 is left out


class PhaseControllerTests(unittest.TestCase):
    def test_late_beat_speeds_up_within_limits(self):
        ctrl = PhaseController(PhaseSettings(latency_s=0.0, delta_rate_per_s=100.0, reset_s=None))
        strike = SimpleNamespace(pc_t=10.4, sigma_s=0.01, number=5)
        # 120 BPM, at beat 0.0 now: beat 1 at base tempo is at 10.5, the strike is at 10.4 -> speed up
        delta = ctrl.update(10.0, 120.0, 0.0, strike)
        delta = ctrl.update(10.001, 120.0, 0.002, strike)
        self.assertGreater(delta, 0)
        self.assertLessEqual(delta, ctrl.s.delta_max)

    def test_large_error_resets_the_phase_on_the_coming_beat(self):
        # Beat 1 at base tempo is 150 ms after the strike: the beat is squeezed at once, beyond δ_max
        ctrl = PhaseController(PhaseSettings(latency_s=0.0))
        strike = SimpleNamespace(pc_t=10.35, sigma_s=0.01, number=5)
        delta = ctrl.update(10.0, 120.0, 0.0, strike)
        self.assertGreater(delta, ctrl.s.delta_max)
        self.assertAlmostEqual(10.0 + 60.0 / (120.0 * (1 + delta)), strike.pc_t, places=6)  # beat 1 on the strike
        # Next strike on time: back within δ_max right away, not at the rate limit
        delta = ctrl.update(10.35, 120.0, 1.05, SimpleNamespace(pc_t=10.8, sigma_s=0.01, number=6))
        self.assertLessEqual(abs(delta), ctrl.s.delta_max)

    def test_uncertain_prediction_fades_to_zero(self):
        ctrl = PhaseController(PhaseSettings(delta_rate_per_s=1.0))
        ctrl.delta = 0.05
        ctrl._last_t = 0.0
        self.assertAlmostEqual(ctrl.update(0.02, 120.0, 0.0, None), 0.03)

    def test_consecutive_strikes_pair_with_consecutive_beats(self):
        ctrl = PhaseController(PhaseSettings(latency_s=0.0))
        ctrl.update(0.0, 120.0, 0.0, SimpleNamespace(pc_t=0.5, sigma_s=0.01, number=0))
        ctrl.update(0.3, 120.0, 0.6, SimpleNamespace(pc_t=1.0, sigma_s=0.01, number=1))
        self.assertEqual(ctrl.pairs, {0: 1, 1: 2})

    def test_bias_learns_from_errors_with_antiwindup(self):
        ctrl = PhaseController(PhaseSettings(ki=0.5))
        ctrl.on_result(0.040)
        self.assertAlmostEqual(ctrl.bias_s, 0.020)
        ctrl.delta = ctrl.s.delta_max  # at the limit: no further growth
        ctrl.on_result(0.040)
        self.assertAlmostEqual(ctrl.bias_s, 0.020)

    def test_standing_holds_the_tempo_on_the_imu_path_only(self):
        from utils.Modes.dynamic_mode import DynamicMode
        for grace, expected in ((float("inf"), 110.0), (1.0, 60.0 / 5.0)):
            context = SimpleNamespace(player=SimpleNamespace(walkingBPM=110.0), logger=None, decay_grace=grace)
            mode = DynamicMode(context)
            mode.on_step(110.0, 100.0)
            target, _ = mode.handle_step(105.0, 110.0, 0.1)  # 5 s without a step
            self.assertAlmostEqual(target, expected)

    def test_tempo_changes_gradually_from_beat_to_beat(self):
        from utils.simulation import fast_loop
        r = fast_loop.run("100:8,0:3,125:14", 1, True, settings=PhaseSettings(max_change_per_beat=0.05),
                          use_model=False)
        gaps = np.diff(r.beat_t)
        tempo = 60.0 / gaps
        playing = (gaps[1:] < 1.5) & (gaps[:-1] < 1.5)  # not across the pause while standing
        self.assertLess(np.max((np.abs(np.diff(tempo)) / tempo[:-1])[playing]), 0.0501)
        back_on_step = r.asynchrony_ms[(r.strike_t > 18.0) & (r.strike_t < 24.0)]  # 7-13 s after the restart
        self.assertLess(np.median(np.abs(back_on_step)), 30)

    def test_music_starts_on_the_step_once_the_pace_is_known(self):
        from utils.simulation import fast_loop
        r = fast_loop.run("110:15", 1, True, use_model=False)
        s = PhaseSettings()
        self.assertFalse(r.with_music[:s.start_steps].any())  # waits for the pace
        first = np.abs(r.asynchrony_ms[r.with_music][:6])
        self.assertLess(np.median(first), 50)  # in step from the first beats, nothing to catch up
        old = fast_loop.run("110:15", 1, False, use_model=False)
        self.assertTrue(old.with_music.all())  # today's system: music right away

    def test_music_pauses_while_standing_and_restarts_on_the_step(self):
        from utils.simulation import fast_loop
        r = fast_loop.run("105:10,0:4,115:12", 1, True, use_model=False)
        standing = (r.beat_t > 11.5) & (r.beat_t < 14.0)
        self.assertFalse(standing.any())  # no beats while standing
        after = r.asynchrony_ms[r.with_music & (r.strike_t > 14.0)]
        self.assertGreater(len(after), 10)
        self.assertLess(np.median(np.abs(after[:6])), 50)

    def test_step_lost_on_the_link_does_not_pause_the_music(self):
        from utils.engine.phase_controller import PhaseAligner
        clock = SimpleNamespace(t=0.0)
        player = SimpleNamespace(playing=True, walkingBPM=120.0, paused=0)
        player.pause = lambda: setattr(player, "paused", player.paused + 1)
        aligner = PhaseAligner(player, SimpleNamespace(), PhaseSettings(), enabled=True)
        aligner.on_strike(0, 1, 10.0)
        import utils.engine.phase_controller as pc
        saved = pc.time
        pc.time = SimpleNamespace(perf_counter=lambda: 11.2)  # 2.4 step periods without a step
        try:
            aligner.last_strike = SimpleNamespace(pc_t=11.4, sigma_s=0.02, number=2)  # the other foot swings
            aligner.update_transport()
            self.assertEqual(player.paused, 0)
            aligner.last_strike = None  # nothing swinging: standing
            aligner.update_transport()
            self.assertEqual(player.paused, 1)
        finally:
            pc.time = saved


class HardwareReadinessTests(unittest.TestCase):
    def receiver(self, protocol):
        from utils.simulation.simulated_receiver import SimulatedReceiver
        from utils.simulation.walker import Walker, parse_scenario
        rx = SimulatedReceiver(Walker(parse_scenario("110:3"), seed=1), protocol=protocol, seed=1)
        rx.timeout = 0.05
        return rx

    def test_fsr_delay_is_measured_from_the_impact_not_the_braking(self):
        from dataclasses import replace
        from utils.gait.events import FootTracker
        from utils.simulation.legs import Person, simulate_legs, ACC_LSB_PER_MS2, GYR_LSB_PER_DPS
        from utils.simulation.walker import Walker, parse_scenario
        person = Person.random(np.random.default_rng(3))
        # Real feet brake up to ~3 g just before landing, more than the simulator's default
        person = Person(right=replace(person.right, brake_ms2=30.0), left=replace(person.left, brake_ms2=30.0))
        rec = simulate_legs(Walker(parse_scenario("110:20"), seed=3), seed=3, person=person)
        for foot in (RIGHT, LEFT):
            tracker = FootTracker(foot)
            strikes = sorted(s.fsr for s in rec.strikes if s.foot == foot)
            for i, t in enumerate(rec.t):
                while strikes and strikes[0] <= t:
                    tracker.on_strike(strikes.pop(0))
                if rec.valid[foot][i]:
                    tracker.add_sample(t, rec.acc_raw[foot][i] / ACC_LSB_PER_MS2, rec.gyr_raw[foot][i] / GYR_LSB_PER_DPS)
            self.assertAlmostEqual(tracker.fsr_delay_s, person.foot(foot).fsr_delay_s, delta=0.008)

    def test_player_starts_on_time_pauses_silently_and_resumes_on_time(self):
        import tempfile
        import time
        import mido
        from utils.hardware.midi_player import BeatClockPlayer
        sent = []

        class Port:
            def send(self, msg):
                sent.append((time.perf_counter(), msg.type if msg.type != "note_on" or msg.velocity else "note_off"))

            def reset(self):
                sent.append((time.perf_counter(), "reset"))

            def close(self):
                pass

        mid = mido.MidiFile(ticks_per_beat=480)
        track = mido.MidiTrack()
        mid.tracks.append(track)
        track.append(mido.MetaMessage("set_tempo", tempo=mido.bpm2tempo(120), time=0))
        for _ in range(64):  # a note on every beat
            track.append(mido.Message("note_on", note=60, velocity=100, time=0))
            track.append(mido.Message("note_off", note=60, velocity=0, time=480))
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "beats.mid"
            mid.save(path)
            player = BeatClockPlayer(str(path), outport=Port())
            try:
                start = time.perf_counter() + 0.1
                self.assertEqual(player.play_at(start, 240.0), 0)
                time.sleep(0.4)
                ons = [t for t, kind in sent if kind == "note_on"]
                self.assertAlmostEqual(ons[0], start, delta=0.01)  # first note exactly at the given time
                player.pause()
                time.sleep(0.15)
                paused = [t for t, kind in sent if kind == "note_on" and t > start + 0.42]
                self.assertEqual(paused, [])
                self.assertIn("reset", [kind for _, kind in sent])  # sounding notes turned off
                position = player.get_beat_now()
                resume = time.perf_counter() + 0.2
                beat = player.play_at(resume, 240.0)
                self.assertGreater(beat, position)
                time.sleep(0.3)
                self.assertAlmostEqual(player.beat_times[beat], resume, delta=0.01)
                after = [t for t, kind in sent if kind == "note_on" and t > resume - 0.15]
                self.assertAlmostEqual(after[0], resume, delta=0.01)  # no burst of the skipped part
            finally:
                player.close()

    def test_version_query_tells_old_and_new_firmware_apart(self):
        from utils.hardware.comms import query_version
        log = SimpleNamespace(log=lambda msg: None)
        self.assertEqual(query_version(self.receiver(2), log), 2)
        self.assertEqual(query_version(self.receiver(1), log, attempts=1), 1)  # no answer = old firmware

    def test_calibration_counts_only_the_senders_threshold_as_success(self):
        from utils.hardware.comms import send_calibration_command
        log = SimpleNamespace(log=lambda msg: None)
        self.assertTrue(send_calibration_command(self.receiver(2), log, retries=1, wait_s=0.5))

        class SenderNeverAnswers:  # the receiver queued the command, nothing came back
            timeout = 0.05

            def __init__(self):
                self.lines = [b"ACK,CAL_WEIGHT\r\n", b"ACK,CAL_WEIGHT,DISPATCHED\r\n"]

            def write(self, data):
                pass

            def readline(self):
                if self.lines:
                    return self.lines.pop(0)
                time.sleep(self.timeout)
                return b""

        self.assertFalse(send_calibration_command(SenderNeverAnswers(), log, retries=1, wait_s=0.3))

    def test_session_commands_apply_when_the_receiver_answers(self):
        from main import _process_line
        from utils.hardware.comms import handle_engine_command
        from utils.hardware.sensor_stream import SensorStream
        rx = self.receiver(2)
        logged = []
        log = SimpleNamespace(log=logged.append)
        engine = SimpleNamespace(set_smoothing_window=lambda n: logged.append(("window", n)),
                                 set_stride=lambda n: logged.append(("stride", n)))
        for cmd in ("SET_WINDOW:5", "SET_STRIDE:9", "SET_STRIDE:2", "CAL_WEIGHT:150"):
            handle_engine_command(cmd, rx, log, engine, None)
        self.assertEqual([x for x in logged if isinstance(x, tuple)], [])  # nothing read or applied yet
        stream = SensorStream()
        while line := rx.readline().decode().strip():
            _process_line(line, 0.0, stream, None, None, None, engine, log, None)
        self.assertEqual([x for x in logged if isinstance(x, tuple)], [("window", 5), ("stride", 2)])  # 9 > window
        self.assertIn("RECEIVER: ACK,CAL_WEIGHT,1000", logged)

    def test_command_line_overrides_phase_settings(self):
        from main import phase_settings
        args = SimpleNamespace(latency_ms=45.0, phase_kp=0.2, phase_deadband_ms=None, phase_max_nudge=5.0)
        s = phase_settings(args)
        self.assertEqual((s.latency_s, s.kp, s.delta_max), (0.045, 0.2, 0.05))
        self.assertEqual(s.deadband_s, PhaseSettings().deadband_s)
        s = phase_settings(SimpleNamespace(latency_ms=30.0, phase_reset_ms=0.0, standing="slow"))
        self.assertEqual((s.reset_s, s.hold_when_standing, s.pause_when_standing), (None, False, False))
        s = phase_settings(SimpleNamespace(latency_ms=30.0, standing="hold", start_steps=0))
        self.assertEqual((s.hold_when_standing, s.pause_when_standing, s.start_on_step), (True, False, False))
        s = phase_settings(SimpleNamespace(latency_ms=30.0, start_steps=7))
        self.assertEqual((s.start_on_step, s.start_steps, s.pause_when_standing), (True, 7, True))
        s = phase_settings(SimpleNamespace(latency_ms=30.0, max_tempo_change=8.0))
        self.assertAlmostEqual(s.max_change_per_beat, 0.08)
        s = phase_settings(SimpleNamespace(latency_ms=30.0, max_tempo_change=0.0))
        self.assertIsNone(s.max_change_per_beat)

    def test_health_warns_once_when_an_imu_goes_silent_and_when_it_is_back(self):
        from utils.engine.sensor_health import SensorHealth
        messages = []
        feet = {f: SimpleNamespace(calib=SimpleNamespace(ready=True)) for f in (RIGHT, LEFT)}
        stream = SimpleNamespace(stats=SimpleNamespace(received={}, lost={}))
        health = SensorHealth(SimpleNamespace(log=messages.append), SimpleNamespace(feet=feet), stream, protocol=2)
        imu = lambda left: ImuMsg(0, 0, left, True, np.zeros(6), np.zeros(6))
        for t in np.arange(0, 6, 0.5):
            health.on_step(t)
            health.on_message(imu(left=t < 2 or t >= 4.5), t)
            health.check(t)
        silent = [m for m in messages if "No data from the left IMU" in m]
        back = [m for m in messages if "left IMU sends data again" in m]
        self.assertEqual((len(silent), len(back)), (1, 1))

    def test_latency_click_detection(self):
        from utils.analysis.measure_latency import find_delays, RATE
        rng = np.random.default_rng(0)
        sends = np.array([1.0, 1.7, 2.5])
        true = np.array([0.062, 0.071, 0.058])
        x = rng.normal(0, 0.001, int(3.5 * RATE))
        for s, d in zip(sends, true):
            i = int((s + d) * RATE)
            x[i:i + 200] += 0.5 * np.exp(-np.arange(200) / 40) * np.sin(np.arange(200) * 0.9)
        blocks = [(k / RATE, x[k:k + 512]) for k in range(0, len(x), 512)]
        delays, _ = find_delays(blocks, sends)
        np.testing.assert_allclose(delays, true, atol=0.001)


if __name__ == "__main__":
    unittest.main()
