"""
Brain-Music Sync - Main Engine

This module orchestrates the real-time BPM synchronization loop:
1. Connects to ESP32 sensor via serial
2. Reads step data and calculates walking tempo
3. Adjusts MIDI playback speed to match user's pace
4. Logs session data for analysis

Usage:
    python main.py [midi_path] [--manual] [--bpm N] [--serial-port COMX]
"""
import sys
import time
import argparse
import threading
from pathlib import Path
from utils.hardware.midi_player import MidiBeatSync, NullOutput
from utils.hardware.sensor_stream import SensorStream, SerialReader, ImuMsg, StepMsg, LiftMsg, StatusMsg, OtherMsg
from utils.engine.strike_tracking import StrikeTracker
from utils.engine.phase_controller import PhaseAligner, PhaseSettings
from utils.engine.sensor_health import SensorHealth
from utils.session.logger import Logger
from utils.session.sensor_logger import SensorLogger
from utils.engine.BPM_estimation import BPM_estimation
from utils.hardware.comms import (session_handshake, handle_engine_command, handle_step, send_calibration_command,
                                  open_receiver)
from utils.prediction_model.LGBM_predictor import LGBMPredictor
from utils.simulation.walker import Walker, parse_scenario
from utils.simulation.simulated_receiver import SimulatedReceiver
from utils.session.plotter import generate_post_session_plot
from utils.paths import get_sim_logs_dir
# from utils.safety import safe_execute

#parse the arguments
def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Brain-music demo player")
    # TWEAK: Change the default MIDI file here if you want a different song by default.
    parser.add_argument(
        "midi_path",
        nargs="?",
        default=r"midi_files\Technion_March1.mid",
        help="Path to the MIDI file to play",
    )
    parser.add_argument(
        "--manual",
        action="store_true",
        help="Enable manual tempo mode (music tempo will not change based on steps)",
    )
    parser.add_argument(
        "--bpm",
        type=float,
        help="Set a fixed BPM for manual mode (optional, defaults to song BPM)",
    )
    parser.add_argument(
        "--smoothing",
        type=int,
        default=3,
        help="Set the smoothing window size (number of steps to average)",
    )
    parser.add_argument(
        "--stride",
        type=int,
        default=1,
        help="Set the BPM update stride (update BPM every N steps)",
    )
    parser.add_argument(
        "--session-name",
        type=str,
        default=None,
        help="Custom name for the session log directory",
    )
    parser.add_argument(
        "--serial-port",
        type=str,
        default="COM3",
        help="Serial port for the ESP32 (default: COM3)",
    )
    parser.add_argument(
        "--alpha-up",
        type=float,
        default=None,
        help="Optional attack smoothing alpha override",
    )
    parser.add_argument(
        "--alpha-down",
        type=float,
        default=None,
        help="Optional decay smoothing alpha override",
    )
    parser.add_argument(
        "--disable-prediction-model",
        "--disable-prediction",
        dest="disable_prediction",
        action="store_true",
        help="Disable prediction model for this run",
    )
    parser.add_argument(
        "--model-path",
        type=str,
        default=None,
        help="Path to prediction model (.joblib). If not specified, uses default base model.",
    )
    parser.add_argument(
        "--hybrid",
        action="store_true",
        help="Enable hybrid mode (starts dynamic, locks if steady)",
    )
    parser.add_argument(
        "--random",
        action="store_true",
        help="Enable Random Drift Mode",
    )
    parser.add_argument(
        "--random-span",
        type=float,
        default=0.20,
        help="Difficulty span for Random Mode (0.0-0.5)",
    )
    parser.add_argument(
        "--random-gamified",
        type=int,
        default=0,
        help="Enable gamified (time-based) random mode (1=True, 0=False)",
    )
    parser.add_argument(
        "--random-simple-threshold",
        type=float,
        default=5.0,
        help="BPM spread to match in simple mode",
    )
    parser.add_argument(
        "--random-simple-steps",
        type=int,
        default=20,
        help="Consecutive steps to match in simple mode",
    )
    parser.add_argument(
        "--random-simple-timeout",
        type=float,
        default=30.0,
        help="Timeout (seconds) to fallback in simple mode",
    )
    parser.add_argument(
        "--hybrid-lock-steps",
        type=int,
        default=5,
        help="Steps needed to lock in hybrid mode",
    )
    parser.add_argument(
        "--hybrid-unlock-time",
        type=float,
        default=1.5,
        help="Seconds of deviation to unlock in hybrid mode",
    )
    parser.add_argument(
        "--hybrid-stability-threshold",
        type=float,
        default=3.0,
        help="BPM spread to consider stable in hybrid mode",
    )
    parser.add_argument(
        "--hybrid-unlock-threshold",
        type=float,
        default=15.0,
        help="BPM deviation to trigger unlock in hybrid mode",
    )
    parser.add_argument(
        "--calibrate-weight",
        type=int,
        default=None,
        help="Run weight calibration only (provide margin), then exit",
    )
    parser.add_argument(
        "--walk-first",
        action="store_true",
        help="Wait for user to walk N steps before starting music",
    )
    parser.add_argument(
        "--walk-steps",
        type=int,
        default=10,
        help="Number of steps to collect before starting music in walk-first mode (default: 10)",
    )
    parser.add_argument(
        "--simulate",
        type=str,
        default=None,
        metavar="SCENARIO",
        help="Run without hardware: a simulated walker replaces the ESP32s. "
             "Preset (steady, speedup, stop_start, mixed) or 'cadence:seconds,...', e.g. '110:20,130:15,0:5'",
    )
    parser.add_argument(
        "--sim-seed",
        type=int,
        default=None,
        help="Random seed for --simulate, to repeat the exact same walk",
    )
    parser.add_argument(
        "--sim-sharp",
        action="store_true",
        help="Stress-test walker for --simulate: instant stops and starts, no natural drift",
    )
    parser.add_argument(
        "--sim-protocol",
        type=int,
        choices=(1, 2),
        default=2,
        help="Serial protocol of the simulated receiver: 1 = today's step lines only, "
             "2 = planned IMU firmware (STEP, IMU, LIFT, STATUS, RXSTAT). Default: 2",
    )
    parser.add_argument(
        "--phase-align",
        choices=("auto", "on", "off"),
        default="auto",
        help="Nudge the tempo so beats land on predicted foot strikes (needs IMU firmware). "
             "auto = on in dynamic and hybrid mode, off in manual and random mode",
    )
    parser.add_argument(
        "--latency-ms",
        type=float,
        default=30.0,
        help="Output latency: time from sending a MIDI note to hearing it (default: 30; "
             "measure it with server/utils/analysis/measure_latency.py)",
    )
    defaults = PhaseSettings()
    parser.add_argument(
        "--phase-kp",
        type=float,
        default=None,
        help=f"Share of a beat's timing error corrected per beat (default: {defaults.kp}). "
             "Lower = calmer tempo, beats less exactly on the steps",
    )
    parser.add_argument(
        "--phase-deadband-ms",
        type=float,
        default=None,
        help=f"Timing errors up to this are left alone (default: {defaults.deadband_s * 1000:.0f})",
    )
    parser.add_argument(
        "--phase-max-nudge",
        type=float,
        default=None,
        help=f"Largest tempo change for alignment, in percent (default: {defaults.delta_max * 100:.0f})",
    )
    parser.add_argument(
        "--phase-reset-ms",
        type=float,
        default=None,
        help=f"From this timing error on (e.g. restarting after standing), the next beat alone is "
             f"stretched or squeezed to land on the step; 0 = off (default: {defaults.reset_s * 1000:.0f})",
    )
    parser.add_argument(
        "--standing",
        choices=("pause", "hold", "slow"),
        default=None,
        help="With phase alignment on, when you stand still: pause the music and restart it on your "
             "steps (pause, default), keep playing at the last tempo (hold), "
             "or slow the music down as without alignment (slow)",
    )
    parser.add_argument(
        "--start-steps",
        type=int,
        default=None,
        help="With phase alignment on, the music starts (and restarts after standing) on the step "
             f"after this many steps, at their pace (default: {defaults.start_steps}); "
             "0 = start the music right away as without alignment",
    )
    parser.add_argument(
        "--max-tempo-change",
        type=float,
        default=None,
        help="With phase alignment on, each beat may be at most this many percent faster or slower "
             f"than the one before, so tempo changes are gradual; 0 = no limit "
             f"(default: {defaults.max_change_per_beat * 100:.0f})",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=None,
        help="Serial baud rate (default: found automatically, 921600 for the IMU firmware, 115200 before it)",
    )
    parser.add_argument(
        "--no-audio",
        action="store_true",
        help="Do not open a MIDI port (silent run, e.g. for simulations)",
    )
    return parser.parse_args()

def start_stdin_listener(command_queue):
    """
    Reads from stdin in a separate thread and puts lines into the command queue.
    Needed for SubprocessManager communication.
    """
    import threading
    import sys
    
    def _listen():
        for line in sys.stdin:
            command_queue.put(line.strip())
    
    t = threading.Thread(target=_listen, daemon=True)
    t.start()

def phase_settings(args) -> PhaseSettings:
    """Phase controller settings: defaults, with the command-line overrides."""
    overrides = {"latency_s": args.latency_ms / 1000.0}
    if getattr(args, "phase_kp", None) is not None:
        overrides["kp"] = args.phase_kp
    if getattr(args, "phase_deadband_ms", None) is not None:
        overrides["deadband_s"] = args.phase_deadband_ms / 1000.0
    if getattr(args, "phase_max_nudge", None) is not None:
        overrides["delta_max"] = args.phase_max_nudge / 100.0
    if getattr(args, "phase_reset_ms", None) is not None:
        overrides["reset_s"] = args.phase_reset_ms / 1000.0 if args.phase_reset_ms > 0 else None
    if getattr(args, "standing", None) is not None:
        overrides["pause_when_standing"] = args.standing == "pause"
        overrides["hold_when_standing"] = args.standing == "hold"
    if getattr(args, "start_steps", None) is not None:
        overrides["start_on_step"] = args.start_steps > 0
        if args.start_steps > 0:
            overrides["start_steps"] = args.start_steps
    if getattr(args, "max_tempo_change", None) is not None:
        overrides["max_change_per_beat"] = args.max_tempo_change / 100.0 if args.max_tempo_change > 0 else None
    return PhaseSettings(**overrides)


def _process_line(line_str, arrival, stream, tracker, aligner, sensor_logger, bpm_estimation, logger, player,
                  health=None):
    """Route one receiver line: steps to the BPM engine and the strike predictor, IMU to the feet."""
    # TWEAK: Check for BUTTON messages first (BTN,<delta>)
    if line_str.startswith("BTN,"):
        parts = line_str.split(",")
        try:
            delta = float(parts[1])
        except (IndexError, ValueError):
            return
        logger.log(f"HARDWARE: Button Delta Received: {delta}")
        bpm_estimation.register_button_delta(delta)
        return

    msg = stream.feed(line_str, arrival)
    if health and msg is not None:
        health.on_message(msg, arrival)
    if isinstance(msg, ImuMsg):
        tracker.on_imu(msg)
        sensor_logger.row("imu_data", [msg.t_us, f"{arrival:.6f}", msg.seq, int(msg.valid_left),
                                       int(msg.valid_right), *msg.left.tolist(), *msg.right.tolist()])
        return
    if isinstance(msg, LiftMsg):
        sensor_logger.row("gait_events", [f"{msg.sender_s:.4f}", "", msg.foot, "LIFT", ""])
        return
    is_step = isinstance(msg, StepMsg) or (isinstance(msg, OtherMsg) and msg.kind == "LEGACY_STEP")
    if not is_step:
        return

    try:
        bpm, instant_bpm, _, _ = handle_step(line_str.encode("utf-8"), player.walkingBPM)
    except ValueError:
        return

    # Log using wall-clock processing time to keep CSV strictly monotonic.
    event_epoch = time.time()
    if health:
        health.on_step(arrival)

    use_model = True
    if isinstance(msg, StepMsg):
        number, contact_pc = tracker.on_step(msg)
        aligner.on_strike(number, msg.foot, contact_pc)
        bpm, use_model = aligner.step_bpm(bpm)
        sensor_logger.row("strikes", [msg.t_us, f"{arrival:.6f}", msg.seq, msg.foot, msg.interval_ms, msg.fsr_value,
                                      msg.instant_bpm, msg.avg_bpm, number,
                                      "" if contact_pc is None else f"{contact_pc:.6f}"])

    # register the step and log the data
    bpm_estimation.register_step(bpm, instant_bpm, use_model=use_model)
    logger.log_data(event_epoch, player.walkingBPM, bpm, step_event=True, instant_bpm=instant_bpm)
    logger.log(f"Processed: BPM={bpm:.2f}")


def _notify(status_callback, message):
    """Send notification to GUI and log."""
    status_callback(message)
    print(f"NOTIFICATION:{message}")
    sys.stdout.flush()

def _collect_calibration_steps(ser, player, logger, status_callback, stop_event, num_steps):
    """Collect calibration steps and return list of BPMs."""
    logger.log(f"WALK-FIRST MODE: Collecting {num_steps} calibration steps...")
    _notify(status_callback, f"Please begin walking. Collecting {num_steps} steps...")
    
    bpms = []
    ser.timeout = 0.2
    
    while len(bpms) < num_steps:
        if stop_event and stop_event.is_set():
            logger.log("Session stopped during calibration.")
            return None
            
        if raw_line := ser.readline():
            try:
                bpm, instant_bpm, _, _ = handle_step(raw_line, player.walkingBPM)
                bpms.append(bpm)
                logger.log_data(time.time(), 0, bpm, step_event=True, instant_bpm=instant_bpm)
                logger.log(f"Calibration step {len(bpms)}/{num_steps}: {bpm:.1f} BPM")
                status_callback(f"Calibrating... {len(bpms)}/{num_steps} steps")
            except ValueError:
                continue
    
    return bpms

def handle_startup_mode(args, ser, player, logger, bpm_estimation, status_callback, stop_event,
                        start_on_step=False):
    """Handle startup: walk-first (calibrate then play), music-first (play immediately), or
    start on the step (phase alignment starts the music once it knows the pace)."""
    if start_on_step and not getattr(args, 'walk_first', False):
        logger.log("Music starts on your steps, once the pace is known")
        logger.log_data(time.time(), player.walkingBPM, player.walkingBPM, step_event=False)
        _notify(status_callback, "System ready - Please begin walking! The music starts on your steps.")
        ser.timeout = 0
        return True

    if getattr(args, 'walk_first', False):
        # Calibrate from walking steps
        bpms = _collect_calibration_steps(ser, player, logger, status_callback, stop_event, 
                                          getattr(args, 'walk_steps', 10))
        if not bpms:
            return False
        
        # Start music at calibrated BPM
        avg_bpm = sum(bpms) / len(bpms)
        player.set_BPM(avg_bpm)
        player.start()
        logger.log(f"Calibration complete. Starting music at {avg_bpm:.1f} BPM")
        
        # Pre-fill prediction model and seed graph data
        for bpm in bpms:
            bpm_estimation.register_step(bpm, bpm)
        logger.log_data(time.time(), player.walkingBPM, avg_bpm, step_event=False)
        
        _notify(status_callback, "Music started! Continue walking.")
    else:
        # Start music immediately at default BPM
        player.start()
        logger.log("Running main loop...")
        
        # Seed initial GUI data so graph starts immediately
        logger.log_data(time.time(), player.walkingBPM, player.walkingBPM, step_event=False)
        
        _notify(status_callback, "System ready - Please begin walking!")
    
    ser.timeout = 0  # Non-blocking for main loop
    return True

def main(args, status_callback=print, stop_event=None, session_dir_callback=None, command_queue=None):
    midi_path = args.midi_path
    smoothing_window = getattr(args, 'smoothing', 3)
    stride = getattr(args, 'stride', 1)
    
    if args.manual:
        run_type = "manual"
    elif getattr(args, "hybrid", False):
        run_type = "hybrid"
    elif getattr(args, "random", False):
        run_type = "random"
    else:
        run_type = "dynamic"

    # Calibration-only flow: open serial, send CAL_WEIGHT, and exit.
    if getattr(args, "calibrate_weight", None) is not None:
        if getattr(args, "simulate", None):
            status_callback("Calibration completed (simulated).")
            return None, None, None
        margin = int(args.calibrate_weight)
        class _Logger:
            def log(self_inner, msg): status_callback(msg)
        try:
            with open_receiver(args.serial_port, _Logger(), getattr(args, "baud", None), timeout=1.0) as ser:
                ok = send_calibration_command(ser, _Logger(), margin=margin, retries=3)
                status_callback("Calibration completed." if ok else "Calibration failed.")
        except Exception as e:
            status_callback(f"Calibration error: {e}")
        return None, None, None

    # 1. Initialize Logger 
    logger = Logger(
        gui_callback=status_callback,
        session_name=getattr(args, 'session_name', None),
        smoothing_window=smoothing_window,
        stride=stride,
        run_type=run_type,
        logs_dir=get_sim_logs_dir() if getattr(args, "simulate", None) else None,
    )
    
    # OUTPUT SESSION DIR FOR GUI PARSING
    # The GUI listens for "SESSION_DIR:..." to know where to save the plot.
    print(f"SESSION_DIR:{logger.path}")
    sys.stdout.flush() # Ensure it sends immediately
    


    if session_dir_callback:
        session_dir_callback(logger.path)
        
    # 2. Initialize the midi player  
    logger.log("DEBUG: Attempting to initialize MidiBeatSync...")
    
    try:
        player = MidiBeatSync(midi_path, outport=NullOutput() if getattr(args, "no_audio", False) else None)
        
        logger.log("DEBUG: MidiBeatSync initialized successfully.")
    except Exception as e:
        logger.log(f"CRITICAL ERROR: Failed to initialize MIDI Player: {e}")
        print(f"CRITICAL ERROR: {e}")
        sys.stdout.flush()
        return None, logger, None
    
    # 3. Log Mode
    if args.manual:
        logger.log("Starting in MANUAL MODE.")
        if args.bpm:
            logger.log(f"Setting fixed BPM to {args.bpm}")
            player.set_BPM(args.bpm)
        else:
            logger.log(f"Using default song BPM: {player.songBPM}")
    elif getattr(args, "random", False):
        logger.log("Starting in RANDOM DRIFT MODE.")
    else:
        logger.log("Starting in DYNAMIC MODE")
    logger.log(f"Run type: {run_type}")
    
    # 4. Initialize prediction model (currently LightGBM predictor)
    if args.disable_prediction:
        prediction_model = None
        logger.log("Prediction model disabled")
    else:
        model_path = args.model_path if hasattr(args, 'model_path') else None
        prediction_model = LGBMPredictor(model_path=model_path, run_type=run_type)
        model_name = Path(model_path).name if model_path else "default base model"
        logger.log(f"Prediction model enabled: {model_name}")
    
    # 5. Initialize BPM Estimation
    bpm_estimation = BPM_estimation(
        player,
        logger,
        initial_mode=run_type,
        manual_bpm=args.bpm,
        prediction_model=prediction_model,
        random_span=args.random_span,
        random_gamified=(args.random_gamified == 1),
        hybrid_lock_steps=args.hybrid_lock_steps,
        hybrid_unlock_time=args.hybrid_unlock_time,
        hybrid_stability_threshold=args.hybrid_stability_threshold,
        hybrid_unlock_threshold=args.hybrid_unlock_threshold,
        random_simple_threshold=args.random_simple_threshold,
        random_simple_steps=args.random_simple_steps,
        random_simple_timeout=args.random_simple_timeout,
        smoothing_window=smoothing_window,
        stride=stride,
    )
    if args.alpha_up is not None:
        bpm_estimation.set_smoothing_alpha_up(args.alpha_up)
    if args.alpha_down is not None:
        bpm_estimation.set_smoothing_alpha_down(args.alpha_down)
    logger.log("Session started")
    
    # 6. Open Serial Port (or the simulated receiver)
    ser = None
    if getattr(args, "simulate", None):
        try:
            walker = Walker(parse_scenario(args.simulate), realistic=not args.sim_sharp, seed=args.sim_seed)
        except ValueError as e:
            logger.log(f"Invalid simulation scenario: {e}")
            return player, logger, bpm_estimation
        ser = SimulatedReceiver(walker, protocol=args.sim_protocol, seed=args.sim_seed)
        ser.timeout = 0.2
        logger.log(f"SIMULATION: scenario '{args.simulate}', {walker.duration:.0f} s, "
                   f"{len(walker.steps)} steps (seed {args.sim_seed}, protocol {args.sim_protocol})")
    else:
        port = args.serial_port
        logger.log(f"DEBUG: Attempting to open serial port {port}...")
        try:
            ser = open_receiver(port, logger, getattr(args, "baud", None))
        except Exception as e:
            logger.log(f"Failed to open serial port {port}: {e}")
            return player, logger, bpm_estimation

    protocol = session_handshake(ser, logger, smoothing_window=smoothing_window, stride=stride)
    if not protocol:
        logger.log("Handshake failed; aborting session")
        return player, logger, bpm_estimation

    # runtime serial reads should be non-blocking to avoid slowing playback
    ser.timeout = 0
    
    phase_align = args.phase_align
    if phase_align == "auto":
        phase_align = "on" if run_type in ("dynamic", "hybrid") else "off"
    if phase_align == "on" and protocol < 2:
        logger.log("Phase alignment needs the IMU firmware (protocol 2); following the tempo only")
        phase_align = "off"
    settings = phase_settings(args)

    # ------------------------------------------------------------------
    # STARTUP MODE: Walk-First vs Music-First vs start on the step
    # ------------------------------------------------------------------
    startup_success = handle_startup_mode(args, ser, player, logger, bpm_estimation, status_callback, stop_event,
                                          start_on_step=phase_align == "on" and settings.start_on_step)
    if not startup_success:
        if ser: ser.close()
        return player, logger, bpm_estimation
    
    # ------------------------------------------------------------------
    # STDIN LISTENER (For Subprocess Mode)
    # If no external queue is provided, we create one and listen to STDIN.
    # ------------------------------------------------------------------
    if command_queue is None:
        from queue import SimpleQueue
        
        command_queue = SimpleQueue()
        
        start_stdin_listener(command_queue)

    # ------------------------------------------------------------------
    # SENSOR PIPELINE: reader thread -> IMU strike prediction -> phase alignment
    # ------------------------------------------------------------------
    stream = SensorStream()
    sensor_logger = SensorLogger(logger.path)
    tracker = StrikeTracker(stream, sensor_logger)
    aligner = PhaseAligner(player, tracker, settings, enabled=phase_align == "on", sensor_logger=sensor_logger)
    if getattr(args, "walk_first", False):
        aligner.settings.start_on_step = False  # walk-first already started the music
    bpm_estimation.phase_aligner = aligner
    health = SensorHealth(logger, tracker, stream, protocol)
    s = aligner.settings
    logger.log(f"Phase alignment {phase_align} (output latency {s.latency_s * 1000:.0f} ms, kp {s.kp}, "
               f"dead-band {s.deadband_s * 1000:.0f} ms, max nudge {s.delta_max * 100:.0f}%, "
               f"phase reset {f'{s.reset_s * 1000:.0f} ms' if s.reset_s else 'off'}, "
               f"start: {f'on the step after {s.start_steps} steps' if s.start_on_step else 'right away'}, "
               f"standing: {'pause' if s.pause_when_standing else 'hold' if s.hold_when_standing else 'slow'}, "
               f"max tempo change {f'{s.max_change_per_beat * 100:.0f}% per beat' if s.max_change_per_beat else 'off'})")
    reader = SerialReader(ser)
    reader.start()

    # ==================================================================================
    # MAIN LOOP
    # ==================================================================================
    while True:
        # CHECK STOP EVENT
        if stop_event and stop_event.is_set():
            logger.log("Session stopped by user.")
            break

        if getattr(ser, "finished", False):
            logger.log("SIMULATION: scenario finished.")
            break

        if reader.error:
            logger.log(f"Serial read failed, ending session: {reader.error}")
            break
            
        # PROCESS COMMAND QUEUE (From GUI Pipe or Thread)
        while not command_queue.empty():
            cmd = command_queue.get_nowait()
            
            quit =handle_engine_command(cmd, ser, logger, bpm_estimation, player)
            if quit:
                stop_event = threading.Event()
                stop_event.set()
                break
        
        # ANIMATION: Run update_bpm() every loop iteration (manual or dynamic).
        # In manual mode this still emits log packets each loop for plotting.
        # Everything the receiver sent since the last pass (read by the reader thread)
        for arrival, line_str in reader.drain():
            _process_line(line_str, arrival, stream, tracker, aligner, sensor_logger, bpm_estimation, logger, player,
                          health)

        # ANIMATION: Run update_bpm() every loop iteration (manual or dynamic).
        # In manual mode this still emits log packets each loop for plotting.
        bpm_estimation.update_bpm()
        sensor_logger.maybe_flush()
        health.check(time.perf_counter())

        # CPU Yield (Prevent 100% core usage)
        time.sleep(0.001)
        
    logger.log("Session ended")
    reader.stop()
    player.stop()
    player.close()
    logger.log(f"Link: {stream.stats.summary()}")
    sensor_logger.close(meta={
        "strike_tracking": tracker.state(),
        "phase_alignment": aligner.state(),
        "link": stream.stats.summary(),
        "receiver_protocol": protocol,
        "sensor_health": health.state(),
        "run_type": run_type,
        "simulated": bool(getattr(args, "simulate", None)),
    })
    if hasattr(ser, "write_truth"):
        ser.write_truth(logger.path)
        # The GUI makes this plot for real sessions; simulated runs are started from the CLI
        generate_post_session_plot(logger.path)
    if ser: ser.close()
    
    print("EXIT_CLEAN")
    return player, logger, bpm_estimation

if __name__ == "__main__":
    main(parse_args())