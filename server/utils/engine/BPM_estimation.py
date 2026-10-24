import math
import time
import threading
from ..hardware.midi_player import MidiBeatSync
from ..session.logger import Logger
from ..prediction_model.LGBM_predictor import LGBMPredictor

# Import Modes
from ..Modes.manual_mode import ManualMode
from ..Modes.random_mode import RandomMode
from ..Modes.dynamic_mode import DynamicMode
from ..Modes.hybrid_mode import HybridMode

class BPM_estimation:
    def __init__(self, player: MidiBeatSync, logger: Logger, initial_mode: str = "dynamic",
                manual_bpm: float = None, prediction_model: LGBMPredictor = None,
                random_span: float = 0.20, random_gamified: bool = False,
                hybrid_lock_steps: int = 5, hybrid_unlock_time: float = 1.5,
                hybrid_stability_threshold: float = 5.0, hybrid_unlock_threshold: float = 10.0,
                random_simple_threshold: float = 5.0, random_simple_steps: int = 20,
                random_simple_timeout: float = 30.0,
                smoothing_window: int = 3, stride: int = 1) -> None:
        
        # ------------ Context ------------
        self.player = player
        self.logger = logger
        self.prediction_model = prediction_model
        # Must match what the receiver ESP32 uses; the model takes them as features
        self.smoothing_window = smoothing_window
        self.stride = stride
        self._logged_prediction_errors = set()
        
        # ------------ State ------------
        self.step_count = 0
        self.last_msg_time = time.time()
        self.last_update_time = time.time()
        self.last_gui_log_time = time.time()
        
        # ------------ Modes ------------
        self.manual_mode_obj = ManualMode(self)
        self.random_mode_obj = RandomMode(self, span=random_span, gamified=random_gamified,
                                        simple_threshold=random_simple_threshold,
                                        simple_steps=random_simple_steps,
                                        simple_timeout=random_simple_timeout)
        self.dynamic_mode_obj = DynamicMode(self)
        self.hybrid_mode_obj = HybridMode(self, lock_steps=hybrid_lock_steps, 
                                        unlock_time=hybrid_unlock_time,
                                        stability_threshold=hybrid_stability_threshold,
                                        unlock_threshold=hybrid_unlock_threshold)
        
        # ------------ Active State ------------
        self.active_mode = initial_mode # "dynamic", "random", "manual", "hybrid"
        
        # ------------ Configuration ------------
        if manual_bpm: 
            self.set_manual_bpm(manual_bpm)
        
        # Apply specific configs if starting in that mode
        if self.active_mode == "random":
            self.set_random_span(random_span)
            self.set_random_gamified(random_gamified)
            self.set_random_simple_threshold(random_simple_threshold)
            self.set_random_simple_steps(random_simple_steps)
            self.set_random_simple_timeout(random_simple_timeout)
            self.random_mode_obj.activate()
        
        elif self.active_mode == "hybrid":
            self.set_hybrid_lock_steps(hybrid_lock_steps)
            self.set_hybrid_unlock_time(hybrid_unlock_time)
            self.set_hybrid_stability_threshold(hybrid_stability_threshold)
            self.set_hybrid_unlock_threshold(hybrid_unlock_threshold)
            self.hybrid_mode_obj.activate()
            
        elif self.active_mode == "manual": 
            self.manual_mode_obj.activate()
            
        elif self.active_mode == "dynamic": 
            self.dynamic_mode_obj.activate()

        # ------------ Smoothing Params ------------
        self.smoothing_alpha_up = 0.1
        self.smoothing_alpha_down = 0.05
        self.target_bpm = player.walkingBPM

        # ------------ Tempo = base (from the mode) x (1 + phase nudge) ------------
        # The mode smooths base_bpm, never the tempo read back from the player, so a
        # phase nudge is not mistaken for a tempo error and undone.
        self.base_bpm = player.walkingBPM
        self.phase_aligner = None  # PhaseAligner, set by main when the IMU pipeline runs
        self.phase_delta = 0.0
        self._sent_bpm = player.walkingBPM
        # Smooth tempo (phase alignment on): (beat index, previous beat's tempo, start time, base at start)
        self._beat_ref = (None, player.walkingBPM, 0.0, player.walkingBPM)
        self._ramped_base = None
        # The modes slow down once the next step is later than one step period x this
        self.decay_grace = 1.0

        # ------------ ML Warmup ------------
        self._warmup_done = threading.Event()
        self._warmup_failed = False
        if self.prediction_model:
            threading.Thread(target=self._run_warmup, args=(player.walkingBPM,), daemon=True).start()

    # ----------- State Switches -----------
    
    # --- manual mode ---
    def set_manual_mode(self, enabled: bool):
        self.active_mode = "manual"
        self.manual_mode_obj.activate()

    def set_manual_bpm(self, bpm: float):
        self.manual_mode_obj.set_bpm(bpm)
    
    # --- random mode ---
    def set_random_mode(self, enabled: bool):
        self.active_mode = "random"
        self.random_mode_obj.activate()

    def set_random_span(self, span: float):
        self.random_mode_obj.set_span(span)

    def set_random_gamified(self, enabled: bool):
        self.random_mode_obj.set_gamified(enabled)

    def set_random_simple_threshold(self, threshold: float):
        self.random_mode_obj.set_simple_threshold(threshold)

    def set_random_simple_steps(self, steps: int):
        self.random_mode_obj.set_simple_steps(steps)

    def set_random_simple_timeout(self, seconds: float):
        self.random_mode_obj.set_simple_timeout(seconds)

    # --- hybrid mode ---
    def set_hybrid_mode(self, enabled: bool):
        self.active_mode = "hybrid"
        self.hybrid_mode_obj.activate()

    def set_hybrid_lock_steps(self, steps: int):
        self.hybrid_mode_obj.set_lock_steps(steps)

    def set_hybrid_unlock_time(self, seconds: float):
        self.hybrid_mode_obj.set_unlock_time(seconds)

    def set_hybrid_stability_threshold(self, bpm: float):
        self.hybrid_mode_obj.set_stability_threshold(bpm)

    def set_hybrid_unlock_threshold(self, bpm: float):
        self.hybrid_mode_obj.set_unlock_threshold(bpm)

    # --- dynamic mode ---
    def set_dynamic_mode(self, enabled: bool):
        self.active_mode = "dynamic"
        self.dynamic_mode_obj.activate()


    def _get_active_mode(self):
        if self.active_mode == "manual": return self.manual_mode_obj
        if self.active_mode == "random": return self.random_mode_obj
        if self.active_mode == "hybrid": return self.hybrid_mode_obj
        return self.dynamic_mode_obj

    def _run_warmup(self, initial_bpm):
        try:
            self.prediction_model.warmup(initial_bpm)
        except Exception as e:
            self._warmup_failed = True
            self.logger.log(f"Prediction model warmup failed, prediction disabled: {e!r}")
        finally:
            self._warmup_done.set()

    def _log_prediction_error(self, e: Exception):
        # Once per distinct error; register_step runs on every step
        key = repr(e)
        if key not in self._logged_prediction_errors:
            self._logged_prediction_errors.add(key)
            self.logger.log(f"Prediction failed, using measured BPM: {key}")
    
    # ----------- Alpha control -----------
    def set_smoothing_alpha_up(self, a): self.smoothing_alpha_up = float(a)
    def set_smoothing_alpha_down(self, a): self.smoothing_alpha_down = float(a)

    # ----------- Receiver BPM settings (model features) -----------
    def set_smoothing_window(self, window: int):
        self.smoothing_window = int(window)
        # Mirrors the receiver, which lowers the stride when the window shrinks below it
        if self.stride > self.smoothing_window:
            self.stride = self.smoothing_window

    def set_stride(self, stride: int):
        self.stride = int(stride)


    # ----------- Input Handling -----------
    def register_step(self, new_bpm, instant_bpm=None, use_model=True):
        """Called when a new step is detected. use_model=False skips the LGBM blend
        (for BPM that is not the receiver's average, which the model was trained on)."""
        self.last_msg_time = time.time()
        self.step_count += 1
        
        # ML Prediction Hook
        if use_model and self.prediction_model and self._warmup_done.is_set() and not self._warmup_failed:
            try:
                self.prediction_model.add_step(new_bpm, instant_bpm)
                pred = self.prediction_model.predict_next(
                    smoothing_window=self.smoothing_window,
                    stride=self.stride,
                )
                if pred: 
                    # Blending prediction
                     new_bpm = float(pred) * 0.65 + new_bpm * 0.35
            except Exception as e:
                self._log_prediction_error(e)

        # Notify active mode
        mode = self._get_active_mode()
        mode.on_step(new_bpm, self.last_msg_time)

    def register_button_delta(self, delta: float):
        """Hardware button press."""
        # Force manual mode if button pressed?
        # Current logic: If in manual mode, adjust it.
        if self.active_mode == "manual":
            new_val = self.manual_mode_obj.adjust_bpm(delta)
            self.player.set_BPM(new_val)
            self.target_bpm = new_val
            
            # Notify GUI
            import sys
            print(f"GUI_SYNC:MANUAL_BPM:{new_val}")
            sys.stdout.flush()
            return new_val
        return None


    def _beat_refs(self) -> tuple[float, float]:
        """(tempo of the previous beat as it was heard, ramped base tempo when this beat began)."""
        now = time.perf_counter()
        position = self.player.get_beat_now()
        beat = math.floor(position)
        index, ref, started, ref_base = self._beat_ref
        if beat != index:
            boundary = now - (position - beat) * 60.0 / self.player.walkingBPM
            if index is not None and beat == index + 1 and boundary > started:
                ref = 60.0 / (boundary - started)
            else:
                ref = self.player.walkingBPM
            ref_base = self._ramped_base or self.base_bpm
            self._beat_ref = (beat, ref, boundary, ref_base)
        return ref, ref_base

    def _apply_smooth(self, max_change: float) -> float:
        """Tempo with beat-to-beat changes limited to max_change. The base tempo ramps at the
        same rate and the controller is told the nudge actually applied, so it plans with the
        tempo that really plays instead of fighting the limit."""
        ref, ref_base = self._beat_refs()
        self._ramped_base = max(ref_base * (1.0 - max_change), min(ref_base * (1.0 + max_change), self.base_bpm))
        delta = self.phase_aligner.update(self._ramped_base)
        bpm = self._ramped_base * (1.0 + delta)
        bpm = max(ref * (1.0 - max_change), min(ref * (1.0 + max_change), bpm))
        self.phase_aligner.ctrl.delta = bpm / self._ramped_base - 1.0
        self.phase_delta = self.phase_aligner.ctrl.delta
        return bpm

    # ----------- Main Loop -----------
    def update_bpm(self):
        """Called ~10Hz by main loop."""
        now_ts = time.time()
        dt = now_ts - self.last_update_time
        self.last_update_time = now_ts
        
        current_bpm = self.player.walkingBPM
        if current_bpm != self._sent_bpm:
            # Set directly elsewhere (startup, manual button): take it as the new base
            self.base_bpm = current_bpm

        aligner = self.phase_aligner
        mode = self._get_active_mode()
        if aligner and aligner.enabled:
            s = aligner.settings
            standing_safe = s.hold_when_standing or s.pause_when_standing
            self.decay_grace = math.inf if standing_safe else s.decay_grace
            started = aligner.update_transport()
            if started:
                # The music (re)started on the step at the measured pace: that is the base now
                self.base_bpm = self._sent_bpm = started
                self._ramped_base = started
                self._beat_ref = (None, started, 0.0, started)
                if hasattr(mode, "last_step_bpm"):
                    mode.last_step_bpm = started
        else:
            self.decay_grace = 1.0

        # Determine Target & Next BPM from Active Mode
        target, self.base_bpm = mode.handle_step(now_ts, self.base_bpm, dt)
        max_change = aligner.ctrl.max_change() if aligner and aligner.enabled else None
        if max_change is not None:
            bpm = self._apply_smooth(max_change)
        else:
            self._ramped_base = None
            self.phase_delta = self.phase_aligner.update(self.base_bpm) if self.phase_aligner else 0.0
            bpm = self.base_bpm * (1.0 + self.phase_delta)

        # Apply
        self.target_bpm = target
        self._sent_bpm = bpm
        self.player.set_BPM(self._sent_bpm)
        
        # Logging
        if now_ts - self.last_gui_log_time > 0.1:
            try:
                self.logger.log_data(now_ts, current_bpm, self.target_bpm, step_event=False)
            except: pass
            self.last_gui_log_time = now_ts
