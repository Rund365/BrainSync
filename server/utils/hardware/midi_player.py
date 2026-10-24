import math
import time
import threading
import mido

mido.set_backend('mido.backends.rtmidi')


class BeatClockPlayer:
    """
    MIDI player driven by a beat clock instead of per-message delays.

    Every note is placed at a beat position (tick / ticks_per_beat) when the file is
    loaded. Playback advances a clock in beats at the current tempo, so the player can
    tell at any moment where it is in the song and when a given beat will be sent.
    Tempo changes inside the MIDI file are ignored for timing; the tempo is set from
    outside via set_BPM(), and the first set_tempo in the file defines songBPM.
    """

    def __init__(self, midi_path, outport=None):
        self.midi_path = midi_path
        self.mid = mido.MidiFile(midi_path)
        self.songBPM = self._extract_song_bpm()

        if self.songBPM is None:
            raise ValueError("This MIDI file has no tempo. ERROR")

        self._events, self.song_length_beats = self._build_beat_events(self.mid)
        if not self._events:
            raise ValueError("This MIDI file has no playable messages. ERROR")

        self.walkingBPM = self.songBPM
        self.TempoFactor = 1.0  # songBPM / walkingBPM, kept for existing readers
        self.outport = outport if outport is not None else mido.open_output()

        self._thread = None
        self._stop = False
        self._lock = threading.Lock()

        # Beat clock: beat position = anchor_beat + (t - anchor_time) * tempo / 60
        self._running = False
        self._anchor_beat = 0.0
        self._anchor_time = time.perf_counter()
        self.beat_times = {}  # whole beat index -> perf_counter time it was reached
        self._last_beat = 0  # last whole beat recorded in beat_times
        self._skip_before = None  # on resume: song events before this beat are skipped, not played late
        self._silence = False  # pause asked: the player thread turns the sounding notes off

    def _extract_song_bpm(self):
        for track in self.mid.tracks:
            for msg in track:
                if msg.type == "set_tempo":
                    return mido.tempo2bpm(msg.tempo)
        return None

    @staticmethod
    def _build_beat_events(mid):
        tpb = mid.ticks_per_beat
        events = []
        tick = 0
        for msg in mido.merge_tracks(mid.tracks):
            tick += msg.time
            if not msg.is_meta:
                events.append((tick / tpb, msg))
        # Whole beats, so every loop of the song starts on the beat grid
        length = max(1.0, float(math.ceil(tick / tpb - 1e-9)))
        return events, length

    # ----------- Beat clock (call with self._lock held) -----------
    def _beat_at(self, t):
        if not self._running:
            return self._anchor_beat
        return self._anchor_beat + (t - self._anchor_time) * self.walkingBPM / 60.0

    def _reanchor(self, t):
        self._anchor_beat = self._beat_at(t)
        self._anchor_time = t

    # ----------- Public API -----------
    def set_BPM(self, bpm: float):
        if bpm <= 0:
            return
        with self._lock:
            # Re-anchor first so the beats already played keep their old tempo
            self._reanchor(time.perf_counter())
            self.walkingBPM = bpm
            self.TempoFactor = self.songBPM / self.walkingBPM

    def get_beat_now(self) -> float:
        """Current position in beats since start() (keeps counting across song loops)."""
        with self._lock:
            return self._beat_at(time.perf_counter())

    def time_of_beat(self, beat: float) -> float:
        """perf_counter time at which `beat` will be reached if the tempo stays as it is now."""
        with self._lock:
            now = time.perf_counter()
            return now + (beat - self._beat_at(now)) * 60.0 / self.walkingBPM

    def next_beat(self) -> tuple[int, float]:
        """(index, perf_counter time) of the next whole beat at the current tempo."""
        with self._lock:
            now = time.perf_counter()
            current = self._beat_at(now)
            index = math.floor(current) + 1
            return index, now + (index - current) * 60.0 / self.walkingBPM

    @property
    def playing(self) -> bool:
        return self._running

    def start(self):
        if self._thread and self._thread.is_alive():
            return
        with self._lock:
            self._anchor_beat = 0.0
            self._anchor_time = time.perf_counter()
            self._running = True
            self.beat_times = {0: self._anchor_time}
            self._last_beat = 0
        self._start_thread()

    def _start_thread(self):
        self._stop = False
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def play_at(self, at_time: float, bpm: float) -> int:
        """Start (or resume after pause()) so that a whole beat is reached exactly at
        `at_time` (perf_counter), at `bpm`. The song goes on from where it was; the part
        between the pause and the lead-in is skipped. Returns the index of that beat."""
        with self._lock:
            now = time.perf_counter()
            lead = max(at_time - now, 0.0) * bpm / 60.0
            first = not self.beat_times
            paused_at = self._anchor_beat
            beat = 0 if first else math.ceil(paused_at + lead - 1e-9)
            self.walkingBPM = bpm
            self.TempoFactor = self.songBPM / bpm
            self._anchor_beat, self._anchor_time = float(beat), at_time
            self._last_beat = beat - 1
            self._skip_before = None if first else beat - lead
            self._running = True
        if not (self._thread and self._thread.is_alive()):
            self._start_thread()
        return beat

    def pause(self):
        """Stop the beat clock where it is and turn the sounding notes off; play_at() resumes."""
        with self._lock:
            if not self._running:
                return
            self._reanchor(time.perf_counter())
            self._running = False
            self._silence = True

    def stop(self):
        self._stop = True
        if self._thread:
            self._thread.join()
            self._thread = None
        with self._lock:
            self._reanchor(time.perf_counter())
            self._running = False
        # Notes that were on when playback stopped would otherwise keep sounding
        self.outport.reset()

    def close(self):
        self.stop()
        self.outport.close()

    def _run(self):
        index = 0
        loop = 0
        while not self._stop:
            event_beat, msg = self._events[index]
            target_beat = loop * self.song_length_beats + event_beat

            with self._lock:
                silence, self._silence = self._silence, False
                skip = self._skip_before is not None and target_beat < self._skip_before
                if self._skip_before is not None and not skip:
                    self._skip_before = None
                now = time.perf_counter()
                beat = self._beat_at(now)
                remaining = (target_beat - beat) * 60.0 / self.walkingBPM
                if self._running:
                    while self._last_beat + 1 <= beat:
                        self._last_beat += 1
                        # Exact crossing time, not the (up to 1 ms later) moment we noticed it
                        self.beat_times[self._last_beat] = now - (beat - self._last_beat) * 60.0 / self.walkingBPM

            if silence:
                self.outport.reset()
            if skip:
                if msg.type not in ("note_on", "note_off"):
                    self.outport.send(msg)  # notes of the skipped part are dropped; program changes still apply
                index += 1
                if index == len(self._events):
                    index = 0
                    loop += 1
                continue

            if remaining > 0 or not self._running:
                # Short sleeps so a tempo change is picked up within ~1 ms
                time.sleep(min(0.001, remaining) if self._running else 0.001)
                continue

            self.outport.send(msg)
            index += 1
            if index == len(self._events):
                index = 0
                loop += 1


class NullOutput:
    """MIDI port that plays nothing (silent runs)."""

    def send(self, msg):
        pass

    def reset(self):
        pass

    def close(self):
        pass


# Existing code imports the player under this name
MidiBeatSync = BeatClockPlayer
