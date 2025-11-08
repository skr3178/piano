# Copyright 2024
# MIDI module for Isaac Sim Piano
# Ported from RoboPianist

"""Piano sound module for Isaac Sim."""

from typing import Callable, List, Optional

import numpy as np

from . import piano_constants
from . import midi_message


class MidiModule:
    """The piano sound module.

    It is responsible for tracking the state of the piano keys and generating
    corresponding MIDI messages. The MIDI messages can be used with a synthesizer
    to produce sound.
    """

    def __init__(self) -> None:
        self._note_on_callback: Optional[Callable[[int, int], None]] = None
        self._note_off_callback: Optional[Callable[[int], None]] = None
        self._sustain_on_callback: Optional[Callable[[], None]] = None
        self._sustain_off_callback: Optional[Callable[[], None]] = None
        self._initialized = False

    def initialize_episode(self, current_time: float = 0.0) -> None:
        """Initialize or reset the MIDI module for a new episode.
        
        Args:
            current_time: Current simulation time in seconds.
        """
        self._prev_activation = np.zeros(piano_constants.NUM_KEYS, dtype=bool)
        self._prev_sustain_activation = np.zeros(1, dtype=bool)
        self._midi_messages: List[List[midi_message.MidiMessage]] = []
        self._current_time = current_time
        self._initialized = True

    def after_substep(
        self,
        current_time: float,
        activation: np.ndarray,
        sustain_activation: np.ndarray,
    ) -> None:
        """Process key state changes and generate MIDI messages.
        
        Args:
            current_time: Current simulation time in seconds.
            activation: Boolean array of key activation states (88 keys).
            sustain_activation: Boolean array of sustain pedal state (1 element).
        """
        if not self._initialized:
            self.initialize_episode(current_time)
        
        # Sanity check dtype since we use bitwise operators.
        assert activation.dtype == bool, f"Expected bool dtype, got {activation.dtype}"
        assert sustain_activation.dtype == bool, f"Expected bool dtype, got {sustain_activation.dtype}"

        timestep_events: List[midi_message.MidiMessage] = []
        message: midi_message.MidiMessage

        state_change = activation ^ self._prev_activation
        sustain_change = sustain_activation ^ self._prev_sustain_activation

        # Note on events.
        for key_id in np.flatnonzero(state_change & ~self._prev_activation):
            message = midi_message.NoteOn(
                note=midi_message.key_number_to_midi_number(key_id),
                # TODO: In the future, we will replace this with the actual
                # key velocity. For now, we hardcode it to the maximum velocity.
                velocity=127,
                time=current_time,
            )
            timestep_events.append(message)
            if self._note_on_callback is not None:
                self._note_on_callback(message.note, message.velocity)

        # Note off events.
        for key_id in np.flatnonzero(state_change & ~activation):
            message = midi_message.NoteOff(
                note=midi_message.key_number_to_midi_number(key_id),
                time=current_time,
            )
            timestep_events.append(message)
            if self._note_off_callback is not None:
                self._note_off_callback(message.note)

        # Sustain pedal events.
        if sustain_change & ~self._prev_sustain_activation:
            timestep_events.append(midi_message.SustainOn(time=current_time))
            if self._sustain_on_callback is not None:
                self._sustain_on_callback()
        if sustain_change & ~sustain_activation:
            timestep_events.append(midi_message.SustainOff(time=current_time))
            if self._sustain_off_callback is not None:
                self._sustain_off_callback()

        self._midi_messages.append(timestep_events)
        self._prev_activation = activation.copy()
        self._prev_sustain_activation = sustain_activation.copy()
        self._current_time = current_time

    def get_latest_midi_messages(self) -> List[midi_message.MidiMessage]:
        """Returns the MIDI messages generated in the last substep."""
        if not self._midi_messages:
            return []
        return self._midi_messages[-1]

    def get_all_midi_messages(self) -> List[midi_message.MidiMessage]:
        """Returns a list of all MIDI messages generated during the episode."""
        return [message for timestep in self._midi_messages for message in timestep]

    # Callbacks for synthesizer events.

    def register_synth_note_on_callback(
        self,
        callback: Callable[[int, int], None],
    ) -> None:
        """Registers a callback for note on events.
        
        Args:
            callback: Function that takes (note, velocity) as arguments.
        """
        self._note_on_callback = callback

    def register_synth_note_off_callback(
        self,
        callback: Callable[[int], None],
    ) -> None:
        """Registers a callback for note off events.
        
        Args:
            callback: Function that takes (note) as argument.
        """
        self._note_off_callback = callback

    def register_synth_sustain_on_callback(
        self,
        callback: Callable[[], None],
    ) -> None:
        """Registers a callback for sustain pedal on events.
        
        Args:
            callback: Function with no arguments.
        """
        self._sustain_on_callback = callback

    def register_synth_sustain_off_callback(
        self,
        callback: Callable[[], None],
    ) -> None:
        """Registers a callback for sustain pedal off events.
        
        Args:
            callback: Function with no arguments.
        """
        self._sustain_off_callback = callback

    def reset(self) -> None:
        """Reset the MIDI module state."""
        self.initialize_episode()

