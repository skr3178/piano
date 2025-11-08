# Copyright 2024
# MIDI message definitions for Isaac Sim Piano
# Ported from RoboPianist

"""MIDI message classes."""

from dataclasses import dataclass
from typing import Union


@dataclass
class MidiMessage:
    """Base class for MIDI messages."""
    time: float  # Simulation time in seconds


@dataclass
class NoteOn(MidiMessage):
    """MIDI Note On message."""
    note: int  # MIDI note number (0-127)
    velocity: int  # Velocity (0-127)


@dataclass
class NoteOff(MidiMessage):
    """MIDI Note Off message."""
    note: int  # MIDI note number (0-127)


@dataclass
class SustainOn(MidiMessage):
    """MIDI Sustain Pedal On message."""
    pass


@dataclass
class SustainOff(MidiMessage):
    """MIDI Sustain Pedal Off message."""
    pass


def key_number_to_midi_number(key_number: int) -> int:
    """Convert piano key number (0-87) to MIDI note number.
    
    Args:
        key_number: Piano key number (0-87, where 0 is A0).
        
    Returns:
        MIDI note number (21-108).
    """
    # Piano key 0 (A0) corresponds to MIDI note 21
    return key_number + 21


def midi_number_to_key_number(midi_number: int) -> int:
    """Convert MIDI note number to piano key number (0-87).
    
    Args:
        midi_number: MIDI note number (21-108).
        
    Returns:
        Piano key number (0-87).
    """
    return midi_number - 21

