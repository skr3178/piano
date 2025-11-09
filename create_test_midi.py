#!/usr/bin/env python3
"""
Helper script to create a simple test MIDI file.
Creates "twinkle.mid" - Twinkle Twinkle Little Star
"""

import mido
from mido import Message, MidiFile, MidiTrack

def create_twinkle_twinkle():
    """Create Twinkle Twinkle Little Star MIDI file."""
    mid = MidiFile()
    track = MidiTrack()
    mid.tracks.append(track)
    
    # Tempo: 120 BPM
    track.append(mido.MetaMessage('set_tempo', tempo=mido.bpm2tempo(120)))
    
    # Middle C is MIDI note 60
    # Twinkle Twinkle: C C G G A A G  F F E E D D C
    # Note pattern with durations (note, duration_beats)
    melody = [
        (60, 1), (60, 1), (67, 1), (67, 1),  # C C G G
        (69, 1), (69, 1), (67, 2),            # A A G-
        (65, 1), (65, 1), (64, 1), (64, 1),  # F F E E
        (62, 1), (62, 1), (60, 2),            # D D C-
        (67, 1), (67, 1), (65, 1), (65, 1),  # G G F F
        (64, 1), (64, 1), (62, 2),            # E E D-
        (67, 1), (67, 1), (65, 1), (65, 1),  # G G F F
        (64, 1), (64, 1), (62, 2),            # E E D-
        (60, 1), (60, 1), (67, 1), (67, 1),  # C C G G
        (69, 1), (69, 1), (67, 2),            # A A G-
        (65, 1), (65, 1), (64, 1), (64, 1),  # F F E E
        (62, 1), (62, 1), (60, 2),            # D D C-
    ]
    
    # Convert to MIDI messages
    ticks_per_beat = mid.ticks_per_beat
    
    for note, duration in melody:
        # Note on
        track.append(Message('note_on', note=note, velocity=80, time=0))
        # Note off after duration
        track.append(Message('note_off', note=note, velocity=0, time=int(duration * ticks_per_beat)))
    
    return mid


def create_scale():
    """Create a simple C major scale."""
    mid = MidiFile()
    track = MidiTrack()
    mid.tracks.append(track)
    
    track.append(mido.MetaMessage('set_tempo', tempo=mido.bpm2tempo(120)))
    
    # C major scale: C D E F G A B C
    scale = [60, 62, 64, 65, 67, 69, 71, 72]
    
    ticks_per_beat = mid.ticks_per_beat
    
    for note in scale:
        track.append(Message('note_on', note=note, velocity=80, time=0))
        track.append(Message('note_off', note=note, velocity=0, time=ticks_per_beat // 2))
    
    # Down the scale
    for note in reversed(scale):
        track.append(Message('note_on', note=note, velocity=80, time=0))
        track.append(Message('note_off', note=note, velocity=0, time=ticks_per_beat // 2))
    
    return mid


def create_chord_progression():
    """Create a simple chord progression."""
    mid = MidiFile()
    track = MidiTrack()
    mid.tracks.append(track)
    
    track.append(mido.MetaMessage('set_tempo', tempo=mido.bpm2tempo(90)))
    
    # Chord progression: C major, F major, G major, C major
    chords = [
        [60, 64, 67],  # C major (C E G)
        [65, 69, 72],  # F major (F A C)
        [67, 71, 74],  # G major (G B D)
        [60, 64, 67],  # C major (C E G)
    ]
    
    ticks_per_beat = mid.ticks_per_beat
    
    for chord in chords:
        # Play chord
        for note in chord:
            track.append(Message('note_on', note=note, velocity=70, time=0))
        
        # Hold for 2 beats
        track.append(Message('note_off', note=chord[0], velocity=0, time=ticks_per_beat * 2))
        for note in chord[1:]:
            track.append(Message('note_off', note=note, velocity=0, time=0))
    
    return mid


if __name__ == "__main__":
    import sys
    from pathlib import Path
    
    output_dir = Path(__file__).parent
    
    print("Creating test MIDI files...")
    
    # Create Twinkle Twinkle
    midi = create_twinkle_twinkle()
    output_path = output_dir / "twinkle.mid"
    midi.save(output_path)
    print(f"✓ Created: {output_path}")
    
    # Create scale
    midi = create_scale()
    output_path = output_dir / "scale.mid"
    midi.save(output_path)
    print(f"✓ Created: {output_path}")
    
    # Create chord progression
    midi = create_chord_progression()
    output_path = output_dir / "chords.mid"
    midi.save(output_path)
    print(f"✓ Created: {output_path}")
    
    print("\nDone! You can now run:")
    print("  /home/skr/isaacsim/python.sh demo_midi_playback.py")

