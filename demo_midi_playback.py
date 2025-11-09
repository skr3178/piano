#!/usr/bin/env python3
"""
Demo: MIDI Playback on Isaac Sim Piano
Similar to RoboPianist's self-actuated piano, but in Isaac Sim.

This demo loads a MIDI file and plays it back on the piano by programmatically
actuating the keys at the correct times.
"""

import sys
from pathlib import Path

# Initialize Isaac Sim FIRST (before other imports)
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import numpy as np
from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.stage import add_reference_to_stage
from pxr import UsdGeom, Gf
import omni.replicator.core as rep

# Import piano controller
sys.path.insert(0, str(Path(__file__).parent))
from exts.omni.isaac.piano.piano_controller import IsaacPianoController

# MIDI parsing
import mido


def midi_note_to_key_index(midi_note: int) -> int:
    """Convert MIDI note number (21-108) to piano key index (0-87)."""
    A0_MIDI = 21  # MIDI note number for A0 (first key on 88-key piano)
    return midi_note - A0_MIDI


def parse_midi_file(midi_path: str, time_stretch: float = 1.0):
    """
    Parse MIDI file and extract note events with timestamps.
    
    Returns:
        List of (time_seconds, note_on, key_index, velocity) tuples
    """
    mid = mido.MidiFile(midi_path)
    events = []
    current_time = 0.0
    
    for msg in mid:
        current_time += msg.time * time_stretch
        
        if msg.type == 'note_on':
            if msg.velocity > 0:
                # Note on
                key_idx = midi_note_to_key_index(msg.note)
                if 0 <= key_idx < 88:
                    events.append((current_time, True, key_idx, msg.velocity))
            else:
                # Note off (velocity = 0)
                key_idx = midi_note_to_key_index(msg.note)
                if 0 <= key_idx < 88:
                    events.append((current_time, False, key_idx, 0))
        elif msg.type == 'note_off':
            # Note off
            key_idx = midi_note_to_key_index(msg.note)
            if 0 <= key_idx < 88:
                events.append((current_time, False, key_idx, 0))
    
    return events


def setup_video_recording():
    """Setup video recording using Replicator."""
    render_product = rep.create.render_product("/OmniverseKit_Persp", (1920, 1080))
    
    timestamp = __import__('datetime').datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = f"recordings/midi_playback_{timestamp}"
    
    writer = rep.WriterRegistry.get("BasicWriter")
    writer.initialize(output_dir=output_dir, rgb=True)
    writer.attach([render_product])
    
    # Replicator saves to omni.replicator_out by default
    import os
    actual_path = os.path.expanduser("~/omni.replicator_out/recordings/midi_playback_" + timestamp)
    
    print(f"✓ Recording to: {actual_path}")
    print(f"  Convert to video:")
    print(f"  ffmpeg -framerate 60 -i {actual_path}/rgb_%04d.png -c:v libx264 -pix_fmt yuv420p midi_playback.mp4")
    return output_dir


def main():
    print("\n" + "="*70)
    print("Isaac Sim Piano - MIDI Playback Demo")
    print("="*70 + "\n")
    
    # MIDI file to play
    midi_files = list(Path(__file__).parent.glob("*.mid")) + list(Path(__file__).parent.glob("*.midi"))
    
    if not midi_files:
        print("❌ No MIDI files found in the current directory!")
        print("   Please add a .mid or .midi file to the piano directory.")
        print("\n   Example: place 'twinkle.mid' or any MIDI file here.")
        simulation_app.close()
        return
    
    # Use first MIDI file found
    midi_file = midi_files[0]
    print(f"🎵 MIDI File: {midi_file.name}")
    
    # Parse MIDI
    time_stretch = 1.0  # Change this to speed up/slow down playback
    print(f"   Parsing MIDI (time stretch: {time_stretch}x)...")
    events = parse_midi_file(str(midi_file), time_stretch=time_stretch)
    
    if not events:
        print("❌ No valid note events found in MIDI file!")
        simulation_app.close()
        return
    
    total_duration = events[-1][0] if events else 0
    note_count = sum(1 for e in events if e[1])  # Count note-on events
    print(f"   Duration: {total_duration:.1f} seconds")
    print(f"   Notes: {note_count}")
    print()
    
    # Enable recording
    recording_enabled = True
    
    # Create world
    world = World(stage_units_in_meters=1.0)
    stage = world.stage
    
    # Load piano FIRST (needed for camera positioning reference)
    piano_usd_path = Path(__file__).parent / "piano_with_physics.usda"
    add_reference_to_stage(str(piano_usd_path), "/Piano")
    
    # Add moderate lighting for clear visibility
    from pxr import UsdLux
    
    # Add a dome light for ambient illumination (reduced intensity)
    dome_light = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome_light.CreateIntensityAttr(500.0)  # Reduced from 2000
    
    # Add directional light (reduced intensity)
    dist_light = UsdLux.DistantLight.Define(stage, "/World/OverheadLight")
    dist_light.CreateIntensityAttr(1000.0)  # Reduced from 5000
    dist_light_xform = UsdGeom.Xformable(dist_light.GetPrim())
    dist_light_xform.AddRotateXYZOp().Set(Gf.Vec3f(-45, 45, 0))  # Angled lighting
    
    # IMPORTANT: Position and zoom camera BEFORE creating render product for recording
    # This ensures the recording captures the correct view
    camera_path = "/OmniverseKit_Persp"
    camera = UsdGeom.Camera.Get(stage, camera_path)
    if camera:
        xform = UsdGeom.Xformable(camera.GetPrim())
        xform_ops = xform.GetOrderedXformOps()
        
        # First-person view: in front of piano, slightly above, looking down at keys
        # Position: slightly forward (Y negative), above keys (Z positive), centered (X=0)
        if len(xform_ops) >= 1:
            xform_ops[0].Set(Gf.Vec3d(0.0, -0.4, 0.35))  # In front and above keys
        else:
            translate_op = xform.AddTranslateOp()
            translate_op.Set(Gf.Vec3d(0.0, -0.4, 0.35))
        
        # Rotate to look down at keys (pitch down ~25 degrees)
        if len(xform_ops) >= 2:
            xform_ops[1].Set(Gf.Vec3f(25, 0, 0))  # Pitch down to see keys
        else:
            rotate_op = xform.AddRotateXYZOp()
            rotate_op.Set(Gf.Vec3f(25, 0, 0))
        
        # Zoom in 5x by reducing horizontal aperture (default ~20.96mm, zoom 5x = ~4.192mm)
        horizontal_aperture = camera.GetHorizontalApertureAttr()
        if horizontal_aperture:
            default_aperture = horizontal_aperture.Get() or 20.96
            camera.GetHorizontalApertureAttr().Set(default_aperture / 5.0)  # 5x zoom
            print(f"✓ Camera positioned and zoomed 5x (aperture: {default_aperture:.2f}mm -> {default_aperture/5.0:.2f}mm)")
    
    # Setup recording AFTER camera is configured (so recording uses the correct view)
    recording_timestamp = None
    if recording_enabled:
        recording_dir = setup_video_recording()
        # Extract timestamp from recording_dir
        recording_timestamp = recording_dir.split("_")[-1] if "_" in recording_dir else None
        print()
    
    # Create articulation
    piano_articulation = world.scene.add(
        Articulation(prim_path="/Piano", name="piano")
    )
    
    # Reset world to initialize physics
    world.reset()
    
    # Create piano controller with actuators
    piano_controller = IsaacPianoController(
        piano_articulation=piano_articulation,
        prim_path="/Piano",
        change_color_on_activation=True,
        add_actuators=True,
    )
    
    # Set all keys to proper black/white colors
    from omni.isaac.core.utils.prims import get_prim_at_path
    from exts.omni.isaac.piano.piano_constants import WHITE_KEY_INDICES, BLACK_KEY_INDICES, WHITE_KEY_COLOR, BLACK_KEY_COLOR
    
    print("Setting piano keys to black and white...")
    for key_id in range(88):
        is_white = key_id in WHITE_KEY_INDICES
        key_type = "white" if is_white else "black"
        mesh_path = f"/Piano/{key_type}_key_{key_id}/{key_type}_key_{key_id}_mesh"
        
        prim = get_prim_at_path(mesh_path)
        if prim and prim.IsValid():
            mesh = UsdGeom.Mesh(prim)
            if mesh:
                color = WHITE_KEY_COLOR[:3] if is_white else BLACK_KEY_COLOR[:3]
                mesh.GetDisplayColorAttr().Set([Gf.Vec3f(*color)])
    
    # MIDI callbacks
    notes_pressed = 0
    notes_released = 0
    
    def on_note_on(note: int, velocity: int):
        nonlocal notes_pressed
        notes_pressed += 1
        print(f"♪ Note ON:  {note:3d} | Key: {midi_note_to_key_index(note):2d} | Velocity: {velocity}")
    
    def on_note_off(note: int):
        nonlocal notes_released
        notes_released += 1
        print(f"♪ Note OFF: {note:3d} | Key: {midi_note_to_key_index(note):2d}")
    
    piano_controller.register_note_on_callback(on_note_on)
    piano_controller.register_note_off_callback(on_note_off)
    
    # Initialize episode
    piano_controller.initialize_episode()
    
    print()
    print("="*70)
    print("Starting MIDI Playback...")
    print("="*70)
    print()
    
    # Action array: 88 keys + 1 sustain pedal
    current_action = np.zeros(89, dtype=np.float64)
    
    # Event index
    event_idx = 0
    sim_time = 0.0
    dt = world.get_physics_dt()
    
    # Simulation loop
    step = 0
    while simulation_app.is_running():
        # Process MIDI events for current time
        while event_idx < len(events) and events[event_idx][0] <= sim_time:
            event_time, note_on, key_idx, velocity = events[event_idx]
            
            if note_on:
                # Press key (action = 1.0 for full press)
                current_action[key_idx] = 1.0
            else:
                # Release key (action = 0.0 for release)
                current_action[key_idx] = 0.0
            
            event_idx += 1
        
        # Apply action using piano controller
        piano_controller.apply_action(current_action)
        
        # Step simulation
        world.step(render=True)
        
        if world.is_playing():
            # Update piano controller
            piano_controller.update(dt)
            sim_time += dt
            step += 1
            
            # Status update every 5 seconds
            if step % (5.0 / dt) == 0:
                remaining = total_duration - sim_time
                print(f"[{sim_time:5.1f}s / {total_duration:.1f}s] Notes: {notes_pressed}/{note_count} | Remaining: {remaining:.1f}s")
        
        # Stop when MIDI is done
        if event_idx >= len(events) and sim_time > total_duration + 2.0:
            print()
            print("="*70)
            print("✓ MIDI Playback Complete!")
            print("="*70)
            print(f"  Notes played: {notes_pressed}")
            print(f"  Notes released: {notes_released}")
            if recording_enabled and recording_timestamp:
                import os
                actual_path = os.path.expanduser(f"~/omni.replicator_out/recordings/midi_playback_{recording_timestamp}")
                print(f"  Recording saved to: {actual_path}/")
                print(f"  Convert with:")
                print(f"  ffmpeg -framerate 60 -i {actual_path}/rgb_%04d.png -c:v libx264 -pix_fmt yuv420p midi_playback.mp4")
            print()
            break
    
    simulation_app.close()


if __name__ == "__main__":
    main()

