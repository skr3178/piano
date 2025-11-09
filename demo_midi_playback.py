#!/usr/bin/env python3
"""
Demo: MIDI Playback on Isaac Sim Piano
Similar to RoboPianist's self-actuated piano, but in Isaac Sim.

This demo loads a MIDI file and plays it back on the piano by programmatically
actuating the keys at the correct times.
"""

import sys
import subprocess
import os
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


def clear_selection():
    """Clear object selection to prevent visual artifacts from selection highlighting."""
    try:
        import omni.kit.selection
        selection = omni.kit.selection.get_selection_interface()
        if selection:
            selection.clear_selected()
            return True
    except Exception:
        try:
            import omni.usd
            context = omni.usd.get_context()
            if context:
                context.get_selection().clear_selected_prim_paths()
                return True
        except Exception:
            pass
    return False


def setup_video_recording():
    """Setup video recording using Replicator."""
    # Use 30fps instead of 60fps for better visibility of key presses
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
    print(f"  Convert to video (30fps for better visibility):")
    print(f"  ffmpeg -framerate 30 -i {actual_path}/rgb_%04d.png -c:v libx264 -pix_fmt yuv420p midi_playback.mp4")
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
    
    # Add bright lighting for clear visibility and better video quality
    from pxr import UsdLux
    
    # Add a dome light for ambient illumination (increased for better visibility)
    dome_light = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome_light.CreateIntensityAttr(1500.0)  # Increased for brighter keys
    dome_light.CreateColorAttr(Gf.Vec3f(1.0, 1.0, 1.0))  # Pure white
    
    # Add directional light (increased for better contrast)
    dist_light = UsdLux.DistantLight.Define(stage, "/World/OverheadLight")
    dist_light.CreateIntensityAttr(3000.0)  # Increased for better visibility
    dist_light.CreateColorAttr(Gf.Vec3f(1.0, 0.98, 0.95))  # Warm white
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
    
    # IMPORTANT: Initialize keys to rest position
    # The joint limits are 0° (lower/rest) to max_angle (upper/pressed)
    # Spring reference is -1° but joint can't go below 0°, so rest is effectively 0°
    print("Initializing keys to rest position (lower limit)...")
    
    # Get joint names and set initial positions to lower limit (rest position)
    joint_names = piano_articulation.dof_names
    if joint_names:
        initial_positions = np.zeros(len(joint_names))
        # Set all key joints to lower limit (0° = rest position)
        for idx, joint_name in enumerate(joint_names):
            if "joint" in joint_name:
                initial_positions[idx] = 0.0  # Lower limit = rest position
        
        # Set initial joint positions
        piano_articulation.set_joint_positions(initial_positions)
        print(f"✓ Keys initialized to rest position (0° = lower limit)")
    else:
        print("⚠️  Warning: Could not get joint names for initialization")
    
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
    
    # Clear selection to prevent visual artifacts from selection highlighting
    if clear_selection():
        print("✓ Cleared object selection for better visuals")
    
    # MIDI callbacks
    notes_pressed = 0
    notes_released = 0
    
    def on_note_on(note: int, velocity: int):
        nonlocal notes_pressed
        notes_pressed += 1
        key_idx = midi_note_to_key_index(note)
        # Note: This callback may not fire in actuator mode, so we also print in the main loop
        # print(f"♪ Note ON:  {note:3d} | Key: {key_idx:2d} | Velocity: {velocity}")
    
    def on_note_off(note: int):
        nonlocal notes_released
        notes_released += 1
        key_idx = midi_note_to_key_index(note)
        # Note: This callback may not fire in actuator mode, so we also print in the main loop
        # print(f"♪ Note OFF: {note:3d} | Key: {key_idx:2d}")
    
    piano_controller.register_note_on_callback(on_note_on)
    piano_controller.register_note_off_callback(on_note_off)
    
    # Initialize episode (this sets up joints and visuals)
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
    
    # Key position monitoring
    enable_position_monitoring = True
    monitored_keys = [0, 20, 40, 60, 80]  # Sample keys to monitor
    key_position_history = {k: [] for k in monitored_keys}
    last_position_log_time = 0.0
    position_log_interval = 1.0  # Log every 1 second
    
    # Track which keys were pressed during playback
    keys_pressed_during_playback = set()  # Set of key IDs that were pressed
    key_press_times = {}  # Dict: key_id -> list of (press_time, release_time) tuples
    
    # Get expected rest positions
    # Note: Joint limits are 0° (lower/rest) to max_angle (upper/pressed)
    # Spring reference is 0° (matches rest position) - spring pulls keys back to 0°
    import math
    expected_rest_positions = {}
    
    # Get actual lower limits from controller (after initialize_episode was called)
    for key_id in range(88):
        # Default to 0.0 (lower limit)
        expected_rest_positions[key_id] = 0.0
    
    # Try to get actual lower limits from controller
    try:
        if hasattr(piano_controller, '_qpos_range') and piano_controller._qpos_range is not None:
            for key_id in range(88):
                if key_id in piano_controller._joint_indices:
                    joint_idx = piano_controller._joint_indices[key_id]
                    if joint_idx < len(piano_controller._qpos_range):
                        expected_rest_positions[key_id] = piano_controller._qpos_range[joint_idx, 0]
    except Exception:
        pass  # Use default 0.0
    
    print("🔍 Key Position Monitoring Enabled")
    print(f"   Monitoring keys: {monitored_keys}")
    print(f"   Expected rest position: {math.degrees(expected_rest_positions[0]):.2f}° (lower limit)")
    print()
    
    # Clear selection one more time before starting simulation loop
    clear_selection()
    
    # Simulation loop
    step = 0
    while simulation_app.is_running():
        # Process MIDI events for current time
        while event_idx < len(events) and events[event_idx][0] <= sim_time:
            event_time, note_on, key_idx, velocity = events[event_idx]
            
            if note_on:
                # Press key (action = 1.0 for full press)
                # Scale effort to overcome spring (spring stiffness is 10 Nm/rad)
                # Use a reasonable effort value that can press the key down
                # Typical key press requires ~0.5-2 Nm to overcome spring
                effort_scale = 2.0  # Nm per unit action
                current_action[key_idx] = 1.0 * effort_scale
                
                # Real-time output: Show key press
                midi_note = key_idx + 21  # Convert key index to MIDI note (A0 = 21)
                print(f"♪ [{sim_time:5.2f}s] Key {key_idx:2d} PRESSED  (MIDI note {midi_note:3d}, velocity {velocity:3d})")
                
                # Track that this key was pressed
                keys_pressed_during_playback.add(key_idx)
                if key_idx not in key_press_times:
                    key_press_times[key_idx] = []
                key_press_times[key_idx].append((sim_time, None))  # (press_time, release_time)
            else:
                # Release key (action = 0.0 for release)
                current_action[key_idx] = 0.0
                
                # Real-time output: Show key release
                midi_note = key_idx + 21  # Convert key index to MIDI note (A0 = 21)
                print(f"♪ [{sim_time:5.2f}s] Key {key_idx:2d} RELEASED (MIDI note {midi_note:3d})")
                
                # Update release time for this key
                if key_idx in key_press_times and len(key_press_times[key_idx]) > 0:
                    last_press = key_press_times[key_idx][-1]
                    if last_press[1] is None:  # Not yet released
                        key_press_times[key_idx][-1] = (last_press[0], sim_time)
            
            event_idx += 1
        
        # Apply action using piano controller
        piano_controller.apply_action(current_action)
        
        # Debug: Check if efforts are being applied (before physics step)
        if step % int(1.0 / dt) == 0 and step > 0:  # Every second
            applied_before = piano_controller.articulation.get_applied_joint_efforts()
            active_actions = np.where(current_action[:88] > 0.5)[0]
            if len(active_actions) > 0:
                key_id = active_actions[0]
                if key_id in piano_controller._joint_indices:
                    joint_idx = piano_controller._joint_indices[key_id]
                    if joint_idx < len(applied_before):
                        effort_val = applied_before[joint_idx]
                        print(f"  [DEBUG] Key {key_id} action={current_action[key_id]:.2f}, effort={effort_val:.3f}Nm")
        
        # Step simulation
        world.step(render=True)
        
        if world.is_playing():
            # Update piano controller
            piano_controller.update(dt)
            sim_time += dt
            step += 1
            
            # Monitor key positions
            if enable_position_monitoring and step % int(position_log_interval / dt) == 0:
                joint_positions = piano_controller.get_joint_positions()
                normalized_states = piano_controller.normalized_state
                activations = piano_controller.activation
                
                # Log positions for monitored keys
                active_keys = np.where(activations)[0]
                pressed_keys = np.where(current_action[:88] > 0.5)[0]
                
                # Get applied joint efforts (for monitoring)
                try:
                    applied_efforts = piano_controller.articulation.get_applied_joint_efforts()
                    # Map joint efforts to key IDs
                    effort_dict = {}
                    # Access joint_indices through the controller's internal mapping
                    # We'll get efforts for monitored keys only
                    joint_names = piano_controller.articulation.dof_names
                    for key_id in monitored_keys:
                        # Find joint name for this key
                        key_type = "white" if key_id not in BLACK_KEY_INDICES else "black"
                        joint_name = f"{key_type}_joint_{key_id}"
                        if joint_name in joint_names:
                            joint_idx = joint_names.index(joint_name)
                            if joint_idx < len(applied_efforts):
                                effort_dict[key_id] = applied_efforts[joint_idx]
                except Exception:
                    effort_dict = {}
                
                print(f"\n[Key Position Report @ {sim_time:.1f}s]")
                print(f"  Keys currently pressed (action > 0.5): {len(pressed_keys)} keys")
                print(f"  Keys detected as active: {len(active_keys)} keys")
                if len(pressed_keys) > 0:
                    print(f"  Pressed key IDs: {pressed_keys[:10].tolist()}")
                
                for key_id in monitored_keys:
                    pos_rad = joint_positions[key_id]
                    pos_deg = math.degrees(pos_rad)
                    normalized = normalized_states[key_id]
                    expected_rest = expected_rest_positions[key_id]
                    rest_deg = math.degrees(expected_rest)
                    is_active = activations[key_id]
                    action_val = current_action[key_id]
                    
                    # Check if key is at rest
                    distance_from_rest = abs(pos_rad - expected_rest)
                    is_at_rest = distance_from_rest < 0.01  # Within 0.01 rad (~0.57°)
                    
                    status = "ACTIVE" if is_active else ("AT REST" if is_at_rest else "MOVING")
                    action_status = f"action={action_val:.2f}" if action_val > 0.01 else "no action"
                    effort_val = effort_dict.get(key_id, 0.0)
                    effort_status = f"effort={effort_val:.3f}Nm" if abs(effort_val) > 0.001 else "no effort"
                    
                    print(f"    Key {key_id:2d}: {pos_deg:7.2f}° (rest: {rest_deg:6.2f}°) | "
                          f"norm={normalized:.3f} | {status:8s} | {action_status} | {effort_status}")
                    
                    # Store in history
                    key_position_history[key_id].append((sim_time, pos_rad, is_active, action_val))
            
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
            
            # Final key position check
            if enable_position_monitoring:
                print()
                print("🔍 Final Key Position Check:")
                joint_positions = piano_controller.get_joint_positions()
                activations = piano_controller.activation
                
                # Show which keys were pressed during playback
                print(f"\n📊 Keys Pressed During Playback: {len(keys_pressed_during_playback)} unique keys")
                if keys_pressed_during_playback:
                    sorted_pressed = sorted(keys_pressed_during_playback)
                    print(f"  Key IDs: {sorted_pressed}")
                    print(f"  Total key presses: {sum(len(times) for times in key_press_times.values())}")
                    
                    # Show some statistics
                    if key_press_times:
                        print(f"\n  Sample key press durations:")
                        sample_keys = sorted(keys_pressed_during_playback)[:5]  # First 5 pressed keys
                        for key_id in sample_keys:
                            if key_id in key_press_times:
                                presses = key_press_times[key_id]
                                durations = []
                                for press_time, release_time in presses:
                                    if release_time is not None:
                                        durations.append(release_time - press_time)
                                if durations:
                                    avg_duration = sum(durations) / len(durations)
                                    print(f"    Key {key_id:2d}: {len(presses)} press(es), avg duration: {avg_duration:.2f}s")
                else:
                    print("  ⚠️  No keys were pressed during playback!")
                
                # Check which keys are at rest vs not at rest
                keys_not_at_rest = []
                keys_at_rest = []
                for key_id in range(88):
                    pos_rad = joint_positions[key_id]
                    expected_rest = expected_rest_positions[key_id]
                    distance_from_rest = abs(pos_rad - expected_rest)
                    if distance_from_rest > 0.01:  # More than ~0.57° from rest
                        keys_not_at_rest.append((key_id, math.degrees(pos_rad), math.degrees(expected_rest)))
                    else:
                        keys_at_rest.append(key_id)
                
                print(f"\n🏠 Keys at Rest Position: {len(keys_at_rest)}/{88} keys")
                if keys_not_at_rest:
                    print(f"  ⚠️  {len(keys_not_at_rest)} keys NOT at rest position:")
                    # Show pressed keys that didn't return first
                    pressed_not_at_rest = [(k, p, r) for k, p, r in keys_not_at_rest if k in keys_pressed_during_playback]
                    unpressed_not_at_rest = [(k, p, r) for k, p, r in keys_not_at_rest if k not in keys_pressed_during_playback]
                    
                    if pressed_not_at_rest:
                        print(f"    Pressed keys that didn't return ({len(pressed_not_at_rest)}):")
                        for key_id, pos_deg, rest_deg in pressed_not_at_rest[:10]:
                            print(f"      Key {key_id:2d}: {pos_deg:7.2f}° (expected: {rest_deg:6.2f}°)")
                        if len(pressed_not_at_rest) > 10:
                            print(f"      ... and {len(pressed_not_at_rest) - 10} more")
                    
                    if unpressed_not_at_rest:
                        print(f"    Unpressed keys not at rest ({len(unpressed_not_at_rest)}):")
                        for key_id, pos_deg, rest_deg in unpressed_not_at_rest[:5]:
                            print(f"      Key {key_id:2d}: {pos_deg:7.2f}° (expected: {rest_deg:6.2f}°)")
                        if len(unpressed_not_at_rest) > 5:
                            print(f"      ... and {len(unpressed_not_at_rest) - 5} more")
                else:
                    print("  ✓ All keys returned to rest position")
                
                # Check for stuck active keys
                active_keys = np.where(activations)[0]
                if len(active_keys) > 0:
                    print(f"\n⚠️  {len(active_keys)} keys still detected as active: {active_keys[:10].tolist()}")
                    # Show which of these were pressed
                    active_and_pressed = [k for k in active_keys if k in keys_pressed_during_playback]
                    if active_and_pressed:
                        print(f"    ({len(active_and_pressed)} of these were pressed during playback)")
                else:
                    print(f"\n✓ No keys detected as active")
            if recording_enabled and recording_timestamp:
                actual_path = os.path.expanduser(f"~/omni.replicator_out/recordings/midi_playback_{recording_timestamp}")
                print(f"  Recording saved to: {actual_path}/")
                
                # Auto-convert to video
                print(f"\n🎬 Converting recording to video (30fps)...")
                output_video = os.path.join(Path(__file__).parent, f"midi_playback_{recording_timestamp}.mp4")
                input_pattern = os.path.join(actual_path, "rgb_%04d.png")
                
                try:
                    # Check if ffmpeg is available
                    subprocess.run(["ffmpeg", "-version"], capture_output=True, check=True)
                    
                    # Run ffmpeg conversion
                    cmd = [
                        "ffmpeg", "-y",  # Overwrite output file
                        "-framerate", "30",
                        "-i", input_pattern,
                        "-c:v", "libx264",
                        "-pix_fmt", "yuv420p",
                        output_video
                    ]
                    
                    result = subprocess.run(cmd, capture_output=True, text=True)
                    
                    if result.returncode == 0:
                        file_size = os.path.getsize(output_video) / (1024 * 1024)  # Size in MB
                        print(f"  ✓ Video created successfully: {output_video}")
                        print(f"  ✓ File size: {file_size:.2f} MB")
                    else:
                        print(f"  ⚠️  FFmpeg conversion failed:")
                        print(f"     {result.stderr}")
                        print(f"  Manual conversion command:")
                        print(f"  ffmpeg -framerate 30 -i {input_pattern} -c:v libx264 -pix_fmt yuv420p {output_video}")
                        
                except FileNotFoundError:
                    print(f"  ⚠️  FFmpeg not found. Please install ffmpeg or convert manually:")
                    print(f"  ffmpeg -framerate 30 -i {input_pattern} -c:v libx264 -pix_fmt yuv420p {output_video}")
                except Exception as e:
                    print(f"  ⚠️  Error during video conversion: {e}")
                    print(f"  Manual conversion command:")
                    print(f"  ffmpeg -framerate 30 -i {input_pattern} -c:v libx264 -pix_fmt yuv420p {output_video}")
            print()
            break
    
    simulation_app.close()


if __name__ == "__main__":
    main()

