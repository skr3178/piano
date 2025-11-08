#!/usr/bin/env python3
"""
Piano Demo with Video and Audio Recording
"""
import sys
from pathlib import Path
import numpy as np
from datetime import datetime

from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False, "width": 1920, "height": 1080})

from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.render_product import RenderProduct
import omni.replicator.core as rep

# Add piano extension
extension_parent = Path(__file__).parent / "exts"
sys.path.insert(0, str(extension_parent))

import omni.isaac
piano_path = str(extension_parent / "omni" / "isaac")
if hasattr(omni.isaac, '__path__') and piano_path not in omni.isaac.__path__:
    omni.isaac.__path__.append(piano_path)

from omni.isaac.piano import IsaacPianoController


def setup_video_recording(output_dir: Path, fps: int = 60):
    """Setup video recording using Isaac Sim's replicator."""
    output_dir.mkdir(exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    video_path = output_dir / f"piano_recording_{timestamp}.mp4"
    
    # Create render product for the active viewport
    render_product = rep.create.render_product("/OmniverseKit_Persp", resolution=(1920, 1080))
    
    # Create video writer
    writer = rep.WriterRegistry.get("BasicWriter")
    writer.initialize(
        output_dir=str(output_dir),
        rgb=True,
        fps=fps
    )
    writer.attach([render_product])
    
    print(f"✓ Video recording setup: {video_path}")
    return writer, video_path


def setup_midi_to_audio(output_dir: Path):
    """Setup MIDI to audio conversion."""
    try:
        import mido
        from mido import MidiFile, MidiTrack, Message
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        midi_path = output_dir / f"piano_recording_{timestamp}.mid"
        
        # Create MIDI file
        mid = MidiFile()
        track = MidiTrack()
        mid.tracks.append(track)
        
        # Set tempo (500,000 microseconds per beat = 120 BPM)
        track.append(mido.MetaMessage('set_tempo', tempo=500000))
        
        print(f"✓ MIDI recording setup: {midi_path}")
        return mid, track, midi_path
    except ImportError:
        print("⚠ Warning: mido not installed. MIDI recording disabled.")
        print("  Install with: pip install mido")
        return None, None, None


def main():
    # Create output directory
    output_dir = Path(__file__).parent / "recordings"
    output_dir.mkdir(exist_ok=True)
    
    # Setup recording
    print("\n" + "="*70)
    print("SETTING UP RECORDING")
    print("="*70)
    
    # Video recording
    writer, video_path = setup_video_recording(output_dir)
    
    # MIDI recording
    midi_file, midi_track, midi_path = setup_midi_to_audio(output_dir)
    
    # Create world
    world = World(stage_units_in_meters=1.0)
    world.get_physics_context().set_gravity(value=0.0)
    
    # Add lighting
    from omni.isaac.core.utils.stage import get_current_stage
    from pxr import UsdLux, Gf, UsdGeom
    
    stage = get_current_stage()
    
    # Dome light
    dome_light = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome_light.CreateIntensityAttr(1000.0)
    
    # Directional light
    sun_light = UsdLux.DistantLight.Define(stage, "/World/SunLight")
    sun_light.CreateIntensityAttr(3000.0)
    sun_light.CreateAngleAttr(0.53)
    sun_light_xform = UsdGeom.Xformable(sun_light.GetPrim())
    sun_light_xform.AddRotateXYZOp().Set(Gf.Vec3f(-45, 45, 0))
    
    # Load piano
    piano_usd_path = Path(__file__).parent / "piano_with_physics.usda"
    add_reference_to_stage(str(piano_usd_path), "/Piano")
    
    # Position camera for better view
    from pxr import UsdGeom, Gf
    camera_path = "/OmniverseKit_Persp"
    camera = UsdGeom.Camera.Get(stage, camera_path)
    if camera:
        xform = UsdGeom.Xformable(camera.GetPrim())
        xform.ClearXformOpOrder()
        xform.AddTranslateOp().Set(Gf.Vec3d(0.5, -1.5, 0.5))
        xform.AddRotateXYZOp().Set(Gf.Vec3f(20, 0, 0))
    
    # Create articulation
    piano_articulation = world.scene.add(
        Articulation(prim_path="/Piano", name="piano")
    )
    
    # Create controller with actuators
    piano_controller = IsaacPianoController(
        piano_articulation=piano_articulation,
        prim_path="/Piano",
        change_color_on_activation=True,
        add_actuators=True,
    )
    
    # Track MIDI events with timestamps
    midi_events = []
    current_time = 0.0
    
    def on_note_on(note: int, velocity: int):
        print(f"♪ Note ON:  {note:3d} | Velocity: {velocity}")
        if midi_track is not None:
            ticks = int(current_time * 480)  # Convert to MIDI ticks
            midi_events.append((ticks, 'note_on', note, velocity))
    
    def on_note_off(note: int):
        print(f"♪ Note OFF: {note:3d}")
        if midi_track is not None:
            ticks = int(current_time * 480)
            midi_events.append((ticks, 'note_off', note))
    
    piano_controller.register_note_on_callback(on_note_on)
    piano_controller.register_note_off_callback(on_note_off)
    
    # Initialize
    world.stop()
    world.reset()
    piano_controller.initialize_episode()
    world.play()
    
    print("\n" + "="*70)
    print("RECORDING STARTED")
    print("="*70)
    print("Playing melody...")
    print()
    
    # Define melody
    melody = [40, 42, 44, 45, 47, 45, 44, 42, 40]  # C D E F G F E D C
    
    step = 0
    note_duration = 40
    current_note_idx = 0
    note_timer = 0
    
    # Main loop
    while simulation_app.is_running() and current_note_idx < len(melody):
        world.step(render=True)
        
        if world.is_playing():
            dt = world.get_physics_dt()
            current_time += dt
            
            # Create action
            action = np.zeros(89)
            
            # Press current note
            if note_timer < note_duration // 2:
                key_id = melody[current_note_idx]
                action[key_id] = 1.0
                
                if note_timer == 0:
                    print(f"[Step {step:4d}] Pressing key {key_id}")
            else:
                if note_timer == note_duration // 2:
                    print(f"[Step {step:4d}] Releasing key")
            
            piano_controller.apply_action(action)
            piano_controller.update(dt)
            
            note_timer += 1
            if note_timer >= note_duration:
                note_timer = 0
                current_note_idx += 1
            
            step += 1
    
    # Wait a bit to capture final release
    for _ in range(60):
        world.step(render=True)
        if world.is_playing():
            dt = world.get_physics_dt()
            current_time += dt
            piano_controller.update(dt)
    
    print("\n" + "="*70)
    print("RECORDING COMPLETE")
    print("="*70)
    
    # Save MIDI file
    if midi_file is not None and len(midi_events) > 0:
        import mido
        
        # Sort events by time
        midi_events.sort(key=lambda x: x[0])
        
        last_tick = 0
        for event in midi_events:
            tick, event_type, note, *args = event
            delta = tick - last_tick
            
            if event_type == 'note_on':
                velocity = args[0]
                midi_track.append(mido.Message('note_on', note=note, velocity=velocity, time=delta))
            else:
                midi_track.append(mido.Message('note_off', note=note, velocity=64, time=delta))
            
            last_tick = tick
        
        midi_file.save(str(midi_path))
        print(f"✓ MIDI saved: {midi_path}")
        print(f"  Total events: {len(midi_events)}")
        print(f"  To convert to audio:")
        print(f"    fluidsynth -ni soundfont.sf2 {midi_path} -F output.wav -r 44100")
    
    print(f"✓ Video saved: {video_path}")
    print("="*70)
    
    simulation_app.close()


if __name__ == "__main__":
    main()

