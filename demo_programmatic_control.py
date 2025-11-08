#!/usr/bin/env python3
"""
Demo: Programmatic Piano Control
Shows how to press piano keys programmatically using actions.
"""

import sys
from pathlib import Path
import numpy as np

from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.stage import add_reference_to_stage

# Add piano extension to path
extension_parent = Path(__file__).parent / "exts"
if str(extension_parent) not in sys.path:
    sys.path.insert(0, str(extension_parent))

# Extend namespace
import omni.isaac
piano_path = str(extension_parent / "omni" / "isaac")
if hasattr(omni.isaac, '__path__') and piano_path not in omni.isaac.__path__:
    omni.isaac.__path__.append(piano_path)

from omni.isaac.piano import IsaacPianoController


def main():
    print("="*70)
    print("PROGRAMMATIC PIANO CONTROL DEMO")
    print("="*70)
    
    # Create world and disable gravity
    world = World(stage_units_in_meters=1.0)
    world.get_physics_context().set_gravity(value=0.0)
    
    # Add lighting for better visuals
    from omni.isaac.core.utils.stage import get_current_stage
    from pxr import UsdLux, Gf, UsdGeom
    
    stage = get_current_stage()
    
    # Add a dome light for ambient lighting
    dome_light = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome_light.CreateIntensityAttr(1000.0)
    dome_light.CreateColorAttr(Gf.Vec3f(1.0, 1.0, 1.0))
    
    # Add a directional light (sun-like)
    sun_light = UsdLux.DistantLight.Define(stage, "/World/SunLight")
    sun_light.CreateIntensityAttr(3000.0)
    sun_light.CreateColorAttr(Gf.Vec3f(1.0, 0.98, 0.95))
    sun_light.CreateAngleAttr(0.5)
    xform = UsdGeom.Xformable(sun_light)
    xform.ClearXformOpOrder()
    rotate_op = xform.AddRotateXYZOp()
    rotate_op.Set(Gf.Vec3f(-45, 45, 0))
    
    # Add a spotlight for key highlights
    spot_light = UsdLux.SphereLight.Define(stage, "/World/SpotLight")
    spot_light.CreateIntensityAttr(5000.0)
    spot_light.CreateRadiusAttr(0.1)
    spot_light.CreateColorAttr(Gf.Vec3f(1.0, 1.0, 1.0))
    xform_spot = UsdGeom.Xformable(spot_light)
    xform_spot.ClearXformOpOrder()
    translate_op = xform_spot.AddTranslateOp()
    translate_op.Set(Gf.Vec3d(0.0, 0.5, 1.0))
    
    # Add a ground plane
    ground_plane = UsdGeom.Mesh.Define(stage, "/World/GroundPlane")
    size = 10.0
    ground_plane.CreatePointsAttr([
        Gf.Vec3f(-size, -size, -0.5),
        Gf.Vec3f(size, -size, -0.5),
        Gf.Vec3f(size, size, -0.5),
        Gf.Vec3f(-size, size, -0.5)
    ])
    ground_plane.CreateFaceVertexCountsAttr([4])
    ground_plane.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    ground_plane.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)] * 4)
    ground_plane.CreateDisplayColorAttr([Gf.Vec3f(0.2, 0.2, 0.2)])
    
    print("✓ Added lighting and ground plane")
    
    # Load piano
    piano_usd_path = Path(__file__).parent / "piano_with_physics.usda"
    piano_prim_path = "/Piano"
    add_reference_to_stage(str(piano_usd_path), piano_prim_path)
    
    # Fix the piano in space BEFORE creating articulation
    from pxr import UsdPhysics, Gf
    piano_xform = UsdGeom.Xformable(stage.GetPrimAtPath(piano_prim_path))
    
    # Set piano position at origin
    translate_op = None
    for op in piano_xform.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            translate_op = op
            break
    if not translate_op:
        translate_op = piano_xform.AddTranslateOp()
    translate_op.Set(Gf.Vec3d(0.0, 0.0, 0.0))
    
    print("✓ Piano positioned at origin")
    
    # Create articulation
    piano_articulation = world.scene.add(
        Articulation(prim_path=piano_prim_path, name="piano")
    )
    
    # Create controller with actuators ENABLED
    piano_controller = IsaacPianoController(
        piano_articulation=piano_articulation,
        prim_path=piano_prim_path,
        change_color_on_activation=True,
        add_actuators=True,  # ← Enable programmatic control
    )
    
    # MIDI callbacks
    def on_note_on(note: int, velocity: int):
        print(f"♪ Note ON:  {note:3d} (velocity: {velocity})")
    
    def on_note_off(note: int):
        print(f"♪ Note OFF: {note:3d}")
    
    piano_controller.register_note_on_callback(on_note_on)
    piano_controller.register_note_off_callback(on_note_off)
    
    # IMPORTANT: Stop the simulation first to prevent auto-play issues
    world.stop()
    
    # Reset the world (this initializes physics properly)
    world.reset()
    
    # Initialize the piano controller AFTER reset
    piano_controller.initialize_episode()
    
    # Now start playing
    world.play()
    
    # Lock the piano base to prevent any movement
    try:
        piano_articulation.set_linear_velocity(np.zeros(3))
        piano_articulation.set_angular_velocity(np.zeros(3))
        print("✓ Piano base velocities locked to zero")
    except:
        pass
    
    print("\n" + "="*70)
    print("SIMULATION STARTED")
    print("="*70)
    print("\nPlaying a simple melody programmatically...")
    print("C-D-E-C pattern (do-re-mi-do)")
    print("="*70 + "\n")
    
    # Middle C is key 40 (MIDI note 60)
    # C major scale: C=40, D=42, E=44, F=45, G=47, A=49, B=51, C=52
    melody = [
        (40, 60),   # C (key 40 = MIDI 60)
        (42, 60),   # D
        (44, 60),   # E
        (40, 60),   # C
    ]
    
    step = 0
    note_duration = 60  # steps (~1 second at 60Hz)
    current_note_idx = 0
    note_timer = 0
    current_action = np.zeros(89)
    
    # Main loop
    while simulation_app.is_running() and current_note_idx < len(melody):
        world.step(render=True)
        
        if world.is_playing():
            dt = world.get_physics_dt()
            
            # Play melody
            if note_timer == 0:
                # Start new note
                key_id, midi_note = melody[current_note_idx]
                print(f"\n[Step {step}] Pressing key {key_id} (MIDI {midi_note})")
                current_action[:] = 0.0
                current_action[key_id] = 1.0  # Press the key
                
            elif note_timer == note_duration // 2:
                # Release note halfway
                print(f"[Step {step}] Releasing key")
                current_action[:] = 0.0  # Release all keys
            
            # Apply action
            piano_controller.apply_action(current_action)
            piano_controller.update(dt)
            
            # Update timing
            note_timer += 1
            if note_timer >= note_duration:
                note_timer = 0
                current_note_idx += 1
            
            step += 1
    
    print("\n" + "="*70)
    print("Melody complete!")
    print("="*70)
    
    # Wait a bit before closing
    for _ in range(120):
        world.step(render=True)
        if world.is_playing():
            piano_controller.update(world.get_physics_dt())
    
    simulation_app.close()


if __name__ == "__main__":
    main()

