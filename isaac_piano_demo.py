#!/usr/bin/env python3
"""
Isaac Sim Piano Demo Script

Main script to run the piano simulation with full MIDI support.

Requirements:
    - Isaac Sim 4.5.0 installed
    - Run with Isaac Sim's python.sh or use the launcher script
    
Usage:
    # Recommended: Use launcher script
    ./run_piano_demo.sh
    
    # Or directly with Isaac Sim's python
    /home/skr/isaacsim/python.sh isaac_piano_demo.py
"""

import sys
from pathlib import Path

# Import Isaac Sim - must be run with Isaac Sim's python.sh
try:
    from omni.isaac.kit import SimulationApp
except ImportError:
    print("=" * 70)
    print("ERROR: Could not import Isaac Sim!")
    print("=" * 70)
    print()
    print("This script must be run with Isaac Sim's Python.")
    print()
    print("Please use:")
    print("  ./run_piano_demo.sh")
    print()
    print("Or directly:")
    print("  /home/skr/isaacsim/python.sh isaac_piano_demo.py")
    print("=" * 70)
    sys.exit(1)

# Launch Isaac Sim
print("Launching Isaac Sim...")
simulation_app = SimulationApp({
    "headless": False,  # Headless mode for testing
    "width": 1920,
    "height": 1080,
})

# Now we can import Isaac Sim modules
import numpy as np
from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.stage import add_reference_to_stage

# Add our piano extension to path  
# The omni package is at /home/skr/Downloads/piano/exts/omni
extension_parent = Path(__file__).parent / "exts"
extension_parent_abs = extension_parent.resolve()

print(f"Script location: {Path(__file__).parent}", flush=True)
print(f"Extension parent: {extension_parent_abs}", flush=True)

if str(extension_parent_abs) not in sys.path:
    sys.path.insert(0, str(extension_parent_abs))
    print(f"✓ Added {extension_parent_abs} to Python path", flush=True)

# Verify omni folder exists
omni_folder = extension_parent_abs / "omni"
if omni_folder.exists():
    print(f"✓ Found omni folder at: {omni_folder}", flush=True)
else:
    print(f"✗ omni folder NOT found at: {omni_folder}", flush=True)

# Manually extend the omni.isaac namespace to include our custom piano module
# This is necessary because Isaac Sim has its own omni.isaac namespace
try:
    import omni
    import omni.isaac
    # Add our piano module to the omni.isaac namespace path
    piano_path = str(extension_parent_abs / "omni" / "isaac")
    if hasattr(omni.isaac, '__path__'):
        if piano_path not in omni.isaac.__path__:
            # Append to namespace path (it's a _NamespacePath object)
            omni.isaac.__path__.append(piano_path)
            print(f"✓ Extended omni.isaac namespace with: {piano_path}", flush=True)
except Exception as e:
    print(f"Warning: Could not extend omni.isaac namespace: {e}", flush=True)

# Import our piano controller
try:
    from omni.isaac.piano import IsaacPianoController
    print("✓ Successfully imported IsaacPianoController", flush=True)
except ImportError as e:
    print(f"ERROR: Could not import piano controller: {e}", flush=True)
    print(f"Extension parent: {extension_parent_abs}", flush=True)
    print(f"Python path: {sys.path[:5]}", flush=True)
    simulation_app.close()
    sys.exit(1)

print("About to call main()...", flush=True)


def main():
    """Main demo function."""
    
    print("=" * 70, flush=True)
    print("ISAAC SIM PIANO DEMO", flush=True)
    print("=" * 70, flush=True)
    print(flush=True)
    
    # Create world with gravity disabled
    world = World(stage_units_in_meters=1.0)
    
    # Disable gravity
    world.get_physics_context().set_gravity(value=0.0)
    
    # Add lighting for better visuals
    from omni.isaac.core.utils.stage import get_current_stage
    from pxr import UsdLux, Gf, UsdGeom, Sdf
    
    stage = get_current_stage()
    
    # Add a dome light for ambient lighting
    dome_light_path = "/World/DomeLight"
    dome_light = UsdLux.DomeLight.Define(stage, dome_light_path)
    dome_light.CreateIntensityAttr(1000.0)
    dome_light.CreateColorAttr(Gf.Vec3f(1.0, 1.0, 1.0))
    
    # Add a directional light (sun-like) for better definition
    sun_light_path = "/World/SunLight"
    sun_light = UsdLux.DistantLight.Define(stage, sun_light_path)
    sun_light.CreateIntensityAttr(3000.0)
    sun_light.CreateColorAttr(Gf.Vec3f(1.0, 0.98, 0.95))  # Warm white
    sun_light.CreateAngleAttr(0.5)
    xform = UsdGeom.Xformable(sun_light)
    xform.ClearXformOpOrder()
    # Rotate to shine from above and slightly to the side
    rotate_op = xform.AddRotateXYZOp()
    rotate_op.Set(Gf.Vec3f(-45, 45, 0))
    
    # Add a spotlight for key highlights
    spot_light_path = "/World/SpotLight"
    spot_light = UsdLux.SphereLight.Define(stage, spot_light_path)
    spot_light.CreateIntensityAttr(5000.0)
    spot_light.CreateRadiusAttr(0.1)
    spot_light.CreateColorAttr(Gf.Vec3f(1.0, 1.0, 1.0))
    xform_spot = UsdGeom.Xformable(spot_light)
    xform_spot.ClearXformOpOrder()
    translate_op = xform_spot.AddTranslateOp()
    translate_op.Set(Gf.Vec3d(0.0, 0.5, 1.0))  # Above and in front of piano
    
    # Add a ground plane for better spatial reference
    ground_path = "/World/GroundPlane"
    ground_plane = UsdGeom.Mesh.Define(stage, ground_path)
    
    # Create a large plane
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
    
    # Set ground color (dark gray)
    ground_plane.CreateDisplayColorAttr([Gf.Vec3f(0.2, 0.2, 0.2)])
    
    print("✓ Added lighting and ground plane for better visuals")
    
    # Piano USD path
    piano_usd_path = Path(__file__).parent / "piano_with_physics.usda"
    
    if not piano_usd_path.exists():
        print(f"ERROR: Piano USD not found at {piano_usd_path}")
        print("Run generate_piano_usda_with_physics.py first!")
        simulation_app.close()
        return
    
    print(f"Loading piano: {piano_usd_path.name}")
    
    # Add piano to stage
    # The piano USD defines the piano at root /Piano, so we add it directly to /Piano path
    piano_prim_path = "/Piano"
    add_reference_to_stage(str(piano_usd_path), piano_prim_path)
    
    # Fix the piano in space BEFORE creating articulation
    # Set the piano base position explicitly
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
    
    print("Creating piano controller...")
    
    # Create controller
    piano_controller = IsaacPianoController(
        piano_articulation=piano_articulation,
        prim_path=piano_prim_path,
        change_color_on_activation=True,
        add_actuators=False,
    )
    
    # MIDI callbacks
    def on_note_on(note: int, velocity: int):
        print(f"♪ Note ON:  {note:3d} | Velocity: {velocity:3d}")
    
    def on_note_off(note: int):
        print(f"♪ Note OFF: {note:3d}")
    
    def on_sustain_on():
        print("🎹 Sustain: ON")
    
    def on_sustain_off():
        print("🎹 Sustain: OFF")
    
    piano_controller.register_note_on_callback(on_note_on)
    piano_controller.register_note_off_callback(on_note_off)
    piano_controller.register_sustain_on_callback(on_sustain_on)
    piano_controller.register_sustain_off_callback(on_sustain_off)
    
    # IMPORTANT: Stop the simulation first to prevent auto-play issues
    world.stop()
    
    # Reset the world (this initializes physics properly)
    world.reset()
    
    # Initialize the piano controller AFTER reset
    piano_controller.initialize_episode()
    
    # Now start playing
    world.play()
    
    print()
    print("=" * 70)
    print("SIMULATION STARTED")
    print("=" * 70)
    print()
    print("Features Enabled:")
    print("  ✓ Real-time key state tracking (88 keys)")
    print("  ✓ MIDI message generation")
    print("  ✓ Key color changes on activation")
    print("  ✓ Callback hooks for synthesizer")
    print("  ✓ Observable states (positions, activations)")
    print()
    print("Instructions:")
    print("  • Click and drag piano keys to press them")
    print("  • Keys will turn green when activated")
    print("  • MIDI events are printed to console")
    print("  • Press ESC or close window to exit")
    print()
    print("=" * 70)
    print()
    
    step_count = 0
    last_active_count = 0
    
    # Main loop
    while simulation_app.is_running():
        world.step(render=True)
        
        if world.is_playing():
            # Update piano
            dt = world.get_physics_dt()
            piano_controller.update(dt)
            
            step_count += 1
            
            # Status every 100 steps
            if step_count % 100 == 0:
                active_count = np.sum(piano_controller.activation)
                if active_count != last_active_count:
                    print(f"[Step {step_count:6d}] Active keys: {active_count:2d}")
                    last_active_count = active_count
    
    print()
    print("=" * 70)
    print("Simulation ended")
    
    # Summary
    all_messages = piano_controller.get_all_midi_messages()
    print(f"Total MIDI messages generated: {len(all_messages)}")
    print("=" * 70)
    
    simulation_app.close()


if __name__ == "__main__":
    main()


