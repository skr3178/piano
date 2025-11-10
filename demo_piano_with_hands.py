#!/usr/bin/env python3
"""
Demo: Piano with Allegro Hands in Isaac Sim

This demo loads the piano with two Allegro hands and provides basic hand control.
Simple implementation to verify hands load correctly and can be controlled.
"""

import sys
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


def main():
    print("\n" + "="*70)
    print("Isaac Sim Piano with Allegro Hands - Demo")
    print("="*70 + "\n")
    
    # Create world
    world = World(stage_units_in_meters=1.0)
    stage = world.stage
    
    # Load piano with hands scene
    piano_hands_usd_path = Path(__file__).parent / "piano_with_allegro_hands.usda"
    if not piano_hands_usd_path.exists():
        print(f"❌ Scene file not found: {piano_hands_usd_path}")
        simulation_app.close()
        return
    
    print(f"📁 Loading scene: {piano_hands_usd_path.name}")
    add_reference_to_stage(str(piano_hands_usd_path), "/World")
    
    # CRITICAL: Configure collision groups BEFORE creating articulations
    # This ensures piano keys and hand fingers have proper collision groups set
    print("\n🔧 Configuring collision groups...")
    try:
        from configure_collisions import configure_collisions
        collision_count = configure_collisions(stage, load_payloads=True)
        print(f"  ✓ Configured {collision_count} collision prims")
    except Exception as e:
        print(f"  ⚠️  Warning: Could not configure collisions: {e}")
        print("  ⚠️  Fingers may pass through piano keys!")
        import traceback
        traceback.print_exc()
    
    # Add bright lighting for clear visibility
    from pxr import UsdLux
    
    # Add a dome light for ambient illumination
    dome_light = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome_light.CreateIntensityAttr(1500.0)
    dome_light.CreateColorAttr(Gf.Vec3f(1.0, 1.0, 1.0))
    
    # Add directional light
    dist_light = UsdLux.DistantLight.Define(stage, "/World/OverheadLight")
    dist_light.CreateIntensityAttr(3000.0)
    dist_light.CreateColorAttr(Gf.Vec3f(1.0, 0.98, 0.95))
    dist_light_xform = UsdGeom.Xformable(dist_light.GetPrim())
    dist_light_xform.AddRotateXYZOp().Set(Gf.Vec3f(-45, 45, 0))
    
    # Position camera to view piano and hands
    camera_path = "/OmniverseKit_Persp"
    camera = UsdGeom.Camera.Get(stage, camera_path)
    if camera:
        xform = UsdGeom.Xformable(camera.GetPrim())
        xform_ops = xform.GetOrderedXformOps()
        
        # Position: in front of piano, slightly above, looking down at keys
        if len(xform_ops) >= 1:
            xform_ops[0].Set(Gf.Vec3d(0.0, -0.6, 0.5))  # Further back to see hands
        else:
            translate_op = xform.AddTranslateOp()
            translate_op.Set(Gf.Vec3d(0.0, -0.6, 0.5))
        
        # Rotate to look at piano and hands
        if len(xform_ops) >= 2:
            xform_ops[1].Set(Gf.Vec3f(20, 0, 0))  # Slight pitch down
        else:
            rotate_op = xform.AddRotateXYZOp()
            rotate_op.Set(Gf.Vec3f(20, 0, 0))
        
        # Zoom out a bit to see hands
        horizontal_aperture = camera.GetHorizontalApertureAttr()
        if horizontal_aperture:
            default_aperture = horizontal_aperture.Get() or 20.96
            camera.GetHorizontalApertureAttr().Set(default_aperture / 2.0)  # 2x zoom (less than piano demo)
            print(f"✓ Camera positioned (aperture: {default_aperture:.2f}mm -> {default_aperture/2.0:.2f}mm)")
    
    # Create articulations for hands
    print("\n🤖 Initializing hands...")
    
    left_hand_articulation = None
    right_hand_articulation = None
    
    try:
        left_hand_articulation = world.scene.add(
            Articulation(prim_path="/World/left_hand", name="left_hand")
        )
        print("  ✓ Left hand articulation created")
    except Exception as e:
        print(f"  ⚠️  Could not create left hand articulation: {e}")
    
    try:
        right_hand_articulation = world.scene.add(
            Articulation(prim_path="/World/right_hand", name="right_hand")
        )
        print("  ✓ Right hand articulation created")
    except Exception as e:
        print(f"  ⚠️  Could not create right hand articulation: {e}")
    
    # Reset world to initialize physics
    world.reset()
    
    # Initialize hands to rest positions
    if left_hand_articulation:
        try:
            joint_names = left_hand_articulation.dof_names
            if joint_names:
                initial_positions = np.zeros(len(joint_names))
                left_hand_articulation.set_joint_positions(initial_positions)
                print(f"  ✓ Left hand initialized to rest position ({len(joint_names)} joints)")
            else:
                print("  ⚠️  Left hand has no joints")
        except Exception as e:
            print(f"  ⚠️  Could not initialize left hand positions: {e}")
    
    if right_hand_articulation:
        try:
            joint_names = right_hand_articulation.dof_names
            if joint_names:
                initial_positions = np.zeros(len(joint_names))
                right_hand_articulation.set_joint_positions(initial_positions)
                print(f"  ✓ Right hand initialized to rest position ({len(joint_names)} joints)")
            else:
                print("  ⚠️  Right hand has no joints")
        except Exception as e:
            print(f"  ⚠️  Could not initialize right hand positions: {e}")
    
    # Clear selection
    if clear_selection():
        print("✓ Cleared object selection for better visuals")
    
    # Disable viewport grid
    try:
        import omni.kit.viewport.utility
        viewport_window = omni.kit.viewport.utility.get_active_viewport_window()
        if viewport_window:
            # Disable grid
            viewport_window.get_viewport_api().get_viewport_grid().set_enabled(False)
            print("✓ Disabled viewport grid")
    except Exception as e:
        # Try alternative method
        try:
            import omni.kit.viewport.utility
            viewport_api = omni.kit.viewport.utility.get_active_viewport()
            if viewport_api:
                viewport_api.get_viewport_grid().set_enabled(False)
                print("✓ Disabled viewport grid (alternative method)")
        except Exception:
            print(f"⚠️  Could not disable grid programmatically: {e}")
            print("   You may need to disable it manually in the viewport settings")
    
    print()
    print("="*70)
    print("Scene loaded successfully!")
    print("="*70)
    print("\nHands are positioned above the piano keys.")
    print("You can now control the hands programmatically or interact with them in the viewport.")
    print("\nPress ESC or close the window to exit.")
    print()
    
    # Simulation loop - just keep running
    step = 0
    while simulation_app.is_running():
        # Step simulation
        world.step(render=True)
        
        if world.is_playing():
            step += 1
            
            # Status update every 5 seconds
            dt = world.get_physics_dt()
            if step % int(5.0 / dt) == 0:
                sim_time = step * dt
                print(f"[{sim_time:5.1f}s] Simulation running... (step {step})")
                
                # Print hand joint positions every 5 seconds for debugging
                if left_hand_articulation and step % int(10.0 / dt) == 0:
                    try:
                        joint_positions = left_hand_articulation.get_joint_positions()
                        print(f"  Left hand joint positions (first 5): {joint_positions[:5]}")
                    except Exception:
                        pass
                
                if right_hand_articulation and step % int(10.0 / dt) == 0:
                    try:
                        joint_positions = right_hand_articulation.get_joint_positions()
                        print(f"  Right hand joint positions (first 5): {joint_positions[:5]}")
                    except Exception:
                        pass
    
    simulation_app.close()


if __name__ == "__main__":
    main()



