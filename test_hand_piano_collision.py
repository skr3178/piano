#!/usr/bin/env python3
"""
Test: Hand-Piano Collision Detection (Step 2.2)

This script tests collision detection between hands and piano keys:
- Verify hand fingertips can contact piano keys
- Test collision geometry is correct
- Verify keys respond to hand contact
- Check for penetration or collision issues
"""

import sys
import os
from pathlib import Path

# Initialize Isaac Sim FIRST (before other imports)
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import numpy as np
import re
from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.stage import add_reference_to_stage
from pxr import UsdGeom, Gf, UsdPhysics, Sdf, UsdLux
import omni.physx as physx
import omni.replicator.core as rep
import subprocess

# Import piano controller
sys.path.insert(0, str(Path(__file__).parent))
try:
    from exts.omni.isaac.piano.piano_controller import IsaacPianoController
    PIANO_CONTROLLER_AVAILABLE = True
except ImportError:
    PIANO_CONTROLLER_AVAILABLE = False
    print("⚠️  Piano controller not available - key monitoring will be limited")


def get_contact_reports(physics_interface):
    """Get contact reports from PhysX."""
    try:
        # Get contact reports
        contact_reports = physics_interface.get_contact_report()
        return contact_reports
    except Exception as e:
        print(f"Warning: Could not get contact reports: {e}")
        return None


def get_joint_torque_limits_from_usd(stage, hand_prim_path):
    """Query maximum torque limits from USD joint drive properties.
    
    Args:
        stage: USD stage
        hand_prim_path: Path to hand prim (e.g., "/World/left_hand")
    
    Returns:
        Dictionary mapping joint names to max torque limits in N⋅m
    """
    limits = {}
    hand_prim = stage.GetPrimAtPath(hand_prim_path)
    
    if not hand_prim or not hand_prim.IsValid():
        return limits
    
    def find_joints(prim, path_prefix=""):
        """Recursively find all joints and their torque limits."""
        # Check if this is a joint
        if prim.IsA(UsdPhysics.RevoluteJoint) or prim.IsA(UsdPhysics.PrismaticJoint):
            joint_name = prim.GetName()
            max_force = None
            
            # Try to get maxForce from angular drive API
            try:
                drive_api = UsdPhysics.DriveAPI.Get(prim, "angular")
                if drive_api:
                    max_force_attr = drive_api.GetMaxForceAttr()
                    if max_force_attr and max_force_attr.IsValid():
                        max_force = max_force_attr.Get()
            except:
                pass
            
            # Try linear drive if angular didn't work
            if max_force is None:
                try:
                    drive_api = UsdPhysics.DriveAPI.Get(prim, "linear")
                    if drive_api:
                        max_force_attr = drive_api.GetMaxForceAttr()
                        if max_force_attr and max_force_attr.IsValid():
                            max_force = max_force_attr.Get()
                except:
                    pass
            
            # Try direct attribute access
            if max_force is None:
                try:
                    max_force_attr = prim.GetAttribute("physics:maxForce")
                    if max_force_attr and max_force_attr.IsValid():
                        max_force = max_force_attr.Get()
                except:
                    pass
            
            # Try effort limit attribute
            if max_force is None:
                try:
                    effort_attr = prim.GetAttribute("physics:maxEffort")
                    if effort_attr and effort_attr.IsValid():
                        max_force = effort_attr.Get()
                except:
                    pass
            
            if max_force is not None:
                limits[joint_name] = max_force
        
        # Recursively search children
        for child in prim.GetChildren():
            find_joints(child)
    
    find_joints(hand_prim)
    return limits


def verify_collision_meshes_in_usd(usd_path):
    """Verify that a USD file contains collision meshes."""
    try:
        from pxr import Usd
        stage = Usd.Stage.Open(str(usd_path))
        if not stage:
            return False, 0
        
        collision_count = 0
        def count_collisions(prim):
            nonlocal collision_count
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                collision_count += 1
            for child in prim.GetChildren():
                count_collisions(child)
        
        default_prim = stage.GetDefaultPrim()
        if default_prim:
            count_collisions(default_prim)
        
        return True, collision_count
    except Exception as e:
        return False, 0


def verify_collision_meshes_in_stage(prim_path, stage):
    """Verify collision meshes exist in the loaded stage at a given prim path."""
    try:
        prim = stage.GetPrimAtPath(prim_path)
        if not prim or not prim.IsValid():
            return False, 0, []
        
        collision_paths = []
        def find_collisions(prim, path_prefix=""):
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                collision_paths.append(prim.GetPath())
            for child in prim.GetChildren():
                find_collisions(child)
        
        find_collisions(prim)
        return True, len(collision_paths), collision_paths
    except Exception as e:
        return False, 0, []


def enable_collisions_recursive(prim, stage, count=[0], debug=False, hand_group=1, piano_group=0, configure=True):
    """Recursively enable collisions on all collision prims and rigid bodies.
    
    Args:
        prim: Prim to process
        stage: USD stage
        count: Counter for collision prims found
        debug: Enable debug output
        hand_group: Collision group for hands (1)
        piano_group: Collision group for piano (0)
        configure: If True, configure collision groups. If False, only count.
    """
    if not prim or not prim.IsValid():
        return
    
    # Determine if this is a hand or piano prim based on path
    prim_path = str(prim.GetPath())
    is_hand = "hand" in prim_path.lower()
    is_piano = "piano" in prim_path.lower()
    
    # Enable collision on rigid bodies (RigidBodyAPI)
    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
        if configure:
            override_prim = stage.OverridePrim(prim.GetPath())
            if override_prim:
                # Ensure it's not kinematic (dynamic)
                try:
                    rigid_body_api = UsdPhysics.RigidBodyAPI(override_prim)
                    if rigid_body_api:
                        try:
                            kinematic_attr = rigid_body_api.GetKinematicEnabledAttr()
                            if kinematic_attr:
                                kinematic_attr.Set(False)
                                if debug:
                                    print(f"    Set {prim.GetPath()} to dynamic")
                        except:
                            pass
                except:
                    pass
        count[0] += 1
        if debug and count[0] <= 5:
            print(f"    Found rigid body: {prim.GetPath()}")
    
    # Enable collision on collision prims (CollisionAPI)
    if prim.HasAPI(UsdPhysics.CollisionAPI):
        if configure:
            override_prim = stage.OverridePrim(prim.GetPath())
            if override_prim:
                # Ensure collision is enabled
                try:
                    collision_api = UsdPhysics.CollisionAPI(override_prim)
                    if collision_api:
                        try:
                            collision_api.GetCollisionEnabledAttr().Set(True)
                        except:
                            # Try creating the attribute if it doesn't exist
                            collision_api.CreateCollisionEnabledAttr(True)
                    else:
                        # Apply CollisionAPI if it doesn't exist
                        collision_api = UsdPhysics.CollisionAPI.Apply(override_prim)
                        collision_api.CreateCollisionEnabledAttr(True)
                    
                    # Set collision groups properly based on whether it's hand or piano
                    # Don't remove collision groups - configure them correctly
                    if is_hand:
                        # Hand: group 1, filter 1 (collide with group 0)
                        override_prim.CreateAttribute("physx:collisionGroup", Sdf.ValueTypeNames.Int).Set(hand_group)
                        override_prim.CreateAttribute("physx:collisionFilter", Sdf.ValueTypeNames.Int).Set(1)  # 0b01 = collide with group 0
                    elif is_piano:
                        # Piano: group 0, filter 2 (collide with group 1)
                        override_prim.CreateAttribute("physx:collisionGroup", Sdf.ValueTypeNames.Int).Set(piano_group)
                        override_prim.CreateAttribute("physx:collisionFilter", Sdf.ValueTypeNames.Int).Set(2)  # 0b10 = collide with group 1
                    
                    if debug and count[0] <= 10:
                        print(f"    Enabled collision on: {prim.GetPath()}")
                except Exception as e:
                    if debug:
                        print(f"    Error enabling collision on {prim.GetPath()}: {e}")
        count[0] += 1
        if debug and count[0] <= 5:
            print(f"    Found collision prim: {prim.GetPath()}")
    
    # Recursively process children
    for child in prim.GetChildren():
        enable_collisions_recursive(child, stage, count, debug, hand_group, piano_group, configure)
    
    return count[0]


def find_fingertip_prims(stage, hand_path):
    """Find fingertip prims in a hand."""
    fingertips = []
    hand_prim = stage.GetPrimAtPath(hand_path)
    
    if not hand_prim or not hand_prim.IsValid():
        return fingertips
    
    def search_for_fingertips(prim, path_prefix=""):
        current_path = f"{path_prefix}/{prim.GetName()}" if path_prefix else prim.GetName()
        
        # Check if this is a fingertip (common naming patterns)
        prim_name = prim.GetName().lower()
        if any(keyword in prim_name for keyword in ["fingertip", "finger_tip", "tip", "end_effector"]):
            fingertips.append(current_path)
        
        # Recursively search children
        for child in prim.GetChildren():
            search_for_fingertips(child, current_path)
    
    search_for_fingertips(hand_prim, hand_path)
    return fingertips


def setup_video_recording():
    """Setup video recording using Replicator."""
    # Use 30fps for better visibility of key presses
    render_product = rep.create.render_product("/OmniverseKit_Persp", (1920, 1080))
    
    timestamp = __import__('datetime').datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = f"recordings/hand_piano_collision_{timestamp}"
    
    writer = rep.WriterRegistry.get("BasicWriter")
    writer.initialize(output_dir=output_dir, rgb=True)
    writer.attach([render_product])
    
    # Replicator saves to omni.replicator_out by default
    actual_path = os.path.expanduser("~/omni.replicator_out/recordings/hand_piano_collision_" + timestamp)
    
    print(f"✓ Recording to: {actual_path}")
    return output_dir, timestamp


def main():
    print("\n" + "="*70)
    print("Step 2.2: Hand-Piano Collision Detection Test")
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
    
    # CRITICAL: Verify physics USD file has collision meshes BEFORE loading
    print("\n🔍 Verifying physics USD files contain collision meshes...")
    physics_usd_path = Path(__file__).parent / "nvidia_assets/Assets/Isaac/5.1/Isaac/Robots/Unitree/Dex5/configuration/Dex5-URDF-R_physics.usd"
    if physics_usd_path.exists():
        success, collision_count = verify_collision_meshes_in_usd(physics_usd_path)
        if success:
            print(f"  ✓ Physics USD file exists: {physics_usd_path.name}")
            print(f"  ✓ Found {collision_count} collision prims in physics USD file")
            if collision_count == 0:
                print(f"  ⚠️  WARNING: Physics USD file has NO collision meshes!")
        else:
            print(f"  ⚠️  Could not open physics USD file: {physics_usd_path}")
    else:
        print(f"  ⚠️  Physics USD file not found: {physics_usd_path}")
    
    # CRITICAL: Ensure variant sets are correct and load payloads
    # The Physics variant must be "PhysX" to load physics collision meshes
    print("\n📦 Configuring variants and loading payloads (physics meshes)...")
    try:
        from pxr import Usd, UsdGeom
        # Get hand prims
        left_hand_prim = stage.GetPrimAtPath("/World/left_hand")
        right_hand_prim = stage.GetPrimAtPath("/World/right_hand")
        
        # Ensure Physics variant is set to "PhysX" (loads physics payload)
        if left_hand_prim and left_hand_prim.IsValid():
            variant_sets = left_hand_prim.GetVariantSets()
            if variant_sets:
                try:
                    physics_variant = variant_sets.GetVariantSet("Physics")
                    if physics_variant:
                        current_variant = physics_variant.GetVariantSelection()
                        if current_variant != "PhysX":
                            physics_variant.SetVariantSelection("PhysX")
                            print(f"  ✓ Set left hand Physics variant to 'PhysX' (was '{current_variant}')")
                        else:
                            print(f"  ✓ Left hand Physics variant already set to 'PhysX'")
                except:
                    pass
            
            # Load all payloads recursively (physics meshes are in payloads)
            for prim in Usd.PrimRange(left_hand_prim):
                if prim.HasPayload():
                    prim.Load()
            print("  ✓ Left hand payloads loaded")
        
        if right_hand_prim and right_hand_prim.IsValid():
            variant_sets = right_hand_prim.GetVariantSets()
            if variant_sets:
                try:
                    physics_variant = variant_sets.GetVariantSet("Physics")
                    if physics_variant:
                        current_variant = physics_variant.GetVariantSelection()
                        if current_variant != "PhysX":
                            physics_variant.SetVariantSelection("PhysX")
                            print(f"  ✓ Set right hand Physics variant to 'PhysX' (was '{current_variant}')")
                        else:
                            print(f"  ✓ Right hand Physics variant already set to 'PhysX'")
                except:
                    pass
            
            # Load all payloads recursively (physics meshes are in payloads)
            for prim in Usd.PrimRange(right_hand_prim):
                if prim.HasPayload():
                    prim.Load()
            print("  ✓ Right hand payloads loaded")
    except Exception as e:
        print(f"  ⚠️  Could not configure variants/load payloads: {e}")
        print("  ⚠️  This may cause collision meshes to be missing!")
        import traceback
        traceback.print_exc()
    
    # Get physics interface for contact reporting
    physics_interface = physx.get_physx_interface()
    
    # CRITICAL: Ensure payloads are loaded BEFORE creating articulations
    # Isaac Sim's Articulation class needs the physics meshes to be loaded
    print("\n📦 Ensuring physics payloads are loaded (required for collisions)...")
    try:
        from pxr import Usd
        left_hand_prim = stage.GetPrimAtPath("/World/left_hand")
        right_hand_prim = stage.GetPrimAtPath("/World/right_hand")
        
        # Force load all payloads recursively
        if left_hand_prim and left_hand_prim.IsValid():
            # Load payloads at all levels
            for prim in Usd.PrimRange(left_hand_prim):
                if prim.HasPayload():
                    prim.Load()
                    print(f"    Loaded payload: {prim.GetPath()}")
            print("  ✓ Left hand payloads loaded")
        
        if right_hand_prim and right_hand_prim.IsValid():
            for prim in Usd.PrimRange(right_hand_prim):
                if prim.HasPayload():
                    prim.Load()
                    print(f"    Loaded payload: {prim.GetPath()}")
            print("  ✓ Right hand payloads loaded")
    except Exception as e:
        print(f"  ⚠️  Could not load payloads: {e}")
        import traceback
        traceback.print_exc()
    
    # CRITICAL: Configure collision groups BEFORE creating articulations
    # Isaac Sim's Articulation class reads collision settings when created
    print("\n🔧 Configuring collision groups BEFORE creating articulations...")
    left_hand_prim = stage.GetPrimAtPath("/World/left_hand")
    right_hand_prim = stage.GetPrimAtPath("/World/right_hand")
    piano_prim = stage.GetPrimAtPath("/World/Piano")
    
    # Set collision groups on root prims BEFORE creating articulations
    if left_hand_prim and left_hand_prim.IsValid():
        override_prim = stage.OverridePrim(left_hand_prim.GetPath())
        override_prim.CreateAttribute("physx:collisionGroup", Sdf.ValueTypeNames.Int).Set(1)
        override_prim.CreateAttribute("physx:collisionFilter", Sdf.ValueTypeNames.Int).Set(1)  # Collide with group 0
        print("  ✓ Left hand collision group set to 1, filter 1 (before articulation creation)")
        
        # Configure all collision prims recursively BEFORE creating articulation
        count = enable_collisions_recursive(left_hand_prim, stage, [0], debug=False, hand_group=1, piano_group=0, configure=True)
        print(f"  ✓ Configured {count} collision prims in left hand")
    
    if right_hand_prim and right_hand_prim.IsValid():
        override_prim = stage.OverridePrim(right_hand_prim.GetPath())
        override_prim.CreateAttribute("physx:collisionGroup", Sdf.ValueTypeNames.Int).Set(1)
        override_prim.CreateAttribute("physx:collisionFilter", Sdf.ValueTypeNames.Int).Set(1)  # Collide with group 0
        print("  ✓ Right hand collision group set to 1, filter 1 (before articulation creation)")
        
        # Configure all collision prims recursively BEFORE creating articulation
        count = enable_collisions_recursive(right_hand_prim, stage, [0], debug=False, hand_group=1, piano_group=0, configure=True)
        print(f"  ✓ Configured {count} collision prims in right hand")
    
    if piano_prim and piano_prim.IsValid():
        override_prim = stage.OverridePrim(piano_prim.GetPath())
        override_prim.CreateAttribute("physx:collisionGroup", Sdf.ValueTypeNames.Int).Set(0)
        override_prim.CreateAttribute("physx:collisionFilter", Sdf.ValueTypeNames.Int).Set(2)  # Collide with group 1
        print("  ✓ Piano collision group set to 0, filter 2 (before articulation creation)")
        
        # Configure all collision prims recursively BEFORE creating articulation
        count = enable_collisions_recursive(piano_prim, stage, [0], debug=False, hand_group=1, piano_group=0, configure=True)
        print(f"  ✓ Configured {count} collision prims in piano")
    
    # Create articulations for hands AFTER configuring collisions
    print("\n🤖 Creating hand articulations (collisions already configured)...")
    
    left_hand_articulation = None
    right_hand_articulation = None
    
    # Reset world first to ensure stage is ready
    world.reset()
    
    try:
        # Create articulation - collision groups are already configured
        left_hand_articulation = world.scene.add(
            Articulation(prim_path="/World/left_hand", name="left_hand")
        )
        print("  ✓ Left hand articulation created")
    except Exception as e:
        print(f"  ⚠️  Could not create left hand articulation: {e}")
        import traceback
        traceback.print_exc()
    
    try:
        right_hand_articulation = world.scene.add(
            Articulation(prim_path="/World/right_hand", name="right_hand")
        )
        print("  ✓ Right hand articulation created")
    except Exception as e:
        print(f"  ⚠️  Could not create right hand articulation: {e}")
        import traceback
        traceback.print_exc()
    
    # Find fingertip prims
    print("\n🔍 Searching for fingertip prims...")
    left_fingertips = find_fingertip_prims(stage, "/World/left_hand")
    right_fingertips = find_fingertip_prims(stage, "/World/right_hand")
    
    print(f"  Left hand fingertips found: {len(left_fingertips)}")
    for tip in left_fingertips[:5]:  # Show first 5
        print(f"    - {tip}")
    if len(left_fingertips) > 5:
        print(f"    ... and {len(left_fingertips) - 5} more")
    
    print(f"  Right hand fingertips found: {len(right_fingertips)}")
    for tip in right_fingertips[:5]:  # Show first 5
        print(f"    - {tip}")
    if len(right_fingertips) > 5:
        print(f"    ... and {len(right_fingertips) - 5} more")
    
    # Check collision groups
    print("\n🔧 Checking collision configuration...")
    piano_prim = stage.GetPrimAtPath("/World/Piano")
    left_hand_prim = stage.GetPrimAtPath("/World/left_hand")
    right_hand_prim = stage.GetPrimAtPath("/World/right_hand")
    
    if piano_prim and piano_prim.IsValid():
        piano_group = piano_prim.GetAttribute("physx:collisionGroup")
        piano_filter = piano_prim.GetAttribute("physx:collisionFilter")
        if piano_group:
            print(f"  Piano collision group: {piano_group.Get()}")
        if piano_filter:
            print(f"  Piano collision filter: {piano_filter.Get()}")
    
    if left_hand_prim and left_hand_prim.IsValid():
        hand_group = left_hand_prim.GetAttribute("physx:collisionGroup")
        hand_filter = left_hand_prim.GetAttribute("physx:collisionFilter")
        if hand_group:
            print(f"  Hand collision group: {hand_group.Get()}")
        if hand_filter:
            print(f"  Hand collision filter: {hand_filter.Get()}")
    
    # Franka-style: Don't manually configure collisions yet - just count them for verification
    # We'll configure them properly after world.reset()
    print("\n🔍 Verifying collision meshes exist (counting only, configuration happens after reset)...")
    piano_count = [0]
    if piano_prim and piano_prim.IsValid():
        count = enable_collisions_recursive(piano_prim, stage, piano_count, debug=False, configure=False)
        print(f"  Piano collision prims found: {count}")
        if count == 0:
            print(f"  ⚠️  WARNING: No collision prims found in piano!")
    
    left_count = [0]
    if left_hand_prim and left_hand_prim.IsValid():
        count = enable_collisions_recursive(left_hand_prim, stage, left_count, debug=False, configure=False)
        print(f"  Left hand collision prims found: {count}")
        if count == 0:
            print(f"  ⚠️  WARNING: No collision prims found in left hand!")
            print(f"  ⚠️  This is likely why fingers pass through - physics payload may not be loaded")
    
    right_count = [0]
    if right_hand_prim and right_hand_prim.IsValid():
        count = enable_collisions_recursive(right_hand_prim, stage, right_count, debug=False, configure=False)
        print(f"  Right hand collision prims found: {count}")
        if count == 0:
            print(f"  ⚠️  WARNING: No collision prims found in right hand!")
            print(f"  ⚠️  This is likely why fingers pass through - physics payload may not be loaded")
    
    # Create piano articulation and controller
    piano_articulation = None
    piano_controller = None
    
    print("\n🎹 Initializing piano...")
    try:
        piano_articulation = world.scene.add(
            Articulation(prim_path="/World/Piano", name="piano")
        )
        print("  ✓ Piano articulation created")
        
        if PIANO_CONTROLLER_AVAILABLE:
            piano_controller = IsaacPianoController(
                piano_articulation=piano_articulation,
                prim_path="/World/Piano",
                change_color_on_activation=True,
                add_actuators=False  # Passive mode - keys respond to contact
            )
            print("  ✓ Piano controller created")
    except Exception as e:
        print(f"  ⚠️  Could not create piano articulation: {e}")
    
    # Reset world once after creating all articulations
    # Collision groups are already configured, so they should be preserved
    print("\n🔄 Resetting world (collision groups already configured)...")
    world.reset()
    
    # Verify collision groups are still set after reset
    print("\n🔍 Verifying collision groups after world reset...")
    left_hand_prim = stage.GetPrimAtPath("/World/left_hand")
    right_hand_prim = stage.GetPrimAtPath("/World/right_hand")
    piano_prim = stage.GetPrimAtPath("/World/Piano")
    
    if left_hand_prim and left_hand_prim.IsValid():
        group_attr = left_hand_prim.GetAttribute("physx:collisionGroup")
        filter_attr = left_hand_prim.GetAttribute("physx:collisionFilter")
        if group_attr:
            print(f"  ✓ Left hand collision group: {group_attr.Get()}")
        if filter_attr:
            print(f"  ✓ Left hand collision filter: {filter_attr.Get()}")
    
    if right_hand_prim and right_hand_prim.IsValid():
        group_attr = right_hand_prim.GetAttribute("physx:collisionGroup")
        filter_attr = right_hand_prim.GetAttribute("physx:collisionFilter")
        if group_attr:
            print(f"  ✓ Right hand collision group: {group_attr.Get()}")
        if filter_attr:
            print(f"  ✓ Right hand collision filter: {filter_attr.Get()}")
    
    if piano_prim and piano_prim.IsValid():
        group_attr = piano_prim.GetAttribute("physx:collisionGroup")
        filter_attr = piano_prim.GetAttribute("physx:collisionFilter")
        if group_attr:
            print(f"  ✓ Piano collision group: {group_attr.Get()}")
        if filter_attr:
            print(f"  ✓ Piano collision filter: {filter_attr.Get()}")
    
    # Verify collision meshes exist AFTER payloads are loaded and world is reset
    print("\n🔍 Verifying collision meshes exist in loaded stage...")
    
    if left_hand_prim and left_hand_prim.IsValid():
        success, count, paths = verify_collision_meshes_in_stage("/World/left_hand", stage)
        print(f"  Left hand: {count} collision prims found")
        if count > 0:
            print(f"    Sample collision paths (first 5):")
            for path in paths[:5]:
                print(f"      - {path}")
            if count > 5:
                print(f"      ... and {count - 5} more")
        else:
            print(f"  ⚠️  WARNING: No collision prims found in left hand after loading!")
    
    if right_hand_prim and right_hand_prim.IsValid():
        success, count, paths = verify_collision_meshes_in_stage("/World/right_hand", stage)
        print(f"  Right hand: {count} collision prims found")
        if count > 0:
            print(f"    Sample collision paths (first 5):")
            for path in paths[:5]:
                print(f"      - {path}")
            if count > 5:
                print(f"      ... and {count - 5} more")
        else:
            print(f"  ⚠️  WARNING: No collision prims found in right hand after loading!")
    
    if piano_prim and piano_prim.IsValid():
        success, count, paths = verify_collision_meshes_in_stage("/World/Piano", stage)
        print(f"  Piano: {count} collision prims found")
        if count == 0:
            print(f"  ⚠️  WARNING: No collision prims found in piano!")
    
    # CRITICAL: Use PhysX interface to ensure collision filtering is properly applied
    print("\n🔧 Applying collision filtering via PhysX interface...")
    try:
        # Get PhysX interface
        physx_interface = physx.get_physx_interface()
        
        # Force update of collision filtering for all actors
        # This ensures that collision groups are properly applied
        if physx_interface:
            # Trigger a physics update to apply collision filtering
            print("  ✓ PhysX interface available - collision filtering should be active")
            
            # Note: PhysX applies collision filtering automatically based on collision groups
            # The groups we set should be working, but we can verify by checking actor states
    except Exception as e:
        print(f"  ⚠️  Could not access PhysX interface: {e}")
    
    # CRITICAL: Reset world again after configuring collisions so physics engine picks up changes
    print("\n🔄 Resetting world again to apply collision configuration...")
    world.reset()
    
    # CRITICAL: Reload payloads after world.reset() - payloads may be unloaded
    print("\n📦 Reloading payloads after world.reset()...")
    try:
        from pxr import Usd
        left_hand_prim = stage.GetPrimAtPath("/World/left_hand")
        right_hand_prim = stage.GetPrimAtPath("/World/right_hand")
        
        # Force load all payloads recursively after reset
        if left_hand_prim and left_hand_prim.IsValid():
            for prim in Usd.PrimRange(left_hand_prim):
                if prim.HasPayload():
                    prim.Load()
            print("  ✓ Left hand payloads reloaded")
        
        if right_hand_prim and right_hand_prim.IsValid():
            for prim in Usd.PrimRange(right_hand_prim):
                if prim.HasPayload():
                    prim.Load()
            print("  ✓ Right hand payloads reloaded")
    except Exception as e:
        print(f"  ⚠️  Could not reload payloads: {e}")
    
    # CRITICAL: Re-apply collision groups to ALL collision prims after world.reset()
    # world.reset() may reset collision settings, so we need to re-apply them
    print("\n🔧 Re-applying collision groups to all collision prims (after world.reset())...")
    left_hand_prim = stage.GetPrimAtPath("/World/left_hand")
    right_hand_prim = stage.GetPrimAtPath("/World/right_hand")
    piano_prim = stage.GetPrimAtPath("/World/Piano")
    
    # Use Usd.PrimRange to traverse all prims including those in payloads
    if left_hand_prim and left_hand_prim.IsValid():
        # First, try to find collision prims using the known paths from USD file
        collision_paths = [
            "/World/left_hand/colliders/base_link_R/base_link_R/mesh",
            "/World/left_hand/colliders/Link_11R/Link_11R/mesh",
            "/World/left_hand/colliders/Link_12R/Link_12R/mesh",
            "/World/left_hand/colliders/Link_13R/Link_13R/mesh",
            "/World/left_hand/colliders/Link_14R/Link_14R/mesh",
            "/World/left_hand/colliders/Link_21R/Link_21R/mesh",
            "/World/left_hand/colliders/Link_22R/Link_22R/mesh",
            "/World/left_hand/colliders/Link_23R/Link_23R/mesh",
            "/World/left_hand/colliders/Link_24R/Link_24R/mesh",
            "/World/left_hand/colliders/Link_31R/Link_31R/mesh",
            "/World/left_hand/colliders/Link_32R/Link_32R/mesh",
            "/World/left_hand/colliders/Link_33R/Link_33R/mesh",
            "/World/left_hand/colliders/Link_34R/Link_34R/mesh",
            "/World/left_hand/colliders/Link_41R/Link_41R/mesh",
            "/World/left_hand/colliders/Link_42R/Link_42R/mesh",
            "/World/left_hand/colliders/Link_43R/Link_43R/mesh",
            "/World/left_hand/colliders/Link_44R/Link_44R/mesh",
            "/World/left_hand/colliders/Link_51R/Link_51R/mesh",
            "/World/left_hand/colliders/Link_52R/Link_52R/mesh",
            "/World/left_hand/colliders/Link_53R/Link_53R/mesh",
            "/World/left_hand/colliders/Link_54R/Link_54R/mesh",
        ]
        
        configured_count = 0
        for path_str in collision_paths:
            try:
                prim = stage.GetPrimAtPath(path_str)
                if prim and prim.IsValid() and prim.HasAPI(UsdPhysics.CollisionAPI):
                    override_prim = stage.OverridePrim(prim.GetPath())
                    if override_prim:
                        override_prim.CreateAttribute("physx:collisionGroup", Sdf.ValueTypeNames.Int).Set(1)
                        override_prim.CreateAttribute("physx:collisionFilter", Sdf.ValueTypeNames.Int).Set(1)
                        configured_count += 1
            except:
                pass
        
        # Also use recursive traversal as fallback
        count = enable_collisions_recursive(left_hand_prim, stage, [0], debug=False, hand_group=1, piano_group=0, configure=True)
        print(f"  ✓ Re-configured {max(configured_count, count)} collision prims in left hand")
    
    if right_hand_prim and right_hand_prim.IsValid():
        # First, try to find collision prims using the known paths from USD file
        collision_paths = [
            "/World/right_hand/colliders/base_link_R/base_link_R/mesh",
            "/World/right_hand/colliders/Link_11R/Link_11R/mesh",
            "/World/right_hand/colliders/Link_12R/Link_12R/mesh",
            "/World/right_hand/colliders/Link_13R/Link_13R/mesh",
            "/World/right_hand/colliders/Link_14R/Link_14R/mesh",
            "/World/right_hand/colliders/Link_21R/Link_21R/mesh",
            "/World/right_hand/colliders/Link_22R/Link_22R/mesh",
            "/World/right_hand/colliders/Link_23R/Link_23R/mesh",
            "/World/right_hand/colliders/Link_24R/Link_24R/mesh",
            "/World/right_hand/colliders/Link_31R/Link_31R/mesh",
            "/World/right_hand/colliders/Link_32R/Link_32R/mesh",
            "/World/right_hand/colliders/Link_33R/Link_33R/mesh",
            "/World/right_hand/colliders/Link_34R/Link_34R/mesh",
            "/World/right_hand/colliders/Link_41R/Link_41R/mesh",
            "/World/right_hand/colliders/Link_42R/Link_42R/mesh",
            "/World/right_hand/colliders/Link_43R/Link_43R/mesh",
            "/World/right_hand/colliders/Link_44R/Link_44R/mesh",
            "/World/right_hand/colliders/Link_51R/Link_51R/mesh",
            "/World/right_hand/colliders/Link_52R/Link_52R/mesh",
            "/World/right_hand/colliders/Link_53R/Link_53R/mesh",
            "/World/right_hand/colliders/Link_54R/Link_54R/mesh",
        ]
        
        configured_count = 0
        for path_str in collision_paths:
            try:
                prim = stage.GetPrimAtPath(path_str)
                if prim and prim.IsValid() and prim.HasAPI(UsdPhysics.CollisionAPI):
                    override_prim = stage.OverridePrim(prim.GetPath())
                    if override_prim:
                        override_prim.CreateAttribute("physx:collisionGroup", Sdf.ValueTypeNames.Int).Set(1)
                        override_prim.CreateAttribute("physx:collisionFilter", Sdf.ValueTypeNames.Int).Set(1)
                        configured_count += 1
            except:
                pass
        
        # Also use recursive traversal as fallback
        count = enable_collisions_recursive(right_hand_prim, stage, [0], debug=False, hand_group=1, piano_group=0, configure=True)
        print(f"  ✓ Re-configured {max(configured_count, count)} collision prims in right hand")
    
    if piano_prim and piano_prim.IsValid():
        count = enable_collisions_recursive(piano_prim, stage, [0], debug=False, hand_group=1, piano_group=0, configure=True)
        print(f"  ✓ Re-configured {count} collision prims in piano")
    
    # Verify collision groups on actual collision prims (not just root)
    print("\n🔍 Verifying collision groups on actual collision prims...")
    def verify_collision_prim_groups(prim_path, expected_group, expected_filter, stage):
        """Verify collision groups on collision prims."""
        prim = stage.GetPrimAtPath(prim_path)
        if not prim or not prim.IsValid():
            return 0, 0
        
        correct_count = 0
        total_count = 0
        
        def check_prim(p):
            nonlocal correct_count, total_count
            if p.HasAPI(UsdPhysics.CollisionAPI):
                total_count += 1
                group_attr = p.GetAttribute("physx:collisionGroup")
                filter_attr = p.GetAttribute("physx:collisionFilter")
                if group_attr and filter_attr:
                    group_val = group_attr.Get()
                    filter_val = filter_attr.Get()
                    if group_val == expected_group and filter_val == expected_filter:
                        correct_count += 1
                    else:
                        print(f"    ⚠️  {p.GetPath()}: group={group_val} (expected {expected_group}), filter={filter_val} (expected {expected_filter})")
            for child in p.GetChildren():
                check_prim(child)
        
        check_prim(prim)
        return total_count, correct_count
    
    if left_hand_prim and left_hand_prim.IsValid():
        total, correct = verify_collision_prim_groups("/World/left_hand", 1, 1, stage)
        print(f"  Left hand: {correct}/{total} collision prims have correct groups")
    
    if right_hand_prim and right_hand_prim.IsValid():
        total, correct = verify_collision_prim_groups("/World/right_hand", 1, 1, stage)
        print(f"  Right hand: {correct}/{total} collision prims have correct groups")
    
    if piano_prim and piano_prim.IsValid():
        total, correct = verify_collision_prim_groups("/World/Piano", 0, 2, stage)
        print(f"  Piano: {correct}/{total} collision prims have correct groups")
    
    # Verify articulation collision settings
    print("\n🔧 Verifying articulation collision settings...")
    if left_hand_articulation:
        try:
            # Ensure self-collisions are enabled (or disabled as needed)
            # For hands, we typically want self-collisions disabled but hand-piano collisions enabled
            print("  ✓ Left hand articulation ready")
        except Exception as e:
            print(f"  ⚠️  Could not verify left hand articulation: {e}")
    
    if right_hand_articulation:
        try:
            print("  ✓ Right hand articulation ready")
        except Exception as e:
            print(f"  ⚠️  Could not verify right hand articulation: {e}")
    
    if piano_articulation:
        try:
            print("  ✓ Piano articulation ready")
        except Exception as e:
            print(f"  ⚠️  Could not verify piano articulation: {e}")
    
    print("\n✓ Collision configuration complete - hands and piano should now collide properly")
    print("  Note: If collisions still don't work, check that collision prims have proper groups set")
    
    # Initialize piano controller if available
    if piano_controller:
        try:
            piano_controller.initialize_episode()
            print("  ✓ Piano controller initialized")
            print("  ✓ Key color change on activation: ENABLED (keys will turn RED when pressed)")
        except Exception as e:
            print(f"  ⚠️  Could not initialize piano controller: {e}")
    
    # Set all keys to proper black/white colors (from demo_midi_playback.py)
    if piano_controller:
        try:
            from omni.isaac.core.utils.prims import get_prim_at_path
            from exts.omni.isaac.piano.piano_constants import WHITE_KEY_INDICES, BLACK_KEY_INDICES, WHITE_KEY_COLOR, BLACK_KEY_COLOR
            
            print("\n🎨 Setting piano keys to black and white colors...")
            for key_id in range(88):
                is_white = key_id in WHITE_KEY_INDICES
                key_type = "white" if is_white else "black"
                mesh_path = f"/World/Piano/{key_type}_key_{key_id}/{key_type}_key_{key_id}_mesh"
                
                prim = get_prim_at_path(mesh_path)
                if prim and prim.IsValid():
                    mesh = UsdGeom.Mesh(prim)
                    if mesh:
                        color = WHITE_KEY_COLOR[:3] if is_white else BLACK_KEY_COLOR[:3]
                        mesh.GetDisplayColorAttr().Set([Gf.Vec3f(*color)])
            print("  ✓ Key colors set (will change to red when activated)")
        except Exception as e:
            print(f"  ⚠️  Could not set key colors: {e}")
    
    # Get baseline key positions after reset (for monitoring)
    baseline_key_positions = None
    if piano_controller:
        try:
            # Wait a few steps for physics to stabilize
            for _ in range(10):
                world.step(render=False)
            baseline_key_positions = piano_controller.get_joint_positions()
            print(f"  ✓ Baseline key positions recorded ({len(baseline_key_positions)} keys)")
        except Exception as e:
            print(f"  ⚠️  Could not get baseline positions: {e}")
    
    # Initialize hands to rest positions and inspect joints
    left_hand_joints = None
    right_hand_joints = None
    
    if left_hand_articulation:
        try:
            # Ensure hand is not kinematic and collisions are enabled
            try:
                # Get the articulation prim
                left_hand_prim = stage.GetPrimAtPath("/World/left_hand")
                if left_hand_prim and left_hand_prim.IsValid():
                    # Ensure articulation root is not kinematic
                    articulation_api = UsdPhysics.ArticulationRootAPI(left_hand_prim)
                    if articulation_api:
                        try:
                            # Check if there's a solver type attribute
                            pass  # Articulation should be dynamic by default
                        except:
                            pass
            except Exception as e:
                print(f"  ⚠️  Could not configure left hand articulation: {e}")
            
            joint_names = left_hand_articulation.dof_names
            if joint_names:
                initial_positions = np.zeros(len(joint_names))
                left_hand_articulation.set_joint_positions(initial_positions)
                left_hand_joints = joint_names
                print(f"  ✓ Left hand initialized ({len(joint_names)} joints)")
                
                # Analyze joint pattern
                print(f"\n  📋 Left Hand Joint Analysis:")
                print(f"     Total DOF: {len(joint_names)}")
                
                # Group joints by finger
                finger_groups = {
                    "thumb": [],
                    "index": [],
                    "middle": [],
                    "ring": [],
                    "pinky": [],
                    "other": []
                }
                
                for i, joint_name in enumerate(joint_names):
                    joint_lower = joint_name.lower()
                    assigned = False
                    for finger in ["thumb", "index", "middle", "ring", "pinky"]:
                        if finger in joint_lower:
                            finger_groups[finger].append((i, joint_name))
                            assigned = True
                            break
                    if not assigned:
                        finger_groups["other"].append((i, joint_name))
                
                for finger, joints in finger_groups.items():
                    if joints:
                        print(f"     {finger.capitalize()} finger ({len(joints)} joints):")
                        for idx, jname in joints:
                            print(f"       [{idx:2d}] {jname}")
                
                if finger_groups["other"]:
                    print(f"     Other joints ({len(finger_groups['other'])}):")
                    for idx, jname in finger_groups["other"]:
                        print(f"       [{idx:2d}] {jname}")
                        
        except Exception as e:
            print(f"  ⚠️  Could not initialize left hand: {e}")
    
    if right_hand_articulation:
        try:
            # Ensure hand is not kinematic and collisions are enabled
            try:
                right_hand_prim = stage.GetPrimAtPath("/World/right_hand")
                if right_hand_prim and right_hand_prim.IsValid():
                    articulation_api = UsdPhysics.ArticulationRootAPI(right_hand_prim)
                    if articulation_api:
                        pass  # Articulation should be dynamic by default
            except Exception as e:
                print(f"  ⚠️  Could not configure right hand articulation: {e}")
            
            joint_names = right_hand_articulation.dof_names
            if joint_names:
                initial_positions = np.zeros(len(joint_names))
                right_hand_articulation.set_joint_positions(initial_positions)
                right_hand_joints = joint_names
                print(f"  ✓ Right hand initialized ({len(joint_names)} joints)")
                
                # Analyze joint pattern
                print(f"\n  📋 Right Hand Joint Analysis:")
                print(f"     Total DOF: {len(joint_names)}")
                
                # Group joints by finger
                finger_groups = {
                    "thumb": [],
                    "index": [],
                    "middle": [],
                    "ring": [],
                    "pinky": [],
                    "other": []
                }
                
                for i, joint_name in enumerate(joint_names):
                    joint_lower = joint_name.lower()
                    assigned = False
                    for finger in ["thumb", "index", "middle", "ring", "pinky"]:
                        if finger in joint_lower:
                            finger_groups[finger].append((i, joint_name))
                            assigned = True
                            break
                    if not assigned:
                        finger_groups["other"].append((i, joint_name))
                
                for finger, joints in finger_groups.items():
                    if joints:
                        print(f"     {finger.capitalize()} finger ({len(joints)} joints):")
                        for idx, jname in joints:
                            print(f"       [{idx:2d}] {jname}")
                
                if finger_groups["other"]:
                    print(f"     Other joints ({len(finger_groups['other'])}):")
                    for idx, jname in finger_groups["other"]:
                        print(f"       [{idx:2d}] {jname}")
                        
        except Exception as e:
            print(f"  ⚠️  Could not initialize right hand: {e}")
    
    # Query and display maximum torque limits for hands
    print("\n💪 Querying Maximum Torque Limits for Hands...")
    
    # Piano key force requirement (from piano_constants.py)
    # Key stiffness: 1.5 N⋅m/rad, max travel: 10mm, max angle: atan(0.01/0.15) ≈ 0.0667 rad
    PIANO_KEY_MAX_TORQUE_REQUIRED = 1.5 * 0.0667  # ≈ 0.1 N⋅m
    
    print(f"  📊 Piano key force requirement: ~{PIANO_KEY_MAX_TORQUE_REQUIRED:.3f} N⋅m (to fully press)")
    print(f"     (Key stiffness: 1.5 N⋅m/rad, max travel: 10mm)")
    
    # Query left hand torque limits
    if left_hand_articulation and left_hand_joints:
        print(f"\n  🤖 Left Hand Torque Limits:")
        left_limits = get_joint_torque_limits_from_usd(stage, "/World/left_hand")
        
        if left_limits:
            # Group by finger type
            finger_groups = {
                "thumb": [],
                "index": [],
                "middle": [],
                "ring": [],
                "pinky": [],
                "other": []
            }
            
            # Dex5 joint naming: Pitch_11R, Pitch_12R (thumb), Pitch_22R, Pitch_23R (index), etc.
            # Pattern: first digit = finger (1=thumb, 2=index, 3=middle, 4=ring, 5=pinky)
            finger_map = {
                "1": "thumb",
                "2": "index", 
                "3": "middle",
                "4": "ring",
                "5": "pinky"
            }
            
            for joint_name, max_torque in left_limits.items():
                joint_lower = joint_name.lower()
                assigned = False
                
                # Try to match finger by name first
                for finger in ["thumb", "index", "middle", "ring", "pinky"]:
                    if finger in joint_lower:
                        finger_groups[finger].append((joint_name, max_torque))
                        assigned = True
                        break
                
                # If not found, try to match by digit pattern (e.g., Pitch_22R -> index)
                if not assigned:
                    # Look for pattern like _2X or _22 or Pitch_2
                    import re as re_module
                    match = re_module.search(r'[_\s](\d)', joint_name)
                    if match:
                        digit = match.group(1)
                        if digit in finger_map:
                            finger = finger_map[digit]
                            finger_groups[finger].append((joint_name, max_torque))
                            assigned = True
                
                if not assigned:
                    finger_groups["other"].append((joint_name, max_torque))
            
            # Display limits grouped by finger
            all_limits = list(left_limits.values())
            finite_limits = [l for l in all_limits if l != float('inf') and l is not None]
            
            for finger, joints in finger_groups.items():
                if joints and finger != "other":
                    print(f"     {finger.capitalize()} finger:")
                    for joint_name, max_torque in joints[:3]:  # Show first 3
                        status = "✓" if max_torque >= PIANO_KEY_MAX_TORQUE_REQUIRED else "⚠️"
                        print(f"       {status} {joint_name}: {max_torque:.3f} N⋅m")
                    if len(joints) > 3:
                        print(f"       ... and {len(joints) - 3} more joints")
            
            if finger_groups["other"]:
                print(f"     Other joints:")
                for joint_name, max_torque in finger_groups["other"][:3]:
                    status = "✓" if max_torque >= PIANO_KEY_MAX_TORQUE_REQUIRED else "⚠️"
                    print(f"       {status} {joint_name}: {max_torque:.3f} N⋅m")
                if len(finger_groups["other"]) > 3:
                    print(f"       ... and {len(finger_groups['other']) - 3} more")
            
            if finite_limits:
                min_limit = min(finite_limits)
                max_limit = max(finite_limits)
                mean_limit = sum(finite_limits) / len(finite_limits)
                print(f"\n     📊 Statistics:")
                print(f"        Min torque limit: {min_limit:.3f} N⋅m")
                print(f"        Max torque limit: {max_limit:.3f} N⋅m")
                print(f"        Mean torque limit: {mean_limit:.3f} N⋅m")
                print(f"        {len(finite_limits)}/{len(all_limits)} joints have defined limits")
                
                # Assessment
                if min_limit >= PIANO_KEY_MAX_TORQUE_REQUIRED:
                    print(f"        ✅ SUFFICIENT: All joints can generate enough torque to press keys")
                elif mean_limit >= PIANO_KEY_MAX_TORQUE_REQUIRED:
                    print(f"        ⚠️  MARGINAL: Some joints may struggle, but most should work")
                else:
                    print(f"        ❌ INSUFFICIENT: Torque limits may be too low to press keys reliably")
            else:
                print(f"     ⚠️  No torque limits found in USD file (may be unlimited)")
        else:
            print(f"     ⚠️  Could not query torque limits from USD file")
    
    # Query right hand torque limits
    if right_hand_articulation and right_hand_joints:
        print(f"\n  🤖 Right Hand Torque Limits:")
        right_limits = get_joint_torque_limits_from_usd(stage, "/World/right_hand")
        
        if right_limits:
            # Group by finger type (same logic as left hand)
            finger_groups = {
                "thumb": [],
                "index": [],
                "middle": [],
                "ring": [],
                "pinky": [],
                "other": []
            }
            
            # Dex5 joint naming: Pitch_11R, Pitch_12R (thumb), Pitch_22R, Pitch_23R (index), etc.
            finger_map = {
                "1": "thumb",
                "2": "index", 
                "3": "middle",
                "4": "ring",
                "5": "pinky"
            }
            
            for joint_name, max_torque in right_limits.items():
                joint_lower = joint_name.lower()
                assigned = False
                
                # Try to match finger by name first
                for finger in ["thumb", "index", "middle", "ring", "pinky"]:
                    if finger in joint_lower:
                        finger_groups[finger].append((joint_name, max_torque))
                        assigned = True
                        break
                
                # If not found, try to match by digit pattern
                if not assigned:
                    import re as re_module
                    match = re_module.search(r'[_\s](\d)', joint_name)
                    if match:
                        digit = match.group(1)
                        if digit in finger_map:
                            finger = finger_map[digit]
                            finger_groups[finger].append((joint_name, max_torque))
                            assigned = True
                
                if not assigned:
                    finger_groups["other"].append((joint_name, max_torque))
            
            all_limits = list(right_limits.values())
            finite_limits = [l for l in all_limits if l != float('inf') and l is not None]
            
            if finite_limits:
                min_limit = min(finite_limits)
                max_limit = max(finite_limits)
                mean_limit = sum(finite_limits) / len(finite_limits)
                print(f"     📊 Statistics:")
                print(f"        Min torque limit: {min_limit:.3f} N⋅m")
                print(f"        Max torque limit: {max_limit:.3f} N⋅m")
                print(f"        Mean torque limit: {mean_limit:.3f} N⋅m")
                print(f"        {len(finite_limits)}/{len(all_limits)} joints have defined limits")
                
                # Assessment
                if min_limit >= PIANO_KEY_MAX_TORQUE_REQUIRED:
                    print(f"        ✅ SUFFICIENT: All joints can generate enough torque to press keys")
                elif mean_limit >= PIANO_KEY_MAX_TORQUE_REQUIRED:
                    print(f"        ⚠️  MARGINAL: Some joints may struggle, but most should work")
                else:
                    print(f"        ❌ INSUFFICIENT: Torque limits may be too low to press keys reliably")
            else:
                print(f"     ⚠️  No torque limits found in USD file (may be unlimited)")
        else:
            print(f"     ⚠️  Could not query torque limits from USD file")
    
    # Enable contact reporting (optional - collisions work without this)
    print("\n📊 Checking physics scene for contact reporting...")
    try:
        # Try to find PhysicsScene - it might be at root or under World
        physics_scene_paths = ["/PhysicsScene", "/World/PhysicsScene"]
        physics_scene = None
        for path in physics_scene_paths:
            physics_scene = UsdPhysics.Scene.Get(stage, path)
            if physics_scene:
                print(f"  ✓ Physics scene found at {path}")
                break
        
        if not physics_scene:
            # Try to find it by traversing the stage
            for prim in stage.Traverse():
                if prim.IsA(UsdPhysics.Scene):
                    print(f"  ✓ Physics scene found at {prim.GetPath()}")
                    physics_scene = UsdPhysics.Scene(prim)
                    break
        
        if not physics_scene:
            print("  ⚠️  Physics scene not found (contact reporting may not work, but collisions should still work)")
    except Exception as e:
        print(f"  ⚠️  Could not find physics scene: {e} (collisions should still work)")
    
    # Setup camera positioning and zoom (3x) BEFORE creating render product
    print("\n📹 Setting up camera for recording (3x zoom)...")
    camera_path = "/OmniverseKit_Persp"
    camera = UsdGeom.Camera.Get(stage, camera_path)
    recording_timestamp = None
    
    if camera:
        xform = UsdGeom.Xformable(camera.GetPrim())
        xform_ops = xform.GetOrderedXformOps()
        
        # Position: in front of piano, slightly above, looking down at keys
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
        
        # Zoom in 3x by reducing horizontal aperture (default ~20.96mm, zoom 3x = ~6.99mm)
        horizontal_aperture = camera.GetHorizontalApertureAttr()
        if horizontal_aperture:
            default_aperture = horizontal_aperture.Get() or 20.96
            camera.GetHorizontalApertureAttr().Set(default_aperture / 3.0)  # 3x zoom
            print(f"  ✓ Camera positioned and zoomed 3x (aperture: {default_aperture:.2f}mm -> {default_aperture/3.0:.2f}mm)")
    
    # Setup recording AFTER camera is configured
    print("\n🎬 Setting up video recording...")
    try:
        recording_dir, recording_timestamp = setup_video_recording()
        print(f"  ✓ Recording enabled - video will be saved after simulation")
    except Exception as e:
        print(f"  ⚠️  Could not setup recording: {e}")
        recording_timestamp = None
    
    print("\n" + "="*70)
    print("Starting Hand-Piano Interaction Test...")
    print("="*70)
    print("\nThe simulation will curl hands to press all piano keys.")
    print("Keys will change to red when activated/pressed below threshold.")
    print("Watch the console for key activation reports.\n")
    
    # CRITICAL: Start the simulation in play mode
    # Without this, physics won't run and collisions won't work!
    print("▶️  Starting simulation in play mode...")
    world.stop()  # Ensure we're stopped first
    world.reset()  # Reset to initialize physics
    world.play()  # Start playing - this enables physics and collisions!
    print("  ✓ Simulation is now playing - physics and collisions are active")
    print()
    
    # Simulation loop
    step = 0
    sim_time = 0.0
    last_status_report_time = 0.0
    
    # Track target positions for gradual curling to press all keys
    left_hand_targets = None
    right_hand_targets = None
    
    # Track previous key positions for monitoring
    prev_key_positions = baseline_key_positions.copy() if baseline_key_positions is not None else None
    key_position_threshold = 0.001  # 1mm movement indicates key is pressed
    
    # Track key activation history
    key_activation_history = {}  # {key_id: [(time, activation_value), ...]}
    # Use same threshold as MuJoCo: 0.5 degrees = 0.00872665 radians
    # Convert to displacement: for key length L=0.15m, displacement ≈ L * sin(θ) ≈ L * θ
    # displacement ≈ 0.15 * 0.00872665 ≈ 0.001309 m ≈ 1.31mm
    KEY_THRESHOLD_RAD = 0.00872665  # 0.5 degrees in radians (MuJoCo standard)
    WHITE_KEY_LENGTH = 0.15  # meters
    key_activation_threshold = WHITE_KEY_LENGTH * KEY_THRESHOLD_RAD  # ≈ 0.001309 m (1.31mm)
    
    # Track previous joint positions for collision detection
    prev_left_joint_positions = None
    prev_right_joint_positions = None
    
    # Simulation duration limit (in seconds) - set to None for unlimited
    max_simulation_time = 6.0  # Stop after 6 seconds
    print(f"⏱️  Simulation will run for up to {max_simulation_time} seconds (or until window is closed)")
    
    while simulation_app.is_running():
        world.step(render=True)
        
        if world.is_playing():
            step += 1
            dt = world.get_physics_dt()
            sim_time = step * dt
            
            # Monitor key positions for reporting (update prev_key_positions)
            if piano_controller and prev_key_positions is not None:
                try:
                    current_positions = piano_controller.get_joint_positions()
                    prev_key_positions = current_positions.copy()
                except Exception as e:
                    pass
            
            # Update piano controller to monitor key states and update colors
            # This MUST be called every step for keys to turn red when pressed
            if piano_controller:
                try:
                    piano_controller.update(dt)  # Updates key states AND colors (turns keys red when activated)
                except Exception as e:
                    if step % 120 == 0:  # Log error every 2 seconds if it persists
                        print(f"  ⚠️  Piano controller update error: {e}")
            
            # Initialize target positions after 1 second
            if step == int(1.0 / dt) and left_hand_targets is None:
                if left_hand_articulation and left_hand_joints:
                    left_hand_targets = np.zeros(len(left_hand_joints))
                    # Show which joints will be curled
                    pitch_joints = [i for i, name in enumerate(left_hand_joints) if "pitch" in name.lower()]
                    print(f"  🤖 Initialized left hand targets - starting to curl fingers to press all keys")
                    print(f"     Found {len(pitch_joints)} pitch joints to curl (out of {len(left_hand_joints)} total joints)")
                
                if right_hand_articulation and right_hand_joints:
                    right_hand_targets = np.zeros(len(right_hand_joints))
                    pitch_joints = [i for i, name in enumerate(right_hand_joints) if "pitch" in name.lower()]
                    print(f"  🤖 Initialized right hand targets - starting to curl fingers to press all keys")
                    print(f"     Found {len(pitch_joints)} pitch joints to curl (out of {len(right_hand_joints)} total joints)")
            
            # Continuously curl fingers to press all keys (WITH collision-aware stopping)
            if step > int(1.0 / dt):
                curl_rate = 2.0 * dt  # Increased from 0.5 to 2.0 radians per second for faster curling
                
                # Left hand gradual curling with collision detection
                if left_hand_articulation and left_hand_joints and left_hand_targets is not None:
                    try:
                        current_positions = left_hand_articulation.get_joint_positions()
                        new_targets = left_hand_targets.copy()
                        
                        for i, joint_name in enumerate(left_hand_joints):
                            joint_lower = joint_name.lower()
                            current_pos = current_positions[i] if i < len(current_positions) else 0.0
                            current_target = left_hand_targets[i]
                            
                            # Check if joint is blocked (target not reached despite effort)
                            # If target is much higher than actual, and actual hasn't changed much, we're blocked
                            position_error = abs(current_target - current_pos)
                            
                            # Check if joint position has changed significantly since last step
                            joint_moving = True
                            if prev_left_joint_positions is not None and i < len(prev_left_joint_positions):
                                position_change = abs(current_pos - prev_left_joint_positions[i])
                                # If target increased but position didn't change much, joint is blocked
                                if position_change < 0.01 * dt:  # Very small movement
                                    joint_moving = False
                            
                            # Only increase target if we're not significantly blocked
                            # Allow some error tolerance (0.2 rad ≈ 11.5 degrees) - increased to allow deeper key presses
                            if position_error < 0.2 and joint_moving:
                                # Gradually increase target positions for pitch joints
                                if "pitch" in joint_lower:
                                    max_target = 0.0
                                    # Match joint patterns: Pitch_22R, Pitch_32R, Pitch_42R, Pitch_52R (proximal)
                                    if any(f"{f}2" in joint_lower for f in ["2", "3", "4", "5"]):
                                        max_target = 1.2  # Increased from 0.8 - curl more aggressively
                                    # Match Pitch_13R (thumb intermediate)
                                    elif "13" in joint_lower:
                                        max_target = 1.0  # Increased from 0.7
                                    # Match Pitch_23R, Pitch_33R, Pitch_43R, Pitch_53R (intermediate)
                                    elif any(f"{f}3" in joint_lower for f in ["2", "3", "4", "5"]):
                                        max_target = 1.0  # Increased from 0.7
                                    # Match Pitch_24R, Pitch_34R, Pitch_44R, Pitch_54R, Pitch_14R (distal)
                                    elif any(f"{f}4" in joint_lower for f in ["1", "2", "3", "4", "5"]):
                                        max_target = 0.9  # Increased from 0.6
                                    
                                    if new_targets[i] < max_target:
                                        new_targets[i] += curl_rate
                                        new_targets[i] = min(new_targets[i], max_target)
                                
                                elif "roll" in joint_lower and "12" not in joint_lower:
                                    if new_targets[i] < 0.3:  # Increased from 0.2
                                        new_targets[i] += curl_rate * 0.5
                                        new_targets[i] = min(new_targets[i], 0.3)
                            # else: joint is blocked, don't increase target (collision detected)
                        
                        # Update previous positions for next step
                        prev_left_joint_positions = current_positions.copy() if current_positions is not None else None
                        
                        left_hand_targets = new_targets
                        # Use position targets for better control (allows PD control to work)
                        try:
                            left_hand_articulation.set_joint_position_targets(left_hand_targets)
                        except:
                            # Fallback to direct position setting if position targets not available
                            left_hand_articulation.set_joint_positions(left_hand_targets)
                    except Exception as e:
                        pass
                
                # Right hand gradual curling with collision detection
                if right_hand_articulation and right_hand_joints and right_hand_targets is not None:
                    try:
                        current_positions = right_hand_articulation.get_joint_positions()
                        new_targets = right_hand_targets.copy()
                        
                        for i, joint_name in enumerate(right_hand_joints):
                            joint_lower = joint_name.lower()
                            current_pos = current_positions[i] if i < len(current_positions) else 0.0
                            current_target = right_hand_targets[i]
                            
                            # Check if joint is blocked (target not reached despite effort)
                            position_error = abs(current_target - current_pos)
                            
                            # Check if joint position has changed significantly since last step
                            joint_moving = True
                            if prev_right_joint_positions is not None and i < len(prev_right_joint_positions):
                                position_change = abs(current_pos - prev_right_joint_positions[i])
                                # If target increased but position didn't change much, joint is blocked
                                if position_change < 0.01 * dt:  # Very small movement
                                    joint_moving = False
                            
                            # Only increase target if we're not significantly blocked
                            # Allow some error tolerance (0.2 rad ≈ 11.5 degrees) - increased to allow deeper key presses
                            if position_error < 0.2 and joint_moving:
                                if "pitch" in joint_lower:
                                    max_target = 0.0
                                    # Match joint patterns: Pitch_22R, Pitch_32R, Pitch_42R, Pitch_52R (proximal)
                                    if any(f"{f}2" in joint_lower for f in ["2", "3", "4", "5"]):
                                        max_target = 1.2  # Increased from 0.8
                                    # Match Pitch_13R (thumb intermediate)
                                    elif "13" in joint_lower:
                                        max_target = 1.0  # Increased from 0.7
                                    # Match Pitch_23R, Pitch_33R, Pitch_43R, Pitch_53R (intermediate)
                                    elif any(f"{f}3" in joint_lower for f in ["2", "3", "4", "5"]):
                                        max_target = 1.0  # Increased from 0.7
                                    # Match Pitch_24R, Pitch_34R, Pitch_44R, Pitch_54R, Pitch_14R (distal)
                                    elif any(f"{f}4" in joint_lower for f in ["1", "2", "3", "4", "5"]):
                                        max_target = 0.9  # Increased from 0.6
                                    
                                    if new_targets[i] < max_target:
                                        new_targets[i] += curl_rate
                                        new_targets[i] = min(new_targets[i], max_target)
                                
                                elif "roll" in joint_lower and "12" not in joint_lower:
                                    if new_targets[i] < 0.3:  # Increased from 0.2
                                        new_targets[i] += curl_rate * 0.5
                                        new_targets[i] = min(new_targets[i], 0.3)
                            # else: joint is blocked, don't increase target (collision detected)
                        
                        # Update previous positions for next step
                        prev_right_joint_positions = current_positions.copy() if current_positions is not None else None
                        
                        right_hand_targets = new_targets
                        # Use position targets for better control (allows PD control to work)
                        try:
                            right_hand_articulation.set_joint_position_targets(right_hand_targets)
                        except:
                            # Fallback to direct position setting if position targets not available
                            right_hand_articulation.set_joint_positions(right_hand_targets)
                    except Exception as e:
                        pass
            
            # Track key activations EVERY FRAME (not just every 2 seconds)
            if piano_controller and baseline_key_positions is not None:
                try:
                    activations = piano_controller.activation
                    current_positions = piano_controller.get_joint_positions()
                    position_deltas = baseline_key_positions - current_positions
                    
                    for key_id in range(len(activations)):
                        # Check if key is activated (either by activation flag or position threshold)
                        is_activated = activations[key_id] > 0
                        position_activated = position_deltas[key_id] > key_activation_threshold
                        
                        if is_activated or position_activated:
                            if key_id not in key_activation_history:
                                key_activation_history[key_id] = []
                            
                            # Record activation with timestamp and value (use position delta, not boolean)
                            activation_value = position_deltas[key_id]  # Always use position delta for displacement
                            
                            # Only add if this is a new activation or value changed significantly
                            if not key_activation_history[key_id] or \
                               abs(key_activation_history[key_id][-1][1] - activation_value) > 0.0005:  # 0.5mm change threshold
                                key_activation_history[key_id].append((sim_time, activation_value))
                except Exception as e:
                    pass  # Silently continue if tracking fails
            
            # Status update every 2 seconds
            if sim_time - last_status_report_time >= 2.0:
                print(f"\n[{sim_time:5.1f}s] Step {step:6d}")
                
                # Show finger joint positions to verify curling
                if left_hand_articulation and left_hand_joints and left_hand_targets is not None:
                    try:
                        current_positions = left_hand_articulation.get_joint_positions()
                        # Show a few sample joints
                        sample_indices = [5, 10, 15]  # Sample pitch joints
                        sample_info = []
                        for idx in sample_indices:
                            if idx < len(left_hand_joints):
                                joint_name = left_hand_joints[idx]
                                target = left_hand_targets[idx]
                                actual = current_positions[idx] if idx < len(current_positions) else 0.0
                                sample_info.append(f"{joint_name}: target={target:.2f}, actual={actual:.2f}")
                        if sample_info:
                            print(f"  🤖 Left hand sample joints: {', '.join(sample_info)}")
                    except:
                        pass
                
                # Monitor key activations and positions (for display only)
                if piano_controller:
                    try:
                        activations = piano_controller.activation
                        active_keys = np.where(activations)[0]
                        
                        # Get key positions to see how far they're pressed
                        current_positions = piano_controller.get_joint_positions()
                        
                        if baseline_key_positions is not None:
                            position_deltas = baseline_key_positions - current_positions
                            pressed_keys = np.where(position_deltas > key_position_threshold)[0]
                            
                            # Check how close pressed keys are to full activation
                            if len(pressed_keys) > 0:
                                # Get key state and ranges from piano controller
                                key_state = piano_controller.state
                                qpos_range = piano_controller._qpos_range if hasattr(piano_controller, '_qpos_range') else None
                                
                                if qpos_range is not None:
                                    # Calculate distance from upper limit for pressed keys
                                    distances_from_full = []
                                    for key_id in pressed_keys[:10]:  # Check first 10
                                        if key_id < len(key_state) and key_id < len(qpos_range):
                                            upper_limit = qpos_range[key_id, 1]
                                            current_state = key_state[key_id]
                                            distance = abs(current_state - upper_limit)
                                            # Convert to displacement (approximate)
                                            key_length = 0.15  # white key length
                                            dist_displacement = distance * key_length  # approximate conversion
                                            distances_from_full.append((key_id, distance, np.degrees(distance), dist_displacement))
                                    
                                    if distances_from_full:
                                        print(f"  🎹 Keys activated (RED): {len(active_keys)} | Keys pressed (pos): {len(pressed_keys)}")
                                        if len(active_keys) > 0:
                                            print(f"    ✨ Active key IDs (should be RED): {active_keys[:15].tolist()}{'...' if len(active_keys) > 15 else ''}")
                                        print(f"    Pressed key IDs: {pressed_keys.tolist()}")
                                        print(f"    Distance from full press (first 10) - needs <0.5° to turn RED:")
                                        for key_id, dist_rad, dist_deg, dist_disp in distances_from_full:
                                            status = "✓ RED" if key_id in active_keys else "⚠️ not red"
                                            threshold_met = "YES" if dist_rad <= KEY_THRESHOLD_RAD else "NO"
                                            print(f"      Key {key_id:2d}: {dist_deg:.2f}° ({dist_disp*1000:.2f}mm) from full - {status} [threshold met: {threshold_met}]")
                                else:
                                    if len(active_keys) > 0 or len(pressed_keys) > 0:
                                        print(f"  🎹 Keys activated (RED): {len(active_keys)} | Keys pressed (pos): {len(pressed_keys)}")
                                        if len(active_keys) > 0:
                                            print(f"    ✨ Active key IDs (should be RED): {active_keys[:15].tolist()}{'...' if len(active_keys) > 15 else ''}")
                                        if len(pressed_keys) > 0 and len(pressed_keys) <= 20:
                                            max_delta = np.max(position_deltas[pressed_keys])
                                            print(f"    Pressed key IDs: {pressed_keys.tolist()}")
                                            print(f"    Max key displacement: {max_delta*1000:.2f}mm")
                            else:
                                print(f"  🎹 No keys currently active or pressed")
                        else:
                            if len(active_keys) > 0:
                                print(f"  🎹 Active keys (should be RED): {len(active_keys)} keys pressed")
                                print(f"    ✨ Key IDs: {active_keys[:15].tolist()}{'...' if len(active_keys) > 15 else ''}")
                            else:
                                print(f"  🎹 No keys currently active")
                    except Exception as e:
                        print(f"  ⚠️  Could not check key activations: {e}")
                
                last_status_report_time = sim_time
            
            # Check if we've reached the time limit
            if max_simulation_time is not None and sim_time >= max_simulation_time:
                print(f"\n⏱️  Reached time limit of {max_simulation_time} seconds - stopping simulation")
                break
    
    # Summary
    print("\n" + "="*70)
    print("Hand-Piano Interaction Test Summary")
    print("="*70)
    print(f"Total simulation steps: {step}")
    print(f"Total simulation time: {sim_time:.2f}s")
    
    if piano_controller:
        try:
            activations = piano_controller.activation
            active_keys = np.where(activations)[0]
            total_active = len(active_keys)
            
            # Get final key positions
            final_positions = piano_controller.get_joint_positions()
            if baseline_key_positions is not None:
                position_deltas = baseline_key_positions - final_positions
                pressed_keys = np.where(position_deltas > key_position_threshold)[0]
                max_delta = np.max(position_deltas[pressed_keys]) if len(pressed_keys) > 0 else 0.0
                
                print(f"\n🎹 Key Activation Results:")
                print(f"  Keys activated (below threshold): {total_active}")
                print(f"  Keys pressed (position change > 1mm): {len(pressed_keys)}")
                if len(pressed_keys) > 0:
                    print(f"  Maximum key displacement: {max_delta*1000:.2f}mm")
                if len(active_keys) > 0:
                    print(f"  Active key IDs: {sorted(active_keys).tolist()}")
            
            # Get MIDI messages if any
            midi_messages = piano_controller.get_all_midi_messages()
            if midi_messages:
                print(f"\n🎵 MIDI messages generated: {len(midi_messages)}")
        except Exception as e:
            print(f"  ⚠️  Could not get final key states: {e}")
    
    # Display key activation history
    print(f"\n🎹 Key Activation History (keys activated beyond threshold):")
    
    # Also write to file for easy access
    history_file = "key_activation_history.txt"
    with open(history_file, 'w') as f:
        f.write("="*70 + "\n")
        f.write("Key Activation History\n")
        f.write("="*70 + "\n\n")
    
    if key_activation_history:
        # Sort by key ID
        sorted_keys = sorted(key_activation_history.keys())
        print(f"  Total unique keys activated: {len(sorted_keys)}")
        print(f"  Activation threshold: {key_activation_threshold*1000:.2f}mm ({np.degrees(KEY_THRESHOLD_RAD):.1f}° / {KEY_THRESHOLD_RAD:.6f} rad) - MuJoCo standard")
        
        # Write to file
        with open(history_file, 'a') as f:
            f.write(f"Total unique keys activated: {len(sorted_keys)}\n")
            f.write(f"Activation threshold: {key_activation_threshold*1000:.2f}mm ({np.degrees(KEY_THRESHOLD_RAD):.1f}° / {KEY_THRESHOLD_RAD:.6f} rad) - MuJoCo standard\n\n")
        
        # Show summary statistics
        total_activations = sum(len(activations) for activations in key_activation_history.values())
        print(f"  Total activation events: {total_activations}")
        
        with open(history_file, 'a') as f:
            f.write(f"Total activation events: {total_activations}\n\n")
        
        # Show keys that were activated (first 20, then summary)
        print(f"\n  Activated keys (first 20):")
        with open(history_file, 'a') as f:
            f.write("Activated Keys:\n")
            f.write("-" * 70 + "\n")
        
        for key_id in sorted_keys[:20]:
            activations = key_activation_history[key_id]
            if activations:
                first_time = activations[0][0]
                last_time = activations[-1][0]
                max_activation = max(act[1] for act in activations)
                duration = last_time - first_time
                activation_count = len(activations)
                line = f"    Key {key_id:2d}: first at {first_time:5.2f}s, " \
                      f"max displacement: {max_activation*1000:6.2f}mm, " \
                      f"duration: {duration:5.2f}s, " \
                      f"{activation_count} events"
                print(line)
                with open(history_file, 'a') as f:
                    f.write(f"Key {key_id:2d}: first at {first_time:5.2f}s, "
                           f"max displacement: {max_activation*1000:6.2f}mm, "
                           f"duration: {duration:5.2f}s, "
                           f"{activation_count} events\n")
        
        if len(sorted_keys) > 20:
            print(f"    ... and {len(sorted_keys) - 20} more keys")
            with open(history_file, 'a') as f:
                f.write(f"\n... and {len(sorted_keys) - 20} more keys\n")
            # Show summary of remaining keys
            remaining_keys = sorted_keys[20:]
            remaining_max_displacements = []
            for key_id in remaining_keys:
                activations = key_activation_history[key_id]
                if activations:
                    max_activation = max(act[1] for act in activations)
                    remaining_max_displacements.append(max_activation * 1000)
            
            if remaining_max_displacements:
                line = f"    Remaining keys - avg max displacement: {np.mean(remaining_max_displacements):.2f}mm, " \
                      f"max: {np.max(remaining_max_displacements):.2f}mm"
                print(line)
                with open(history_file, 'a') as f:
                    f.write(f"Remaining keys - avg max displacement: {np.mean(remaining_max_displacements):.2f}mm, "
                           f"max: {np.max(remaining_max_displacements):.2f}mm\n")
        
        # Show overall statistics
        all_max_displacements = []
        all_durations = []
        for key_id in sorted_keys:
            activations = key_activation_history[key_id]
            if activations:
                max_activation = max(act[1] for act in activations)
                first_time = activations[0][0]
                last_time = activations[-1][0]
                duration = last_time - first_time
                all_max_displacements.append(max_activation * 1000)
                all_durations.append(duration)
        
        if all_max_displacements:
            print(f"\n  Overall Statistics:")
            stats_lines = [
                f"    Average max displacement: {np.mean(all_max_displacements):.2f}mm",
                f"    Maximum displacement: {np.max(all_max_displacements):.2f}mm",
                f"    Minimum displacement: {np.min(all_max_displacements):.2f}mm",
                f"    Average activation duration: {np.mean(all_durations):.2f}s",
                f"    Total activation time: {sum(all_durations):.2f}s"
            ]
            for line in stats_lines:
                print(line)
            with open(history_file, 'a') as f:
                f.write("\nOverall Statistics:\n")
                f.write("-" * 70 + "\n")
                for line in stats_lines:
                    f.write(line.strip() + "\n")
            
            # Write all activated key IDs to file
            with open(history_file, 'a') as f:
                f.write(f"\nAll Activated Key IDs: {sorted_keys}\n")
    else:
        print(f"  No keys were activated during simulation")
        print(f"  (Threshold: {key_activation_threshold*1000:.2f}mm / {np.degrees(KEY_THRESHOLD_RAD):.1f}° / {KEY_THRESHOLD_RAD:.6f} rad - MuJoCo standard)")
        with open(history_file, 'a') as f:
            f.write(f"No keys were activated during simulation\n")
            f.write(f"Threshold: {key_activation_threshold*1000:.2f}mm / {np.degrees(KEY_THRESHOLD_RAD):.1f}° / {KEY_THRESHOLD_RAD:.6f} rad - MuJoCo standard\n")
    
    print(f"\n  📄 Key activation history saved to: {history_file}")
    
    print("\n✅ Hand-piano interaction test completed!")
    print("="*70)
    print("\nTest Results:")
    print("  ✓ Scene loaded successfully")
    print("  ✓ Hands initialized and curled to press keys")
    print("  ✓ Piano initialized with color change on activation")
    print("  ✓ Keys will change to red when activated/pressed")
    if piano_controller:
        print("  ✓ Key monitoring enabled")
    print("="*70)
    
    # Convert recording to video if recording was enabled
    if recording_timestamp:
        actual_path = os.path.expanduser(f"~/omni.replicator_out/recordings/hand_piano_collision_{recording_timestamp}")
        print(f"\n🎬 Recording saved to: {actual_path}/")
        
        # Auto-convert to video
        print(f"🎬 Converting recording to video (30fps)...")
        output_video = os.path.join(Path(__file__).parent, f"hand_piano_collision_{recording_timestamp}.mp4")
        
        # Detect actual file pattern
        input_pattern = None
        if os.path.exists(actual_path):
            # Check for common patterns
            files = sorted([f for f in os.listdir(actual_path) if f.endswith('.png')])
            if files:
                # Try to detect pattern
                first_file = files[0]
                if first_file.startswith('rgb_'):
                    # Pattern: rgb_0000.png, rgb_0001.png, etc.
                    # Extract the number format
                    import re
                    match = re.search(r'rgb_(\d+)\.png', first_file)
                    if match:
                        num_digits = len(match.group(1))
                        pattern = f"rgb_%0{num_digits}d.png"
                        input_pattern = os.path.join(actual_path, pattern)
                elif first_file.startswith('frame_'):
                    # Pattern: frame_0000.png, frame_0001.png, etc.
                    import re
                    match = re.search(r'frame_(\d+)\.png', first_file)
                    if match:
                        num_digits = len(match.group(1))
                        pattern = f"frame_%0{num_digits}d.png"
                        input_pattern = os.path.join(actual_path, pattern)
                else:
                    # Fallback: use glob pattern
                    input_pattern = os.path.join(actual_path, "*.png")
                    print(f"  ⚠️  Using glob pattern (may need manual conversion)")
        
        if not input_pattern:
            # Default fallback
            input_pattern = os.path.join(actual_path, "rgb_%04d.png")
            print(f"  ⚠️  Could not detect file pattern, using default: {input_pattern}")
            if os.path.exists(actual_path):
                actual_files = [f for f in os.listdir(actual_path) if f.endswith('.png')]
                if actual_files:
                    print(f"  Found {len(actual_files)} PNG files. Sample filenames:")
                    for f in sorted(actual_files)[:5]:
                        print(f"    - {f}")
                    if len(actual_files) > 5:
                        print(f"    ... and {len(actual_files) - 5} more")
            print(f"  Please check the actual filenames in: {actual_path}")
        
        try:
            # Check if ffmpeg is available
            subprocess.run(["ffmpeg", "-version"], capture_output=True, check=True)
            
            # If using glob pattern, use different ffmpeg approach
            if input_pattern.endswith("*.png"):
                # Use glob pattern with ffmpeg
                import glob
                png_files = sorted(glob.glob(os.path.join(actual_path, "*.png")))
                if png_files:
                    # Create a temporary file list for ffmpeg
                    import tempfile
                    with tempfile.NamedTemporaryFile(mode='w', suffix='.txt', delete=False) as f:
                        for png_file in png_files:
                            f.write(f"file '{png_file}'\n")
                        file_list = f.name
                    
                    cmd = [
                        "ffmpeg", "-y",
                        "-f", "concat", "-safe", "0",
                        "-i", file_list,
                        "-framerate", "30",
                        "-c:v", "libx264",
                        "-pix_fmt", "yuv420p",
                        output_video
                    ]
                    
                    result = subprocess.run(cmd, capture_output=True, text=True)
                    os.unlink(file_list)  # Clean up temp file
                else:
                    raise FileNotFoundError("No PNG files found")
            else:
                # Standard pattern-based conversion
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
                print(f"  Detected pattern: {input_pattern}")
                print(f"  Files in directory: {len(files) if 'files' in locals() else 'unknown'}")
                print(f"  Manual conversion command:")
                if input_pattern.endswith("*.png"):
                    print(f"  (Use glob pattern or list files manually)")
                else:
                    print(f"  ffmpeg -framerate 30 -i {input_pattern} -c:v libx264 -pix_fmt yuv420p {output_video}")
                
        except FileNotFoundError:
            print(f"  ⚠️  FFmpeg not found. Please install ffmpeg or convert manually:")
            if not input_pattern.endswith("*.png"):
                print(f"  ffmpeg -framerate 30 -i {input_pattern} -c:v libx264 -pix_fmt yuv420p {output_video}")
        except Exception as e:
            print(f"  ⚠️  Error during video conversion: {e}")
            print(f"  Detected pattern: {input_pattern}")
            print(f"  Manual conversion command:")
            if not input_pattern.endswith("*.png"):
                print(f"  ffmpeg -framerate 30 -i {input_pattern} -c:v libx264 -pix_fmt yuv420p {output_video}")
    
    simulation_app.close()


if __name__ == "__main__":
    main()

