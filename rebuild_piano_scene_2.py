#!/usr/bin/env python3
"""
Rebuild the integrated piano scene USD file using the new hand files.
This script creates a complete USD scene with proper physics, positioning, and references.
"""

import os
import sys
from pathlib import Path
from typing import Optional

try:
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdLux, UsdShade
    USD_AVAILABLE = True
except ImportError:
    USD_AVAILABLE = False
    print("Error: pxr (USD Python API) not available.")
    print("This script requires USD Python bindings (usually available in Isaac Sim).")
    sys.exit(1)


def configure_joint_drives(prim: Usd.Prim, stage: Optional[Usd.Stage] = None, default_stiffness: float = 10.0, default_damping: float = 1.0):
    """
    Recursively configure joint drives to prevent joints from collapsing under gravity.
    This applies DriveAPI with stiffness and damping to maintain joint positions.
    
    Args:
        prim: Prim to check for joints
        stage: USD stage (required for creating overrides)
        default_stiffness: Default joint stiffness if not found in MjcJointAPI
        default_damping: Default joint damping if not found in MjcJointAPI
    """
    if not stage:
        return
    
    # Check if this prim is a joint
    if prim.IsA(UsdPhysics.RevoluteJoint) or prim.IsA(UsdPhysics.PrismaticJoint) or prim.IsA(UsdPhysics.SphericalJoint):
        try:
            # Get stiffness and damping from MjcJointAPI if available
            stiffness = default_stiffness
            damping = default_damping
            
            # Try to read from MjcJointAPI attributes
            try:
                stiffness_attr = prim.GetAttribute("mjc:stiffness")
                if stiffness_attr and stiffness_attr.HasAuthoredValue():
                    stiffness = stiffness_attr.Get()
            except:
                pass
            
            try:
                damping_attr = prim.GetAttribute("mjc:damping")
                if damping_attr and damping_attr.HasAuthoredValue():
                    damping = damping_attr.Get()
            except:
                pass
            
            # Create override for the joint
            override_prim = stage.OverridePrim(prim.GetPath())
            if override_prim:
                # Apply DriveAPI to the joint
                # For Isaac Sim, we typically use angular drive for revolute joints
                if prim.IsA(UsdPhysics.RevoluteJoint):
                    # Angular drive for revolute joints
                    drive_api = UsdPhysics.DriveAPI.Apply(override_prim, UsdPhysics.Tokens.angular)
                elif prim.IsA(UsdPhysics.PrismaticJoint):
                    # Linear drive for prismatic joints
                    drive_api = UsdPhysics.DriveAPI.Apply(override_prim, UsdPhysics.Tokens.linear)
                elif prim.IsA(UsdPhysics.SphericalJoint):
                    # Angular drive for spherical joints
                    drive_api = UsdPhysics.DriveAPI.Apply(override_prim, UsdPhysics.Tokens.angular)
                else:
                    drive_api = None
                
                if drive_api:
                    # Set drive stiffness and damping
                    drive_api.CreateStiffnessAttr().Set(stiffness)
                    drive_api.CreateDampingAttr().Set(damping)
                    # Set target to rest position (0) - this will maintain the joint position
                    # Target position of 0 means maintain the joint's rest angle
                    drive_api.CreateTargetPositionAttr().Set(0.0)
                    drive_api.CreateTargetVelocityAttr().Set(0.0)
                    # Optional: Set max force if available
                    try:
                        drive_api.CreateMaxForceAttr().Set(1000.0)  # High max force to maintain position
                    except:
                        pass
        except Exception as e:
            # Silently skip if we can't modify
            pass
    
    # Recursively process children
    for child in prim.GetChildren():
        configure_joint_drives(child, stage, default_stiffness, default_damping)


def configure_collision_prims(prim: Usd.Prim, hand_material_shade: Optional[UsdShade.Material] = None, stage: Optional[Usd.Stage] = None):
    """
    Recursively configure collision prims to ensure they have valid geometry.
    This helps fix 'Invalid PxGeometry' errors by ensuring collision API is properly set.
    
    Since we're working with referenced USD files, we need to create overrides to modify
    the collision prims. However, for now, we'll just ensure the API is properly set.
    """
    # Check if this prim has CollisionAPI
    if prim.HasAPI(UsdPhysics.CollisionAPI):
        # Try to get the collision API (may fail if it's in a referenced file)
        try:
            collision_api = UsdPhysics.CollisionAPI(prim)
            
            # Ensure collision is enabled (create override if needed)
            if stage:
                # Create an override prim to modify the referenced collision
                override_prim = stage.OverridePrim(prim.GetPath())
                if override_prim:
                    collision_api_override = UsdPhysics.CollisionAPI.Apply(override_prim)
                    collision_api_override.CreateCollisionEnabledAttr().Set(True)
            
            # Check if it's a mesh collision
            mesh = UsdGeom.Mesh(prim)
            if mesh:
                # Check if mesh has valid geometry
                points_attr = mesh.GetPointsAttr()
                if points_attr:
                    points = points_attr.Get()
                    if points and len(points) > 0:
                        # Mesh has valid points - ensure MeshCollisionAPI is applied
                        if not prim.HasAPI(UsdPhysics.MeshCollisionAPI):
                            # Create override if we have stage
                            if stage:
                                override_prim = stage.OverridePrim(prim.GetPath())
                                if override_prim:
                                    mesh_collision_api = UsdPhysics.MeshCollisionAPI.Apply(override_prim)
                                    try:
                                        mesh_collision_api.CreateApproximationAttr().Set(UsdPhysics.Tokens.convexHull)
                                    except:
                                        pass
                        else:
                            # Already has MeshCollisionAPI, just ensure approximation is set
                            if stage:
                                override_prim = stage.OverridePrim(prim.GetPath())
                                if override_prim:
                                    mesh_collision_api = UsdPhysics.MeshCollisionAPI(override_prim)
                                    if not mesh_collision_api.GetApproximationAttr():
                                        mesh_collision_api.CreateApproximationAttr().Set(UsdPhysics.Tokens.convexHull)
            
            # Bind material if provided (create override for binding)
            if hand_material_shade and stage:
                try:
                    override_prim = stage.OverridePrim(prim.GetPath())
                    if override_prim:
                        binding_api = UsdShade.MaterialBindingAPI.Apply(override_prim)
                        binding_api.Bind(hand_material_shade, materialPurpose="physics")
                except:
                    pass
        except Exception as e:
            # Silently skip if we can't modify (likely in referenced file without override)
            pass
    
    # Recursively process children
    for child in prim.GetChildren():
        configure_collision_prims(child, hand_material_shade, stage)


def rebuild_piano_scene(
    output_path: str = "piano_with_hands.usd",
    base_dir: Optional[str] = None
) -> bool:
    """
    Rebuild the integrated piano scene with new hand files.
    
    Args:
        output_path: Path where the USD file will be saved
        base_dir: Base directory for resolving relative paths
        
    Returns:
        True if successful
    """
    if not USD_AVAILABLE:
        return False
    
    # Get base directory
    if base_dir is None:
        base_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Define paths
    piano_usd = os.path.join(base_dir, "piano_fixed.usd")
    left_hand_usd = os.path.join(base_dir, "hand_left.usd")
    right_hand_usd = os.path.join(base_dir, "hand_right.usd")
    output_path = os.path.join(base_dir, output_path)
    
    # Convert to absolute paths
    piano_usd = os.path.abspath(piano_usd)
    left_hand_usd = os.path.abspath(left_hand_usd)
    right_hand_usd = os.path.abspath(right_hand_usd)
    output_path = os.path.abspath(output_path)
    
    # Validate files exist
    for path, name in [
        (piano_usd, "piano_fixed.usd"),
        (left_hand_usd, "hand_left.usd"),
        (right_hand_usd, "hand_right.usd")
    ]:
        if not os.path.exists(path):
            print(f"Error: {name} not found at: {path}")
            return False
    
    print("=" * 60)
    print("Rebuilding Piano + Shadow Hands USD Scene")
    print("=" * 60)
    print(f"\nInput files:")
    print(f"  Piano: {piano_usd}")
    print(f"  Left Hand: {left_hand_usd}")
    print(f"  Right Hand: {right_hand_usd}")
    print(f"\nOutput: {output_path}")
    print()
    
    try:
        # Create new USD stage
        stage = Usd.Stage.CreateNew(output_path)
        
        # Set stage metadata
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)  # Z-up for Isaac Sim
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
        stage.SetMetadata("defaultPrim", "World")
        
        # Create /World root prim
        world_prim = stage.DefinePrim("/World", "Xform")
        stage.SetDefaultPrim(world_prim)
        
        print("✓ Created World prim")
        
        # ========== Create Lighting ==========
        print("\nSetting up lighting...")
        
        # Create a Lights container
        lights_prim = stage.DefinePrim("/World/Lights", "Xform")
        lights_xform = UsdGeom.Xform(lights_prim)
        
        # Add DomeLight for ambient/global illumination (similar to MuJoCo's first light at (0, 0, 1))
        dome_light = UsdLux.DomeLight.Define(stage, "/World/Lights/DomeLight")
        dome_light.CreateIntensityAttr().Set(3.0)  # Increased intensity for better color visibility
        dome_light.CreateColorAttr().Set(Gf.Vec3f(1.0, 1.0, 1.0))  # White light
        dome_light.CreateEnableColorTemperatureAttr().Set(False)
        print("  ✓ Added DomeLight for ambient illumination (intensity: 3.0)")
        
        # Add SphereLight for point lighting (similar to MuJoCo's second light)
        # MuJoCo has light at (0.3, 0, 1) pointing down (dir=(0, 0, -1), directional=False)
        # This is a point light, so we use SphereLight
        sphere_light = UsdLux.SphereLight.Define(stage, "/World/Lights/SphereLight")
        sphere_light.CreateIntensityAttr().Set(3000.0)  # Match the intensity from piano_fixed.usda
        sphere_light.CreateColorAttr().Set(Gf.Vec3f(1.0, 1.0, 1.0))  # White light
        sphere_light.CreateRadiusAttr().Set(0.1)  # Small radius for point-like behavior
        
        # Position the sphere light
        # In MuJoCo: pos=(0.3, 0, 1) - light from above
        # In Isaac Sim Z-up: same position (0.3, 0, 1.0)
        sphere_light_xform = UsdGeom.Xform(sphere_light)
        sphere_light_xform.ClearXformOpOrder()
        translate_op = sphere_light_xform.AddTranslateOp()
        translate_op.Set(Gf.Vec3d(0.3, 0.0, 1.0))
        
        print("  ✓ Added SphereLight for point lighting (from above)")
        print("    Position: (0.3, 0.0, 1.0), Intensity: 3000")
        
        # Add additional RectLight for better overall illumination (similar to MuJoCo's setup)
        # This helps ensure colors are visible and not washed out
        rect_light = UsdLux.RectLight.Define(stage, "/World/Lights/RectLight")
        rect_light.CreateIntensityAttr().Set(5000.0)
        rect_light.CreateColorAttr().Set(Gf.Vec3f(1.0, 1.0, 1.0))  # White light
        rect_light.CreateWidthAttr().Set(2.0)
        rect_light.CreateHeightAttr().Set(2.0)
        
        # Position the rect light above the scene, facing down
        rect_light_xform = UsdGeom.Xform(rect_light)
        rect_light_xform.ClearXformOpOrder()
        translate_op = rect_light_xform.AddTranslateOp()
        translate_op.Set(Gf.Vec3d(0.0, 0.0, 1.5))  # Above the scene
        # Rotate to face down (-Z direction)
        orient_op = rect_light_xform.AddOrientOp()
        rotation = Gf.Rotation(Gf.Vec3d(1, 0, 0), 180.0)  # 180° around X axis to face down
        quat_d = rotation.GetQuat()
        quat = Gf.Quatf(quat_d.GetReal(), quat_d.GetImaginary()[0], quat_d.GetImaginary()[1], quat_d.GetImaginary()[2])
        orient_op.Set(quat)
        
        print("  ✓ Added RectLight for additional illumination (from above, facing down)")
        print("    Position: (0.0, 0.0, 1.5), Intensity: 5000, Size: 2.0x2.0")
        
        # ========== Create Physics Scene ==========
        print("\nConfiguring physics scene...")
        physics_scene = UsdPhysics.Scene.Define(stage, "/PhysicsScene")
        
        # Set gravity (Z-down, 9.81 m/s²)
        physics_scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        physics_scene.CreateGravityMagnitudeAttr().Set(9.81)
        
        print("  ✓ Physics scene configured with gravity (Z-down, 9.81 m/s²)")
        
        # ========== Create Physics Materials ==========
        print("\nCreating physics materials...")
        
        # Piano material (stiff contacts, equivalent to MuJoCo solref = (0.01, 1))
        piano_material_prim = stage.DefinePrim("/World/Materials/piano_material", "Material")
        piano_material = UsdShade.Material(piano_material_prim)
        piano_material_api = UsdPhysics.MaterialAPI.Apply(piano_material_prim)
        piano_material_api.CreateStaticFrictionAttr().Set(1.0)
        piano_material_api.CreateDynamicFrictionAttr().Set(1.0)
        piano_material_api.CreateRestitutionAttr().Set(0.0)
        
        # Add PhysX-specific material properties (Isaac Sim extensions)
        # These use custom attributes that Isaac Sim/PhysX will read
        piano_material_prim.CreateAttribute("physx:staticFriction", Sdf.ValueTypeNames.Float).Set(1.0)
        piano_material_prim.CreateAttribute("physx:dynamicFriction", Sdf.ValueTypeNames.Float).Set(1.0)
        piano_material_prim.CreateAttribute("physx:restitution", Sdf.ValueTypeNames.Float).Set(0.0)
        piano_material_prim.CreateAttribute("physx:stiffness", Sdf.ValueTypeNames.Float).Set(1000000.0)
        piano_material_prim.CreateAttribute("physx:damping", Sdf.ValueTypeNames.Float).Set(1000.0)
        
        print("  ✓ Piano material created (stiff contacts)")
        print("    Friction: 1.0, Restitution: 0.0, Stiffness: 1e6, Damping: 1000")
        
        # Hand material (softer contacts, equivalent to MuJoCo solref = (0.005, 1))
        hand_material_prim = stage.DefinePrim("/World/Materials/hand_material", "Material")
        hand_material = UsdShade.Material(hand_material_prim)
        hand_material_api = UsdPhysics.MaterialAPI.Apply(hand_material_prim)
        hand_material_api.CreateStaticFrictionAttr().Set(1.0)
        hand_material_api.CreateDynamicFrictionAttr().Set(1.0)
        hand_material_api.CreateRestitutionAttr().Set(0.0)
        
        # Add PhysX-specific material properties
        hand_material_prim.CreateAttribute("physx:staticFriction", Sdf.ValueTypeNames.Float).Set(1.0)
        hand_material_prim.CreateAttribute("physx:dynamicFriction", Sdf.ValueTypeNames.Float).Set(1.0)
        hand_material_prim.CreateAttribute("physx:restitution", Sdf.ValueTypeNames.Float).Set(0.0)
        hand_material_prim.CreateAttribute("physx:stiffness", Sdf.ValueTypeNames.Float).Set(500000.0)
        hand_material_prim.CreateAttribute("physx:damping", Sdf.ValueTypeNames.Float).Set(500.0)
        
        print("  ✓ Hand material created (softer contacts)")
        print("    Friction: 1.0, Restitution: 0.0, Stiffness: 5e5, Damping: 500")
        
        # ========== Load Piano ==========
        print("\nLoading piano...")
        piano_prim = stage.DefinePrim("/World/piano", "Xform")
        
        # Add reference to piano USD file
        piano_prim.GetReferences().AddReference(Sdf.Reference(piano_usd))
        
        # Piano positioning:
        # In MuJoCo: piano base at x=-0.127, keys center at x=0
        # If piano USD origin is at base, we need to offset by +0.127 to align keys at x=0
        # If piano USD origin is at keys, we keep at origin
        # For now, place piano at origin and adjust hands if needed
        piano_xform = UsdGeom.Xform(piano_prim)
        translate_op = piano_xform.AddTranslateOp()
        translate_op.Set(Gf.Vec3d(0.0, 0.0, 0.0))
        
        # ========== Apply Physics Properties to Piano ==========
        # Collision group 0, filter 2 (collide with group 1 only)
        # This ensures piano keys don't collide with each other
        piano_prim.CreateAttribute("physx:collisionGroup", Sdf.ValueTypeNames.Int).Set(0)
        piano_prim.CreateAttribute("physx:collisionFilter", Sdf.ValueTypeNames.Int).Set(2)  # 0b10 = collide with group 1
        
        # Bind piano material using MaterialBindingAPI
        piano_material_path = "/World/Materials/piano_material"
        piano_material_prim = stage.GetPrimAtPath(piano_material_path)
        if piano_material_prim.IsValid():
            piano_material_shade = UsdShade.Material(piano_material_prim)
            binding_api = UsdShade.MaterialBindingAPI.Apply(piano_prim)
            # Bind with "physics" purpose for physics materials (use string if token not available)
            try:
                binding_api.Bind(piano_material_shade, materialPurpose=UsdShade.Tokens.physics)
            except AttributeError:
                # Fallback: use string "physics" if token not available
                binding_api.Bind(piano_material_shade, materialPurpose="physics")
            # Also bind as preview (visual) if needed
            binding_api.Bind(piano_material_shade, materialPurpose=UsdShade.Tokens.preview)
        
        print(f"  ✓ Piano loaded at origin")
        print(f"    Collision group: 0, Filter: 2 (collide with group 1 only)")
        print(f"    Physics material: piano_material")
        
        # ========== Load Left Hand ==========
        print("\nLoading left hand...")
        # MuJoCo position: (0.4, -0.15, 0.13)
        # In MuJoCo: keys center at x=0, base at x=-0.127, hands at x=0.4
        # 
        # If piano USD has base at origin (x=-0.127 in MuJoCo), then:
        # - Keys would be at x=0.127 in USD
        # - Hands should be at x=0.4 + 0.127 = 0.527 to maintain same relative position
        #
        # If piano USD has keys at origin (x=0 in MuJoCo), then:
        # - Hands should be at x=0.4 (same as MuJoCo)
        #
        # Adjust hand X position to account for base offset
        # MuJoCo exact positions: Left (0.4, -0.15, 0.13), Right (0.4, 0.15, 0.13)
        BASE_X_OFFSET_MUJOCO = -0.127  # Piano base offset in MuJoCo
        left_hand_pos_mujoco = (0.4, -0.15, 0.13)
        # If piano USD origin is at base (not keys), add base offset to X
        # Keep Y and Z exactly as in MuJoCo to match positioning
        left_hand_x_adjusted = left_hand_pos_mujoco[0] - BASE_X_OFFSET_MUJOCO
        left_hand_pos_isaac = (left_hand_x_adjusted, left_hand_pos_mujoco[1], left_hand_pos_mujoco[2])
        
        left_hand_prim = stage.DefinePrim("/World/left_hand", "Xform")
        left_hand_prim.GetReferences().AddReference(Sdf.Reference(left_hand_usd))
        
        # Set position and orientation
        left_hand_xform = UsdGeom.Xform(left_hand_prim)
        # Clear existing xform ops if any
        left_hand_xform.ClearXformOpOrder()
        
        translate_op = left_hand_xform.AddTranslateOp()
        translate_op.Set(Gf.Vec3d(*left_hand_pos_isaac))
        
        # Orient hand to face X direction (toward piano) with palm facing down
        # First rotate -90° around Z axis, then 180° around X axis to flip palm down
        orient_op = left_hand_xform.AddOrientOp()
        rotation_z = Gf.Rotation(Gf.Vec3d(0, 0, 1), -90.0)  # -90° around Z
        rotation_x = Gf.Rotation(Gf.Vec3d(1, 0, 0), 180.0)  # 180° around X
        # Combine rotations: first Z, then X
        combined_rotation = rotation_z * rotation_x
        quat_d = combined_rotation.GetQuat()  # Returns GfQuatd
        quat = Gf.Quatf(quat_d.GetReal(), quat_d.GetImaginary()[0], quat_d.GetImaginary()[1], quat_d.GetImaginary()[2])
        orient_op.Set(quat)
        
        # Configure as articulation
        # Check if the hand USD already has ArticulationRootAPI at its root
        # If it does, we should NOT apply it again (nested articulation roots are not allowed)
        # The articulation root should be inside the referenced hand file, not on the parent Xform
        hand_stage = Usd.Stage.Open(left_hand_usd)
        hand_has_articulation = False
        if hand_stage:
            hand_default_prim = hand_stage.GetDefaultPrim()
            if hand_default_prim and hand_default_prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                hand_has_articulation = True
                print("  ✓ Hand USD already has ArticulationRootAPI at root - not applying duplicate")
        
        # Only apply ArticulationRootAPI if the hand file doesn't already have it
        if not hand_has_articulation:
            articulation_api = UsdPhysics.ArticulationRootAPI.Apply(left_hand_prim)
            try:
                if hasattr(articulation_api, 'CreateSolverTypeAttr'):
                    articulation_api.CreateSolverTypeAttr().Set("PGS")
            except (AttributeError, Exception):
                pass
        else:
            print("  ✓ Using existing ArticulationRootAPI from hand USD file")
        
        # ========== Apply Physics Properties to Left Hand ==========
        # Collision group 1, filter 1 (collide with group 0 only)
        # This ensures hands don't collide with each other
        left_hand_prim.CreateAttribute("physx:collisionGroup", Sdf.ValueTypeNames.Int).Set(1)
        left_hand_prim.CreateAttribute("physx:collisionFilter", Sdf.ValueTypeNames.Int).Set(1)  # 0b01 = collide with group 0 only
        
        # Bind hand material using MaterialBindingAPI
        hand_material_path = "/World/Materials/hand_material"
        hand_material_prim = stage.GetPrimAtPath(hand_material_path)
        hand_material_shade = None
        if hand_material_prim.IsValid():
            hand_material_shade = UsdShade.Material(hand_material_prim)
            binding_api = UsdShade.MaterialBindingAPI.Apply(left_hand_prim)
            # Bind with "physics" purpose for physics materials
            try:
                binding_api.Bind(hand_material_shade, materialPurpose=UsdShade.Tokens.physics)
            except AttributeError:
                binding_api.Bind(hand_material_shade, materialPurpose="physics")
            binding_api.Bind(hand_material_shade, materialPurpose=UsdShade.Tokens.preview)
        
        # Configure collision prims recursively to fix invalid geometry errors
        print("  Configuring collision prims...")
        configure_collision_prims(left_hand_prim, hand_material_shade, stage)
        
        # Note: Joints are passive (no DriveAPI) to match MuJoCo behavior
        # In MuJoCo, joints collapse under gravity unless actively controlled via actuators
        # To prevent collapse, joints must be actively controlled via Isaac Sim's joint control API
        
        print(f"  ✓ Left hand loaded at {left_hand_pos_isaac} (Isaac Sim Z-up)")
        print(f"    Original MuJoCo position: {left_hand_pos_mujoco}")
        print(f"    Adjusted X position: {left_hand_x_adjusted} (accounting for piano base offset)")
        print(f"    Orientation: -90° around Z axis, then 180° around X axis (facing X, palm down)")
        print(f"    Collision group: 1, Filter: 1 (collide with group 0 only)")
        print(f"    Physics material: hand_material")
        print(f"    Configured as articulation")
        print(f"    Note: Warning about external reference is expected if hand file references Isaac Sim assets")
        
        # ========== Load Right Hand ==========
        print("\nLoading right hand...")
        # MuJoCo position: (0.4, 0.15, 0.13)
        # Apply same X adjustment as left hand to account for piano base offset
        # Keep Y and Z exactly as in MuJoCo to match positioning
        right_hand_pos_mujoco = (0.4, 0.15, 0.13)
        right_hand_x_adjusted = right_hand_pos_mujoco[0] - BASE_X_OFFSET_MUJOCO
        right_hand_pos_isaac = (right_hand_x_adjusted, right_hand_pos_mujoco[1], right_hand_pos_mujoco[2])
        
        right_hand_prim = stage.DefinePrim("/World/right_hand", "Xform")
        right_hand_prim.GetReferences().AddReference(Sdf.Reference(right_hand_usd))
        
        # Set position and orientation
        right_hand_xform = UsdGeom.Xform(right_hand_prim)
        # Clear existing xform ops if any
        right_hand_xform.ClearXformOpOrder()
        
        translate_op = right_hand_xform.AddTranslateOp()
        translate_op.Set(Gf.Vec3d(*right_hand_pos_isaac))
        
        # Orient hand to face X direction (toward piano) with palm facing down
        # First rotate -90° around Z axis, then 180° around X axis to flip palm down
        orient_op = right_hand_xform.AddOrientOp()
        rotation_z = Gf.Rotation(Gf.Vec3d(0, 0, 1), -90.0)  # -90° around Z
        rotation_x = Gf.Rotation(Gf.Vec3d(1, 0, 0), 180.0)  # 180° around X
        # Combine rotations: first Z, then X
        combined_rotation = rotation_z * rotation_x
        quat_d = combined_rotation.GetQuat()  # Returns GfQuatd
        quat = Gf.Quatf(quat_d.GetReal(), quat_d.GetImaginary()[0], quat_d.GetImaginary()[1], quat_d.GetImaginary()[2])
        orient_op.Set(quat)
        
        # Mirror the right hand (scale -1 in X axis to flip left-to-right)
        # The right hand USD file has scale -1 when viewed standalone, so we need to preserve that
        scale_op = right_hand_xform.AddScaleOp()
        scale_op.Set(Gf.Vec3f(-1.0, 1.0, 1.0))  # Mirror in X axis
        
        # Configure as articulation
        # Check if the hand USD already has ArticulationRootAPI at its root
        # If it does, we should NOT apply it again (nested articulation roots are not allowed)
        hand_stage = Usd.Stage.Open(right_hand_usd)
        hand_has_articulation = False
        if hand_stage:
            hand_default_prim = hand_stage.GetDefaultPrim()
            if hand_default_prim and hand_default_prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                hand_has_articulation = True
                print("  ✓ Hand USD already has ArticulationRootAPI at root - not applying duplicate")
        
        # Only apply ArticulationRootAPI if the hand file doesn't already have it
        if not hand_has_articulation:
            articulation_api = UsdPhysics.ArticulationRootAPI.Apply(right_hand_prim)
            try:
                if hasattr(articulation_api, 'CreateSolverTypeAttr'):
                    articulation_api.CreateSolverTypeAttr().Set("PGS")
            except (AttributeError, Exception):
                pass
        else:
            print("  ✓ Using existing ArticulationRootAPI from hand USD file")
        
        # ========== Apply Physics Properties to Right Hand ==========
        # Collision group 1, filter 1 (collide with group 0 only)
        # Same as left hand - ensures hands don't collide with each other
        right_hand_prim.CreateAttribute("physx:collisionGroup", Sdf.ValueTypeNames.Int).Set(1)
        right_hand_prim.CreateAttribute("physx:collisionFilter", Sdf.ValueTypeNames.Int).Set(1)  # 0b01 = collide with group 0 only
        
        # Bind hand material using MaterialBindingAPI
        hand_material_path = "/World/Materials/hand_material"
        hand_material_prim = stage.GetPrimAtPath(hand_material_path)
        hand_material_shade = None
        if hand_material_prim.IsValid():
            hand_material_shade = UsdShade.Material(hand_material_prim)
            binding_api = UsdShade.MaterialBindingAPI.Apply(right_hand_prim)
            # Bind with "physics" purpose for physics materials
            try:
                binding_api.Bind(hand_material_shade, materialPurpose=UsdShade.Tokens.physics)
            except AttributeError:
                binding_api.Bind(hand_material_shade, materialPurpose="physics")
            binding_api.Bind(hand_material_shade, materialPurpose=UsdShade.Tokens.preview)
        
        # Configure collision prims recursively to fix invalid geometry errors
        print("  Configuring collision prims...")
        configure_collision_prims(right_hand_prim, hand_material_shade, stage)
        
        # Note: Joints are passive (no DriveAPI) to match MuJoCo behavior
        # In MuJoCo, joints collapse under gravity unless actively controlled via actuators
        # To prevent collapse, joints must be actively controlled via Isaac Sim's joint control API
        
        print(f"  ✓ Right hand loaded at {right_hand_pos_isaac} (Isaac Sim Z-up)")
        print(f"    Original MuJoCo position: {right_hand_pos_mujoco}")
        print(f"    Adjusted X position: {right_hand_x_adjusted} (accounting for piano base offset)")
        print(f"    Orientation: -90° around Z axis, then 180° around X axis (facing X, palm down)")
        print(f"    Scale: (-1.0, 1.0, 1.0) - mirrored in X axis")
        print(f"    Collision group: 1, Filter: 1 (collide with group 0 only)")
        print(f"    Physics material: hand_material")
        print(f"    Configured as articulation")
        print(f"    Note: Warning about external reference is expected if hand file references Isaac Sim assets")
        
        # ========== Save the stage ==========
        print(f"\nSaving scene to: {output_path}")
        stage.Save()
        
        print("\n" + "=" * 60)
        print("✓ Successfully rebuilt piano scene!")
        print("=" * 60)
        print(f"\nScene hierarchy:")
        print("  /World")
        print("    ├── Lights")
        print("    │   ├── DomeLight (ambient)")
        print("    │   ├── SphereLight (point light)")
        print("    │   └── RectLight (area light)")
        print("    ├── piano")
        print("    ├── left_hand (articulation)")
        print("    └── right_hand (articulation)")
        print("  /PhysicsScene")
        print(f"\nYou can now load this file in Isaac Sim:")
        print(f"  {output_path}")
        
        return True
        
    except Exception as e:
        print(f"\nError rebuilding scene: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    """Main function."""
    import argparse
    
    parser = argparse.ArgumentParser(description="Rebuild piano scene with new hand files")
    parser.add_argument(
        "--output",
        "-o",
        default="piano_with_hands.usd",
        help="Output USD file path (default: piano_with_hands.usd)"
    )
    parser.add_argument(
        "--base-dir",
        "-d",
        default=None,
        help="Base directory for resolving relative paths (default: script directory)"
    )
    
    args = parser.parse_args()
    
    success = rebuild_piano_scene(
        output_path=args.output,
        base_dir=args.base_dir
    )
    
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())

