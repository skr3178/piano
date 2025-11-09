#!/usr/bin/env python3
"""
Generate a comprehensive piano USDA file with full physics properties.

This script creates a USD file that includes all physics features from the MuJoCo XML:
- Rigid body dynamics (masses, inertias)
- Revolute joints with limits
- Spring stiffness and reference positions
- Damping coefficients
- Collision geometries
- Material properties

The resulting USD can be used in physics engines that support USD physics schemas.
"""

import sys
import math
from pathlib import Path

# Add the robopianist directory to path
sys.path.insert(0, str(Path(__file__).parent / "robopianist"))

# Try to use local constants first, fallback to robopianist
try:
    from exts.omni.isaac.piano import piano_constants as consts
except ImportError:
    from robopianist.models.piano import piano_constants as consts

try:
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt
    USD_AVAILABLE = True
except ImportError:
    print("ERROR: USD Python API (pxr) is not available.")
    print("Please install it with: pip install usd-core")
    sys.exit(1)


def create_physics_material(stage, name, static_friction=0.5, dynamic_friction=0.4, restitution=0.1):
    """Create a physics material for collisions."""
    material_path = f"/PhysicsMaterials/{name}"
    material = UsdPhysics.MaterialAPI.Apply(stage.DefinePrim(material_path, "Material"))
    material.CreateStaticFrictionAttr().Set(static_friction)
    material.CreateDynamicFrictionAttr().Set(dynamic_friction)
    material.CreateRestitutionAttr().Set(restitution)
    return material_path


def create_box_collision(stage, prim_path, size, physics_material_path=None):
    """Create a box collision shape."""
    collision_prim = stage.DefinePrim(prim_path, "Cube")
    collision_geom = UsdGeom.Cube(collision_prim)
    # USD Cube has size=2.0 by default (from -1 to 1), so we scale by size
    collision_geom.GetSizeAttr().Set(2.0)
    
    # Scale to match the box dimensions (no coordinate conversion needed)
    xform = UsdGeom.Xformable(collision_prim)
    xform.AddScaleOp().Set(Gf.Vec3d(size[0], size[1], size[2]))
    
    # Add collision API and ALWAYS enable it
    collision_api = UsdPhysics.CollisionAPI.Apply(collision_prim)
    collision_api.GetCollisionEnabledAttr().Set(True)
    
    # Bind physics material if provided
    if physics_material_path:
        # Get the material binding API
        material_api = UsdShade.MaterialBindingAPI.Apply(collision_prim)
        # Bind the physics material
        material_api.Bind(UsdShade.Material(stage.GetPrimAtPath(physics_material_path)), 
                         UsdShade.Tokens.weakerThanDescendants,
                         "physics")
    
    return collision_prim


def create_visual_box_mesh(mesh, size, color):
    """Create a box mesh with vertices and faces."""
    x, y, z = size[0], size[1], size[2]
    
    # Both MuJoCo and USD use Z-up, so no coordinate conversion needed
    points = [
        Gf.Vec3f(-x, -y, -z),  # 0: back-left-bottom
        Gf.Vec3f(x, -y, -z),   # 1: back-right-bottom
        Gf.Vec3f(-x, y, -z),   # 2: back-left-top
        Gf.Vec3f(x, y, -z),    # 3: back-right-top
        Gf.Vec3f(-x, -y, z),   # 4: front-left-bottom
        Gf.Vec3f(x, -y, z),    # 5: front-right-bottom
        Gf.Vec3f(-x, y, z),    # 6: front-left-top
        Gf.Vec3f(x, y, z),     # 7: front-right-top
    ]
    
    face_vertex_counts = [4, 4, 4, 4, 4, 4]
    face_vertex_indices = [
        0, 1, 3, 2,  # back face
        0, 4, 5, 1,  # bottom face
        0, 2, 6, 4,  # left face
        2, 3, 7, 6,  # top face
        4, 6, 7, 5,  # front face
        1, 5, 7, 3,  # right face
    ]
    
    # Compute normals for each face
    normals = [
        Gf.Vec3f(0, 0, -1),   # back
        Gf.Vec3f(0, -1, 0),   # bottom
        Gf.Vec3f(-1, 0, 0),   # left
        Gf.Vec3f(0, 1, 0),    # top
        Gf.Vec3f(0, 0, 1),    # front
        Gf.Vec3f(1, 0, 0),    # right
    ]
    
    mesh.GetPointsAttr().Set(points)
    mesh.GetFaceVertexCountsAttr().Set(face_vertex_counts)
    mesh.GetFaceVertexIndicesAttr().Set(face_vertex_indices)
    mesh.GetSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
    
    # Set per-face normals
    mesh.GetNormalsAttr().Set(normals)
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.uniform)
    
    # Set display color
    color_primvar = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
        "displayColor", Sdf.ValueTypeNames.Color3fArray
    )
    color_primvar.Set([Gf.Vec3f(color[0], color[1], color[2])])
    color_primvar.SetInterpolation(UsdGeom.Tokens.constant)


def add_joint_with_spring(
    stage,
    joint_path,
    body0_path,
    body1_path,
    joint_local_pos,
    axis,
    lower_limit,
    upper_limit,
    stiffness,
    damping,
    spring_ref,
    armature,
    body1_position=None,
    body0_position=None,
):
    """
    Create a revolute joint with spring properties.
    
    Args:
        stage: USD stage
        joint_path: Path for the joint prim
        body0_path: Parent body path
        body1_path: Child body path
        joint_local_pos: Joint position in local coordinates
        axis: Joint axis ('X', 'Y', or 'Z')
        lower_limit: Lower joint limit (degrees)
        upper_limit: Upper joint limit (degrees)
        stiffness: Spring stiffness (Nm/rad)
        damping: Joint damping
        spring_ref: Spring reference angle (radians)
        armature: Joint armature (inertia)
    """
    joint = UsdPhysics.RevoluteJoint.Define(stage, joint_path)
    
    # Set body relationships
    joint.CreateBody0Rel().SetTargets([Sdf.Path(body0_path)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(body1_path)])
    
    # Set joint axis (MuJoCo Y-axis maps to USD Z-axis)
    joint.CreateAxisAttr().Set(axis)
    
    # Set joint limits (in degrees for USD)
    joint.CreateLowerLimitAttr().Set(lower_limit)
    joint.CreateUpperLimitAttr().Set(upper_limit)
    
    # Set joint anchor frames properly
    # The joint position relative to body1 (the key)
    joint.CreateLocalPos1Attr().Set(joint_local_pos)
    
    # The joint position relative to body0 (base)
    # If body1_position is provided, compute the joint position in base's local frame
    if body1_position is not None and body0_position is not None:
        # Joint position in world space = key position + joint offset
        joint_world_x = body1_position[0] + joint_local_pos[0]
        joint_world_y = body1_position[1] + joint_local_pos[1]
        joint_world_z = body1_position[2] + joint_local_pos[2]
        
        # Convert to base's local frame = world position - base position
        joint_pos_in_base = Gf.Vec3d(
            joint_world_x - body0_position[0],
            joint_world_y - body0_position[1],
            joint_world_z - body0_position[2]
        )
        joint.CreateLocalPos0Attr().Set(joint_pos_in_base)
    else:
        # Fallback: use same as body1 local pos (assumes bodies are at same location)
        joint.CreateLocalPos0Attr().Set(joint_local_pos)
    
    # Add drive API for spring behavior
    # USD uses PhysxJoint schema for advanced properties
    joint_prim = joint.GetPrim()
    
    # Store MuJoCo-specific properties as custom attributes
    # These can be read by MuJoCo-compatible USD importers
    joint_prim.CreateAttribute(
        "physics:stiffness", Sdf.ValueTypeNames.Float, custom=True
    ).Set(stiffness)
    
    joint_prim.CreateAttribute(
        "physics:damping", Sdf.ValueTypeNames.Float, custom=True
    ).Set(damping)
    
    joint_prim.CreateAttribute(
        "physics:springReference", Sdf.ValueTypeNames.Float, custom=True
    ).Set(spring_ref)
    
    joint_prim.CreateAttribute(
        "physics:armature", Sdf.ValueTypeNames.Float, custom=True
    ).Set(armature)
    
    # Add USD Physics drive for spring-like behavior
    # This uses the DriveAPI which is standard in USD Physics
    drive = UsdPhysics.DriveAPI.Apply(joint_prim, "angular")
    drive.CreateTypeAttr().Set("force")
    drive.CreateDampingAttr().Set(damping)
    drive.CreateStiffnessAttr().Set(stiffness)
    # Target position is the spring reference
    drive.CreateTargetPositionAttr().Set(math.degrees(spring_ref))
    # Set a high max force to ensure springs can overcome any resistance
    drive.CreateMaxForceAttr().Set(1000.0)  # High enough to pull keys back reliably
    
    return joint


def build_piano_usd_with_physics(out_file):
    """Build complete piano USD with full physics properties."""
    
    print("Creating USD stage...")
    stage = Usd.Stage.CreateNew(str(out_file))
    
    # Set stage metadata
    stage.SetMetadata("defaultPrim", "Piano")
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)  # Z-up
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    
    # Set time codes
    stage.SetStartTimeCode(0)
    stage.SetEndTimeCode(100)
    
    print("Creating physics scene...")
    # Create physics scene with gravity
    physics_scene = UsdPhysics.Scene.Define(stage, "/PhysicsScene")
    physics_scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0, 0, -1))  # -Z is down in Z-up coordinate system
    physics_scene.CreateGravityMagnitudeAttr().Set(9.81)
    
    # Create physics materials
    print("Creating physics materials...")
    key_material = create_physics_material(
        stage, "KeyMaterial",
        static_friction=0.6,
        dynamic_friction=0.5,
        restitution=0.1
    )
    
    # Create root piano Xform
    print("Creating piano root...")
    piano_prim = stage.DefinePrim("/Piano", "Xform")
    stage.SetDefaultPrim(piano_prim)
    piano_xform = UsdGeom.Xform(piano_prim)
    
    # Apply ArticulationRootAPI to make Isaac Sim recognize this as an articulation
    UsdPhysics.ArticulationRootAPI.Apply(piano_prim)
    
    # Create a fixed joint to anchor the piano base to the world frame
    # This keeps the piano completely stationary
    # By not setting body0, it defaults to the world frame
    fixed_joint_path = "/Piano/WorldFixedJoint"
    fixed_joint = UsdPhysics.FixedJoint.Define(stage, fixed_joint_path)
    # body0 is implicitly the world frame (no target = world)
    # body1 is the piano base
    fixed_joint.CreateBody1Rel().SetTargets([Sdf.Path("/Piano/Base")])
    print("Added fixed joint to anchor piano base to world frame")
    
    # Add base (static body)
    print("Creating piano base...")
    base_path = "/Piano/Base"
    base_xform = UsdGeom.Xform.Define(stage, base_path)
    base_transform = base_xform.AddTranslateOp()
    
    # Both MuJoCo and USD use Z-up, so keep coordinates as-is
    base_pos = consts.BASE_POS
    base_transform.Set(Gf.Vec3d(base_pos[0], base_pos[1], base_pos[2]))
    
    # Create base mesh (visual)
    base_mesh_path = f"{base_path}/BaseMesh"
    base_mesh = UsdGeom.Mesh.Define(stage, base_mesh_path)
    create_visual_box_mesh(base_mesh, consts.BASE_SIZE, consts.BASE_COLOR)
    
    # Make base a dynamic rigid body with very high mass (acts like static)
    # Note: Can't use kinematic because Isaac Sim doesn't support kinematic bodies in articulations
    base_rb = UsdPhysics.RigidBodyAPI.Apply(base_xform.GetPrim())
    base_rb.CreateRigidBodyEnabledAttr().Set(True)
    
    # Add mass API with very high mass to make it effectively immovable
    base_mass_api = UsdPhysics.MassAPI.Apply(base_xform.GetPrim())
    base_mass_api.CreateMassAttr().Set(10000.0)  # 10 tons - effectively immovable
    
    # Add collision to base
    base_collision_path = f"{base_path}/BaseCollision"
    create_box_collision(stage, base_collision_path, consts.BASE_SIZE, key_material)
    
    # Define key indices (same as in piano_mjcf.py)
    WHITE_KEY_INDICES = [
        0, 2, 3, 5, 7, 8, 10, 12, 14, 15, 17, 19, 20, 22, 24, 26, 27, 29, 31, 32,
        34, 36, 38, 39, 41, 43, 44, 46, 48, 50, 51, 53, 55, 56, 58, 60, 62, 63,
        65, 67, 68, 70, 72, 74, 75, 77, 79, 80, 82, 84, 86, 87,
    ]
    
    BLACK_TWIN_KEY_INDICES = [
        4, 6, 16, 18, 28, 30, 40, 42, 52, 54, 64, 66, 76, 78,
    ]
    
    BLACK_TRIPLET_KEY_INDICES = [
        1, 9, 11, 13, 21, 23, 25, 33, 35, 37, 45, 47, 49, 57, 59, 61, 69, 71, 73,
        81, 83, 85,
    ]
    
    print(f"Creating {consts.NUM_WHITE_KEYS} white keys...")
    # Create white keys
    for i in range(consts.NUM_WHITE_KEYS):
        y_coord = (
            -consts.PIANO_LENGTH * 0.5
            + consts.WHITE_KEY_WIDTH * 0.5
            + i * (consts.WHITE_KEY_WIDTH + consts.SPACING_BETWEEN_WHITE_KEYS)
        )
        key_index = WHITE_KEY_INDICES[i]
        create_key(
            stage=stage,
            key_index=key_index,
            is_white=True,
            position=[consts.WHITE_KEY_X_OFFSET, y_coord, consts.WHITE_KEY_Z_OFFSET],
            parent_path=base_path,
            physics_material=key_material,
        )
        
        if (i + 1) % 10 == 0:
            print(f"  Created {i + 1}/{consts.NUM_WHITE_KEYS} white keys...")
    
    print(f"Creating black keys...")
    # Create black keys
    
    # Lone black key on far left
    y_coord = consts.WHITE_KEY_WIDTH + 0.5 * (
        -consts.PIANO_LENGTH + consts.SPACING_BETWEEN_WHITE_KEYS
    )
    key_index = BLACK_TRIPLET_KEY_INDICES[0]
    create_key(
        stage=stage,
        key_index=key_index,
        is_white=False,
        position=[consts.BLACK_KEY_X_OFFSET, y_coord, consts.BLACK_KEY_Z_OFFSET],
        parent_path=base_path,
        physics_material=key_material,
    )
    
    # Twin black keys
    n = 0
    TWIN_INDICES = list(range(2, consts.NUM_WHITE_KEYS - 1, 7))
    for twin_index in TWIN_INDICES:
        for j in range(2):
            y_coord = (
                -consts.PIANO_LENGTH * 0.5
                + (j + 1) * (consts.WHITE_KEY_WIDTH + consts.SPACING_BETWEEN_WHITE_KEYS)
                + twin_index * (consts.WHITE_KEY_WIDTH + consts.SPACING_BETWEEN_WHITE_KEYS)
            )
            key_index = BLACK_TWIN_KEY_INDICES[n]
            create_key(
                stage=stage,
                key_index=key_index,
                is_white=False,
                position=[consts.BLACK_KEY_X_OFFSET, y_coord, consts.BLACK_KEY_Z_OFFSET],
                parent_path=base_path,
                physics_material=key_material,
            )
            n += 1
    
    # Triplet black keys
    n = 1  # Skip the lone black key
    TRIPLET_INDICES = list(range(5, consts.NUM_WHITE_KEYS - 1, 7))
    for triplet_index in TRIPLET_INDICES:
        for j in range(3):
            y_coord = (
                -consts.PIANO_LENGTH * 0.5
                + (j + 1) * (consts.WHITE_KEY_WIDTH + consts.SPACING_BETWEEN_WHITE_KEYS)
                + triplet_index * (consts.WHITE_KEY_WIDTH + consts.SPACING_BETWEEN_WHITE_KEYS)
            )
            key_index = BLACK_TRIPLET_KEY_INDICES[n]
            create_key(
                stage=stage,
                key_index=key_index,
                is_white=False,
                position=[consts.BLACK_KEY_X_OFFSET, y_coord, consts.BLACK_KEY_Z_OFFSET],
                parent_path=base_path,
                physics_material=key_material,
            )
            n += 1
    
    print(f"  Created {consts.NUM_KEYS - consts.NUM_WHITE_KEYS} black keys")
    
    return stage


def create_key(stage, key_index, is_white, position, parent_path, physics_material):
    """
    Create a single piano key with full physics properties.
    
    Args:
        stage: USD stage
        key_index: Key number (0-87)
        is_white: True for white key, False for black key
        position: [x, y, z] position in MuJoCo coordinates
        parent_path: Path to parent body (base)
        physics_material: Physics material path
    """
    key_type = "white" if is_white else "black"
    key_name = f"{key_type}_key_{key_index}"
    key_path = f"/Piano/{key_name}"
    
    # Get key properties
    if is_white:
        key_length = consts.WHITE_KEY_LENGTH
        key_width = consts.WHITE_KEY_WIDTH
        key_height = consts.WHITE_KEY_HEIGHT
        key_size = [key_length / 2, key_width / 2, key_height / 2]
        key_mass = consts.WHITE_KEY_MASS
        key_color = consts.WHITE_KEY_COLOR
        joint_max_angle = consts.WHITE_KEY_JOINT_MAX_ANGLE
        stiffness = consts.WHITE_KEY_STIFFNESS
        damping = consts.WHITE_JOINT_DAMPING
        spring_ref = consts.WHITE_KEY_SPRINGREF * math.pi / 180
        armature = consts.WHITE_JOINT_ARMATURE
    else:
        key_length = consts.BLACK_KEY_LENGTH
        key_width = consts.BLACK_KEY_WIDTH
        key_height = consts.BLACK_KEY_HEIGHT
        key_size = [key_length / 2, key_width / 2, key_height / 2]
        key_mass = consts.BLACK_KEY_MASS
        key_color = consts.BLACK_KEY_COLOR
        joint_max_angle = consts.BLACK_KEY_JOINT_MAX_ANGLE
        stiffness = consts.BLACK_KEY_STIFFNESS
        damping = consts.BLACK_JOINT_DAMPING
        spring_ref = consts.BLACK_KEY_SPRINGREF * math.pi / 180
        armature = consts.BLACK_JOINT_ARMATURE
    
    # Create key Xform
    key_xform = UsdGeom.Xform.Define(stage, key_path)
    key_transform = key_xform.AddTranslateOp()
    # Both MuJoCo and USD use Z-up, so keep coordinates as-is
    key_transform.Set(Gf.Vec3d(position[0], position[1], position[2]))
    
    # Create visual mesh
    mesh_path = f"{key_path}/{key_name}_mesh"
    key_mesh = UsdGeom.Mesh.Define(stage, mesh_path)
    create_visual_box_mesh(key_mesh, key_size, key_color)
    
    # Create collision geometry
    collision_path = f"{key_path}/{key_name}_collision"
    create_box_collision(stage, collision_path, key_size, physics_material)
    
    # Add rigid body API with mass
    rb_api = UsdPhysics.RigidBodyAPI.Apply(key_xform.GetPrim())
    
    # Add mass API
    mass_api = UsdPhysics.MassAPI.Apply(key_xform.GetPrim())
    mass_api.CreateMassAttr().Set(key_mass)
    
    # Compute center of mass (at geometric center for box)
    mass_api.CreateCenterOfMassAttr().Set(Gf.Vec3f(0, 0, 0))
    
    # Create revolute joint
    joint_name = f"{key_type}_joint_{key_index}"
    joint_path = f"/Piano/{joint_name}"
    
    # Joint position: at the back of the key (in key's local frame)
    # Both MuJoCo and USD use Z-up, joint is at [-key_length/2, 0, 0]
    joint_local_pos = Gf.Vec3d(-key_length / 2, 0, 0)
    
    # Joint axis: Y-axis in both MuJoCo and USD (horizontal axis for key rotation)
    joint_axis = UsdPhysics.Tokens.y
    
    # Create joint with spring properties
    add_joint_with_spring(
        stage=stage,
        joint_path=joint_path,
        body0_path=parent_path,
        body1_path=key_path,
        joint_local_pos=joint_local_pos,
        axis=joint_axis,
        lower_limit=0.0,  # degrees
        upper_limit=math.degrees(joint_max_angle),  # degrees
        stiffness=stiffness,
        damping=damping,
        spring_ref=spring_ref,
        armature=armature,
        body1_position=position,  # Pass key position for proper joint anchor computation
        body0_position=consts.BASE_POS,  # Pass base position to compute LocalPos0 correctly
    )


def main():
    """Generate piano USDA file with full physics."""
    out_dir = Path(__file__).parent
    out_file = out_dir / "piano_with_physics.usda"
    
    print("="*70)
    print("PIANO USD GENERATOR WITH FULL PHYSICS")
    print("="*70)
    print()
    print("This script generates a comprehensive USD file with:")
    print("  • Rigid body dynamics (masses, inertias)")
    print("  • Revolute joints with spring properties")
    print("  • Joint limits and ranges")
    print("  • Spring stiffness and damping")
    print("  • Collision geometries")
    print("  • Physics materials")
    print()
    print(f"Output file: {out_file}")
    print()
    
    # Build USD stage
    stage = build_piano_usd_with_physics(out_file)
    
    # Save the stage
    print()
    print("Saving USD file...")
    stage.Save()
    
    print()
    print("="*70)
    print("✓ SUCCESS!")
    print("="*70)
    print(f"USD file created: {out_file}")
    print()
    print("File contains:")
    print(f"  • {consts.NUM_KEYS} piano keys with full physics")
    print(f"  • {consts.NUM_KEYS} revolute joints with springs")
    print(f"  • Collision geometries for all keys")
    print(f"  • Physics materials and properties")
    print()
    print("Physics properties preserved from MuJoCo XML:")
    print(f"  • Joint stiffness: {consts.WHITE_KEY_STIFFNESS} Nm/rad")
    print(f"  • Joint damping: {consts.WHITE_JOINT_DAMPING}")
    print(f"  • Spring reference: {consts.WHITE_KEY_SPRINGREF}° (matches rest position)")
    print(f"  • White key mass: {consts.WHITE_KEY_MASS} kg")
    print(f"  • Black key mass: {consts.BLACK_KEY_MASS} kg")
    print()
    print("You can:")
    print("  • View in USD Composer / Omniverse")
    print("  • Import into Blender (with USD plugin)")
    print("  • Use in physics simulations supporting USD Physics")
    print("  • Reference from other USD files")
    print()
    print("Note: The USD file contains standard USD Physics schemas.")
    print("For MuJoCo-specific features, custom attributes are included.")
    print("="*70)


if __name__ == "__main__":
    main()

