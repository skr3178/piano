# Copyright 2023 The RoboPianist Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Export piano MJCF model to USD format and build USD from scratch."""

import math
from pathlib import Path
from typing import TYPE_CHECKING, Optional

from dm_control import mjcf
from mujoco_utils import types

from robopianist.models.piano import piano_constants as consts

if TYPE_CHECKING:
    from pxr import Usd

try:
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, Vt
    USD_AVAILABLE = True
except ImportError:
    USD_AVAILABLE = False
    Usd = None  # type: ignore
    UsdPhysics = None  # type: ignore


def build_usd(
    file_path: Optional[str | Path] = None,
    add_physics: bool = True,
    add_actuators: bool = False,
) -> "Usd.Stage":
    """Programmatically build a piano USD model from scratch.
    
    This function creates a USD stage programmatically, similar to how
    piano_mjcf.build() creates an MJCF model. It uses the USD Python API
    to create all prims, meshes, and transforms directly.
    
    Args:
        file_path: Optional path to save the USD file. If None, creates
            an in-memory stage.
        add_physics: Whether to add physics properties (masses, joints, etc.).
            If True, includes RigidBodyAPI, MassAPI, and RevoluteJoints for keys.
        add_actuators: Whether to add actuators to the piano keys (requires
            add_physics=True). Note: Actuators in USD require MjcPhysics schemas.
    
    Returns:
        Usd.Stage: The created USD stage.
    
    Example:
        ```python
        from robopianist.models.piano import piano_usd
        
        # Create USD stage in memory (visual only)
        stage = piano_usd.build_usd(add_physics=False)
        
        # Create with physics properties
        stage = piano_usd.build_usd("piano.usda", add_physics=True)
        stage.Save()
        ```
    """
    if not USD_AVAILABLE:
        raise ImportError(
            "USD Python API (pxr) is not available. "
            "Please install usd-core or similar package."
        )
    
    # Create stage (in-memory or file-based)
    if file_path is not None:
        file_path = Path(file_path)
        file_path.parent.mkdir(parents=True, exist_ok=True)
        stage = Usd.Stage.CreateNew(str(file_path))
    else:
        stage = Usd.Stage.CreateInMemory()
    
    # Set stage metadata
    stage.SetMetadata("defaultPrim", "piano")
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)  # Z-up (USD convention)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    if add_physics:
        UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    
    # Create root piano Xform
    root_prim = stage.DefinePrim("/piano", "Xform")
    stage.SetDefaultPrim(root_prim)
    root_xform = UsdGeom.Xform(root_prim)
    
    # Create physics scene if needed
    if add_physics:
        physics_scene = UsdPhysics.Scene.Define(stage, "/PhysicsScene")
        physics_scene_prim = physics_scene.GetPrim()
        # Try to apply MjcPhysicsSceneAPI if available (optional)
        try:
            physics_scene_prim.ApplyAPI(Usd.SchemaRegistry.GetSchemaTypeName("MjcPhysicsSceneAPI"))
        except Exception:
            pass  # MjcPhysics schemas may not be available
    
    # Add base (static/kinematic body)
    base_xform = UsdGeom.Xform.Define(stage, "/piano/base")
    base_transform = base_xform.AddTranslateOp()
    # Convert MuJoCo coordinates (Y-up) to USD (Z-up): (x, y, z) -> (x, z, -y)
    base_pos = consts.BASE_POS
    base_transform.Set(Gf.Vec3d(base_pos[0], base_pos[2], -base_pos[1]))
    
    base_mesh = UsdGeom.Mesh.Define(stage, "/piano/base/base_geom")
    _create_box_mesh(base_mesh, consts.BASE_SIZE, consts.BASE_COLOR)
    
    if add_physics:
        # Base is kinematic (static)
        base_rb = UsdPhysics.RigidBodyAPI.Apply(base_xform.GetPrim())
        base_rb.CreateKinematicEnabledAttr().Set(True)
    
    # Create white keys
    WHITE_KEY_INDICES = [
        0, 2, 3, 5, 7, 8, 10, 12, 14, 15, 17, 19, 20, 22, 24, 26, 27, 29, 31, 32,
        34, 36, 38, 39, 41, 43, 44, 46, 48, 50, 51, 53, 55, 56, 58, 60, 62, 63,
        65, 67, 68, 70, 72, 74, 75, 77, 79, 80, 82, 84, 86, 87,
    ]
    
    for i in range(consts.NUM_WHITE_KEYS):
        y_coord = (
            -consts.PIANO_LENGTH * 0.5
            + consts.WHITE_KEY_WIDTH * 0.5
            + i * (consts.WHITE_KEY_WIDTH + consts.SPACING_BETWEEN_WHITE_KEYS)
        )
        key_index = WHITE_KEY_INDICES[i]
        key_name = f"white_key_{key_index}"
        key_path = f"/piano/{key_name}"
        
        key_xform = UsdGeom.Xform.Define(stage, key_path)
        key_transform = key_xform.AddTranslateOp()
        key_pos = [
            consts.WHITE_KEY_X_OFFSET,
            y_coord,
            consts.WHITE_KEY_Z_OFFSET,
        ]
        key_transform.Set(Gf.Vec3d(key_pos[0], key_pos[2], -key_pos[1]))
        
        key_mesh = UsdGeom.Mesh.Define(stage, f"{key_path}/{key_name}_geom")
        key_size = [
            consts.WHITE_KEY_LENGTH / 2,
            consts.WHITE_KEY_WIDTH / 2,
            consts.WHITE_KEY_HEIGHT / 2,
        ]
        _create_box_mesh(key_mesh, key_size, consts.WHITE_KEY_COLOR)
        
        if add_physics:
            _add_key_physics(
                stage,
                key_xform.GetPrim(),
                key_name,
                is_white=True,
                key_size=key_size,
                key_pos=key_pos,
                add_actuators=add_actuators,
            )
    
    # Create black keys
    BLACK_TWIN_KEY_INDICES = [
        4, 6, 16, 18, 28, 30, 40, 42, 52, 54, 64, 66, 76, 78,
    ]
    BLACK_TRIPLET_KEY_INDICES = [
        1, 9, 11, 13, 21, 23, 25, 33, 35, 37, 45, 47, 49, 57, 59, 61, 69, 71, 73,
        81, 83, 85,
    ]
    
    # Place the lone black key on the far left
    y_coord = consts.WHITE_KEY_WIDTH + 0.5 * (
        -consts.PIANO_LENGTH + consts.SPACING_BETWEEN_WHITE_KEYS
    )
    key_index = BLACK_TRIPLET_KEY_INDICES[0]
    key_name = f"black_key_{key_index}"
    key_path = f"/piano/{key_name}"
    
    key_xform = UsdGeom.Xform.Define(stage, key_path)
    key_transform = key_xform.AddTranslateOp()
    key_pos = [
        consts.BLACK_KEY_X_OFFSET,
        y_coord,
        consts.BLACK_KEY_Z_OFFSET,
    ]
    key_transform.Set(Gf.Vec3d(key_pos[0], key_pos[2], -key_pos[1]))
    
    key_mesh = UsdGeom.Mesh.Define(stage, f"{key_path}/{key_name}_geom")
    key_size = [
        consts.BLACK_KEY_LENGTH / 2,
        consts.BLACK_KEY_WIDTH / 2,
        consts.BLACK_KEY_HEIGHT / 2,
    ]
    _create_box_mesh(key_mesh, key_size, consts.BLACK_KEY_COLOR)
    
    if add_physics:
        _add_key_physics(
            stage,
            key_xform.GetPrim(),
            key_name,
            is_white=False,
            key_size=key_size,
            key_pos=key_pos,
            add_actuators=add_actuators,
        )
    
    # Place the twin black keys
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
            key_name = f"black_key_{key_index}"
            key_path = f"/piano/{key_name}"
            
            key_xform = UsdGeom.Xform.Define(stage, key_path)
            key_transform = key_xform.AddTranslateOp()
            key_pos = [
                consts.BLACK_KEY_X_OFFSET,
                y_coord,
                consts.BLACK_KEY_Z_OFFSET,
            ]
            key_transform.Set(Gf.Vec3d(key_pos[0], key_pos[2], -key_pos[1]))
            
            key_mesh = UsdGeom.Mesh.Define(stage, f"{key_path}/{key_name}_geom")
            key_size = [
                consts.BLACK_KEY_LENGTH / 2,
                consts.BLACK_KEY_WIDTH / 2,
                consts.BLACK_KEY_HEIGHT / 2,
            ]
            _create_box_mesh(key_mesh, key_size, consts.BLACK_KEY_COLOR)
            
            if add_physics:
                _add_key_physics(
                    stage,
                    key_xform.GetPrim(),
                    key_name,
                    is_white=False,
                    key_size=key_size,
                    key_pos=key_pos,
                    add_actuators=add_actuators,
                )
            n += 1
    
    # Place the triplet black keys
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
            key_name = f"black_key_{key_index}"
            key_path = f"/piano/{key_name}"
            
            key_xform = UsdGeom.Xform.Define(stage, key_path)
            key_transform = key_xform.AddTranslateOp()
            key_pos = [
                consts.BLACK_KEY_X_OFFSET,
                y_coord,
                consts.BLACK_KEY_Z_OFFSET,
            ]
            key_transform.Set(Gf.Vec3d(key_pos[0], key_pos[2], -key_pos[1]))
            
            key_mesh = UsdGeom.Mesh.Define(stage, f"{key_path}/{key_name}_geom")
            key_size = [
                consts.BLACK_KEY_LENGTH / 2,
                consts.BLACK_KEY_WIDTH / 2,
                consts.BLACK_KEY_HEIGHT / 2,
            ]
            _create_box_mesh(key_mesh, key_size, consts.BLACK_KEY_COLOR)
            
            if add_physics:
                _add_key_physics(
                    stage,
                    key_xform.GetPrim(),
                    key_name,
                    is_white=False,
                    key_size=key_size,
                    key_pos=key_pos,
                    add_actuators=add_actuators,
                )
            n += 1
    
    return stage


def _add_key_physics(
    stage: "Usd.Stage",
    key_prim: "Usd.Prim",
    key_name: str,
    is_white: bool,
    key_size: list[float],
    key_pos: list[float],
    add_actuators: bool,
) -> None:
    """Add physics properties to a key (mass, joint, etc.).
    
    Args:
        stage: The USD stage.
        key_prim: The key prim to add physics to.
        key_name: Name of the key.
        is_white: Whether this is a white or black key.
        key_size: Half-extents of the key.
        key_pos: Position of the key.
        add_actuators: Whether to add actuators.
    """
    if not USD_AVAILABLE or UsdPhysics is None:
        return
    
    # Add RigidBodyAPI
    rb_api = UsdPhysics.RigidBodyAPI.Apply(key_prim)
    
    # Add MassAPI with mass
    mass_api = UsdPhysics.MassAPI.Apply(key_prim)
    if is_white:
        mass_api.CreateMassAttr().Set(consts.WHITE_KEY_MASS)
    else:
        mass_api.CreateMassAttr().Set(consts.BLACK_KEY_MASS)
    
    # Create joint between key and base/piano root
    # Joint position is at the back of the key (MuJoCo convention)
    joint_name = f"{key_name}_joint"
    joint_path = f"{key_prim.GetPath()}/{joint_name}"
    
    # Joint axis is Y-axis (up) in MuJoCo, which becomes Z-axis in USD
    joint_axis = Gf.Vec3f(0, 0, 1)  # Z-axis in USD (was Y in MuJoCo)
    
    # Joint position: at the back of the key relative to key center
    if is_white:
        joint_pos_rel = [-consts.WHITE_KEY_LENGTH / 2, 0, 0]
    else:
        joint_pos_rel = [-consts.BLACK_KEY_LENGTH / 2, 0, 0]
    
    # Convert joint position to USD coordinates
    joint_pos_usd = Gf.Vec3d(
        joint_pos_rel[0],
        joint_pos_rel[2],
        -joint_pos_rel[1],
    )
    
    # Parent body is the base (or piano root for articulation)
    base_prim = stage.GetPrimAtPath("/piano/base")
    body0 = base_prim if base_prim.IsValid() else stage.GetDefaultPrim()
    body1 = key_prim
    
    # Create revolute joint
    joint = UsdPhysics.RevoluteJoint.Define(stage, joint_path)
    joint.CreateBody0Rel().SetTargets([body0.GetPath()])
    joint.CreateBody1Rel().SetTargets([body1.GetPath()])
    
    # Set joint axis (Y in MuJoCo -> Z in USD)
    joint.CreateAxisAttr().Set(UsdPhysics.Tokens.z)
    
    # Set joint limits
    if is_white:
        joint.CreateLowerLimitAttr().Set(0.0)
        joint.CreateUpperLimitAttr().Set(math.degrees(consts.WHITE_KEY_JOINT_MAX_ANGLE))
        springref_deg = consts.WHITE_KEY_SPRINGREF
        stiffness = consts.WHITE_KEY_STIFFNESS
        damping = consts.WHITE_JOINT_DAMPING
        armature = consts.WHITE_JOINT_ARMATURE
    else:
        joint.CreateLowerLimitAttr().Set(0.0)
        joint.CreateUpperLimitAttr().Set(math.degrees(consts.BLACK_KEY_JOINT_MAX_ANGLE))
        springref_deg = consts.BLACK_KEY_SPRINGREF
        stiffness = consts.BLACK_KEY_STIFFNESS
        damping = consts.BLACK_JOINT_DAMPING
        armature = consts.BLACK_JOINT_ARMATURE
    
    # Set joint frame (local transform)
    # The joint is positioned at the back of the key
    joint_frame_xform = UsdGeom.Xform(joint.GetPrim())
    joint_frame_xform.AddTranslateOp().Set(joint_pos_usd)
    
    # Try to apply MjcJointAPI for MuJoCo-specific properties
    # This requires MjcPhysics schemas to be available
    try:
        joint_prim = joint.GetPrim()
        joint_prim.ApplyAPI(Usd.SchemaRegistry.GetSchemaTypeName("MjcPhysicsJointAPI"))
        
        # Set MuJoCo-specific joint properties
        joint_prim.CreateAttribute(
            "mjc:damping", Sdf.ValueTypeNames.Float, custom=True
        ).Set(damping)
        joint_prim.CreateAttribute(
            "mjc:stiffness", Sdf.ValueTypeNames.Float, custom=True
        ).Set(stiffness)
        joint_prim.CreateAttribute(
            "mjc:springref", Sdf.ValueTypeNames.Float, custom=True
        ).Set(math.radians(springref_deg))
        joint_prim.CreateAttribute(
            "mjc:armature", Sdf.ValueTypeNames.Float, custom=True
        ).Set(armature)
    except Exception:
        # MjcPhysics schemas may not be available - this is optional
        pass
    
    # Add collision API to the mesh
    mesh_prim = stage.GetPrimAtPath(f"{key_prim.GetPath()}/{key_name}_geom")
    if mesh_prim.IsValid():
        collision_api = UsdPhysics.CollisionAPI.Apply(mesh_prim)
    
    # Add actuators if requested (requires MjcPhysics schemas)
    if add_actuators:
        try:
            actuator_name = f"{key_name}_actuator"
            actuator_path = f"{key_prim.GetPath()}/{actuator_name}"
            actuator_prim = stage.DefinePrim(actuator_path, "Xform")
            actuator_prim.ApplyAPI(Usd.SchemaRegistry.GetSchemaTypeName("MjcActuatorAPI"))
            
            # Set actuator properties
            actuator_prim.CreateAttribute(
                "mjc:joint", Sdf.ValueTypeNames.Token, custom=True
            ).Set(joint_name)
            actuator_prim.CreateAttribute(
                "mjc:ctrlrange:min", Sdf.ValueTypeNames.Float, custom=True
            ).Set(0.0)
            if is_white:
                actuator_prim.CreateAttribute(
                    "mjc:ctrlrange:max", Sdf.ValueTypeNames.Float, custom=True
                ).Set(consts.WHITE_KEY_JOINT_MAX_ANGLE)
            else:
                actuator_prim.CreateAttribute(
                    "mjc:ctrlrange:max", Sdf.ValueTypeNames.Float, custom=True
                ).Set(consts.BLACK_KEY_JOINT_MAX_ANGLE)
        except Exception:
            # Actuators require MjcPhysics schemas
            pass


def _create_box_mesh(mesh: UsdGeom.Mesh, size: list[float], color: list[float]) -> None:
    """Create a box mesh from half-extents.
    
    Args:
        mesh: The UsdGeom.Mesh to configure.
        size: Half-extents [length/2, width/2, height/2].
        color: RGBA color [r, g, b, a].
    """
    # Create box vertices (8 vertices of a box)
    # Box extends from -size to +size in each dimension
    x, y, z = size[0], size[1], size[2]
    
    points = [
        Gf.Vec3f(-x, -z, -y),  # 0: back-left-bottom (MuJoCo Y-up -> USD Z-up)
        Gf.Vec3f(x, -z, -y),   # 1: back-right-bottom
        Gf.Vec3f(-x, z, -y),   # 2: back-left-top
        Gf.Vec3f(x, z, -y),    # 3: back-right-top
        Gf.Vec3f(-x, -z, y),   # 4: front-left-bottom
        Gf.Vec3f(x, -z, y),    # 5: front-right-bottom
        Gf.Vec3f(-x, z, y),    # 6: front-left-top
        Gf.Vec3f(x, z, y),     # 7: front-right-top
    ]
    
    # Box faces (6 faces, each with 4 vertices)
    face_vertex_counts = [4, 4, 4, 4, 4, 4]
    face_vertex_indices = [
        0, 1, 3, 2,  # back face
        0, 4, 5, 1,  # bottom face
        0, 2, 6, 4,  # left face
        2, 3, 7, 6,  # top face
        4, 6, 7, 5,  # front face
        1, 5, 7, 3,  # right face
    ]
    
    mesh.GetPointsAttr().Set(points)
    mesh.GetFaceVertexCountsAttr().Set(face_vertex_counts)
    mesh.GetFaceVertexIndicesAttr().Set(face_vertex_indices)
    mesh.GetSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
    
    # Set color
    color_primvar = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
        "displayColor", Sdf.ValueTypeNames.Color3fArray
    )
    color_primvar.Set([Gf.Vec3f(color[0], color[1], color[2])])


def export_to_usd(
    mjcf_model: types.MjcfRootElement,
    out_file: str | Path,
    out_dir: Optional[str | Path] = None,
) -> Path:
    """Export an MJCF piano model to USD format.

    This function converts the programmatically built piano MJCF model to USD
    format, similar to how export_with_assets works for XML files.

    Args:
        mjcf_model: The MJCF root element (from piano_mjcf.build() or
            task.root_entity.mjcf_model).
        out_file: Output USD file path (e.g., "piano.usda").
        out_dir: Optional output directory. If None, uses the directory of out_file.

    Returns:
        Path to the created USD file.

    Example:
        ```python
        from robopianist.models.piano import piano_mjcf, piano_usd
        
        piano_model = piano_mjcf.build()
        piano_usd.export_to_usd(
            piano_model,
            out_file="piano.usda",
            out_dir="/tmp/robopianist"
        )
        ```
    """
    out_path = Path(out_file)
    if out_dir is not None:
        out_dir = Path(out_dir)
        out_dir.mkdir(parents=True, exist_ok=True)
        out_path = out_dir / out_path.name
    
    out_path.parent.mkdir(parents=True, exist_ok=True)
    
    # Generate USD ASCII format
    usd_content = _generate_usd_content(mjcf_model)
    
    # Write to file
    with open(out_path, 'w') as f:
        f.write(usd_content)
    
    return out_path


def _generate_usd_content(mjcf_model: types.MjcfRootElement) -> str:
    """Generate USD ASCII content from MJCF model."""
    
    lines = [
        "#usda 1.0",
        "",
        'def Xform "piano" {',
        '    token visibility = "inherited"',
        '    double3 xformOp:translate = (0, 0, 0)',
        '    uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:rotateXYZ", "xformOp:scale"]',
        "",
    ]
    
    # Export base
    base_body = mjcf_model.find('body', 'base')
    if base_body:
        base_pos = getattr(base_body, 'pos', consts.BASE_POS)
        base_size = consts.BASE_SIZE
        base_color = consts.BASE_COLOR
        
        lines.append('    def Xform "base" {')
        lines.append(f'        double3 xformOp:translate = ({base_pos[0]}, {base_pos[2]}, {-base_pos[1]})')
        lines.append('        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:rotateXYZ", "xformOp:scale"]')
        lines.append('')
        lines.append('        def Mesh "base_geom" {')
        lines.append('            int[] faceVertexCounts = [4, 4, 4, 4, 4, 4]')
        lines.append('            int[] faceVertexIndices = [0, 1, 3, 2, 0, 4, 5, 1, 0, 2, 6, 4, 2, 3, 7, 6, 4, 6, 7, 5, 1, 5, 7, 3]')
        lines.append(f'            point3f[] points = [(-{base_size[0]}, -{base_size[2]}, -{base_size[1]}), ({base_size[0]}, -{base_size[2]}, -{base_size[1]}), (-{base_size[0]}, {base_size[2]}, -{base_size[1]}), ({base_size[0]}, {base_size[2]}, -{base_size[1]}), (-{base_size[0]}, -{base_size[2]}, {base_size[1]}), ({base_size[0]}, -{base_size[2]}, {base_size[1]}), (-{base_size[0]}, {base_size[2]}, {base_size[1]}), ({base_size[0]}, {base_size[2]}, {base_size[1]})]')
        lines.append('            color3f[] primvars:displayColor = [')
        lines.append(f'                ({base_color[0]}, {base_color[1]}, {base_color[2]})')
        lines.append('            ]')
        lines.append('            uniform token subdivisionScheme = "none"')
        lines.append('        }')
        lines.append('    }')
        lines.append('')
    
    # Export keys (white and black)
    keys = _get_all_keys(mjcf_model)
    
    for key_body in keys:
        key_name = key_body.name
        is_white = 'white' in key_name
        
        # Get position
        key_pos = getattr(key_body, 'pos', [0, 0, 0])
        
        # Get geometry size - use find_all to get geoms
        geoms = key_body.find_all('geom')
        if geoms and len(geoms) > 0:
            geom = geoms[0]
            size = getattr(geom, 'size', None)
            if size is None:
                # Use defaults
                if is_white:
                    size = [
                        consts.WHITE_KEY_LENGTH / 2,
                        consts.WHITE_KEY_WIDTH / 2,
                        consts.WHITE_KEY_HEIGHT / 2,
                    ]
                else:
                    size = [
                        consts.BLACK_KEY_LENGTH / 2,
                        consts.BLACK_KEY_WIDTH / 2,
                        consts.BLACK_KEY_HEIGHT / 2,
                    ]
        else:
            # Use defaults if no geom found
            if is_white:
                size = [
                    consts.WHITE_KEY_LENGTH / 2,
                    consts.WHITE_KEY_WIDTH / 2,
                    consts.WHITE_KEY_HEIGHT / 2,
                ]
            else:
                size = [
                    consts.BLACK_KEY_LENGTH / 2,
                    consts.BLACK_KEY_WIDTH / 2,
                    consts.BLACK_KEY_HEIGHT / 2,
                ]
        
        # Get color
        if is_white:
            color = consts.WHITE_KEY_COLOR
        else:
            color = consts.BLACK_KEY_COLOR
        
        lines.append(f'    def Xform "{key_name}" {{')
        # Convert MuJoCo coordinates (Y-up) to USD coordinates (Z-up)
        lines.append(f'        double3 xformOp:translate = ({key_pos[0]}, {key_pos[2]}, {-key_pos[1]})')
        lines.append('        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:rotateXYZ", "xformOp:scale"]')
        lines.append('')
        lines.append(f'        def Mesh "{key_name}_geom" {{')
        lines.append('            int[] faceVertexCounts = [4, 4, 4, 4, 4, 4]')
        lines.append('            int[] faceVertexIndices = [0, 1, 3, 2, 0, 4, 5, 1, 0, 2, 6, 4, 2, 3, 7, 6, 4, 6, 7, 5, 1, 5, 7, 3]')
        lines.append(f'            point3f[] points = [(-{size[0]}, -{size[2]}, -{size[1]}), ({size[0]}, -{size[2]}, -{size[1]}), (-{size[0]}, {size[2]}, -{size[1]}), ({size[0]}, {size[2]}, -{size[1]}), (-{size[0]}, -{size[2]}, {size[1]}), ({size[0]}, -{size[2]}, {size[1]}), (-{size[0]}, {size[2]}, {size[1]}), ({size[0]}, {size[2]}, {size[1]})]')
        lines.append('            color3f[] primvars:displayColor = [')
        lines.append(f'                ({color[0]}, {color[1]}, {color[2]})')
        lines.append('            ]')
        lines.append('            uniform token subdivisionScheme = "none"')
        lines.append('        }')
        lines.append('    }')
        lines.append('')
    
    lines.append('}')
    
    return '\n'.join(lines)


def _get_all_keys(mjcf_model: types.MjcfRootElement) -> list:
    """Get all key bodies from the MJCF model."""
    all_bodies = mjcf_model.find_all('body')
    keys = []
    for body in all_bodies:
        if body.name and ('white_key' in body.name or 'black_key' in body.name):
            keys.append(body)
    # Sort by key number
    keys.sort(key=lambda b: int(b.name.split('_')[-1]) if b.name.split('_')[-1].isdigit() else 0)
    return keys

