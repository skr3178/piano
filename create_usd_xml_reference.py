#!/usr/bin/env python3
"""
Create a USD file that references the piano.xml for MuJoCo simulation.

This creates a lightweight USD file that contains visual geometry and
metadata pointing to the piano.xml file for physics simulation. This is
useful for workflows where:
1. USD is used for rendering/visualization
2. MuJoCo XML is used for physics simulation
3. Both need to stay in sync

The USD file contains:
- Visual geometry (meshes, colors, transforms)
- Custom metadata pointing to the XML file
- Asset structure matching the XML
"""

import sys
from pathlib import Path

try:
    from pxr import Usd, UsdGeom, Sdf, Gf
    USD_AVAILABLE = True
except ImportError:
    print("ERROR: USD Python API (pxr) not available.")
    print("Install with: pip install usd-core")
    sys.exit(1)

# Add robopianist to path
sys.path.insert(0, str(Path(__file__).parent / "robopianist"))
from robopianist.models.piano import piano_constants as consts


def create_visual_box_mesh(mesh, size, color):
    """Create a box mesh with vertices and faces."""
    x, y, z = size[0], size[1], size[2]
    
    # Both MuJoCo and USD use Z-up, so no coordinate conversion needed
    points = [
        Gf.Vec3f(-x, -y, -z),
        Gf.Vec3f(x, -y, -z),
        Gf.Vec3f(-x, y, -z),
        Gf.Vec3f(x, y, -z),
        Gf.Vec3f(-x, -y, z),
        Gf.Vec3f(x, -y, z),
        Gf.Vec3f(-x, y, z),
        Gf.Vec3f(x, y, z),
    ]
    
    face_vertex_counts = [4, 4, 4, 4, 4, 4]
    face_vertex_indices = [
        0, 1, 3, 2,
        0, 4, 5, 1,
        0, 2, 6, 4,
        2, 3, 7, 6,
        4, 6, 7, 5,
        1, 5, 7, 3,
    ]
    
    normals = [
        Gf.Vec3f(0, 0, -1),
        Gf.Vec3f(0, -1, 0),
        Gf.Vec3f(-1, 0, 0),
        Gf.Vec3f(0, 1, 0),
        Gf.Vec3f(0, 0, 1),
        Gf.Vec3f(1, 0, 0),
    ]
    
    mesh.GetPointsAttr().Set(points)
    mesh.GetFaceVertexCountsAttr().Set(face_vertex_counts)
    mesh.GetFaceVertexIndicesAttr().Set(face_vertex_indices)
    mesh.GetSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
    mesh.GetNormalsAttr().Set(normals)
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.uniform)
    
    color_primvar = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
        "displayColor", Sdf.ValueTypeNames.Color3fArray
    )
    color_primvar.Set([Gf.Vec3f(color[0], color[1], color[2])])
    color_primvar.SetInterpolation(UsdGeom.Tokens.constant)


def create_usd_with_xml_reference(out_file, xml_file):
    """
    Create a USD file with visual geometry and XML reference.
    
    Args:
        out_file: Output USD file path
        xml_file: Path to the MuJoCo XML file
    """
    print("Creating USD stage with XML reference...")
    stage = Usd.Stage.CreateNew(str(out_file))
    
    # Set stage metadata
    stage.SetMetadata("defaultPrim", "Piano")
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    
    # Create root
    piano_prim = stage.DefinePrim("/Piano", "Xform")
    stage.SetDefaultPrim(piano_prim)
    
    # Add custom metadata pointing to XML file
    piano_prim.SetCustomDataByKey("mujoco:xmlFile", str(xml_file))
    piano_prim.SetCustomDataByKey("mujoco:description", 
                                   "Visual USD representation. Physics in XML file.")
    piano_prim.SetDocumentation(
        f"This USD file provides visual geometry for rendering. "
        f"Physics simulation should use the MuJoCo XML file: {xml_file.name}"
    )
    
    # Create base
    print("Creating base...")
    base_xform = UsdGeom.Xform.Define(stage, "/Piano/Base")
    base_transform = base_xform.AddTranslateOp()
    base_pos = consts.BASE_POS
    base_transform.Set(Gf.Vec3d(base_pos[0], base_pos[1], base_pos[2]))
    
    base_mesh = UsdGeom.Mesh.Define(stage, "/Piano/Base/BaseMesh")
    create_visual_box_mesh(base_mesh, consts.BASE_SIZE, consts.BASE_COLOR)
    
    # Key indices
    WHITE_KEY_INDICES = [
        0, 2, 3, 5, 7, 8, 10, 12, 14, 15, 17, 19, 20, 22, 24, 26, 27, 29, 31, 32,
        34, 36, 38, 39, 41, 43, 44, 46, 48, 50, 51, 53, 55, 56, 58, 60, 62, 63,
        65, 67, 68, 70, 72, 74, 75, 77, 79, 80, 82, 84, 86, 87,
    ]
    
    BLACK_TWIN_KEY_INDICES = [4, 6, 16, 18, 28, 30, 40, 42, 52, 54, 64, 66, 76, 78]
    BLACK_TRIPLET_KEY_INDICES = [
        1, 9, 11, 13, 21, 23, 25, 33, 35, 37, 45, 47, 49, 57, 59, 61, 69, 71, 73,
        81, 83, 85,
    ]
    
    print("Creating white keys...")
    # White keys
    for i in range(consts.NUM_WHITE_KEYS):
        y_coord = (
            -consts.PIANO_LENGTH * 0.5
            + consts.WHITE_KEY_WIDTH * 0.5
            + i * (consts.WHITE_KEY_WIDTH + consts.SPACING_BETWEEN_WHITE_KEYS)
        )
        key_index = WHITE_KEY_INDICES[i]
        key_name = f"white_key_{key_index}"
        key_path = f"/Piano/{key_name}"
        
        key_xform = UsdGeom.Xform.Define(stage, key_path)
        key_transform = key_xform.AddTranslateOp()
        key_pos = [consts.WHITE_KEY_X_OFFSET, y_coord, consts.WHITE_KEY_Z_OFFSET]
        key_transform.Set(Gf.Vec3d(key_pos[0], key_pos[1], key_pos[2]))
        
        # Add XML body name as custom data
        key_xform.GetPrim().SetCustomDataByKey("mujoco:bodyName", key_name)
        
        key_mesh = UsdGeom.Mesh.Define(stage, f"{key_path}/Mesh")
        key_size = [
            consts.WHITE_KEY_LENGTH / 2,
            consts.WHITE_KEY_WIDTH / 2,
            consts.WHITE_KEY_HEIGHT / 2,
        ]
        create_visual_box_mesh(key_mesh, key_size, consts.WHITE_KEY_COLOR)
    
    print("Creating black keys...")
    # Lone black key
    y_coord = consts.WHITE_KEY_WIDTH + 0.5 * (
        -consts.PIANO_LENGTH + consts.SPACING_BETWEEN_WHITE_KEYS
    )
    key_index = BLACK_TRIPLET_KEY_INDICES[0]
    key_name = f"black_key_{key_index}"
    key_path = f"/Piano/{key_name}"
    
    key_xform = UsdGeom.Xform.Define(stage, key_path)
    key_transform = key_xform.AddTranslateOp()
    key_pos = [consts.BLACK_KEY_X_OFFSET, y_coord, consts.BLACK_KEY_Z_OFFSET]
    key_transform.Set(Gf.Vec3d(key_pos[0], key_pos[1], key_pos[2]))
    key_xform.GetPrim().SetCustomDataByKey("mujoco:bodyName", key_name)
    
    key_mesh = UsdGeom.Mesh.Define(stage, f"{key_path}/Mesh")
    key_size = [
        consts.BLACK_KEY_LENGTH / 2,
        consts.BLACK_KEY_WIDTH / 2,
        consts.BLACK_KEY_HEIGHT / 2,
    ]
    create_visual_box_mesh(key_mesh, key_size, consts.BLACK_KEY_COLOR)
    
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
            key_name = f"black_key_{key_index}"
            key_path = f"/Piano/{key_name}"
            
            key_xform = UsdGeom.Xform.Define(stage, key_path)
            key_transform = key_xform.AddTranslateOp()
            key_pos = [consts.BLACK_KEY_X_OFFSET, y_coord, consts.BLACK_KEY_Z_OFFSET]
            key_transform.Set(Gf.Vec3d(key_pos[0], key_pos[1], key_pos[2]))
            key_xform.GetPrim().SetCustomDataByKey("mujoco:bodyName", key_name)
            
            key_mesh = UsdGeom.Mesh.Define(stage, f"{key_path}/Mesh")
            create_visual_box_mesh(key_mesh, key_size, consts.BLACK_KEY_COLOR)
            n += 1
    
    # Triplet black keys
    n = 1
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
            key_path = f"/Piano/{key_name}"
            
            key_xform = UsdGeom.Xform.Define(stage, key_path)
            key_transform = key_xform.AddTranslateOp()
            key_pos = [consts.BLACK_KEY_X_OFFSET, y_coord, consts.BLACK_KEY_Z_OFFSET]
            key_transform.Set(Gf.Vec3d(key_pos[0], key_pos[1], key_pos[2]))
            key_xform.GetPrim().SetCustomDataByKey("mujoco:bodyName", key_name)
            
            key_mesh = UsdGeom.Mesh.Define(stage, f"{key_path}/Mesh")
            create_visual_box_mesh(key_mesh, key_size, consts.BLACK_KEY_COLOR)
            n += 1
    
    return stage


def main():
    """Create USD file with XML reference."""
    out_dir = Path(__file__).parent
    usd_file = out_dir / "piano_visual.usda"
    xml_file = out_dir / "piano.xml"
    
    if not xml_file.exists():
        print(f"ERROR: {xml_file} not found!")
        print("Please run generate_piano_xml.py first.")
        sys.exit(1)
    
    print("="*70)
    print("CREATE USD WITH XML REFERENCE")
    print("="*70)
    print()
    print("This creates a lightweight USD file for visualization that")
    print("references the MuJoCo XML file for physics simulation.")
    print()
    print(f"XML file (physics): {xml_file}")
    print(f"USD file (visual): {usd_file}")
    print()
    
    stage = create_usd_with_xml_reference(usd_file, xml_file)
    
    print("\nSaving USD file...")
    stage.Save()
    
    print()
    print("="*70)
    print("✓ SUCCESS!")
    print("="*70)
    print(f"Created: {usd_file}")
    print()
    print("File contains:")
    print("  • Visual geometry for 88 piano keys")
    print("  • Custom metadata pointing to piano.xml")
    print("  • Same structure as XML file")
    print()
    print("Usage workflow:")
    print("  1. Use piano_visual.usda for rendering/visualization")
    print("  2. Use piano.xml for physics simulation in MuJoCo")
    print("  3. Sync state between USD and XML using body names")
    print()
    print("The USD file is lightweight (~80 KB) and contains only visual data.")
    print("Physics simulation should always use the piano.xml file.")
    print("="*70)


if __name__ == "__main__":
    main()

