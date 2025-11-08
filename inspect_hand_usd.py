#!/usr/bin/env python3
"""
Inspect hand USD files to understand their structure, articulation roots, and collision geometry.
"""

import os
import sys
from pathlib import Path

try:
    from pxr import Usd, UsdPhysics, UsdGeom
    USD_AVAILABLE = True
except ImportError:
    USD_AVAILABLE = False
    print("Error: pxr (USD Python API) not available.")
    sys.exit(1)


def inspect_hand_usd(usd_path: str):
    """Inspect a hand USD file and print its structure."""
    print(f"\n{'='*60}")
    print(f"Inspecting: {usd_path}")
    print(f"{'='*60}\n")
    
    if not os.path.exists(usd_path):
        print(f"Error: File not found: {usd_path}")
        return
    
    stage = Usd.Stage.Open(usd_path)
    if not stage:
        print(f"Error: Failed to open USD file: {usd_path}")
        return
    
    # Get default prim
    default_prim = stage.GetDefaultPrim()
    if not default_prim:
        print("Warning: No default prim found")
        return
    
    print(f"Default Prim: {default_prim.GetPath()}")
    print(f"Prim Type: {default_prim.GetTypeName()}")
    
    # Check for ArticulationRootAPI
    if default_prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        print(f"\n✓ ArticulationRootAPI found at: {default_prim.GetPath()}")
        articulation_api = UsdPhysics.ArticulationRootAPI(default_prim)
        if articulation_api:
            try:
                solver_type = articulation_api.GetSolverTypeAttr()
                if solver_type:
                    print(f"  Solver Type: {solver_type.Get()}")
            except:
                pass
    else:
        print(f"\n✗ No ArticulationRootAPI at root")
    
    # Recursively find all articulation roots
    print("\nSearching for ArticulationRootAPI in hierarchy...")
    def find_articulation_roots(prim, level=0):
        indent = "  " * level
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            print(f"{indent}✓ ArticulationRootAPI at: {prim.GetPath()}")
        for child in prim.GetChildren():
            find_articulation_roots(child, level + 1)
    
    find_articulation_roots(default_prim)
    
    # Find collision prims
    print("\nSearching for collision prims...")
    collision_count = 0
    invalid_collision_count = 0
    
    def find_collisions(prim, level=0):
        nonlocal collision_count, invalid_collision_count
        indent = "  " * level
        
        # Check if this prim has CollisionAPI
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            collision_count += 1
            print(f"{indent}Collision prim: {prim.GetPath()}")
            
            # Check for geometry
            mesh = UsdGeom.Mesh(prim)
            if mesh:
                points = mesh.GetPointsAttr()
                if points:
                    points_val = points.Get()
                    if points_val and len(points_val) > 0:
                        print(f"{indent}  ✓ Has valid mesh geometry ({len(points_val)} points)")
                    else:
                        print(f"{indent}  ✗ Mesh has no points!")
                        invalid_collision_count += 1
                else:
                    print(f"{indent}  ✗ No points attribute!")
                    invalid_collision_count += 1
            else:
                # Check for other geometry types
                sphere = UsdGeom.Sphere(prim)
                cube = UsdGeom.Cube(prim)
                cylinder = UsdGeom.Cylinder(prim)
                capsule = UsdGeom.Capsule(prim)
                
                if sphere or cube or cylinder or capsule:
                    geom_type = "Sphere" if sphere else ("Cube" if cube else ("Cylinder" if cylinder else "Capsule"))
                    print(f"{indent}  ✓ Has {geom_type} geometry")
                else:
                    print(f"{indent}  ✗ No valid geometry type found!")
                    invalid_collision_count += 1
            
            # Check for MeshCollisionAPI
            if prim.HasAPI(UsdPhysics.MeshCollisionAPI):
                mesh_collision = UsdPhysics.MeshCollisionAPI(prim)
                print(f"{indent}  ✓ Has MeshCollisionAPI")
                try:
                    approx = mesh_collision.GetApproximationAttr()
                    if approx:
                        print(f"{indent}    Approximation: {approx.Get()}")
                except:
                    pass
        
        for child in prim.GetChildren():
            find_collisions(child, level + 1)
    
    find_collisions(default_prim)
    
    print(f"\nSummary:")
    print(f"  Total collision prims: {collision_count}")
    print(f"  Invalid collision prims: {invalid_collision_count}")
    
    # Print hierarchy
    print(f"\nPrim hierarchy:")
    def print_hierarchy(prim, level=0):
        indent = "  " * level
        prim_type = prim.GetTypeName()
        prim_path = prim.GetPath()
        print(f"{indent}{prim_path} ({prim_type})")
        for child in prim.GetChildren():
            print_hierarchy(child, level + 1)
    
    print_hierarchy(default_prim)


def main():
    """Main function."""
    base_dir = os.path.dirname(os.path.abspath(__file__))
    left_hand_usd = os.path.join(base_dir, "hand_left.usd")
    right_hand_usd = os.path.join(base_dir, "hand_right.usd")
    
    inspect_hand_usd(left_hand_usd)
    inspect_hand_usd(right_hand_usd)


if __name__ == "__main__":
    main()

