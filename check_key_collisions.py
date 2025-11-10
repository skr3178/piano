#!/usr/bin/env python3
"""
Check if all piano keys have collision meshes in the USD file.
"""

import sys
from pathlib import Path

try:
    from pxr import Usd, UsdPhysics
except ImportError:
    print("ERROR: USD Python API (pxr) is not available.")
    print("Please run this script in Isaac Sim environment or install usd-core")
    sys.exit(1)

# Import constants to know expected number of keys
try:
    from exts.omni.isaac.piano import piano_constants as consts
except ImportError:
    # Fallback
    NUM_KEYS = 88
    NUM_WHITE_KEYS = 52
    NUM_BLACK_KEYS = 36
else:
    NUM_KEYS = consts.NUM_KEYS
    NUM_WHITE_KEYS = consts.NUM_WHITE_KEYS
    NUM_BLACK_KEYS = NUM_KEYS - NUM_WHITE_KEYS


def check_key_collisions(usd_file_path):
    """Check all piano keys for collision meshes."""
    
    print(f"\n{'='*70}")
    print(f"Checking collision meshes in: {usd_file_path.name}")
    print(f"{'='*70}\n")
    
    # Open USD stage
    stage = Usd.Stage.Open(str(usd_file_path))
    if not stage:
        print(f"❌ Failed to open USD file: {usd_file_path}")
        return False
    
    # Get piano root
    piano_prim = stage.GetPrimAtPath("/Piano")
    if not piano_prim:
        print("❌ Piano prim not found at /Piano")
        return False
    
    print(f"✓ Found Piano prim at: {piano_prim.GetPath()}\n")
    
    # Collect all key prims
    white_keys = []
    black_keys = []
    missing_collisions = []
    
    # Traverse all children of Piano
    for child in piano_prim.GetChildren():
        child_name = child.GetName()
        
        if child_name.startswith("white_key_"):
            key_index = int(child_name.split("_")[-1])
            white_keys.append((key_index, child))
        elif child_name.startswith("black_key_"):
            key_index = int(child_name.split("_")[-1])
            black_keys.append((key_index, child))
    
    # Sort by index
    white_keys.sort(key=lambda x: x[0])
    black_keys.sort(key=lambda x: x[0])
    
    print(f"Found {len(white_keys)} white keys, {len(black_keys)} black keys")
    print(f"Expected {NUM_WHITE_KEYS} white keys, {NUM_BLACK_KEYS} black keys\n")
    
    # Check each key for collision mesh
    print("Checking white keys for collision meshes...")
    for key_index, key_prim in white_keys:
        collision_found = False
        
        # Check all children for collision mesh
        for child in key_prim.GetChildren():
            child_name = child.GetName()
            if child_name.endswith("_collision"):
                # Check if it has PhysicsCollisionAPI schema
                if child.HasAPI(UsdPhysics.CollisionAPI):
                    collision_found = True
                    break
        
        if not collision_found:
            missing_collisions.append(("white", key_index, key_prim.GetPath()))
    
    print(f"  ✓ Checked {len(white_keys)} white keys")
    if missing_collisions:
        white_missing = [m for m in missing_collisions if m[0] == "white"]
        if white_missing:
            print(f"  ❌ {len(white_missing)} white keys missing collisions")
    
    print(f"\nChecking black keys for collision meshes...")
    for key_index, key_prim in black_keys:
        collision_found = False
        
        # Check all children for collision mesh
        for child in key_prim.GetChildren():
            child_name = child.GetName()
            if child_name.endswith("_collision"):
                # Check if it has PhysicsCollisionAPI schema
                if child.HasAPI(UsdPhysics.CollisionAPI):
                    collision_found = True
                    break
        
        if not collision_found:
            missing_collisions.append(("black", key_index, key_prim.GetPath()))
    
    print(f"  ✓ Checked {len(black_keys)} black keys")
    if missing_collisions:
        black_missing = [m for m in missing_collisions if m[0] == "black"]
        if black_missing:
            print(f"  ❌ {len(black_missing)} black keys missing collisions")
    
    # Summary
    print(f"\n{'='*70}")
    print("Summary")
    print(f"{'='*70}")
    total_keys_checked = len(white_keys) + len(black_keys)
    total_with_collisions = total_keys_checked - len(missing_collisions)
    
    print(f"Total keys found: {total_keys_checked}")
    print(f"Keys with collision meshes: {total_with_collisions}")
    print(f"Keys missing collision meshes: {len(missing_collisions)}")
    
    if len(missing_collisions) > 0:
        print(f"\n❌ Missing collisions:")
        for key_type, key_index, key_path in missing_collisions:
            print(f"  - {key_type}_key_{key_index} at {key_path}")
        return False
    else:
        print(f"\n✅ All keys have collision meshes!")
        return True


def main():
    # Check piano_with_physics.usda
    usd_file = Path(__file__).parent / "piano_with_physics.usda"
    
    if not usd_file.exists():
        print(f"❌ USD file not found: {usd_file}")
        return 1
    
    success = check_key_collisions(usd_file)
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())

