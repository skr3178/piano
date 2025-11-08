#!/usr/bin/env python3
"""
Fix USD file to enable joint visualization in IsaacSim.
IsaacSim requires specific properties and APIs to visualize joints properly.
"""

import os
import sys
from pathlib import Path

try:
    from pxr import Usd, UsdPhysics, UsdGeom, Sdf
    USD_AVAILABLE = True
except ImportError:
    USD_AVAILABLE = False
    print("Error: pxr (USD Python API) not available.")
    print("Please install with: pip install usd-core")
    sys.exit(1)


def fix_joint_visualization(usd_path: str, output_path: str = None):
    """
    Fix joint visualization in USD file for IsaacSim.
    
    Args:
        usd_path: Path to input USD file
        output_path: Path to output USD file (if None, creates _fixed version)
    """
    print(f"\n{'='*60}")
    print(f"Fixing joint visualization for: {usd_path}")
    print(f"{'='*60}\n")
    
    if not os.path.exists(usd_path):
        print(f"Error: File not found: {usd_path}")
        return False
    
    # Determine output path
    if output_path is None:
        path = Path(usd_path)
        output_path = str(path.parent / f"{path.stem}_joints_fixed{path.suffix}")
    
    # Open the stage
    stage = Usd.Stage.Open(usd_path)
    if not stage:
        print(f"Error: Failed to open USD file: {usd_path}")
        return False
    
    # Get default prim
    default_prim = stage.GetDefaultPrim()
    if not default_prim:
        print("Warning: No default prim found, using root prim")
        default_prim = stage.GetPseudoRoot()
    
    print(f"Default Prim: {default_prim.GetPath()}")
    
    # Find all joints in the stage
    joints_found = 0
    joints_fixed = 0
    
    def process_prim(prim):
        nonlocal joints_found, joints_fixed
        
        # Check if this is a joint prim
        prim_type = prim.GetTypeName()
        if 'Joint' in prim_type:
            joints_found += 1
            print(f"Found joint: {prim.GetPath()} (type: {prim_type})")
            
            # Add PhysicsJointStateAPI if not present
            if not prim.HasAPI(UsdPhysics.JointStateAPI):
                UsdPhysics.JointStateAPI.Apply(prim)
                print(f"  ✓ Added PhysicsJointStateAPI")
                joints_fixed += 1
            
            # Enable visualization by setting display properties
            # Create or get the joint's drawing properties
            if prim_type == 'PhysicsRevoluteJoint':
                joint = UsdPhysics.RevoluteJoint(prim)
                
                # Ensure the joint has proper body relationships
                body0_rel = joint.GetBody0Rel()
                body1_rel = joint.GetBody1Rel()
                
                if body0_rel and body1_rel:
                    targets0 = body0_rel.GetTargets()
                    targets1 = body1_rel.GetTargets()
                    if targets0 and targets1:
                        print(f"  ✓ Body0: {targets0[0]}")
                        print(f"  ✓ Body1: {targets1[0]}")
                    else:
                        print(f"  ✗ Warning: Joint has invalid body relationships")
                else:
                    print(f"  ✗ Warning: Joint missing body relationships")
            
            # Add visualization metadata
            # This helps IsaacSim's joint visualizer detect and display the joints
            prim.SetMetadata('kind', 'component')
            
        # Recursively process children
        for child in prim.GetChildren():
            process_prim(child)
    
    # Start processing from the default prim
    process_prim(default_prim)
    
    # Also check if we need to add ArticulationRootAPI to the root
    if joints_found > 0:
        # Find the top-level body that should be the articulation root
        print(f"\nSearching for articulation root candidates...")
        
        def find_rigid_bodies(prim, level=0):
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                indent = "  " * level
                print(f"{indent}RigidBody: {prim.GetPath()}")
                
                # Check if this should be an articulation root
                # (a body with joints but no parent joint)
                has_parent_joint = False
                parent = prim.GetParent()
                while parent and parent != stage.GetPseudoRoot():
                    if 'Joint' in parent.GetTypeName():
                        has_parent_joint = True
                        break
                    parent = parent.GetParent()
                
                if not has_parent_joint and joints_found > 0:
                    print(f"{indent}  → Potential articulation root")
                    if not prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                        UsdPhysics.ArticulationRootAPI.Apply(prim)
                        print(f"{indent}  ✓ Added ArticulationRootAPI")
                        return True
            
            for child in prim.GetChildren():
                result = find_rigid_bodies(child, level + 1)
                if result:
                    return True
            return False
        
        find_rigid_bodies(default_prim)
    
    # Save the fixed stage
    stage.GetRootLayer().Export(output_path)
    
    print(f"\n{'='*60}")
    print(f"Results:")
    print(f"  Joints found: {joints_found}")
    print(f"  Joints fixed: {joints_fixed}")
    print(f"  Output saved to: {output_path}")
    print(f"{'='*60}\n")
    
    if joints_found == 0:
        print("⚠ WARNING: No joints found in the USD file!")
        print("This might be because:")
        print("  1. The joints are defined in a separate layer (check Payload/Physics.usda)")
        print("  2. The stage references or payloads are not being loaded")
        print("\nTry fixing the Payload/Physics.usda file directly.")
        return False
    
    return True


def inspect_joints(usd_path: str):
    """Inspect joint structure in USD file."""
    print(f"\n{'='*60}")
    print(f"Inspecting joints in: {usd_path}")
    print(f"{'='*60}\n")
    
    if not os.path.exists(usd_path):
        print(f"Error: File not found: {usd_path}")
        return
    
    stage = Usd.Stage.Open(usd_path)
    if not stage:
        print(f"Error: Failed to open USD file: {usd_path}")
        return
    
    joints_count = 0
    
    def inspect_prim(prim, level=0):
        nonlocal joints_count
        indent = "  " * level
        prim_type = prim.GetTypeName()
        
        if 'Joint' in prim_type:
            joints_count += 1
            print(f"{indent}Joint: {prim.GetPath()}")
            print(f"{indent}  Type: {prim_type}")
            print(f"{indent}  Has JointStateAPI: {prim.HasAPI(UsdPhysics.JointStateAPI)}")
            
            if prim_type == 'PhysicsRevoluteJoint':
                joint = UsdPhysics.RevoluteJoint(prim)
                axis_attr = joint.GetAxisAttr()
                if axis_attr:
                    print(f"{indent}  Axis: {axis_attr.Get()}")
                
                lower_limit = joint.GetLowerLimitAttr()
                upper_limit = joint.GetUpperLimitAttr()
                if lower_limit and upper_limit:
                    print(f"{indent}  Limits: [{lower_limit.Get()}, {upper_limit.Get()}]")
        
        for child in prim.GetChildren():
            inspect_prim(child, level + 1)
    
    default_prim = stage.GetDefaultPrim()
    if default_prim:
        inspect_prim(default_prim)
    
    print(f"\nTotal joints found: {joints_count}")


def main():
    """Main function."""
    if len(sys.argv) < 2:
        print("Usage: python3 fix_joints_visualization.py <usd_file> [output_file]")
        print("\nOptions:")
        print("  --inspect    Only inspect joints without fixing")
        print("\nExamples:")
        print("  python3 fix_joints_visualization.py convert/stage.usda")
        print("  python3 fix_joints_visualization.py convert/stage.usda convert/stage_fixed.usda")
        print("  python3 fix_joints_visualization.py --inspect convert/stage.usda")
        sys.exit(1)
    
    if sys.argv[1] == "--inspect":
        if len(sys.argv) < 3:
            print("Error: Please specify USD file to inspect")
            sys.exit(1)
        inspect_joints(sys.argv[2])
    else:
        usd_path = sys.argv[1]
        output_path = sys.argv[2] if len(sys.argv) > 2 else None
        success = fix_joint_visualization(usd_path, output_path)
        sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()

