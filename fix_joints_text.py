#!/usr/bin/env python3
"""
Fix USD/USDA files to add PhysicsJointStateAPI to all joints.
This is required for IsaacSim to properly visualize and control joints.
"""

import re
import sys
from pathlib import Path


def fix_joints_in_usda(input_path: str, output_path: str = None):
    """
    Add PhysicsJointStateAPI to all joint definitions in a USDA file.
    
    Args:
        input_path: Path to input USDA file
        output_path: Path to output USDA file (if None, creates _fixed version)
    
    Returns:
        bool: True if successful
    """
    input_path = Path(input_path)
    
    if not input_path.exists():
        print(f"Error: File not found: {input_path}")
        return False
    
    if output_path is None:
        output_path = input_path.parent / f"{input_path.stem}_joints_fixed{input_path.suffix}"
    else:
        output_path = Path(output_path)
    
    print(f"\n{'='*60}")
    print(f"Fixing joints in: {input_path}")
    print(f"Output will be: {output_path}")
    print(f"{'='*60}\n")
    
    # Read the file
    with open(input_path, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # Pattern to match joint definitions with apiSchemas
    # We need to find lines like:
    #   def PhysicsRevoluteJoint "name" (
    #       prepend apiSchemas = ["MjcJointAPI"]
    #       ...
    #   )
    
    # First, let's find all joint definitions
    joint_pattern = re.compile(
        r'(def\s+Physics\w*Joint\s+"[^"]+"\s*\(\s*\n)'  # Joint definition line
        r'(\s*prepend\s+apiSchemas\s*=\s*\[)([^\]]+)(\])',  # apiSchemas line
        re.MULTILINE
    )
    
    joints_found = 0
    joints_fixed = 0
    
    def fix_joint(match):
        nonlocal joints_found, joints_fixed
        joints_found += 1
        
        joint_def = match.group(1)  # def PhysicsXXXJoint ...
        api_prefix = match.group(2)  # prepend apiSchemas = [
        api_list = match.group(3)    # "API1", "API2", etc.
        api_suffix = match.group(4)  # ]
        
        # Check if PhysicsJointStateAPI is already present
        if 'PhysicsJointStateAPI' in api_list:
            # Already has it, don't modify
            return match.group(0)
        
        # Add PhysicsJointStateAPI to the list
        # Place it first for better compatibility
        new_api_list = '"PhysicsJointStateAPI", ' + api_list
        joints_fixed += 1
        
        return joint_def + api_prefix + new_api_list + api_suffix
    
    # Apply the fix
    fixed_content = joint_pattern.sub(fix_joint, content)
    
    # Write the fixed content
    with open(output_path, 'w', encoding='utf-8') as f:
        f.write(fixed_content)
    
    print(f"Results:")
    print(f"  Joints found: {joints_found}")
    print(f"  Joints fixed: {joints_fixed}")
    print(f"  Output saved to: {output_path}")
    print(f"\n{'='*60}\n")
    
    if joints_fixed > 0:
        print("✓ Successfully added PhysicsJointStateAPI to joints!")
        print("\nTo use the fixed file in IsaacSim:")
        print(f"  1. Load: {output_path}")
        print("  2. In IsaacSim, go to Window > Simulation > Physics Inspector")
        print("  3. Joints should now be visible in the scene hierarchy")
        print("  4. You can also use Create > Physics > Joint State to visualize them")
    elif joints_found == 0:
        print("⚠ WARNING: No joints found in this file.")
        print("This might be a composed USD file. Try fixing the Physics.usda layer:")
        print(f"  python3 {Path(__file__).name} {input_path.parent}/Payload/Physics.usda")
    
    return True


def inspect_joints(input_path: str):
    """Inspect joint definitions in a USDA file."""
    input_path = Path(input_path)
    
    if not input_path.exists():
        print(f"Error: File not found: {input_path}")
        return
    
    print(f"\n{'='*60}")
    print(f"Inspecting joints in: {input_path}")
    print(f"{'='*60}\n")
    
    with open(input_path, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # Find all joint definitions
    joint_pattern = re.compile(
        r'def\s+(Physics\w*Joint)\s+"([^"]+)"\s*\([^)]*\n\s*prepend\s+apiSchemas\s*=\s*\[([^\]]+)\]',
        re.MULTILINE | re.DOTALL
    )
    
    matches = joint_pattern.findall(content)
    
    print(f"Found {len(matches)} joints:\n")
    
    has_joint_state_api_count = 0
    missing_joint_state_api_count = 0
    
    for i, (joint_type, joint_name, api_list) in enumerate(matches[:10], 1):  # Show first 10
        has_state_api = 'PhysicsJointStateAPI' in api_list
        status = "✓" if has_state_api else "✗"
        
        if has_state_api:
            has_joint_state_api_count += 1
        else:
            missing_joint_state_api_count += 1
        
        print(f"{status} {joint_type} \"{joint_name}\"")
        apis = [api.strip().strip('"') for api in api_list.split(',')]
        for api in apis:
            if api:
                marker = "  ✓" if api == "PhysicsJointStateAPI" else "   "
                print(f"    {marker} {api}")
        print()
    
    if len(matches) > 10:
        print(f"... and {len(matches) - 10} more joints\n")
    
    print(f"Summary:")
    print(f"  Total joints: {len(matches)}")
    print(f"  With PhysicsJointStateAPI: {has_joint_state_api_count}")
    print(f"  Missing PhysicsJointStateAPI: {missing_joint_state_api_count}")
    
    if missing_joint_state_api_count > 0:
        print(f"\n⚠ {missing_joint_state_api_count} joints are missing PhysicsJointStateAPI")
        print("  This is why they're not visible in IsaacSim!")
        print(f"\nTo fix, run:")
        print(f"  python3 {Path(__file__).name} {input_path}")


def main():
    """Main function."""
    if len(sys.argv) < 2:
        print("Fix USD/USDA joint visualization for IsaacSim")
        print("\nUsage:")
        print("  python3 fix_joints_text.py <usda_file> [output_file]")
        print("  python3 fix_joints_text.py --inspect <usda_file>")
        print("\nExamples:")
        print("  # Fix the Physics layer (recommended):")
        print("  python3 fix_joints_text.py convert/Payload/Physics.usda")
        print()
        print("  # Fix the main stage file:")
        print("  python3 fix_joints_text.py convert/stage.usda")
        print()
        print("  # Inspect joints before fixing:")
        print("  python3 fix_joints_text.py --inspect convert/Payload/Physics.usda")
        print()
        print("What this script does:")
        print("  - Adds 'PhysicsJointStateAPI' to all joint definitions")
        print("  - This API is REQUIRED for IsaacSim to visualize joints")
        print("  - Without it, joints exist but are invisible in the viewport")
        sys.exit(1)
    
    if sys.argv[1] == "--inspect":
        if len(sys.argv) < 3:
            print("Error: Please specify USDA file to inspect")
            sys.exit(1)
        inspect_joints(sys.argv[2])
    else:
        input_path = sys.argv[1]
        output_path = sys.argv[2] if len(sys.argv) > 2 else None
        success = fix_joints_in_usda(input_path, output_path)
        sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()

