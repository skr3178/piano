#!/usr/bin/env python3
"""
Export the complete MuJoCo XML file containing:
- Piano (88 keys with physics)
- Left Shadow Hand
- Right Shadow Hand
- All interactions and physics properties

This script creates a complete XML file that can be loaded directly in MuJoCo.

Requirements:
    - dm_control
    - robopianist (installed or in PYTHONPATH)
    - mujoco

Usage:
    # Activate conda environment first:
    conda activate pianist
    
    # Then run the script:
    python3 export_piano_with_hands_xml.py --output piano_with_hands.xml
    
    # Or use conda run:
    conda run -n pianist python3 export_piano_with_hands_xml.py --output piano_with_hands.xml
"""

import argparse
import sys
from pathlib import Path

# Add robopianist to path
sys.path.insert(0, str(Path(__file__).parent / "robopianist"))

from dm_control.mjcf import export_with_assets
from robopianist import suite


def export_piano_with_hands_xml(
    output_path: str = "piano_with_hands.xml",
    midi_file: str = None,
    control_timestep: float = 0.05,
    gravity_compensation: bool = False,
    primitive_fingertip_collisions: bool = False,
    reduced_action_space: bool = False,
    attachment_yaw: float = 0.0,
) -> None:
    """
    Export the complete MuJoCo XML file with piano and two hands.
    
    Args:
        output_path: Path where to save the XML file
        midi_file: Optional MIDI file path (for environment setup)
        control_timestep: Control timestep in seconds
        gravity_compensation: Whether to enable gravity compensation
        primitive_fingertip_collisions: Use primitive fingertip collisions
        reduced_action_space: Use reduced action space
        attachment_yaw: Hand attachment yaw angle in degrees
    """
    print("=" * 70)
    print("Exporting Piano with Shadow Hands MuJoCo XML")
    print("=" * 70)
    
    # Create environment with piano and two shadow hands
    env_name = "RoboPianist-debug-TwinkleTwinkleLittleStar-v0"
    if midi_file is None:
        print(f"\nCreating environment: {env_name}")
    else:
        print(f"\nCreating environment with MIDI file: {midi_file}")
    
    env = suite.load(
        environment_name=env_name,
        midi_file=midi_file,
        task_kwargs=dict(
            control_timestep=control_timestep,
            gravity_compensation=gravity_compensation,
            primitive_fingertip_collisions=primitive_fingertip_collisions,
            reduced_action_space=reduced_action_space,
            attachment_yaw=attachment_yaw,
        ),
    )
    
    print(f"\n✓ Environment created successfully")
    print(f"  - Piano: {len(env.task._piano.keys)} keys")
    print(f"  - Left Hand: {env.task.left_hand}")
    print(f"  - Right Hand: {env.task.right_hand}")
    
    # Get the root entity MJCF model (contains arena + piano + hands)
    root_mjcf = env.task.root_entity.mjcf_model
    
    print(f"\n✓ Root entity MJCF model retrieved")
    print(f"  - Model name: {root_mjcf.model}")
    
    # Create output directory structure
    # Use a subdirectory to keep assets (.obj files) organized and prevent clutter
    output_path = Path(output_path)
    
    # Check if output path is just a filename (no directory specified)
    # If so, use an exports/ subdirectory to contain everything
    if len(output_path.parts) == 1 or output_path.parent == Path("."):
        # Create exports directory to contain all files (XML + assets)
        exports_dir = Path.cwd() / "exports"
        exports_dir.mkdir(exist_ok=True)
        output_dir = exports_dir
        output_file = output_path.name
        final_output_path = exports_dir / output_file
    else:
        # User specified a directory path, use that directory
        output_dir = output_path.parent.resolve()
        output_file = output_path.name
        final_output_path = output_path.resolve()
        output_dir.mkdir(parents=True, exist_ok=True)
    
    print(f"\nExporting complete XML to: {final_output_path}")
    print(f"  - Output directory: {output_dir}")
    print(f"  - Output filename: {output_file}")
    print(f"  - Asset files (.obj, etc.) will be placed in: {output_dir}")
    
    # Export the complete scene with all assets
    export_with_assets(
        root_mjcf,
        out_dir=str(output_dir),
        out_file_name=output_file,
    )
    
    print(f"\n{'='*70}")
    print("✓ Export completed successfully!")
    print(f"{'='*70}")
    print(f"\nThe complete MuJoCo XML file has been saved to:")
    print(f"  {final_output_path.resolve()}")
    print(f"\nAll asset files (.obj meshes, etc.) are in:")
    print(f"  {output_dir.resolve()}")
    print(f"\nThis file contains:")
    print(f"  ✓ Piano model (88 keys with physics)")
    print(f"  ✓ Left Shadow Hand (with all joints and actuators)")
    print(f"  ✓ Right Shadow Hand (with all joints and actuators)")
    print(f"  ✓ All collision geometries and interactions")
    print(f"  ✓ All physics properties (masses, joints, actuators, etc.)")
    print(f"  ✓ Arena/stage environment")
    print(f"\nTo view in MuJoCo viewer, use one of these commands:")
    print(f"\n  Option 1 - Using the helper script:")
    print(f"    python3 view_piano_with_hands.py")
    print(f"\n  Option 2 - Direct Python command:")
    print(f"    python3 -c \"from mujoco import viewer; viewer.launch_from_path('{final_output_path}')\"")
    print(f"\n  Option 3 - Using Python script:")
    print(f"    python3 view_piano_with_hands.py '{final_output_path}'")
    print(f"{'='*70}\n")


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Export complete MuJoCo XML with piano and two shadow hands"
    )
    parser.add_argument(
        "--output",
        "-o",
        type=str,
        default="piano_with_hands.xml",
        help="Output XML file path (default: piano_with_hands.xml)",
    )
    parser.add_argument(
        "--midi",
        type=str,
        default=None,
        help="Optional MIDI file path for environment setup",
    )
    parser.add_argument(
        "--control-timestep",
        type=float,
        default=0.05,
        help="Control timestep in seconds (default: 0.05)",
    )
    parser.add_argument(
        "--gravity-compensation",
        action="store_true",
        help="Enable gravity compensation for hands",
    )
    parser.add_argument(
        "--primitive-fingertip-collisions",
        action="store_true",
        help="Use primitive fingertip collisions (faster simulation)",
    )
    parser.add_argument(
        "--reduced-action-space",
        action="store_true",
        help="Use reduced action space",
    )
    parser.add_argument(
        "--attachment-yaw",
        type=float,
        default=0.0,
        help="Hand attachment yaw angle in degrees (default: 0.0)",
    )
    
    args = parser.parse_args()
    
    export_piano_with_hands_xml(
        output_path=args.output,
        midi_file=args.midi,
        control_timestep=args.control_timestep,
        gravity_compensation=args.gravity_compensation,
        primitive_fingertip_collisions=args.primitive_fingertip_collisions,
        reduced_action_space=args.reduced_action_space,
        attachment_yaw=args.attachment_yaw,
    )


if __name__ == "__main__":
    main()

