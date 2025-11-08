#!/usr/bin/env python3
"""Generate piano.xml from piano_mjcf.py"""

import sys
from pathlib import Path

# Add the robopianist directory to path
sys.path.insert(0, str(Path(__file__).parent / "robopianist"))

from dm_control.mjcf import export_with_assets
from robopianist.models.piano import piano_mjcf


def main():
    """Generate piano.xml file."""
    print("Building piano model...")
    piano_model = piano_mjcf.build(add_actuators=False)
    
    # Save to current directory
    out_dir = Path(__file__).parent
    out_dir.mkdir(parents=True, exist_ok=True)
    
    print("\nExporting to MuJoCo XML format...")
    export_with_assets(
        piano_model,
        out_dir=str(out_dir),
        out_file_name="piano.xml",
    )
    
    xml_path = out_dir / "piano.xml"
    print(f"\n✓ XML file exported: {xml_path}")
    print(f"  - Contains: Full physics simulation data")
    print(f"  - Includes: Masses, joints, actuators, constraints, materials")
    print(f"  - Use in: MuJoCo viewer, physics simulators")


if __name__ == "__main__":
    main()

