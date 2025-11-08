#!/usr/bin/env python3
"""Export piano model to both USD (visual) and XML (physics) formats."""

import sys
from pathlib import Path

# Add the robopianist directory to path
sys.path.insert(0, str(Path(__file__).parent / "robopianist"))

from dm_control.mjcf import export_with_assets
from robopianist.models.piano import piano_mjcf
from robopianist.models.piano import piano_usd


def main():
    """Export piano model to USD and XML formats."""
    # Build the piano model programmatically
    print("Building piano model...")
    piano_model = piano_mjcf.build(add_actuators=False)
    
    # Save to the piano assets directory
    out_dir = Path(__file__).parent / "robopianist" / "robopianist" / "models" / "piano" / "assets"
    out_dir.mkdir(parents=True, exist_ok=True)
    
    # Export to USD (visual geometry only - no physics)
    print("\n" + "="*60)
    print("Exporting to USD format (visual geometry only)...")
    print("="*60)
    usd_path = piano_usd.export_to_usd(
        piano_model,
        out_file="piano.usda",
        out_dir=out_dir,
    )
    print(f"✓ USD file exported: {usd_path}")
    print(f"  - Contains: Visual geometry only (meshes, colors, transforms)")
    print(f"  - Does NOT contain: Physics properties (masses, joints, actuators)")
    print(f"  - Use in: USD Composer, Blender, Houdini, Omniverse")
    
    # Export to XML (with physics - masses, joints, actuators, etc.)
    print("\n" + "="*60)
    print("Exporting to MuJoCo XML format (with physics)...")
    print("="*60)
    export_with_assets(
        piano_model,
        out_dir=out_dir,
        out_file_name="piano.xml",
    )
    xml_path = out_dir / "piano.xml"
    print(f"✓ XML file exported: {xml_path}")
    print(f"  - Contains: FULL physics simulation data")
    print(f"  - Includes: Masses, joints, actuators, constraints, materials")
    print(f"  - Use in: MuJoCo viewer, physics simulators")
    
    print("\n" + "="*60)
    print("SUMMARY:")
    print("="*60)
    print(f"  USD file: {usd_path}")
    print(f"    → Visual rendering only (no physics)")
    print(f"\n  XML file: {xml_path}")
    print(f"    → Full physics simulation (with masses, joints, etc.)")
    print("="*60)


if __name__ == "__main__":
    main()

