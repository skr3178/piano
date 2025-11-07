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

"""Example script to export piano model to USD and XML formats."""

from pathlib import Path

from dm_control.mjcf import export_with_assets

from robopianist.models.piano import piano_mjcf, piano_usd


def main() -> None:
    """Export piano model to USD and XML formats."""
    # Save to the piano assets directory
    out_dir = Path(__file__).parent.parent / "robopianist" / "models" / "piano" / "assets"
    
    # Method 1: Build USD from scratch (similar to piano_mjcf.build())
    print("="*60)
    print("Method 1: Building USD from scratch (like piano_mjcf.build())")
    print("="*60)
    try:
        usd_file = out_dir / "piano_from_scratch.usda"
        stage = piano_usd.build_usd(file_path=usd_file)
        stage.Save()
        print(f"✓ USD file created from scratch: {usd_file}")
        print(f"  - Built directly using USD Python API (pxr)")
        print(f"  - No MJCF model needed")
    except ImportError as e:
        print(f"✗ Failed to create USD from scratch: {e}")
        print(f"  Note: USD Python API (pxr) is required for build_usd()")
    
    # Method 2: Build MJCF first, then export to USD
    print("\n" + "="*60)
    print("Method 2: Exporting from MJCF model")
    print("="*60)
    piano_model = piano_mjcf.build(add_actuators=False)
    
    # Export to USD (visual geometry only - no physics)
    print("Exporting MJCF model to USD format (visual geometry only)...")
    usd_path = piano_usd.export_to_usd(
        piano_model,
        out_file="piano_from_mjcf.usda",
        out_dir=out_dir,
    )
    print(f"✓ USD file exported from MJCF: {usd_path}")
    print(f"  - Converted from programmatically built MJCF model")
    print(f"  - Note: USD file contains VISUAL geometry only, NO physics properties")
    
    # Export to XML (with physics - masses, joints, actuators, etc.)
    print("\nExporting to MuJoCo XML format (with physics)...")
    export_with_assets(
        piano_model,
        out_dir=out_dir,
        out_file_name="piano.xml",
    )
    xml_path = out_dir / "piano.xml"
    print(f"✓ XML file exported: {xml_path}")
    print(f"  Note: XML file contains FULL physics simulation data")
    
    print("\n" + "="*60)
    print("Summary:")
    print(f"  USD file: {usd_path}")
    print(f"    - Visual geometry only (meshes, colors, transforms)")
    print(f"    - No physics properties")
    print(f"    - Use in: USD Composer, Blender, Houdini")
    print(f"\n  XML file: {xml_path}")
    print(f"    - Full physics simulation (masses, joints, actuators)")
    print(f"    - Use in: MuJoCo viewer, physics simulators")
    print("="*60)


if __name__ == "__main__":
    main()

