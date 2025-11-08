#!/usr/bin/env python3
"""
Simple script to open the exported piano_with_hands.xml in MuJoCo viewer.

Usage:
    python3 view_piano_with_hands.py [path_to_xml]
    
If no path is provided, it will look for exports/piano_with_hands.xml
"""

import sys
from pathlib import Path
import mujoco
import mujoco.viewer


def main():
    """Open the XML file in MuJoCo viewer."""
    # Get XML path from command line or use default
    if len(sys.argv) > 1:
        xml_path = Path(sys.argv[1])
    else:
        # Default location from export script
        xml_path = Path(__file__).parent / "exports" / "piano_with_hands.xml"
    
    if not xml_path.exists():
        print(f"Error: XML file not found at: {xml_path}")
        print(f"\nPlease provide the path to your exported XML file:")
        print(f"  python3 view_piano_with_hands.py <path_to_xml>")
        sys.exit(1)
    
    print(f"Loading MuJoCo model from: {xml_path}")
    print("Opening MuJoCo viewer...")
    print("Press ESC or close the window to exit.\n")
    
    # Load the model
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    
    # Launch viewer
    with mujoco.viewer.launch_passive(model, data) as viewer:
        print("MuJoCo viewer opened successfully!")
        print("Controls:")
        print("  - Mouse: Rotate camera (left click + drag)")
        print("  - Scroll: Zoom in/out")
        print("  - Right click + drag: Pan camera")
        print("  - ESC: Close viewer\n")
        
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()


if __name__ == "__main__":
    main()

