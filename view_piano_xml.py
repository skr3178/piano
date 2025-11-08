#!/usr/bin/env python3
"""Open piano.xml in MuJoCo viewer."""

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
        # Default location - the piano.xml we just generated
        xml_path = Path(__file__).parent / "piano.xml"
    
    if not xml_path.exists():
        print(f"Error: XML file not found at: {xml_path}")
        print(f"\nPlease provide the path to your XML file:")
        print(f"  python3 view_piano_xml.py <path_to_xml>")
        print(f"\nOr generate it first:")
        print(f"  conda run -n pianist python generate_piano_xml.py")
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

