#!/usr/bin/env /home/skr/miniconda3/envs/pianist/bin/python
"""Open piano with hands USD scene in MuJoCo viewer."""

import sys
from pathlib import Path
import mujoco
import mujoco.viewer


def main():
    """Launch MuJoCo viewer with the piano and hands scene from USD."""
    # Get file path from command line or use default
    if len(sys.argv) > 1:
        model_path = Path(sys.argv[1])
    else:
        # Default to the MJCF wrapper that loads the USD file
        model_path = Path("/home/skr/Downloads/piano/load_piano_usd.xml")
    
    if not model_path.exists():
        print(f"Error: Model file not found at: {model_path}")
        print(f"\nPlease provide the path to a model file:")
        print(f"  python view_piano_xml_2.py <path_to_model>")
        sys.exit(1)
    
    print("=" * 60)
    print("Launching MuJoCo Viewer with USD Scene")
    print("=" * 60)
    print(f"\nLoading model from: {model_path}")
    print("\nViewer Controls:")
    print("  - Left click + drag: Rotate camera")
    print("  - Right click + drag: Pan camera")
    print("  - Scroll: Zoom in/out")
    print("  - Double-click: Select object")
    print("  - Space: Pause/Resume simulation")
    print("  - ESC: Close viewer")
    print("\nStarting viewer...\n")
    
    try:
        # Load the model (MJCF wrapper with USD)
        print("Loading model with MuJoCo...")
        
        # MuJoCo can load USD files via MJCF wrapper
        model = mujoco.MjModel.from_xml_path(str(model_path))
        data = mujoco.MjData(model)
        
        print(f"✓ Model loaded successfully!")
        print(f"  - Bodies: {model.nbody}")
        print(f"  - Joints: {model.njnt}")
        print(f"  - Geometries: {model.ngeom}")
        print(f"  - Actuators: {model.nu}")
        print()
        
        # Launch interactive viewer
        with mujoco.viewer.launch_passive(model, data) as viewer:
            print("✓ MuJoCo viewer opened!")
            print("\nViewer is running. Press ESC or close window to exit.\n")
            
            while viewer.is_running():
                mujoco.mj_step(model, data)
                viewer.sync()
                
    except Exception as e:
        print(f"\n✗ Error loading model: {e}")
        print("\nTroubleshooting:")
        print("  - Make sure the USD file and its Payload/ folder exist")
        print("  - Try the direct XML export: /home/skr/Downloads/piano/exports/piano_with_hands.xml")
        print("  - Or use a USD viewer like usdview or Isaac Sim")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()

