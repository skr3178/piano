#!/bin/bash
# Launcher script for Hand-Piano Collision Test
# Automatically finds and uses Isaac Sim's Python

set -e

echo "======================================================================"
echo "Hand-Piano Collision Test Launcher"
echo "======================================================================"
echo

# Find Isaac Sim installation
ISAAC_SIM_PATH=""

# Try environment variable
if [ ! -z "$ISAACSIM_PATH" ]; then
    ISAAC_SIM_PATH="$ISAACSIM_PATH"
    echo "Using ISAACSIM_PATH: $ISAAC_SIM_PATH"
# Try user's specific location first
elif [ -d "/home/skr/isaacsim" ]; then
    # Check if python.sh is directly in the folder
    if [ -f "/home/skr/isaacsim/python.sh" ]; then
        ISAAC_SIM_PATH="/home/skr/isaacsim"
    else
        # Search for isaac-sim subdirectories
        for dir in /home/skr/isaacsim/isaac*sim* /home/skr/isaacsim/*; do
            if [ -f "$dir/python.sh" ]; then
                ISAAC_SIM_PATH="$dir"
                break
            fi
        done
    fi
# Try common locations
elif [ -f "$HOME/.local/share/ov/pkg/isaac-sim-4.5.0/python.sh" ]; then
    ISAAC_SIM_PATH="$HOME/.local/share/ov/pkg/isaac-sim-4.5.0"
elif [ -f "$HOME/.local/share/ov/pkg/isaac-sim-2024.1.1/python.sh" ]; then
    ISAAC_SIM_PATH="$HOME/.local/share/ov/pkg/isaac-sim-2024.1.1"
elif [ -f "$HOME/.local/share/ov/pkg/isaac_sim-2024.1.1/python.sh" ]; then
    ISAAC_SIM_PATH="$HOME/.local/share/ov/pkg/isaac_sim-2024.1.1"
elif [ -f "/opt/nvidia/isaac-sim/python.sh" ]; then
    ISAAC_SIM_PATH="/opt/nvidia/isaac-sim"
else
    # Search in ov/pkg directory
    OV_PKG="$HOME/.local/share/ov/pkg"
    if [ -d "$OV_PKG" ]; then
        for dir in "$OV_PKG"/isaac*sim*; do
            if [ -f "$dir/python.sh" ]; then
                ISAAC_SIM_PATH="$dir"
                break
            fi
        done
    fi
fi

if [ -z "$ISAAC_SIM_PATH" ] || [ ! -f "$ISAAC_SIM_PATH/python.sh" ]; then
    echo "ERROR: Isaac Sim installation not found!"
    echo
    echo "Please install Isaac Sim from:"
    echo "  https://developer.nvidia.com/isaac-sim"
    echo
    echo "Or set the ISAACSIM_PATH environment variable:"
    echo "  export ISAACSIM_PATH=/path/to/isaac-sim"
    echo
    echo "Common install locations:"
    echo "  ~/.local/share/ov/pkg/isaac-sim-4.5.0/"
    echo "  ~/.local/share/ov/pkg/isaac-sim-2024.1.1/"
    echo "  /opt/nvidia/isaac-sim/"
    echo
    exit 1
fi

echo "Found Isaac Sim: $ISAAC_SIM_PATH"
echo

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Run the test using Isaac Sim's Python
echo "Launching hand-piano collision test..."
echo "======================================================================"
echo

"$ISAAC_SIM_PATH/python.sh" "$SCRIPT_DIR/test_hand_piano_collision.py" "$@"

