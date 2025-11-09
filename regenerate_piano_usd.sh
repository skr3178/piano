#!/bin/bash
# Script to regenerate piano_with_physics.usda with updated spring reference
# Uses conda environment 'pianist'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "Regenerating piano_with_physics.usda with updated spring reference (0° instead of -1°)..."
echo "Using conda environment: pianist"
echo ""

cd "$SCRIPT_DIR"

# Initialize conda
eval "$(conda shell.bash hook)"

# Activate pianist environment and run the script
conda activate pianist && python3 generate_piano_usda_with_physics.py

if [ $? -eq 0 ]; then
    echo ""
    echo "✓ Successfully regenerated piano_with_physics.usda"
    echo "  Spring reference is now 0° (matches rest position)"
else
    echo ""
    echo "✗ Failed to regenerate USD file"
    exit 1
fi

