#!/bin/bash
# Helper script to convert the latest MIDI playback recording to video

RECORDINGS_DIR="$HOME/omni.replicator_out/recordings"
LATEST=$(ls -dt "$RECORDINGS_DIR"/midi_playback_* 2>/dev/null | head -1)

if [ -z "$LATEST" ]; then
    echo "❌ No recordings found in $RECORDINGS_DIR"
    exit 1
fi

echo "📹 Latest recording: $LATEST"
echo "🔄 Converting to video..."

OUTPUT_FILE="midi_playback_$(date +%Y%m%d_%H%M%S).mp4"

ffmpeg -y -framerate 60 -i "$LATEST/rgb_%04d.png" \
       -c:v libx264 -pix_fmt yuv420p \
       "$OUTPUT_FILE" 2>&1 | grep -E "(Duration|frame=|Lsize)" | tail -3

if [ $? -eq 0 ]; then
    echo "✅ Video saved: $OUTPUT_FILE"
    echo "   Location: $(pwd)/$OUTPUT_FILE"
else
    echo "❌ Conversion failed"
    exit 1
fi

