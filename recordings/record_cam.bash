#!/bin/bash

# Script: record_camera.sh
# Description: Records camera video in 10-second segments with timestamped filenames

# Set variables
DURATION=1800  # Duration in seconds, 1800 seconds = 30 minutes
WIDTH=1280
HEIGHT=720
FRAMERATE=30
OUTPUT_DIR="./recordings/cam${1:-0}"  # Directory to store recordings
CAMERA_ID=${1:-0}  # Camera ID, terminal input parameter

echo "Recording from camera ID: $CAMERA_ID"

# Create output directory if it doesn't exist
mkdir -p "$OUTPUT_DIR"

# Function to generate filename with timestamp (including seconds)
generate_filename() {
    local timestamp=$(date +"%Y%m%d_%H_%M_%S")  # Added seconds
    echo "${OUTPUT_DIR}/${timestamp}.mp4"
}

# Function to record a single segment
record_segment() {
    local filename=$(generate_filename)
    echo "Recording: $filename"
    
    rpicam-vid --camera $CAMERA_ID -t "$((DURATION * 1000))" \
        --codec yuv420 \
        --width "$WIDTH" \
        --height "$HEIGHT" \
        -o - 2>/dev/null | \
    ffmpeg -hide_banner -loglevel error \
        -f rawvideo \
        -pix_fmt yuv420p \
        -s "${WIDTH}x${HEIGHT}" \
        -framerate "$FRAMERATE" \
        -i - \
        -c:v libx264 \
        -preset veryfast \
        -t "$DURATION" \
        -y \
        "$filename" 2>/dev/null
    
    if [ $? -eq 0 ] && [ -f "$filename" ]; then
        local filesize=$(du -h "$filename" | cut -f1)
        echo "✓ Successfully recorded: $filename ($filesize)"
        return 0
    else
        echo "✗ Error recording: $filename"
        return 1
    fi
}

# Main loop - continuous recording
echo "Starting continuous recording (${DURATION}-second segments)"
echo "Press Ctrl+C to stop"
echo "----------------------------------------"

# Trap Ctrl+C for clean exit
trap 'echo -e "\nStopping recording..."; exit 0' INT

while true; do
    record_segment
    # Small delay between recordings to prevent overlap
    sleep 0.5
done