#!/bin/bash

# ROS2 Map saving script

MAP_NAME="map"
MAP_DIR="$HOME"

# Check if a map name is provided as argument
if [ $# -gt 0 ]; then
    MAP_NAME="$1"
fi

echo "Saving map as: $MAP_DIR/$MAP_NAME"
echo "This will create: $MAP_DIR/$MAP_NAME.yaml and $MAP_DIR/$MAP_NAME.pgm"

# Save the map
ros2 run nav2_map_server map_saver_cli -f "$MAP_DIR/$MAP_NAME"

if [ $? -eq 0 ]; then
    echo "Map saved successfully!"
    echo "Files created:"
    echo "  - $MAP_DIR/$MAP_NAME.yaml"
    echo "  - $MAP_DIR/$MAP_NAME.pgm"
else
    echo "Error: Failed to save map"
    exit 1
fi