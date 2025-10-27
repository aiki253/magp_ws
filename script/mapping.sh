#!/bin/bash

# ROS2 LiDAR mapping script
# URG node and Cartographer launch

# echo "Starting URG node2..."
# ros2 launch urg_node2 urg_node2.launch.py &
# URG_PID=$!

# Wait a moment for the first node to initialize
# sleep 3

echo "Starting Cartographer..."
ros2 launch urg_node2 urg_cartographer.launch.py &
CARTOGRAPHER_PID=$!

echo "Mapping started. Press Ctrl+C to stop both processes."

# Function to handle cleanup on exit
cleanup() {
    echo "Stopping processes..."
    kill $URG_PID 2>/dev/null
    kill $CARTOGRAPHER_PID 2>/dev/null
    exit 0
}

# Set trap to handle Ctrl+C
trap cleanup SIGINT

# Wait for processes to finish
wait