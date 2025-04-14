# Hailo ROS Bridge

This package provides a bridge between Hailo AI accelerator detections and ROS 2 topics using ZeroMQ for communication.

## Overview

The system consists of two main components:

1. **ZeroMQ Publisher** (`hailo_zmq_publisher.py`): Runs inside a Docker container with the Hailo SDK and publishes detection results via ZeroMQ.
2. **ROS ZeroMQ Subscriber** (`hailo_zmq_subscriber.py`): Runs as a ROS 2 node outside the container, subscribes to ZeroMQ messages, and republishes them as ROS topics.

## Installation

### Prerequisites

- ROS 2 (Jazzy or later)
- Python 3
- PyZMQ (`pip install pyzmq`)
- Docker (for running the Hailo container)

### Building the Package

```bash
# Clone the repository
cd ~/hailo_ros_ws/src
# The package should already be here

# Build the workspace
cd ~/hailo_ros_ws
colcon build --symlink-install
```

## Usage

### 1. Start the ZeroMQ Publisher in Docker

```bash
# Start the Docker container with access to the Hailo device and webcam
docker start 985089d0053e && docker exec -it 985089d0053e bash

# Inside the container, set up the environment and run the publisher
cd /root/hailo-rpi5-examples && source setup_env.sh
python3 /path/to/hailo_zmq_publisher.py
```

### 2. Start the ROS ZeroMQ Subscriber

```bash
# Source the ROS workspace
source ~/hailo_ros_ws/install/setup.bash

# Launch the subscriber node
ros2 launch hailo_ros_bridge hailo_bridge.launch.py zmq_address:=tcp://CONTAINER_IP:5555
```

Replace `CONTAINER_IP` with the IP address of your Docker container.

## Topics

The bridge publishes to the following ROS topics:

- `/object_detection/detections` (vision_msgs/Detection2DArray): Structured detection messages
- `/object_detection/detections_string` (std_msgs/String): JSON-formatted detection data
- `/object_detection/custom_format` (std_msgs/String): Human-readable format: "object at distance° angle"

## Extending for LiDAR Integration

To correlate detected objects with 2D LiDAR data:

1. Subscribe to your LiDAR topic (e.g., `/scan`)
2. Use the angle information from detections to find corresponding LiDAR points
3. Compare the estimated distance from object size with actual LiDAR distance

Example code for this integration will be added in a future update.

## License

MIT 