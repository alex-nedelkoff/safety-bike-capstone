#!/bin/bash

# Function to cleanup
cleanup() {
    echo "Cleaning up..."
    # Kill the HUD
    pkill -f "python3.*lidar_hud.py"
    # Kill the LiDAR process
    pkill -f "lidar_zmq_refined"
    # Kill openbox and X server
    pkill openbox
    pkill Xorg
    # Return to previous tty
    chvt 1
}

# Set trap for cleanup
trap cleanup EXIT

# Check for required commands
for cmd in docker docker-compose python3; do
    if ! command -v "$cmd" >/dev/null 2>&1; then
        echo "Error: $cmd is not installed"
        exit 1
    fi
done

# Check if docker-compose.yml exists
DOCKER_COMPOSE_PATH="/home/alex/pi-ai-kit-ubuntu/docker-compose.yml"
if [ ! -f "$DOCKER_COMPOSE_PATH" ]; then
    echo "Error: docker-compose.yml not found at $DOCKER_COMPOSE_PATH"
    exit 1
fi

# Try to stop and remove any existing container
echo "Cleaning up any existing containers..."
cd /home/alex/pi-ai-kit-ubuntu
docker-compose down

# Create and start container
echo "Starting container..."
docker-compose up -d hailo-ubuntu-pi
sleep 5  # Wait for container to be ready

# Verify container is running
if ! docker-compose ps hailo-ubuntu-pi | grep -q "Up"; then
    echo "Error: Container failed to start"
    exit 1
fi

# Start the detection script inside the container in background
echo "Starting detection script in container..."
docker-compose exec -d hailo-ubuntu-pi /bin/bash -c "cd /hailo-rpi5-examples && python basic_pipelines/docker_detection_refined.py -i /dev/video0"

# Wait for ZMQ port to become active
echo "Waiting for detection script to initialize..."
for i in {1..30}; do  # Try for 30 seconds
    if timeout 1 bash -c '>/dev/tcp/localhost/5555' 2>/dev/null; then
        echo "Detection script is running (ZMQ port active)"
        break
    fi
    if [ $i -eq 30 ]; then
        echo "Error: Detection script failed to start (ZMQ port not active)"
        echo "Container logs:"
        docker-compose logs hailo-ubuntu-pi
        exit 1
    fi
    sleep 1
done

# Start the LiDAR process in background
echo "Starting LiDAR..."
cd /home/alex/rplidar_sdk
./app/lidar_zmq_refined &
LIDAR_PID=$!
sleep 2  # Wait for LiDAR to initialize

# Start the HUD without sudo password
echo "Starting HUD..."
cd /home/alex
sudo -n ./start_hud.sh 2>/dev/null || {
    echo "Error: sudo access not configured. Please run:"
    echo "echo 'alex ALL=(ALL) NOPASSWD: /home/alex/start_hud.sh' | sudo tee /etc/sudoers.d/hud"
    exit 1
}

# The cleanup function will be called automatically when the script exits 