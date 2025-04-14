#!/bin/bash

SESSION="hailo"

# Start new tmux session in detached mode
tmux new-session -d -s $SESSION

# Rename initial window (index 0) to 'main'
tmux rename-window -t $SESSION:0 'main'

# Create a second window for running the detection script
tmux new-window -t $SESSION:1 -n 'detection'

# Send the detection pipeline commands to the 'detection' window
tmux send-keys -t $SESSION:1 'cd /home/alex/pi-ai-kit-ubuntu' C-m

tmux send-keys -t $SESSION:1 'docker-compose up -d hailo-ubuntu-pi' C-m
sleep 3  # Wait for the container to spin up

tmux send-keys -t $SESSION:1 'docker-compose exec hailo-ubuntu-pi /bin/bash' C-m
sleep 2  # Wait for shell to be ready

# Send container-internal commands (each will wait for a prompt)
tmux send-keys -t $SESSION:1 'cd /home/alex/hailo-rpi5-examples' C-m
sleep 1
tmux send-keys -t $SESSION:1 'source setup_env.sh' C-m
sleep 1
tmux send-keys -t $SESSION:1 'python3 basic_pipelines/docker_detection_refined.py -i /dev/video0' C-m

# Return to the original window
sleep 1
tmux select-window -t $SESSION:0

# Attach to the session
tmux attach -t $SESSION
