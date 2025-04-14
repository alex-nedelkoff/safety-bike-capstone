#!/usr/bin/env python3
import zmq
import json
import time
from collections import deque
from datetime import datetime
import threading
import signal
import sys
import os

# Configuration
PORTS = {
    'detection': 5555,  # Camera detections
    'lidar': 5556,      # Raw LIDAR data
    'correlated': 5557  # Correlated objects
}

# Keep last 1000 messages for analysis
MAX_HISTORY = 1000
UPDATE_INTERVAL = 2.0  # Update display every 2 seconds

class ZMQDebugger:
    def __init__(self):
        print("Initializing ZMQ debugger...")
        self.context = zmq.Context()
        self.sockets = {}
        self.message_history = {port: deque(maxlen=MAX_HISTORY) for port in PORTS}
        self.running = True
        self.last_display_update = 0
        
        # Initialize sockets
        for name, port in PORTS.items():
            print(f"Connecting to {name} on port {port}...")
            socket = self.context.socket(zmq.SUB)
            socket.setsockopt_string(zmq.SUBSCRIBE, "")
            socket.setsockopt(zmq.RCVHWM, 10)
            socket.setsockopt(zmq.CONFLATE, 0)
            socket.setsockopt(zmq.RCVTIMEO, 100)
            socket.setsockopt(zmq.LINGER, 0)
            
            try:
                socket.connect(f"tcp://localhost:{port}")
                print(f"Connected to {name} on port {port}")
                self.sockets[name] = socket
            except Exception as e:
                print(f"Error connecting to {name}: {e}")
        
        # Setup signal handler
        signal.signal(signal.SIGINT, self.signal_handler)
    
    def signal_handler(self, signum, frame):
        print("\nShutting down debugger...")
        self.running = False
    
    def process_message(self, name, msg):
        try:
            if name == 'detection':
                data = json.loads(msg)
                timestamp = data.get('send_time', 0)
                frame = data.get('frame', 0)
                detections = data.get('detections', [])
                
                self.message_history[name].append({
                    'timestamp': timestamp,
                    'frame': frame,
                    'receive_time': time.time(),
                    'detections': detections
                })
                
            elif name == 'correlated':
                # Only log the message, no debug prints
                try:
                    data = json.loads(msg)
                    objects = data.get("objects", [])
                    timestamp = data.get("timestamp", 0)
                    
                    self.message_history[name].append({
                        'timestamp': timestamp,
                        'receive_time': time.time(),
                        'objects': objects
                    })
                except json.JSONDecodeError as e:
                    pass  # Silently ignore JSON errors
                
            elif name == 'lidar':
                # Store LIDAR data with minimal processing
                self.message_history[name].append({
                    'receive_time': time.time()
                })
                
        except Exception:
            pass  # Silently ignore errors
    
    def analyze_latencies(self):
        """Thread that analyzes latencies but only displays results periodically."""
        while self.running:
            current_time = time.time()
            
            # Only update the display once per UPDATE_INTERVAL
            if current_time - self.last_display_update >= UPDATE_INTERVAL:
                self.last_display_update = current_time
                
                # Clear the screen before new output
                print("\033[2J\033[H", end="")  # ANSI escape code to clear screen and move cursor to home
                
                print("\n=== Object Detection System ===")
                print(f"Time: {datetime.now().strftime('%H:%M:%S')}")
                
                # Check for correlated objects
                if 'correlated' in self.message_history and len(self.message_history['correlated']) > 0:
                    latest = self.message_history['correlated'][-1]
                    objects = latest.get('objects', [])
                    if objects:
                        print("\nCorrelated Objects (Camera + LIDAR):")
                        print("----------------------------------")
                        print("   Object |   Angle | Distance | Confidence | Match Diff")
                        print("--------------------------------------------------------")
                        for obj in objects:
                            label = obj.get('label', 'unknown')
                            angle = obj.get('angle_deg', 0)
                            distance = obj.get('distance_mm', 0) / 1000.0  # Convert to meters
                            confidence = obj.get('confidence', 0) * 100  # Convert to percentage
                            angle_diff = obj.get('angle_diff', 0)  # Get angle difference if available
                            
                            print(f"{label:>10}: {angle:>6.1f}° | {distance:>5.2f}m |  {confidence:>5.1f}% | {angle_diff:>5.1f}°")
                    else:
                        print("\nNo objects detected")
                else:
                    print("\nNo correlated objects received yet")
                
                # Only show update rate for correlated data
                if 'correlated' in self.message_history and len(self.message_history['correlated']) > 1:
                    history = self.message_history['correlated']
                    time_diff = history[-1]['receive_time'] - history[0]['receive_time']
                    if time_diff > 0:
                        rate = len(history) / time_diff
                        print(f"\nUpdate rate: {rate:>5.1f} objects/sec")
            
            # Sleep briefly to avoid high CPU usage
            time.sleep(0.05)  # 50ms sleep for more responsive updates
    
    def run(self):
        print("\nStarting Camera-LIDAR Correlation Monitor...")
        print("Press Ctrl+C to exit\n")
        
        # Start analysis thread
        analysis_thread = threading.Thread(target=self.analyze_latencies)
        analysis_thread.daemon = True
        analysis_thread.start()
        
        # Main receive loop
        poller = zmq.Poller()
        for name, socket in self.sockets.items():
            poller.register(socket, zmq.POLLIN)
        
        while self.running:
            try:
                socks = dict(poller.poll(100))  # 100ms timeout
                
                for name, socket in self.sockets.items():
                    if socket in socks:
                        try:
                            msg = socket.recv_string(zmq.NOBLOCK)
                            self.process_message(name, msg)
                        except zmq.Again:
                            continue
                        except Exception:
                            pass  # Silently ignore errors
                            
            except Exception:
                continue  # Silently ignore errors
        
        # Cleanup
        for socket in self.sockets.values():
            socket.close()
        self.context.term()
        print("Debugger stopped")

if __name__ == "__main__":
    debugger = ZMQDebugger()
    debugger.run()
