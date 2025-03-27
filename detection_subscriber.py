import zmq
import json
from datetime import datetime
import numpy as np
from collections import deque
import threading
import time

class CombinedSubscriber:
    def __init__(self):
        self.context = zmq.Context()
        
        # Detection subscriber
        self.det_socket = self.context.socket(zmq.SUB)
        self.det_socket.connect("tcp://localhost:5555")
        self.det_socket.setsockopt_string(zmq.SUBSCRIBE, "")
        
        # LIDAR subscriber
        self.lidar_socket = self.context.socket(zmq.SUB)
        self.lidar_socket.connect("tcp://localhost:5556")
        self.lidar_socket.setsockopt_string(zmq.SUBSCRIBE, "LIDAR_DATA")
        
        # Store recent data
        self.latest_detections = None
        self.latest_lidar = None
        self.lock = threading.Lock()
        
        # Rate limiting
        self.last_print_time = 0
        self.print_interval = 2.0  # Print every 2 seconds

    def parse_lidar_message(self, message):
        """Parse plain text LIDAR message into structured data"""
        # Remove the "LIDAR_DATA " prefix
        data_str = message.replace("LIDAR_DATA ", "")
        
        # Parse the measurements
        scan_points = []
        for measurement in data_str.split(';'):
            if measurement.strip():
                angle, distance = map(float, measurement.split(','))
                scan_points.append({
                    'angle': angle,
                    'distance': distance
                })
        
        return {
            'timestamp': time.time(),
            'scan_points': scan_points
        }

    def detection_listener(self):
        while True:
            message = self.det_socket.recv_string()
            data = json.loads(message)
            
            with self.lock:
                self.latest_detections = data
                self.process_combined_data()

    def lidar_listener(self):
        while True:
            message = self.lidar_socket.recv_string()
            data = self.parse_lidar_message(message)
            
            with self.lock:
                self.latest_lidar = data
                self.process_combined_data()

    def process_combined_data(self):
        """Process detection and LIDAR data when both are available"""
        if self.latest_detections is None or self.latest_lidar is None:
            return

        # Rate limit the output
        current_time = time.time()
        if current_time - self.last_print_time < self.print_interval:
            return

        self.last_print_time = current_time

        print("\n" + "="*50)
        print(f"Time: {datetime.fromtimestamp(self.latest_detections['timestamp']).strftime('%H:%M:%S.%f')}")
        
        # Process detections
        print("\nDetections:")
        for det in self.latest_detections['detections']:
            print(f"\nObject: {det['label']}")
            print(f"Confidence: {det['confidence']:.2f}")
            print(f"Track ID: {det['track_id']}")
            print(f"BBox: x1={det['bbox'][0]:.1f}, y1={det['bbox'][1]:.1f}, "
                  f"x2={det['bbox'][2]:.1f}, y2={det['bbox'][3]:.1f}")
            
            # Try to estimate object distance using LIDAR data
            center_x = (det['bbox'][0] + det['bbox'][2]) / 2
            angle_estimate = (center_x / 640) * 60  # Assuming 60° FOV camera
            
            # Find closest LIDAR point to this angle
            closest_point = min(self.latest_lidar['scan_points'], 
                              key=lambda p: abs(p['angle'] - angle_estimate))
            
            print(f"Estimated distance: {closest_point['distance']/1000:.2f}m at {closest_point['angle']:.1f}°")

        # Process LIDAR
        print("\nLIDAR Summary:")
        distances = [p['distance'] for p in self.latest_lidar['scan_points']]
        print(f"Points in scan: {len(distances)}")
        print(f"Min distance: {min(distances)/1000:.2f}m")
        print(f"Max distance: {max(distances)/1000:.2f}m")

    def run(self):
        # Start listener threads
        det_thread = threading.Thread(target=self.detection_listener)
        lidar_thread = threading.Thread(target=self.lidar_listener)
        
        det_thread.daemon = True
        lidar_thread.daemon = True
        
        det_thread.start()
        lidar_thread.start()
        
        print("Combined Subscriber started, waiting for messages...")
        print(f"Output rate: every {self.print_interval} seconds")
        
        try:
            # Keep main thread alive
            while True:
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\nClosing subscriber...")
        finally:
            self.det_socket.close()
            self.lidar_socket.close()
            self.context.term()

if __name__ == "__main__":
    subscriber = CombinedSubscriber()
    subscriber.run() 