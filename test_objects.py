#!/usr/bin/env python3
import zmq
import time
import json
import random
import math

# ZMQ publisher to send simulated object data
context = zmq.Context()
publisher = context.socket(zmq.PUB)
publisher.bind("tcp://*:5557")  # Object detection data on port 5557

# Object classes to simulate
OBJECT_CLASSES = ['person', 'vehicle', 'animal', 'object']

# Generate random objects with random distances and angles
def generate_objects(num_objects=3):
    objects = []
    
    for _ in range(num_objects):
        obj_class = random.choice(OBJECT_CLASSES)
        angle = random.uniform(-85, 85)  # Front 170° field of view
        distance = random.uniform(500, 5000)  # 0.5 to 5 meters
        
        obj = {
            'class': obj_class,
            'angle_deg': angle,
            'distance_mm': distance,
            'confidence': random.uniform(0.7, 1.0),
            'size_mm': random.uniform(200, 1000)  # Size in mm
        }
        objects.append(obj)
    
    return objects

# Randomly generate new detections
def get_new_detections(all_objects, prob=0.2):
    new_detections = []
    for obj in all_objects:
        if random.random() < prob:
            new_detections.append(obj)
    return new_detections

# Move objects to simulate motion
def update_objects(objects):
    for obj in objects:
        # Randomly change angle slightly
        obj['angle_deg'] += random.uniform(-2, 2)
        obj['angle_deg'] = max(-85, min(85, obj['angle_deg']))  # Keep in FOV
        
        # Randomly change distance slightly
        obj['distance_mm'] += random.uniform(-200, 100)
        obj['distance_mm'] = max(500, min(5000, obj['distance_mm']))
    
    # Add or remove objects occasionally
    if random.random() < 0.1:  # 10% chance to add an object
        if len(objects) < 10:  # Maximum number of objects
            objects.append(generate_objects(1)[0])
    
    if random.random() < 0.05 and objects:  # 5% chance to remove an object
        objects.pop(random.randrange(len(objects)))
        
    return objects

def main():
    print("Starting simulated object detection...")
    print("Sending data to port 5557")
    print("Press Ctrl+C to stop")
    
    # Initial objects
    objects = generate_objects(3)
    
    try:
        while True:
            # Update object positions
            objects = update_objects(objects)
            
            # Determine which are new detections
            new_detections = get_new_detections(objects)
            
            # Create message data
            data = {
                'timestamp': time.time(),
                'objects': objects,
                'new_detections': new_detections
            }
            
            # Send the data
            message = "OBJECT_DATA " + json.dumps(data)
            publisher.send_string(message)
            
            print(f"Sent {len(objects)} objects, {len(new_detections)} new detections")
            
            # Wait a bit
            time.sleep(0.5)
            
    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        publisher.close()
        context.term()
        print("Done")

if __name__ == "__main__":
    main() 