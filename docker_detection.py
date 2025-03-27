import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import os
import numpy as np
import cv2
import hailo
import zmq
import json
import time

from hailo_apps_infra.hailo_rpi_common import (
    get_caps_from_pad,
    get_numpy_from_buffer,
    app_callback_class,
)
from hailo_apps_infra.detection_pipeline import GStreamerDetectionApp

# -----------------------------------------------------------------------------------------------
# User-defined class to be used in the callback function
# -----------------------------------------------------------------------------------------------
class user_app_callback_class(app_callback_class):
    def __init__(self):
        super().__init__()
        # Initialize ZMQ publisher
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind("tcp://*:5555")
        print("ZMQ publisher started on port 5555")

    def __del__(self):
        # Cleanup ZMQ
        if hasattr(self, 'socket'):
            self.socket.close()
        if hasattr(self, 'context'):
            self.context.term()

# -----------------------------------------------------------------------------------------------
# User-defined callback function
# -----------------------------------------------------------------------------------------------
def app_callback(pad, info, user_data):
    # Get the GstBuffer from the probe info
    buffer = info.get_buffer()
    if buffer is None:
        return Gst.PadProbeReturn.OK

    # Using the user_data to count the number of frames
    user_data.increment()
    
    # Get the detections from the buffer
    roi = hailo.get_roi_from_buffer(buffer)
    detections = roi.get_objects_typed(hailo.HAILO_DETECTION)

    # Create list to store detection data
    detection_list = []

    # Parse the detections
    for detection in detections:
        label = detection.get_label()
        bbox = detection.get_bbox()
        confidence = detection.get_confidence()
        
        # Get track ID
        track_id = 0
        track = detection.get_objects_typed(hailo.HAILO_UNIQUE_ID)
        if len(track) == 1:
            track_id = track[0].get_id()

        # Create detection data dictionary
        det_data = {
            'label': label,
            'confidence': float(confidence),  # Convert to float for JSON serialization
            'bbox': [
                float(bbox.xmin()),
                float(bbox.ymin()),
                float(bbox.xmax()),
                float(bbox.ymax())
            ],
            'track_id': track_id
        }
        detection_list.append(det_data)

        # Print to console (keeping original output)
        print(f"Detection: ID: {track_id} Label: {label} Confidence: {confidence:.2f}")

    # Create message for ZMQ
    message = {
        'timestamp': time.time(),
        'frame': user_data.get_count(),
        'detections': detection_list
    }

    # Publish via ZMQ
    try:
        user_data.socket.send_string(json.dumps(message))
    except Exception as e:
        print(f"Error publishing to ZMQ: {e}")

    return Gst.PadProbeReturn.OK

if __name__ == "__main__":
    # Create an instance of the user app callback class
    user_data = user_app_callback_class()
    app = GStreamerDetectionApp(app_callback, user_data)
    app.run()