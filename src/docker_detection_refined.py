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
import math

from hailo_apps_infra.hailo_rpi_common import (
    get_caps_from_pad,
    get_numpy_from_buffer,
    app_callback_class,
)
from hailo_apps_infra.detection_pipeline import GStreamerDetectionApp

# -----------------------------------------------------------------------------------------------
# Camera / Aspect Ratio Parameters
# -----------------------------------------------------------------------------------------------
DIAGONAL_FOV_DEG = 78.0     # Diagonal FOV for Logitech C920
ASPECT_WIDTH     = 16.0
ASPECT_HEIGHT    = 9.0
IMAGE_WIDTH      = 1920.0   # Pipeline's actual width in pixels
IMAGE_HEIGHT     = 1080.0   # Pipeline's actual height in pixels
PROCESS_INTERVAL = 2.0      # Process detections every N seconds
MIN_BBOX_AREA = 50000      # Minimum bounding box area in pixels²

# Debug mode to print detailed angle calculations
DEBUG_ANGLES = True

# -----------------------------------------------------------------------------------------------
# Compute Horizontal FOV from the diagonal FOV
# -----------------------------------------------------------------------------------------------
def compute_horizontal_fov(diag_fov_deg, width, height):
    """
    Given a diagonal FOV (in degrees) and an aspect ratio (width, height),
    compute the resulting horizontal FOV (in degrees).
    """
    diag_fov_rad = math.radians(diag_fov_deg)
    diag_aspect = math.sqrt(width**2 + height**2)
    horizontal_scale = width / diag_aspect
    horizontal_fov_rad = 2.0 * math.atan(horizontal_scale * math.tan(diag_fov_rad / 2.0))
    return math.degrees(horizontal_fov_rad)

CAMERA_HFOV_DEG = compute_horizontal_fov(DIAGONAL_FOV_DEG, ASPECT_WIDTH, ASPECT_HEIGHT)
IMAGE_CENTER_X  = IMAGE_WIDTH / 2.0

print(f"Camera Parameters:")
print(f"- Horizontal FOV: {CAMERA_HFOV_DEG:.1f}°")
print(f"- Image Center: {IMAGE_CENTER_X:.1f} px")
print(f"- Image Width: {IMAGE_WIDTH:.1f} px")
print(f"Publishing detections on port 5555 for LiDAR correlation")

# -----------------------------------------------------------------------------------------------
# Helper function: compute bounding box area
# -----------------------------------------------------------------------------------------------
def compute_bbox_area(bbox):
    """
    Given a bounding box [xmin_px, ymin_px, xmax_px, ymax_px],
    compute its area in pixels².
    """
    x_min, y_min, x_max, y_max = bbox
    width = x_max - x_min
    height = y_max - y_min
    return width * height

# -----------------------------------------------------------------------------------------------
# User-defined class for callback
# -----------------------------------------------------------------------------------------------
class user_app_callback_class(app_callback_class):
    def __init__(self):
        super().__init__()
        # Initialize ZMQ publisher
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind("tcp://*:5555")
        print("ZMQ publisher started on port 5555")
        self.last_process_time = time.time()

    def __del__(self):
        # Cleanup ZMQ
        if hasattr(self, 'socket'):
            self.socket.close()
        if hasattr(self, 'context'):
            self.context.term()

# -----------------------------------------------------------------------------------------------
# Helper function: bounding box center -> angle for 2D LiDAR correlation
# -----------------------------------------------------------------------------------------------
def compute_detection_angle(bbox):
    """
    Given a bounding box [xmin_px, ymin_px, xmax_px, ymax_px] in pixel coordinates,
    compute the horizontal center in pixels, then convert that to an angle in degrees
    relative to the camera's optical center.
    
    Returns angle in degrees where:
    - Negative angles are to the left of center
    - Positive angles are to the right of center
    - Range is approximately -CAMERA_HFOV_DEG/2 to +CAMERA_HFOV_DEG/2
    """
    x_min, y_min, x_max, y_max = bbox
    bbox_center_x = (x_min + x_max) / 2.0

    # Pixel offset from the optical center (-960 to +960 for 1920px width)
    offset_pixels = bbox_center_x - IMAGE_CENTER_X
    
    # Convert to angle using FOV
    # If CAMERA_HFOV_DEG is 70°, then at IMAGE_WIDTH/2 pixels offset we want ±35°
    angle_deg = (offset_pixels / (IMAGE_WIDTH/2)) * (CAMERA_HFOV_DEG/2)

    if DEBUG_ANGLES:
        print(f"\nAngle Calculation Debug:")
        print(f"- Bbox X range: {x_min:.1f} to {x_max:.1f} px")
        print(f"- Bbox center: {bbox_center_x:.1f} px")
        print(f"- Offset from center: {offset_pixels:+.1f} px")
        print(f"- Resulting angle: {angle_deg:+.1f}°")

    return angle_deg

# -----------------------------------------------------------------------------------------------
# Callback for each frame
# -----------------------------------------------------------------------------------------------
def app_callback(pad, info, user_data):
    # Get GstBuffer
    buffer = info.get_buffer()
    if buffer is None:
        return Gst.PadProbeReturn.OK

    # Frame counter
    user_data.increment()

    # Check if enough time has passed since last processing
    current_time = time.time()
    if current_time - user_data.last_process_time < PROCESS_INTERVAL:
        return Gst.PadProbeReturn.OK

    user_data.last_process_time = current_time

    # Get detections
    roi = hailo.get_roi_from_buffer(buffer)
    detections = roi.get_objects_typed(hailo.HAILO_DETECTION)

    # Collection for all detections
    detection_list = []

    for detection in detections:
        label = detection.get_label()
        confidence = detection.get_confidence()

        # Hailo bounding box (likely 0..1 normalized coords)
        bbox = detection.get_bbox()
        x_min_norm = float(bbox.xmin())
        y_min_norm = float(bbox.ymin())
        x_max_norm = float(bbox.xmax())
        y_max_norm = float(bbox.ymax())

        # Convert normalized coords to pixel coords
        x_min_px = x_min_norm * IMAGE_WIDTH
        y_min_px = y_min_norm * IMAGE_HEIGHT
        x_max_px = x_max_norm * IMAGE_WIDTH
        y_max_px = y_max_norm * IMAGE_HEIGHT

        # Build pixel-coordinate bbox
        bbox_coords = [x_min_px, y_min_px, x_max_px, y_max_px]

        # Compute bbox area
        bbox_area = compute_bbox_area(bbox_coords)

        # Skip small detections
        if bbox_area < MIN_BBOX_AREA:
            continue

        # Attempt to get track ID
        track_id = 0
        track = detection.get_objects_typed(hailo.HAILO_UNIQUE_ID)
        if len(track) == 1:
            track_id = track[0].get_id()

        # Compute angle in degrees (for 2D LiDAR correlation)
        angle_deg = compute_detection_angle(bbox_coords)

        # Only include detections within LiDAR's front 180° field of view
        if angle_deg < -90 or angle_deg > 90:
            continue

        # Prepare dictionary
        det_data = {
            'label': label,
            'confidence': float(confidence),
            'angle_deg': angle_deg,    # Most important for LiDAR correlation
            'area': bbox_area,         # Add bbox area for object size
            'bbox': bbox_coords,       # Full data if needed
            'bbox_norm': [x_min_norm, y_min_norm, x_max_norm, y_max_norm],
            'track_id': track_id
        }
        detection_list.append(det_data)

        # Print for debugging (only large objects in LiDAR FOV)
        print(f"\nCamera Detection:")
        print(f"- Label: {label}")
        print(f"- Angle: {angle_deg:+.1f}°")
        print(f"- Area: {bbox_area/1000:.1f}k px²")
        print(f"- Confidence: {confidence:.2f}")

    # Publish via ZMQ (filtered detections)
    if detection_list:  # Only publish if we have detections
        message = {
            'timestamp': time.time(),
            'frame': user_data.get_count(),
            'detections': detection_list
        }
        try:
            user_data.socket.send_string(json.dumps(message))
        except Exception as e:
            print(f"Error publishing to ZMQ: {e}")

    return Gst.PadProbeReturn.OK

if __name__ == "__main__":
    # Instantiate callback class and detection app
    user_data = user_app_callback_class()
    app = GStreamerDetectionApp(app_callback, user_data)
    app.run()