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
import psutil  # Add this for CPU affinity control

from hailo_apps_infra.hailo_rpi_common import (
    get_caps_from_pad,
    get_numpy_from_buffer,
    app_callback_class,
)
from hailo_apps_infra.detection_pipeline import GStreamerDetectionApp

# Set process priority and CPU affinity
try:
    process = psutil.Process()
    # Use only cores 2 and 3, leaving 0 and 1 for system
    process.cpu_affinity([2, 3])
    # Set nice value to give other processes priority
    os.nice(10)
    
    # Set GStreamer environment variables for performance
    os.environ["GST_DEBUG"] = "0"
    os.environ["GST_GL_XINITTHREADS"] = "0"
    os.environ["GST_PIPELINE_LATENCY"] = "0"
    os.environ["GST_V4L2_USE_LIBV4L2"] = "1"
    os.environ["GST_V4L2_MIN_BUFFERS"] = "2"
    os.environ["GST_V4L2_MAX_BUFFERS"] = "2"
    os.environ["GST_V4L2_FORCE_LEGACY"] = "1"
    os.environ["GST_V4L2_FORCE_MMAP"] = "1"
    
    # Disable video display/rendering
    os.environ["GST_PLUGIN_FEATURE_RANK"] = "fakesink:HIGH"   # Prioritize fakesink
    os.environ["DISABLE_DISPLAY"] = "1"  # Custom environment var for apps to check
    os.environ["GST_VIDEO_SINK"] = "fakesink"  # Force fakesink
    os.environ["NO_AT_BRIDGE"] = "1"  # Disable accessibility bus

except Exception as e:
    print(f"Warning: Could not set performance options: {e}")

# Performance tuning constants
OBJECT_PERSISTENCE = 0.25  # Keep objects for 250ms
POLL_TIMEOUT = 0  # No timeout for fastest updates
MAX_FPS = 20  # Further reduced from 30 to 20 FPS
ZMQ_HWM = 1
ANGLE_BUCKET_SIZE = 5.0
MAX_ANGLE_DIFF = 10.0
SMOOTHING_ALPHA = 0.3  # Added smoothing factor for measurements

# Camera / Aspect Ratio Parameters
DIAGONAL_FOV_DEG = 78.0     # Diagonal FOV for Logitech C920
ASPECT_WIDTH     = 16.0
ASPECT_HEIGHT    = 9.0
IMAGE_WIDTH      = 1280.0   # Reduced from 1920 to 1280
IMAGE_HEIGHT     = 720.0    # Reduced from 1080 to 720
PROCESS_INTERVAL = 0.05     # Process at 20fps (50ms) instead of 60fps
MIN_BBOX_AREA = 5000       # Adjusted for lower resolution
MAX_QUEUE_SIZE = 1         # Only keep latest detection
CONFIDENCE_THRESHOLD = 0.45  # Slightly increased to reduce false positives

# Performance optimization flags
ENABLE_TRACKING = False     # Disable tracking if not needed
DEBUG_ANGLES = False       # Keep debug disabled for performance
BATCH_PROCESSING = True    # Enable batch processing of detections
USE_CUDA = False          # Disable CUDA to reduce CPU overhead
ENABLE_THREADING = False   # Disable threading to reduce overhead

# GStreamer pipeline optimization
GST_PIPELINE_FLAGS = {
    "sync": False,           # Disable pipeline sync
    "async": False,          # Disable async state changes
    "max-lateness": 0,       # Drop late frames
    "drop": True,           # Allow frame dropping
    "throttle-time": 50000000  # 50ms throttle (in ns)
}

# Camera optimization
GST_CAMERA_FLAGS = {
    "device": "/dev/video0",
    "io-mode": 2,           # Use memory mapped buffers
    "num-buffers": 2,       # Minimize buffer queue
    "do-timestamp": False   # Disable timestamping
}

# Define optimized GStreamer caps
GST_CAPS = (
    "video/x-raw,format=RGB,width={},height={},"
    "framerate=20/1,pixel-aspect-ratio=1/1"
).format(int(IMAGE_WIDTH), int(IMAGE_HEIGHT))

# Compute scale factor for bbox coordinates
SCALE_FACTOR = 1280.0/1920.0  # Scale factor for coordinate conversion

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
print("Camera detection system initialized and running...")

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
        # Initialize ZMQ publisher with optimized settings
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        
        # Set socket options for performance
        self.socket.setsockopt(zmq.SNDHWM, 1)  # Only queue 1 message
        self.socket.setsockopt(zmq.LINGER, 0)  # Don't wait when closing
        self.socket.setsockopt(zmq.CONFLATE, 1)  # Only keep latest message
        self.socket.setsockopt(zmq.RCVBUF, 8192)  # Smaller receive buffer
        self.socket.setsockopt(zmq.SNDBUF, 8192)  # Smaller send buffer
        
        self.socket.bind("tcp://*:5555")
        print("ZMQ publisher started on port 5555")
        self.last_process_time = time.time()
        self.camera_hfov = compute_horizontal_fov(DIAGONAL_FOV_DEG, ASPECT_WIDTH, ASPECT_HEIGHT)
        print(f"Camera horizontal FOV: {self.camera_hfov:.1f}°")
        
        # Pre-compute constants for angle calculation
        self.angle_scale = (self.camera_hfov/2) / (IMAGE_WIDTH/2)
        self.image_center = IMAGE_WIDTH / 2.0

    def __del__(self):
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
    buffer = info.get_buffer()
    if buffer is None:
        return Gst.PadProbeReturn.OK

    # Check processing interval (throttle to 20fps)
    current_time = time.time()
    if current_time - user_data.last_process_time < PROCESS_INTERVAL:
        return Gst.PadProbeReturn.OK
    user_data.last_process_time = current_time

    # Get detections efficiently
    roi = hailo.get_roi_from_buffer(buffer)
    detections = roi.get_objects_typed(hailo.HAILO_DETECTION)

    # Process all detections in batch
    detection_list = []
    
    # Pre-compute constants outside loop
    image_center = user_data.image_center
    angle_scale = user_data.angle_scale
    
    for detection in detections:
        # Quick confidence check first
        confidence = detection.get_confidence()
        if confidence < CONFIDENCE_THRESHOLD:
            continue

        bbox = detection.get_bbox()
        
        # Compute bbox coordinates more efficiently
        x_min = float(bbox.xmin()) * IMAGE_WIDTH
        x_max = float(bbox.xmax()) * IMAGE_WIDTH
        
        # Quick area check using only width (faster than full area)
        width = x_max - x_min
        if width * width < MIN_BBOX_AREA:  # Approximate area check
            continue

        # Compute angle efficiently
        bbox_center_x = (x_min + x_max) * 0.5
        angle_deg = (bbox_center_x - image_center) * angle_scale
        
        # Quick angle check
        if angle_deg < -90.0 or angle_deg > 90.0:
            continue

        # Only compute y coordinates if we passed other checks
        y_min = float(bbox.ymin()) * IMAGE_HEIGHT
        y_max = float(bbox.ymax()) * IMAGE_HEIGHT
        area = width * (y_max - y_min)

        # Get detection info
        label = detection.get_label()

        # Only get track ID if tracking is enabled
        track_id = 0
        if ENABLE_TRACKING:
            track = detection.get_objects_typed(hailo.HAILO_UNIQUE_ID)
            if len(track) == 1:
                track_id = track[0].get_id()

        # Add to detection list
        detection_list.append({
            'label': label,
            'confidence': float(confidence),
            'angle_deg': angle_deg,
            'area': area,
            'bbox': [x_min, y_min, x_max, y_max],
            'track_id': track_id
        })

    # Always publish via ZMQ, even if detection_list is empty
    message = {
        'timestamp': current_time,
        'frame': user_data.get_count(),
        'detections': detection_list
    }
    
    try:
        # Use pre-encoded JSON string for better performance
        json_str = json.dumps(message)
        user_data.socket.send_string(json_str, zmq.NOBLOCK)
    except zmq.error.Again:
        pass  # Skip if can't send immediately
    except Exception as e:
        if DEBUG_ANGLES:
            print(f"Error publishing to ZMQ: {e}")

    return Gst.PadProbeReturn.OK

# -----------------------------------------------------------------------------------------------
# Custom GStreamer Detection App with disabled video sink
# -----------------------------------------------------------------------------------------------
class NoDisplayDetectionApp(GStreamerDetectionApp):
    def __init__(self, callback, user_data):
        # Initialize with fakesink explicitly
        super().__init__(callback, user_data)
        print("Using headless detection app (no display output)")

if __name__ == "__main__":
    # Instantiate callback class and detection app
    user_data = user_app_callback_class()
    
    try:
        # First approach: try to pass custom source/sink parameters if supported
        app = GStreamerDetectionApp(
            app_callback,
            user_data,
            source="/dev/video0",  # Use direct device access
            network_sink="fakesink",  # Use fakesink for network
            display_sink="fakesink"  # Use fakesink for display
        )
    except TypeError:
        # Fallback: use the simplified approach
        print("Using simplified headless app initialization")
        app = NoDisplayDetectionApp(
            app_callback, 
            user_data
        )
    
    print("Starting detection app with display disabled...")
    app.run()
