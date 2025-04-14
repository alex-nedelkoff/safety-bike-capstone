#!/usr/bin/env python3
import sys
import math
import time
import zmq
import pygame
import threading
import os
import json
from collections import defaultdict

# ----------------------------
# Display & Radar Config
# ----------------------------
SCREEN_WIDTH = 800   # Landscape mode
SCREEN_HEIGHT = 480  # Landscape mode

CENTER_X = SCREEN_WIDTH // 2
CENTER_Y = 40  # Fixed position 40 pixels from top

# We assume your LiDAR data goes up to about 10m (10000 mm) for bike safety
MAX_RANGE_MM = 5000  # Maximum range in millimeters

# The radar circle radius is fixed at 400 pixels for the bottom half
RADAR_RADIUS = 400  # Fixed radius for bottom half of display

# Performance tuning constants
OBJECT_PERSISTENCE = 0.25  # Keep objects for 250ms
POLL_TIMEOUT = 0  # No timeout for fastest updates
MAX_FPS = 30  # Reduced from 60 to 30 FPS
ZMQ_HWM = 1
ANGLE_BUCKET_SIZE = 5.0
MAX_ANGLE_DIFF = 10.0
SMOOTHING_ALPHA = 0.3  # Added smoothing factor for measurements

# Animation parameters
ANIMATION_SPEED = 0.4  # Slightly increased for smoother motion at lower FPS
POSITION_LERP_ALPHA = 0.2  # Slightly increased for faster response
FADE_IN_TIME = 0.1  # Time to fade in new objects
FADE_OUT_TIME = 0.2  # Reduced fade out time
MIN_FADE_FACTOR = 0.85  # Minimum opacity for objects
MIN_ICON_ALPHA = 0.9  # Minimum opacity for icons

# New optimization flags
ENABLE_SMOOTH_RENDERING = True  # Use pygame.SCALED for better performance
ENABLE_ICON_CACHING = True     # Cache scaled icons
BATCH_RENDERING = True         # Batch similar draw operations
DEBUG_PERFORMANCE = False      # Only show performance metrics when needed

# Colors: ARGB format, fully opaque (0xFF at the top bits)
COLORS = {
    'background': 0xFF000000,   # black
    'radar_grid': 0xFF303030,   # dark gray
    'radar_line': 0xFF00FF00,   # green
    'lidar_pt':   0xFFFF0000,   # red
    'text':       0xFFFFFFFF,   # white
    'bike_obj':   0xFFFF6A00,   # orange for bike-relevant objects
    'other_obj':  0xFF00FF00,   # green for other objects
    'warning':    0xFFFF0000,   # red
    'safe':       0xFF00FF00,   # green
    'unknown':    0xFF0088FF,   # light blue
}

# Bike-relevant object classes
BIKE_OBJECTS = {
    'person': {'color': 0xFFFF6A00, 'priority': 1},
    'bicycle': {'color': 0xFF00FF00, 'priority': 2},
    'car': {'color': 0xFF0000FF, 'priority': 3},
    'motorcycle': {'color': 0xFF00FFFF, 'priority': 2},
    'bus': {'color': 0xFFFF00FF, 'priority': 4},
    'truck': {'color': 0xFFFF00FF, 'priority': 4}
}

# Add IMU configuration
IMU_PORT = 5558
IMU_THRESHOLD = 10.0  # m/s²

# ----------------------------
# Performance Optimization
# ----------------------------
USE_FRAMEBUFFER = True    # Force framebuffer mode for better performance
DIRTY_RECTS = True        # Use dirty rectangle updates
UPDATE_INTERVAL = 0.1     # Update screen at 10Hz (100ms)
FAST_MODE = True          # Use fastest possible rendering
SKIP_FRAMES = 3           # Only draw every 3rd frame

class LidarHUD:
    def __init__(self):
        # Print current environment info
        print("Current environment:")
        print(f"User: {os.getenv('USER')}")
        print(f"DISPLAY: {os.getenv('DISPLAY')}")
        print(f"XDG_RUNTIME_DIR: {os.getenv('XDG_RUNTIME_DIR')}")
        
        # Initialize SDL with more debug info
        print("\nInitializing display...")
        os.environ['SDL_DEBUG'] = '1'  # Enable SDL debug output
        
        # Don't force framebuffer initially
        if 'SDL_VIDEODRIVER' in os.environ:
            print(f"Removing existing SDL_VIDEODRIVER: {os.environ['SDL_VIDEODRIVER']}")
            del os.environ['SDL_VIDEODRIVER']
        
        pygame.init()
        
        # Try display drivers in order of preference for Raspberry Pi
        drivers = ['x11', 'wayland', 'fbdev', 'kmsdrm', 'directfb', 'dummy']
        screen = None
        
        for driver in drivers:
            try:
                print(f"\nTrying SDL video driver: {driver}")
                
                # Clean up any previous attempts
                pygame.display.quit()
                
                # Set the driver
                os.environ['SDL_VIDEODRIVER'] = driver
                
                # Initialize display system
                pygame.display.init()
                
                # Get available display modes
                print(f"Available display modes for {driver}:")
                try:
                    modes = pygame.display.list_modes()
                    print(f"Display modes: {modes}")
                except:
                    print("Could not get display modes")
                
                # Try to set display mode
                try:
                    if driver == 'dummy':
                        # Dummy driver just for testing
                        screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
                    else:
                        # Try fullscreen first
                        flags = pygame.FULLSCREEN | pygame.NOFRAME
                        screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT), flags)
                except pygame.error as e:
                    # If fullscreen fails, try windowed mode
                    print(f"Fullscreen failed: {e}, trying windowed mode")
                    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
                
                print(f"Success with driver: {driver}")
                break
                
            except Exception as e:
                print(f"Driver {driver} failed: {str(e)}")
                continue
        
        if screen is None:
            print("ERROR: All display drivers failed")
            print("Available pygame drivers:", pygame.display.get_drivers())
            print("Current driver:", pygame.display.get_driver())
            sys.exit(1)
        
        self.screen = screen
        pygame.display.set_caption("LiDAR Radar (Front 180°)")
        
        # Hide mouse cursor
        try:
            pygame.mouse.set_visible(False)
        except:
            pass

        self.font = pygame.font.Font(None, 32)
        self.large_font = pygame.font.Font(None, 42) 
        
        # Load images (will be None if file doesn't exist)
        self.loaded_images = {}
        self.current_image = None
        self.alpha_overlay = True  # Whether to make images semi-transparent
        
        # Try to load test.png if exists
        self.load_image("test.png")
        
        # Load object icons
        self.object_icons = {}
        self.load_object_icons()

        # ZMQ subscriber for LiDAR data and object detections
        self.context = zmq.Context()
        
        # Configure ZMQ sockets with optimized settings
        self.lidar_subscriber = self.context.socket(zmq.SUB)
        self.object_subscriber = self.context.socket(zmq.SUB)
        
        # Set socket options for performance
        for socket in [self.lidar_subscriber, self.object_subscriber]:
            socket.setsockopt(zmq.RCVHWM, 1)  # Only keep 1 message in receive buffer
            socket.setsockopt(zmq.LINGER, 0)  # Don't wait when closing
            socket.setsockopt(zmq.CONFLATE, 1)  # Only keep latest message
            socket.setsockopt(zmq.RCVTIMEO, 0)  # Non-blocking receive
            socket.setsockopt_string(zmq.SUBSCRIBE, "")
            socket.setsockopt(zmq.RCVBUF, 1024)  # Smaller receive buffer (1KB)

        # Connect sockets
        try:
            self.lidar_subscriber.connect("tcp://localhost:5556")  # LIDAR data
            self.object_subscriber.connect("tcp://localhost:5557")  # Correlated objects
            print("Connected to ZMQ ports 5556 and 5557")
        except zmq.error.ZMQError as e:
            print(f"Failed to connect ZMQ sockets: {e}")
            print("Make sure lidar_zmq_refined is running!")
            sys.exit(1)

        # Add IMU subscriber
        self.imu_subscriber = self.context.socket(zmq.SUB)
        self.imu_subscriber.setsockopt(zmq.RCVHWM, 1)
        self.imu_subscriber.setsockopt(zmq.LINGER, 0)
        self.imu_subscriber.setsockopt(zmq.CONFLATE, 1)
        self.imu_subscriber.setsockopt(zmq.RCVTIMEO, 0)
        self.imu_subscriber.setsockopt_string(zmq.SUBSCRIBE, "")
        self.imu_subscriber.setsockopt(zmq.RCVBUF, 1024)

        try:
            self.imu_subscriber.connect(f"tcp://localhost:{IMU_PORT}")
            print(f"Connected to IMU port {IMU_PORT}")
        except zmq.error.ZMQError as e:
            print(f"Failed to connect to IMU socket: {e}")
            sys.exit(1)

        # Initialize data structures
        self.lidar_points = []
        self.lidar_angle_map = {}
        self.detected_objects = []
        self.object_history = {}
        self.history_length = 3  # Reduced for faster updates
        self.last_lidar_update = 0
        self.last_object_update = 0
        
        # Display settings
        self.running = True
        self.show_radar = True
        self.show_debug = True
        self.angle_offset = 90
        self.show_lidar = True
        
        # Start data collection thread
        self.data_thread = threading.Thread(target=self.collect_data, daemon=True)
        self.data_thread.start()
        
        # Initialize animation state
        self.object_positions = {}  # Current interpolated positions
        self.object_prev_pos = {}   # Previous positions for interpolation
        self.object_target_pos = {} # Target positions for interpolation
        self.object_alphas = {}     # Current fade values
        self.last_frame_time = time.time()
        
        # Initialize icon cache
        self.icon_cache = {}
        self.last_cleanup = time.time()
        
        # Create surface for batch rendering
        self.radar_surface = pygame.Surface((SCREEN_WIDTH, SCREEN_HEIGHT), pygame.SRCALPHA)
        self.text_surface = pygame.Surface((SCREEN_WIDTH, SCREEN_HEIGHT), pygame.SRCALPHA)
        
        # Optimize pygame
        pygame.display.set_allow_screensaver(True)  # Allow screensaver when inactive
        if ENABLE_SMOOTH_RENDERING:
            self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT), 
                                                pygame.SCALED | pygame.FULLSCREEN)
        
        # Create dirty rectangle tracking
        self.dirty_rects = []
        self.last_screen_update = 0
        self.frame_counter = 0
        
        # Add IMU state tracking
        self.imu_warning = False
        self.last_imu_update = 0
    
    def load_object_icons(self):
        """Load icons for different object types from the icons directory"""
        icons_dir = os.path.join(os.path.dirname(__file__), "icons")
        print(f"Loading icons from: {icons_dir}")
        
        for obj_type in BIKE_OBJECTS:
            icon_path = os.path.join(icons_dir, f"{obj_type}.png")
            if os.path.exists(icon_path):
                try:
                    icon = pygame.image.load(icon_path)
                    # Convert to RGBA if not already
                    if icon.get_alpha() is None:
                        icon = icon.convert_alpha()
                    self.object_icons[obj_type] = icon
                    print(f"Loaded icon for {obj_type}")
                except Exception as e:
                    print(f"Error loading icon for {obj_type}: {e}")
            else:
                print(f"No icon found for {obj_type} at {icon_path}")

    def load_image(self, path):
        """Load a PNG image from the given path."""
        if os.path.exists(path):
            try:
                img = pygame.image.load(path)
                self.loaded_images[path] = img
                print(f"Loaded image: {path}")
                # Use the first loaded image as current
                if self.current_image is None:
                    self.current_image = path
                return True
            except Exception as e:
                print(f"Error loading image {path}: {e}")
        else:
            print(f"Image file not found: {path}")
        return False
    
    def set_current_image(self, path):
        """Set the current image to display."""
        if path in self.loaded_images:
            self.current_image = path
            return True
        return self.load_image(path)  # Try to load if not already loaded

    def predict_position(self, obj_id, current_time):
        """Simplified prediction (only used if target position is missing)"""
        if not ENABLE_SMOOTH_RENDERING or obj_id not in self.object_history:
            return None
            
        hist = self.object_history[obj_id]
        time_delta = current_time - hist['last_update']
        
        # Only predict for a short time
        if time_delta > MAX_PREDICTION_TIME:
            return None
        
        # Simply use the last known position (no velocity prediction)
        return {
            'distance_mm': hist['distance'],
            'angle_deg': hist['angle'],
            'area': hist['area'],
            'predicted': True
        }

    def smooth_measurement(self, obj_id, measurement):
        """Simplified smoothing without velocity calculation"""
        current_time = time.time()
        
        # Initialize history if needed
        if obj_id not in self.object_history:
            self.object_history[obj_id] = {
                'distance': measurement['distance_mm'],
                'angle': measurement['angle_deg'],
                'area': measurement['area'],
                'last_update': current_time
            }
            return measurement

        hist = self.object_history[obj_id]
        
        # Simple position smoothing
        hist['distance'] = SMOOTHING_ALPHA * measurement['distance_mm'] + (1 - SMOOTHING_ALPHA) * hist['distance']
        hist['angle'] = SMOOTHING_ALPHA * measurement['angle_deg'] + (1 - SMOOTHING_ALPHA) * hist['angle']
        hist['area'] = SMOOTHING_ALPHA * measurement['area'] + (1 - SMOOTHING_ALPHA) * hist['area']
        hist['last_update'] = current_time

        # Update measurement with smoothed values
        measurement['distance_mm'] = hist['distance']
        measurement['angle_deg'] = hist['angle']
        measurement['area'] = hist['area']
        return measurement

    def update_lidar_map(self):
        """Update angle-to-distance map for faster correlation"""
        self.lidar_angle_map.clear()
        for angle, dist in self.lidar_points:
            # Round angle to nearest bucket
            bucket = round(angle / ANGLE_BUCKET_SIZE) * ANGLE_BUCKET_SIZE
            # Keep shortest distance for each angle bucket
            if bucket not in self.lidar_angle_map or dist < self.lidar_angle_map[bucket]:
                self.lidar_angle_map[bucket] = dist

    def find_closest_lidar_point(self, target_angle):
        """Find closest LIDAR point to target angle efficiently"""
        if not self.lidar_angle_map:
            return None, float('inf')

        # Get angle bucket
        bucket = round(target_angle / ANGLE_BUCKET_SIZE) * ANGLE_BUCKET_SIZE
        
        # Check target bucket and adjacent buckets
        min_diff = float('inf')
        best_dist = 0
        
        for offset in [0, -ANGLE_BUCKET_SIZE, ANGLE_BUCKET_SIZE]:
            check_bucket = bucket + offset
            if check_bucket in self.lidar_angle_map:
                diff = abs(target_angle - check_bucket)
                if diff < min_diff:
                    min_diff = diff
                    best_dist = self.lidar_angle_map[check_bucket]
        
        return best_dist, min_diff

    def collect_data(self):
        """Continuously receive LiDAR, object detection, and IMU data from ZMQ."""
        poller = zmq.Poller()
        # Register all subscribers
        poller.register(self.object_subscriber, zmq.POLLIN)
        poller.register(self.lidar_subscriber, zmq.POLLIN)
        poller.register(self.imu_subscriber, zmq.POLLIN)
        
        last_print_time = time.time()
        message_count = 0
        
        while self.running:
            try:
                socks = dict(poller.poll(1))
                current_time = time.time()
                
                # Check IMU data
                if self.imu_subscriber in socks:
                    try:
                        msg = self.imu_subscriber.recv_string(zmq.NOBLOCK)
                        if msg:
                            data = json.loads(msg)
                            if "imu" in data:
                                imu_data = data["imu"]
                                # Check for extreme acceleration (both positive and negative)
                                accel_x = imu_data.get("accel_x", 0)
                                accel_y = imu_data.get("accel_y", 0)
                                accel_z = imu_data.get("accel_z", 0)
                                
                                # Check if we're entering or exiting warning state
                                was_warning = self.imu_warning
                                self.imu_warning = (accel_x > IMU_THRESHOLD or accel_x < -IMU_THRESHOLD or
                                                   accel_y > IMU_THRESHOLD or accel_y < -IMU_THRESHOLD or
                                                   accel_z > IMU_THRESHOLD or accel_z < -IMU_THRESHOLD)
                                self.last_imu_update = current_time
                                
                                # Force HUD update if warning state changed
                                if self.imu_warning != was_warning:
                                    self.last_screen_update = 0  # Force next frame to update immediately
                                    if self.imu_warning:
                                        print(f"IMU Warning! Acceleration: X={accel_x:.1f}, Y={accel_y:.1f}, Z={accel_z:.1f} m/s²")
                                    else:
                                        print("IMU returned to normal values")
                    except Exception as e:
                        print(f"Error processing IMU data: {e}")

                # Always check object data first (higher priority)
                if self.object_subscriber in socks:
                    # Skip intermediate messages, get only the latest
                    msg = None
                    for _ in range(10):  # Try up to 10 times to get latest message
                        try:
                            new_msg = self.object_subscriber.recv_string(zmq.NOBLOCK)
                            if new_msg:  # Only keep if valid
                                msg = new_msg
                                message_count += 1
                        except zmq.Again:
                            break  # No more messages
                        except Exception as e:
                            print(f"Error receiving object data: {e}")
                            break
                    
                    if msg:
                        try:
                            data = json.loads(msg)
                            if data.get("type") == "OBJECTS" and "objects" in data:                                
                                # Print timing diagnostics
                                timestamp = data.get("timestamp", 0)
                                forced = data.get("forced", False)
                                
                                # Calculate and print latency
                                if timestamp:
                                    timestamp_sec = timestamp / 1000.0  # Convert from ms to seconds
                                    latency = (current_time - timestamp_sec) * 1000.0  # Convert to ms
                                    if latency > 100:  # Only print significant latencies
                                        print(f"HUD: Object latency {latency:.1f}ms (send: {timestamp_sec:.3f}, receive: {current_time:.3f})")
                                
                                # Process objects (this needs to be fast)
                                new_objects = []
                                for obj_data in data["objects"]:
                                    obj = {
                                        'class': obj_data.get('label', 'unknown'),
                                        'confidence': float(obj_data.get('confidence', 0)),
                                        'angle_deg': float(obj_data.get('angle_deg', 0)),
                                        'distance_mm': float(obj_data.get('distance_mm', 0)),
                                        'area': float(obj_data.get('area', 10000)),
                                        'timestamp': current_time,
                                        'last_seen': current_time,
                                        'predicted': False
                                    }
                                    
                                    # Apply simple smoothing
                                    obj_id = f"{obj['class']}_{int(obj['angle_deg'])}"
                                    self.smooth_measurement(obj_id, obj)
                                    
                                    new_objects.append(obj)
                                
                                # Replace objects rather than appending
                                self.detected_objects = new_objects
                                self.last_object_update = current_time
                        except json.JSONDecodeError:
                            pass
                        except Exception as e:
                            print(f"Error processing objects: {e}")
                
                # Process LIDAR data with lower priority
                if self.lidar_subscriber in socks:
                    try:
                        msg = self.lidar_subscriber.recv_string(zmq.NOBLOCK)
                        if msg.startswith("LIDAR_DATA"):
                            data_str = msg[10:].strip()
                            self.lidar_points = [
                                (float(p.split(',')[0]), float(p.split(',')[1]))
                                for p in data_str.split(';')
                                if p and ',' in p
                            ]
                            self.update_lidar_map()
                            self.last_lidar_update = current_time
                    except Exception:
                        pass
                
                # Only remove very old objects (hasn't been updated in OBJECT_PERSISTENCE seconds)
                self.detected_objects = [
                    obj for obj in self.detected_objects
                    if (current_time - obj.get('last_seen', 0)) < OBJECT_PERSISTENCE
                ]

            except zmq.Again:
                continue
            except Exception as e:
                print(f"Error in collect_data: {e}")
                continue

    def interpolate_position(self, current, target, alpha):
        """Smoothly interpolate between two positions with easing"""
        # Apply easing function to alpha for smoother transitions
        eased_alpha = 1.0 - (1.0 - alpha) * (1.0 - alpha)  # Ease out quad
        
        return {
            'distance_mm': current['distance_mm'] + (target['distance_mm'] - current['distance_mm']) * eased_alpha,
            'angle_deg': current['angle_deg'] + (target['angle_deg'] - current['angle_deg']) * eased_alpha,
            'area': current['area'] + (target['area'] - current['area']) * eased_alpha
        }

    def update_object_animation(self, obj_id, target_pos):
        """Update animation state for an object using simple interpolation with easing"""
        current_time = time.time()
        dt = current_time - self.last_frame_time
        dt = min(dt, 0.1)  # Cap dt to prevent large jumps
        
        # If this is a new object, initialize it
        if obj_id not in self.object_positions:
            self.object_positions[obj_id] = target_pos.copy()
            self.object_prev_pos[obj_id] = target_pos.copy()
            self.object_target_pos[obj_id] = target_pos.copy()
            self.object_alphas[obj_id] = 0  # Start fully transparent
            return
        
        # Only update target position if significant change
        current_pos = self.object_positions[obj_id]
        dist_change = abs(target_pos['distance_mm'] - self.object_target_pos[obj_id]['distance_mm'])
        angle_change = abs(target_pos['angle_deg'] - self.object_target_pos[obj_id]['angle_deg'])
        
        # If position changed significantly, update interpolation targets
        if dist_change > 5 or angle_change > 0.5:
            self.object_prev_pos[obj_id] = current_pos.copy()
            self.object_target_pos[obj_id] = target_pos.copy()
        
        # Update fade in
        if obj_id in self.object_alphas:
            self.object_alphas[obj_id] = min(1.0, self.object_alphas[obj_id] + dt / FADE_IN_TIME)
        
        # Calculate interpolation factor based on time
        # This makes the movement smoother with consistent speed
        lerp_factor = min(1.0, ANIMATION_SPEED * dt * 5)
        
        # Interpolate between current position and target position
        self.object_positions[obj_id] = self.interpolate_position(
            current_pos, 
            self.object_target_pos[obj_id], 
            lerp_factor
        )

    def draw_object(self, obj, surface=None):
        """Enhanced draw_object with animation support"""
        if surface is None:
            surface = self.screen
            
        # Get object properties
        obj_class = obj.get('class', 'object').lower()
        obj_id = f"{obj_class}_{int(obj.get('angle_deg', 0))}"
        
        # Get animated position
        if obj_id in self.object_positions:
            animated_pos = self.object_positions[obj_id]
            alpha = self.object_alphas.get(obj_id, 1.0)
        else:
            animated_pos = obj
            alpha = 1.0
            
        angle_deg = animated_pos['angle_deg']
        distance_mm = animated_pos['distance_mm']
        
        # No longer forcing distance to be rounded
        
        bbox_area = animated_pos['area']
        confidence = obj.get('confidence', 0)
        last_seen = obj.get('last_seen', 0)
        predicted = obj.get('predicted', False)
        
        # Skip invalid measurements
        if distance_mm <= 0 or distance_mm > MAX_RANGE_MM:
            return
            
        # First apply angle offset and REVERSE the angle for correct orientation
        adjusted_angle = -angle_deg + self.angle_offset
        
        # Convert polar coordinates (distance, angle) to Cartesian (x, y)
        x_mm = distance_mm * math.cos(math.radians(adjusted_angle))
        y_mm = distance_mm * math.sin(math.radians(adjusted_angle))
        
        # Scale distances to screen coordinates
        scale = RADAR_RADIUS / MAX_RANGE_MM
        screen_x = CENTER_X + (x_mm * scale)
        screen_y = CENTER_Y + (y_mm * scale)
        
        # Calculate circle radius based on bbox area
        min_radius = 16  # Increased from 12 to 16 for larger minimum size
        max_radius = 40  # Increased from 32 to 40 for larger maximum size
        area_scale = min(1.0, bbox_area / 100000)
        radius = int(min_radius + (max_radius - min_radius) * area_scale)
        
        # Choose color and apply fade
        if obj_class in BIKE_OBJECTS:
            base_color = BIKE_OBJECTS[obj_class]['color']
        else:
            base_color = COLORS['bike_obj']
            
        # Extract RGB components
        r = (base_color >> 16) & 0xFF
        g = (base_color >> 8) & 0xFF
        b = base_color & 0xFF
        
        # Calculate fade factor
        age = time.time() - last_seen
        if predicted:
            fade_factor = max(MIN_FADE_FACTOR, 1.0 - (age / MAX_PREDICTION_TIME))
        else:
            fade_factor = max(MIN_FADE_FACTOR, 1.0 - (age / OBJECT_PERSISTENCE))
            
        # Apply animation fade but keep minimum visibility
        fade_factor = max(MIN_FADE_FACTOR, fade_factor * alpha)
        
        # Draw the object circle with animation
        if (0 <= screen_x <= SCREEN_WIDTH) and (0 <= screen_y <= SCREEN_HEIGHT):
            # Draw base circle with full opacity
            color = (int(r * fade_factor), int(g * fade_factor), int(b * fade_factor))
            pygame.draw.circle(surface, color, (int(screen_x), int(screen_y)), radius)
            
            # Draw icon if available
            if ENABLE_ICON_CACHING and obj_class in self.object_icons:
                # Make icons 2.0x larger than the circle for better visibility
                icon_size = int(radius * 2.0)  # Increased from 1.5x to 2.0x
                cache_key = (obj_class, icon_size)
                if cache_key not in self.icon_cache:
                    icon = self.object_icons[obj_class]
                    scaled_icon = pygame.transform.scale(icon, (icon_size, icon_size))
                    self.icon_cache[cache_key] = scaled_icon
                scaled_icon = self.icon_cache[cache_key]
                
                # Draw the icon centered on the circle
                icon_x = int(screen_x - icon_size/2)
                icon_y = int(screen_y - icon_size/2)
                surface.blit(scaled_icon, (icon_x, icon_y))
            else:
                # Draw a question mark if no icon
                question_text = "?"
                question_surf = self.font.render(question_text, True, (255, 255, 255))
                question_surf.set_alpha(int(255 * fade_factor))
                question_x = int(screen_x - question_surf.get_width()/2)
                question_y = int(screen_y - question_surf.get_height()/2)
                surface.blit(question_surf, (question_x, question_y))
            
            # Draw text with higher minimum opacity
            text_alpha = int(255 * max(MIN_FADE_FACTOR, fade_factor))
            
            # Format distance and angle info
            distance_m = distance_mm / 1000.0
            distance_text = f"{distance_m:.1f}m"
            angle_text = f"{-angle_deg:.0f}°"
            
            # Combine distance and angle on one line for cleaner display
            info_text = f"{distance_text} | {angle_text}"
            
            # Add text shadow for better readability
            shadow_offset = 2
            
            # Position text to the right of the icon
            text_x = int(screen_x) + radius + 5
            text_y = int(screen_y) - self.font.get_height()//2
            
            # Create text surfaces
            info_surf = self.font.render(info_text, True, (255, 255, 255))
            text_width = info_surf.get_width()
            text_height = info_surf.get_height()
            
            # Create background surface with semi-transparent black background
            bg_padding = 4
            bg_rect = pygame.Rect(
                text_x - bg_padding,
                text_y - bg_padding,
                text_width + bg_padding * 2,
                text_height + bg_padding * 2
            )
            
            # Draw semi-transparent background
            bg_surf = pygame.Surface((bg_rect.width, bg_rect.height), pygame.SRCALPHA)
            bg_surf.fill((0, 0, 0, int(180 * fade_factor)))  # Semi-transparent black
            surface.blit(bg_surf, bg_rect.topleft)
            
            # Draw text on top of background
            info_surf.set_alpha(text_alpha)
            surface.blit(info_surf, (text_x, text_y))

    def draw_frame(self):
        current_time = time.time()
        dt = current_time - self.last_frame_time
        self.last_frame_time = current_time
        
        # Clear surfaces for batch rendering
        if BATCH_RENDERING:
            self.radar_surface.fill((0,0,0,0))
            self.text_surface.fill((0,0,0,0))
        
        # Draw warning screen if IMU warning is active
        if self.imu_warning:
            # Create orange overlay
            warning_surface = pygame.Surface((SCREEN_WIDTH, SCREEN_HEIGHT))
            warning_surface.fill((255, 165, 0))  # Orange color
            warning_surface.set_alpha(180)  # Semi-transparent
            
            # Draw warning text
            warning_text = "WARNING: Abnormal Bicycle Orientation Detected!"
            text_surf = self.large_font.render(warning_text, True, (255, 255, 255))
            text_x = SCREEN_WIDTH//2 - text_surf.get_width()//2
            text_y = SCREEN_HEIGHT//2 - text_surf.get_height()//2
            warning_surface.blit(text_surf, (text_x, text_y))
            
            # Apply warning overlay
            self.screen.blit(warning_surface, (0, 0))
            return  # Skip normal radar display when warning is active
        
        # Clear screen before drawing normal display
        self.screen.fill(self._rgb('background'))
        
        # Performance tracking
        start_process = time.time()
        
        # Update animations
        active_count = len(self.detected_objects)
        interp_count = 0
        fading_count = 0
        
        # Batch update all animations
        for obj in self.detected_objects:
            obj_id = f"{obj.get('class', 'unknown')}_{int(obj.get('angle_deg', 0))}"
            self.update_object_animation(obj_id, obj)
            if obj_id in self.object_positions:
                interp_count += 1
        
        # Cleanup icon cache periodically (every 60 seconds)
        if current_time - self.last_cleanup > 60:
            self.icon_cache.clear()
            self.last_cleanup = current_time
        
        # Draw frame with batching
        if self.show_radar:
            if BATCH_RENDERING:
                # Draw all radar elements to radar surface
                self.draw_cartesian_grid([], self.radar_surface)
                if self.show_lidar:
                    self.draw_lidar_points(self.radar_surface)
                for obj in self.detected_objects:
                    self.draw_object(obj, self.radar_surface)
                
                # Blit radar surface to screen
                self.screen.blit(self.radar_surface, (0,0))
            else:
                # Original drawing code
                self.draw_cartesian_grid([])
                if self.show_lidar:
                    self.draw_lidar_points(self.screen)
                for obj in self.detected_objects:
                    self.draw_object(obj)
        
        # Draw debug info if enabled
        if self.show_debug and DEBUG_PERFORMANCE:
            fps = 1.0 / dt if dt > 0 else 0
            debug_text = f"FPS: {fps:.1f} | Objects: {active_count}"
            debug_surf = self.font.render(debug_text, True, self._rgb('text'))
            self.screen.blit(debug_surf, (10, SCREEN_HEIGHT - 30))

    def draw_cartesian_grid(self, debug_lines, surface):
        """ Draw a Cartesian coordinate grid """
        # Draw main circle for radar bounds (only top half)
        pygame.draw.arc(surface, self._rgb('radar_grid'), 
                       (CENTER_X - RADAR_RADIUS, CENTER_Y - RADAR_RADIUS,
                        RADAR_RADIUS * 2, RADAR_RADIUS * 2),
                       math.pi, 2 * math.pi, 1)  # Draw only top half (π to 2π)
        
        # Draw grid lines every 0.5m (only top half)
        grid_interval_mm = 500  # 0.5m
        num_lines = int(MAX_RANGE_MM / grid_interval_mm)
        
        # Draw concentric circles for distance (only top half)
        for i in range(1, num_lines + 1):
            radius = int((i * grid_interval_mm / MAX_RANGE_MM) * RADAR_RADIUS)
            # Draw as dashed arc (only top half)
            pygame.draw.arc(surface, self._rgb('radar_grid'), 
                          (CENTER_X - radius, CENTER_Y - radius,
                           radius * 2, radius * 2),
                          math.pi, 2 * math.pi, 1)
            
            # Add distance label
            distance_m = i * grid_interval_mm / 1000.0
            if i % 2 == 0:  # Only label every 1m
                distance_text = f"{int(distance_m)}m"
                text_surf = self.font.render(distance_text, True, self._rgb('text'))
                self.screen.blit(text_surf, 
                               (CENTER_X + 25 - text_surf.get_width()//2,
                                CENTER_Y + radius - text_surf.get_height()//2))
        
        # Draw angle markers at 30° intervals (only bottom half)
        for angle in range(-90, 91, 30):  # Every 30 degrees from -90 to +90
            if angle == 0:
                continue  # Skip 0 degrees, already drawn as Y axis
            rad = math.radians(angle + 90)
            # Calculate end point for angle line (ending before text labels)
            end_x = CENTER_X + (RADAR_RADIUS) * math.cos(rad)
            end_y = CENTER_Y + (RADAR_RADIUS) * math.sin(rad)
            
            # Draw line from center to edge at specified angle
            pygame.draw.line(surface, self._rgb('radar_grid'),
                           (CENTER_X, CENTER_Y), (end_x, end_y), 1)
            
            # Add angle label at the edge
            label = f"{angle}°"
            text_surf = self.font.render(label, True, self._rgb('text'))
            
            # Calculate position for text (rotate 90° CCW)
            rad_text = math.radians(angle + 90)  # Shift text position to start from bottom
            label_x = CENTER_X + (RADAR_RADIUS + 25) * math.cos(rad_text) - text_surf.get_width()//2
            label_y = CENTER_Y + (RADAR_RADIUS + 25) * math.sin(rad_text) - text_surf.get_height()//2
            
            # Draw the text
            self.screen.blit(text_surf, (label_x, label_y))
        
        # Draw main axes (only bottom half) - now on top of grid lines
        # X axis endpoints
        x_start = (CENTER_X - RADAR_RADIUS, CENTER_Y)
        x_end = (CENTER_X + RADAR_RADIUS, CENTER_Y)
        pygame.draw.line(surface, self._rgb('radar_line'),
                        x_start, x_end, 2)
        
        # Y axis endpoints
        y_start = (CENTER_X, CENTER_Y)
        y_end = (CENTER_X, CENTER_Y + RADAR_RADIUS)
        pygame.draw.line(surface, self._rgb('radar_line'),
                        y_start, y_end, 2)
        
        # Draw bright green circle at origin
        origin_radius = 20
        pygame.draw.circle(surface, self._rgb('radar_line'),
                         (CENTER_X, CENTER_Y), origin_radius)
        
        # Draw rider icon on top of the circle
        rider_path = os.path.join(os.path.dirname(__file__), "icons", "rider.png")
        if os.path.exists(rider_path):
            try:
                rider_icon = pygame.image.load(rider_path)
                if rider_icon.get_alpha() is None:
                    rider_icon = rider_icon.convert_alpha()
                
                # Scale the icon to fit nicely in the circle
                icon_size = origin_radius * 2.0  # Increased from 1.5 to 2.0
                scaled_icon = pygame.transform.scale(rider_icon, (int(icon_size), int(icon_size)))
                
                # Draw the icon centered on the circle
                icon_x = int(CENTER_X - icon_size/2)
                icon_y = int(CENTER_Y - icon_size/2)
                surface.blit(scaled_icon, (icon_x, icon_y))
            except Exception as e:
                print(f"Error loading rider icon: {e}")

    def draw_lidar_points(self, surface):
        """Batch render all LIDAR points"""
        if not self.lidar_points:
            return
            
        # Create points list for batch drawing
        for angle_deg, dist_mm in self.lidar_points:
            if dist_mm <= 0 or dist_mm > MAX_RANGE_MM or dist_mm < 100:
                continue
                
            adjusted_angle = -angle_deg + self.angle_offset
            x_mm = dist_mm * math.cos(math.radians(adjusted_angle))
            y_mm = dist_mm * math.sin(math.radians(adjusted_angle))
            
            scale = RADAR_RADIUS / MAX_RANGE_MM
            screen_x = CENTER_X + (x_mm * scale)
            screen_y = CENTER_Y + (y_mm * scale)
            
            # Draw individual points since pygame.draw.points() isn't available
            pygame.draw.circle(surface, self._rgb('lidar_pt'), (int(screen_x), int(screen_y)), 1)

    def _rgb(self, key):
        """ Convert a 0xAARRGGBB color into (R, G, B) for Pygame. """
        c = COLORS[key]
        r = (c >> 16) & 0xFF
        g = (c >> 8)  & 0xFF
        b = c & 0xFF
        return (r, g, b)

    def run(self):
        """ Main Pygame loop. """
        print("Starting HUD main loop")
        clock = pygame.time.Clock()
        
        # Pre-allocate event list to avoid creating new lists
        event_list = []
        
        # Track performance metrics
        frame_times = []
        last_fps_print = time.time()
        
        # Create a background surface to compare against for dirty rect detection
        if DIRTY_RECTS:
            self.background = pygame.Surface((SCREEN_WIDTH, SCREEN_HEIGHT))
            self.background.fill(self._rgb('background'))
        
        try:
            while self.running:
                # Faster event processing using get_events instead of event.get()
                event_list.clear()
                pygame.event.get(event_list)
                
                for event in event_list:
                    if event.type == pygame.QUIT:
                        self.running = False
                    elif event.type == pygame.KEYDOWN:
                        if event.key == pygame.K_ESCAPE:
                            self.running = False
                        elif event.key == pygame.K_l:
                            self.show_lidar = not self.show_lidar
                        elif event.key == pygame.K_LEFT:
                            self.angle_offset -= 1
                        elif event.key == pygame.K_RIGHT:
                            self.angle_offset += 1

                # Skip frames for better performance
                self.frame_counter += 1
                if SKIP_FRAMES and self.frame_counter % SKIP_FRAMES != 0:
                    # Still ensure we don't hog CPU
                    pygame.time.delay(5)
                    continue
                
                # Only update the screen at specified intervals
                current_time = time.time()
                if current_time - self.last_screen_update < UPDATE_INTERVAL:
                    # Still process events but don't redraw
                    pygame.time.delay(5)
                    continue
                
                self.last_screen_update = current_time
                
                # Draw and flip
                start_time = time.time()
                
                if DIRTY_RECTS:
                    # Clear dirty rects list
                    self.dirty_rects = []
                    
                    # Store screen state before drawing
                    old_screen = self.screen.copy()
                    
                    # Draw everything
                    self.draw_frame()
                    
                    # Find differences between old and new screens
                    diff = pygame.Surface((SCREEN_WIDTH, SCREEN_HEIGHT))
                    diff.blit(self.screen, (0, 0))
                    diff.blit(old_screen, (0, 0), None, pygame.BLEND_SUB)
                    
                    # Convert to grayscale for easier analysis
                    diff_array = pygame.surfarray.pixels3d(diff)
                    changed = (diff_array[:,:,0] > 10) | (diff_array[:,:,1] > 10) | (diff_array[:,:,2] > 10)
                    del diff_array
                    
                    # Find regions that changed (simplified approach)
                    # Just use entire rows that changed for simplicity
                    changed_rows = []
                    for y in range(0, SCREEN_HEIGHT, 20):  # Check every 20 pixels
                        if any(changed[0:SCREEN_WIDTH, y]):
                            # Mark this row as dirty
                            changed_rows.append(y)
                    
                    # Update only changed rows
                    for y in changed_rows:
                        rect = pygame.Rect(0, y, SCREEN_WIDTH, 20)
                        self.dirty_rects.append(rect)
                        pygame.display.update(rect)
                else:
                    # Traditional full screen update
                    self.draw_frame()
                    pygame.display.flip()
                
                # Track frame times for performance metrics
                frame_time = time.time() - start_time
                frame_times.append(frame_time)
                if len(frame_times) > 30:
                    frame_times.pop(0)  # Keep only last 30 frames
                    
                # Print FPS every 5 seconds
                now = time.time()
                if now - last_fps_print > 5.0:
                    fps = 1.0 / (sum(frame_times) / len(frame_times)) if frame_times else 0
                    print(f"HUD performance: {fps:.1f} FPS, {len(self.detected_objects)} objects")
                    if DIRTY_RECTS:
                        print(f"Using {len(self.dirty_rects)} dirty rectangles")
                    last_fps_print = now
                    
                # Use efficient sleep for throttling
                if FAST_MODE:
                    # Just a minimal delay to prevent 100% CPU usage
                    pygame.time.delay(1)
                else:
                    # Normal frame rate control
                    clock.tick_busy_loop(MAX_FPS)  # More precise than regular tick()
        except KeyboardInterrupt:
            print("Keyboard interrupt received")
        finally:
            # Cleanup
            print("Cleaning up HUD...")
            self.running = False
            self.lidar_subscriber.close()
            self.object_subscriber.close()
            self.imu_subscriber.close()  # Close IMU subscriber
            self.context.term()
            pygame.quit()

def main():
    hud = LidarHUD()
    hud.run()

if __name__ == "__main__":
    main()