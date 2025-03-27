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
MAX_RANGE_MM = 6000  # Maximum range in millimeters

# The radar circle radius is fixed at 400 pixels for the bottom half
RADAR_RADIUS = 400  # Fixed radius for bottom half of display

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
    'train': {'color': 0xFFFF0000, 'priority': 5},
    'truck': {'color': 0xFFFF00FF, 'priority': 4},
    'traffic light': {'color': 0xFFFFFF00, 'priority': 1},
    'fire hydrant': {'color': 0xFFFF0000, 'priority': 1},
    'parking meter': {'color': 0xFF888888, 'priority': 1}
}

# Performance tuning constants
OBJECT_PERSISTENCE = 2.0  # Increased to 2s - only used for stale object cleanup
POLL_TIMEOUT = 10  # Keep at 10ms
MAX_FPS = 60  # Keep at 60 FPS
ZMQ_HWM = 2  # Keep at 2
ANGLE_BUCKET_SIZE = 5.0  # Keep at 5.0
MAX_ANGLE_DIFF = 10.0  # Keep at 10.0

class LidarHUD:
    def __init__(self):
        # Print current environment info
        print("Current environment:")
        print(f"User: {os.getenv('USER')}")
        print(f"DISPLAY: {os.getenv('DISPLAY')}")
        print(f"XDG_RUNTIME_DIR: {os.getenv('XDG_RUNTIME_DIR')}")
        
        # Initialize SDL
        pygame.init()
        
        # If we're in X11, use that, otherwise try direct framebuffer
        if os.getenv('DISPLAY'):
            print("Using X11 mode")
            # Use the default X11 driver
            screen = None
            try:
                pygame.display.init()
                screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT), 
                                               pygame.FULLSCREEN | pygame.NOFRAME)
            except pygame.error as e:
                print(f"X11 fullscreen failed: {str(e)}")
                screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        else:
            # Try different video drivers in order
            drivers = ['fbdev', 'kmsdrm', 'directfb']
            screen = None
            
            for driver in drivers:
                os.environ['SDL_VIDEODRIVER'] = driver
                print(f"\nTrying SDL video driver: {driver}")
                try:
                    pygame.display.quit()  # Reset display
                    pygame.display.init()
                    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT), 
                                                   pygame.FULLSCREEN | pygame.NOFRAME)
                    print(f"Success with driver: {driver}")
                    break
                except pygame.error as e:
                    print(f"Driver {driver} failed: {str(e)}")
                    continue
        
        if screen is None:
            print("All display mode attempts failed")
            sys.exit(1)
        
        self.screen = screen
        pygame.display.set_caption("LiDAR Radar (Front 180°)")
        
        # Hide mouse cursor
        try:
            pygame.mouse.set_visible(False)
        except:
            pass

        self.font = pygame.font.Font(None, 24)
        self.large_font = pygame.font.Font(None, 32) 
        
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
            socket.setsockopt(zmq.RCVHWM, ZMQ_HWM)  # Receive high water mark
            socket.setsockopt(zmq.LINGER, 0)  # Don't wait on close
            socket.setsockopt(zmq.CONFLATE, 1)  # Only keep latest message
            socket.setsockopt_string(zmq.SUBSCRIBE, "")  # Subscribe to all messages

        # Connect sockets
        try:
            self.lidar_subscriber.connect("tcp://localhost:5556")  # LIDAR data
            self.object_subscriber.connect("tcp://localhost:5557")  # Correlated objects
        except zmq.error.ZMQError as e:
            print(f"Failed to connect ZMQ sockets: {e}")
            print("Make sure lidar_zmq_refined is running!")
            sys.exit(1)

        # Initialize data structures with pre-allocated memory
        self.lidar_points = []
        self.lidar_angle_map = {}  # Map angles to distances for faster lookup
        self.detected_objects = []
        self.object_history = {}
        self.history_length = 3  # Increased from 2 to 3 for smoother tracking
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

    def smooth_measurement(self, obj_id, measurement):
        """Apply exponential moving average smoothing to measurements"""
        alpha = 0.3  # Reduced from 0.7 to 0.3 for smoother transitions
        
        if obj_id not in self.object_history:
            self.object_history[obj_id] = {
                'distance': measurement['distance_mm'],
                'angle': measurement['angle_deg'],
                'area': measurement['area'],
                'last_update': time.time()
            }
            return measurement
        
        hist = self.object_history[obj_id]
        hist['distance'] = alpha * measurement['distance_mm'] + (1 - alpha) * hist['distance']
        hist['angle'] = alpha * measurement['angle_deg'] + (1 - alpha) * hist['angle']
        hist['area'] = alpha * measurement['area'] + (1 - alpha) * hist['area']
        hist['last_update'] = time.time()
        
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
        """Continuously receive both LiDAR and object detection data from ZMQ."""
        poller = zmq.Poller()
        poller.register(self.lidar_subscriber, zmq.POLLIN)
        poller.register(self.object_subscriber, zmq.POLLIN)
        
        while self.running:
            try:
                socks = dict(poller.poll(POLL_TIMEOUT))
                current_time = time.time()
                
                # Process LIDAR data
                if self.lidar_subscriber in socks:
                    msg = self.lidar_subscriber.recv_string(zmq.NOBLOCK)
                    if msg.startswith("LIDAR_DATA"):
                        try:
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

                # Process correlated objects
                if self.object_subscriber in socks:
                    msg = self.object_subscriber.recv_string(zmq.NOBLOCK)
                    try:
                        data = json.loads(msg)
                        if data.get("type") == "OBJECTS" and "objects" in data:
                            current_time = time.time()
                            
                            # Process each object in the message
                            for obj_data in data["objects"]:
                                obj = {
                                    'class': obj_data.get('label', 'unknown'),
                                    'confidence': float(obj_data.get('confidence', 0)),
                                    'angle_deg': float(obj_data.get('angle_deg', 0)),
                                    'distance_mm': float(obj_data.get('distance_mm', 0)),
                                    'area': float(obj_data.get('area', 10000)),
                                    'timestamp': current_time,
                                    'last_seen': current_time
                                }
                                
                                # Apply smoothing
                                obj_id = f"{obj['class']}_{int(obj['angle_deg'])}"
                                self.smooth_measurement(obj_id, obj)
                                
                                # Update or add object
                                found = False
                                for i, existing in enumerate(self.detected_objects):
                                    if (existing['class'] == obj['class'] and 
                                        abs(existing['angle_deg'] - obj['angle_deg']) < 5.0):
                                        # Update existing object
                                        self.detected_objects[i] = obj
                                        found = True
                                        break
                                
                                if not found:
                                    self.detected_objects.append(obj)
                            
                            self.last_object_update = current_time
                    except Exception as e:
                        print(f"Error processing objects: {e}")
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
                pass

    def draw_frame(self):
        """ Render the Cartesian grid and LiDAR points """
        self.screen.fill(self._rgb('background'))
        
        # Initialize debug lines
        debug_lines = [
            f"Points: {len(self.lidar_points)}",
            f"Objects: {len(self.detected_objects)}",
        ]
        
        if self.show_radar:
            # Draw the Cartesian grid first
            self.draw_cartesian_grid(debug_lines)
            
            # Draw LiDAR points
            if self.show_lidar:
                for (angle_deg, dist_mm) in self.lidar_points:
                    if dist_mm <= 0 or dist_mm > MAX_RANGE_MM:
                        continue
                    if dist_mm < 100:  # Filter only extremely close points (<10cm)
                        continue
                    
                    adjusted_angle = -angle_deg + self.angle_offset
                    x_mm = dist_mm * math.cos(math.radians(adjusted_angle))
                    y_mm = dist_mm * math.sin(math.radians(adjusted_angle))
                    
                    scale = RADAR_RADIUS / MAX_RANGE_MM
                    screen_x = CENTER_X + (x_mm * scale)
                    screen_y = CENTER_Y + (y_mm * scale)
                    
                    pygame.draw.circle(self.screen, self._rgb('lidar_pt'),
                                     (int(screen_x), int(screen_y)), 2)
            
            # Draw detected objects
            for obj in self.detected_objects:
                self.draw_object(obj)
        
        # Draw semi-transparent background for debug text
        debug_height = len(debug_lines) * 20 + 10
        debug_width = 200
        debug_surface = pygame.Surface((debug_width, debug_height))
        debug_surface.fill((0,0,0))
        debug_surface.set_alpha(128)
        self.screen.blit(debug_surface, (10, SCREEN_HEIGHT - debug_height - 10))
        
        # Draw debug text
        y = SCREEN_HEIGHT - debug_height + 5
        for line in debug_lines:
            text = self.font.render(line, True, self._rgb('text'))
            self.screen.blit(text, (15, y))
            y += 20

    def draw_object(self, obj, surface=None, alpha=255):
        """Draw a detected object using Cartesian coordinates"""
        if surface is None:
            surface = self.screen
            
        # Get object properties
        obj_class = obj.get('class', 'object').lower()
        angle_deg = obj.get('angle_deg', 0)
        distance_mm = obj.get('distance_mm', 0)
        bbox_area = obj.get('area', 10000)
        confidence = obj.get('confidence', 0)
        last_seen = obj.get('last_seen', 0)
        age = time.time() - last_seen
        
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
        min_radius = 5
        max_radius = 20
        area_scale = min(1.0, bbox_area / 100000)
        radius = int(min_radius + (max_radius - min_radius) * area_scale)
        
        # Choose color and fade based on age
        if obj_class in BIKE_OBJECTS:
            base_color = BIKE_OBJECTS[obj_class]['color']
        else:
            base_color = COLORS['bike_obj']
            
        # Extract RGB components
        r = (base_color >> 16) & 0xFF
        g = (base_color >> 8) & 0xFF
        b = base_color & 0xFF
        
        # Fade color based on age
        fade_factor = max(0.3, 1.0 - (age / OBJECT_PERSISTENCE))
        color = (int(r * fade_factor), int(g * fade_factor), int(b * fade_factor))
            
        # Make sure screen coordinates are valid
        if (0 <= screen_x <= SCREEN_WIDTH) and (0 <= screen_y <= SCREEN_HEIGHT):
            # Draw the object circle
            pygame.draw.circle(surface, color, (int(screen_x), int(screen_y)), radius)
            
            # Draw icon or question mark
            if obj_class in self.object_icons:
                icon = self.object_icons[obj_class]
                # Scale icon based on distance (closer = bigger)
                icon_scale = 1.0 - (distance_mm / MAX_RANGE_MM)
                icon_scale = max(0.3, min(1.0, icon_scale))  # Limit scale between 0.3 and 1.0
                
                # Calculate icon size (base size * scale)
                base_size = 32
                icon_size = int(base_size * icon_scale)
                
                # Scale the icon
                scaled_icon = pygame.transform.scale(icon, (icon_size, icon_size))
                
                # Create a surface for the icon with alpha
                icon_surface = pygame.Surface((icon_size, icon_size), pygame.SRCALPHA)
                icon_surface.fill((255, 255, 255, int(255 * fade_factor)))  # White with fade
                icon_surface.blit(scaled_icon, (0, 0), special_flags=pygame.BLEND_RGBA_MULT)
                
                # Draw the icon centered on the circle
                icon_x = int(screen_x - icon_size/2)
                icon_y = int(screen_y - icon_size/2)
                surface.blit(icon_surface, (icon_x, icon_y))
            else:
                # Draw a white circle with black question mark
                question_radius = radius - 2  # Slightly smaller than the colored circle
                pygame.draw.circle(surface, (255, 255, 255), (int(screen_x), int(screen_y)), question_radius)
                
                # Draw question mark
                question_text = "?"
                question_surf = self.font.render(question_text, True, (0, 0, 0))
                question_x = int(screen_x - question_surf.get_width()/2)
                question_y = int(screen_y - question_surf.get_height()/2)
                surface.blit(question_surf, (question_x, question_y))
            
            # Draw object info with fade effect
            conf = confidence * 100
            if distance_mm < 1000:
                distance_text = f"{distance_mm/1000:.3f}m"
            else:
                distance_text = f"{distance_mm/1000:.2f}m"
                
            label_text = f"{distance_text}\n{-angle_deg:.0f}°"
            
            # Split and render each line of text
            lines = label_text.split('\n')
            line_height = self.font.get_height()
            total_height = line_height * len(lines)
            text_y = int(screen_y) - total_height//2  # Center text vertically with circle
            
            # Fade text color too
            text_color = tuple(int(c * fade_factor) for c in self._rgb('text'))
            
            for line in lines:
                text_surf = self.font.render(line, True, text_color)
                text_x = int(screen_x) + 50 - text_surf.get_width()//2  # 50 pixels right of circle
                surface.blit(text_surf, (text_x, text_y))
                text_y += line_height

    def draw_cartesian_grid(self, debug_lines):
        """ Draw a Cartesian coordinate grid """
        # Draw main circle for radar bounds (only top half)
        pygame.draw.arc(self.screen, self._rgb('radar_grid'), 
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
            pygame.draw.arc(self.screen, self._rgb('radar_grid'), 
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
            pygame.draw.line(self.screen, self._rgb('radar_grid'),
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
        pygame.draw.line(self.screen, self._rgb('radar_line'),
                        x_start, x_end, 2)
        
        # Y axis endpoints
        y_start = (CENTER_X, CENTER_Y)
        y_end = (CENTER_X, CENTER_Y + RADAR_RADIUS)
        pygame.draw.line(self.screen, self._rgb('radar_line'),
                        y_start, y_end, 2)
        
        # Draw bright green circle at origin
        origin_radius = 20
        pygame.draw.circle(self.screen, self._rgb('radar_line'),
                         (CENTER_X, CENTER_Y), origin_radius)
        
        # Draw rider icon on top of the circle
        rider_path = os.path.join(os.path.dirname(__file__), "icons", "rider.png")
        if os.path.exists(rider_path):
            try:
                rider_icon = pygame.image.load(rider_path)
                if rider_icon.get_alpha() is None:
                    rider_icon = rider_icon.convert_alpha()
                
                # Scale the icon to fit nicely in the circle
                icon_size = origin_radius * 1.5
                scaled_icon = pygame.transform.scale(rider_icon, (int(icon_size), int(icon_size)))
                
                # Draw the icon centered on the circle
                icon_x = int(CENTER_X - icon_size/2)
                icon_y = int(CENTER_Y - icon_size/2)
                self.screen.blit(scaled_icon, (icon_x, icon_y))
            except Exception as e:
                print(f"Error loading rider icon: {e}")

    def _rgb(self, key):
        """ Convert a 0xAARRGGBB color into (R, G, B) for Pygame. """
        c = COLORS[key]
        r = (c >> 16) & 0xFF
        g = (c >> 8)  & 0xFF
        b = c & 0xFF
        return (r, g, b)

    def run(self):
        """ Main Pygame loop. """
        clock = pygame.time.Clock()
        while self.running:
            for event in pygame.event.get():
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

            self.draw_frame()
            pygame.display.flip()
            clock.tick(MAX_FPS)  # Increased to 120 FPS

        # Cleanup
        self.lidar_subscriber.close()
        self.object_subscriber.close()
        self.context.term()
        pygame.quit()
        print("Exiting Lidar HUD.")

def main():
    hud = LidarHUD()
    hud.run()

if __name__ == "__main__":
    main()