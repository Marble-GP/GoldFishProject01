#!/usr/bin/env python3
"""
Fish Display System with IMU-based Scrolling
Integrates data-driven fish simulation with tilt-based scroll control
Master program that:
1. Receives IMU data from ESP32 slave
2. Detects device tilt and generates scroll commands
3. Sends realistic fish movement data
4. Provides smooth scrolling based on accelerometer input
"""

import serial
import time
import random
import struct
import sys
import os
import pickle
import numpy as np
import math
import threading
from collections import deque
from scipy import signal
import subprocess
import serial.tools.list_ports

# Display parameters matching ESP32 code
SCREEN_WIDTH = 480
SCREEN_HEIGHT = 480
SCREEN_CENTER_X = SCREEN_WIDTH // 2
SCREEN_CENTER_Y = SCREEN_HEIGHT // 2
CIRCLE_RADIUS = 240
INIT_FISH_RADIUS = 240 * 2.5

# Fish catching game parameters
CATCH_TARGET_X = -180  # Target position relative to screen center
CATCH_TARGET_Y = 0
CATCH_RADIUS = 60       # Success detection radius (as specified)
DEFAULT_HEALTH = 5      # Starting health points (3 poi)

class BandpassFilter:
    """1st order bandpass filter for attitude detection"""
    def __init__(self, fs=100.0, lowcut=5.0, highcut=100.0):
        self.fs = fs
        self.lowcut = lowcut
        self.highcut = highcut

        # Design bandpass filter
        nyquist = 0.5 * fs
        low = lowcut / nyquist
        high = highcut / nyquist
        self.b, self.a = signal.butter(1, [low, high], btype='band', fs=fs)

        # Filter state
        self.zi = signal.lfilter_zi(self.b, self.a)

    def apply(self, x):
        y, self.zi = signal.lfilter(self.b, self.a, [x], zi=self.zi)
        return y[0]

class MotionDetector:
    """Detects fish catching motion using highpass-filtered acceleration"""
    def __init__(self, sample_rate=30.0):
        self.sample_rate = sample_rate

        # Highpass filters for each axis
        self.filter_x = BandpassFilter(sample_rate, lowcut=2.0, highcut=20.0)
        self.filter_y = BandpassFilter(sample_rate, lowcut=2.0, highcut=20.0)
        self.filter_z = BandpassFilter(sample_rate, lowcut=2.0, highcut=20.0)

        # Motion detection parameters (adjustable)
        self.min_acceleration_threshold = 0.5   # Minimum motion (G)
        self.max_acceleration_threshold = 1.0   # Maximum motion (G)
        self.motion_duration_threshold = 0.1    # Minimum motion duration (seconds)

        # Motion state tracking
        self.motion_history = deque(maxlen=10)  # Last 10 samples
        self.last_motion_time = 0
        self.motion_active = False

    def update_acceleration(self, accel_x, accel_y, accel_z):
        """Process new acceleration data and detect catching motion"""
        # Apply highpass filtering to remove gravity and low-frequency noise
        filtered_y = self.filter_x.apply(accel_x)
        filtered_x = self.filter_y.apply(accel_y)
        filtered_z = self.filter_z.apply(accel_z)

        # Calculate magnitude of filtered acceleration
        magnitude = math.sqrt(filtered_x**2 + filtered_y**2 + filtered_z**2)

        # Store in history
        current_time = time.time()
        self.motion_history.append({
            'time': current_time,
            'magnitude': magnitude,
            'y_accel': filtered_y  # Y-axis is horizontal motion for catching
        })

        # Check for catching motion detection
        return self._detect_catching_motion(current_time)

    def _detect_catching_motion(self, current_time):
        """Detect if current motion pattern matches fish catching"""
        if len(self.motion_history) < 3:
            return False

        # Look for significant Y-axis motion (horizontal catching motion)
        recent_samples = [s for s in self.motion_history if current_time - s['time'] < self.motion_duration_threshold]

        if len(recent_samples) < 2:
            return False

        # Check for appropriate motion magnitude
        max_magnitude = max(s['magnitude'] for s in recent_samples)
        avg_magnitude = sum(s['magnitude'] for s in recent_samples) / len(recent_samples)

        # Motion must be within thresholds
        magnitude_ok = (self.min_acceleration_threshold <= avg_magnitude <= self.max_acceleration_threshold)

        # Check for primarily horizontal motion (Y-axis)
        y_motion = max(abs(s['y_accel']) for s in recent_samples)
        horizontal_dominant = y_motion > (max_magnitude * 0.4)  # Y-axis should be significant

        # Detect motion start (wasn't moving, now moving)
        was_still = not self.motion_active
        self.motion_active = avg_magnitude > self.min_acceleration_threshold

        # Trigger detection on motion start with appropriate characteristics
        catching_detected = (was_still and self.motion_active and
                           magnitude_ok and horizontal_dominant)

        if catching_detected:
            self.last_motion_time = current_time

        return catching_detected

    def get_debug_info(self):
        """Get debug information for motion detection"""
        if not self.motion_history:
            return {
                'magnitude': 0.0,
                'y_accel': 0.0,
                'motion_active': False,
                'threshold_min': self.min_acceleration_threshold,
                'threshold_max': self.max_acceleration_threshold
            }

        latest = self.motion_history[-1]
        return {
            'magnitude': latest['magnitude'],
            'y_accel': latest['y_accel'],
            'motion_active': self.motion_active,
            'threshold_min': self.min_acceleration_threshold,
            'threshold_max': self.max_acceleration_threshold,
            'history_size': len(self.motion_history)
        }

class IMUDataProcessor:
    """Processes IMU data from ESP32 and generates scroll commands with proper attitude estimation"""
    
    def __init__(self, sample_rate=30.0):
        self.sample_rate = sample_rate
        self.dt = 1.0 / sample_rate
        
        # Raw sensor data
        self.accel_x = 0.0
        self.accel_y = 0.0
        self.accel_z = 0.0
        self.gyro_roll = 0.0
        self.gyro_pitch = 0.0
        self.gyro_yaw = 0.0
        
        # Attitude estimation (roll, pitch in degrees)
        self.roll_deg = 0.0
        self.pitch_deg = 0.0
        self.prev_roll_deg = 0.0
        self.prev_pitch_deg = 0.0
        
        # Scroll state
        self.scroll_velocity_x = 0.0
        self.scroll_velocity_y = 0.0
        self.scroll_position_x = 0.0
        self.scroll_position_y = 0.0
        
        # Scroll input visualization
        self.scroll_input_x = 0.0  # -1.0 to 1.0
        self.scroll_input_y = 0.0  # -1.0 to 1.0
        self.scroll_input_magnitude = 0.0  # 0.0 to 1.0
        
        # Physics parameters (optimized)
        self.dead_zone_threshold = 10.0     # degrees (reduced for better sensitivity)
        self.tilt_force_scale = 5000.0        # Force scaling factor
        self.velocity_decay_base = 0.9     # Base decay factor
        self.max_scroll_velocity = 480     # pixels/second
        
        # Scroll area limitation
        self.max_scroll_area = 1440
        self.scroll_limit = self.max_scroll_area // 2
        
        self.last_update_time = time.time()
        self.is_initialized = False
        self.button_a_pressed = False
        self.button_b_pressed = False
        
        # Stability detection
        self.stability_window = deque(maxlen=5)  # Last 5 samples for stability check
        
    def process_imu_packet(self, packet_data):
        """Process received IMU data packet from ESP32"""
        try:
            # Unpack IMU data packet
            protocol_id, timestamp, accel_x_raw, accel_y_raw, accel_z_raw, \
            gyro_roll_raw, gyro_pitch_raw, gyro_yaw_raw, button_state = struct.unpack('<BIhhhhhhB', packet_data)
            
            if protocol_id != 0x01:
                return False
            
            # Convert raw values to physical units
            # Note: X and Y axes are swapped in the original code
            self.accel_x = accel_y_raw / 8191.75  # Swapped axes
            self.accel_y = accel_x_raw / 8191.75  # Swapped axes
            self.accel_z = accel_z_raw / 8191.75
            
            # Gyroscope data (deg/s)
            self.gyro_roll = gyro_roll_raw / 16.3835
            self.gyro_pitch = gyro_pitch_raw / 16.3835
            self.gyro_yaw = gyro_yaw_raw / 16.3835
            
            # Calculate attitude from accelerometer
            gravity_magnitude = math.sqrt(self.accel_x**2 + self.accel_y**2 + self.accel_z**2)
            
            # Only update attitude if gravity vector is reasonable (0.8g to 1.2g)
            if 0.8 < gravity_magnitude < 1.2:
                # Store previous values for stability detection
                self.prev_roll_deg = self.roll_deg
                self.prev_pitch_deg = self.pitch_deg
                
                # Calculate roll and pitch
                # Roll: rotation around X-axis (left-right tilt)
                # Pitch: rotation around Y-axis (forward-back tilt)
                self.roll_deg = math.degrees(math.atan2(-self.accel_y, math.sqrt(self.accel_x**2 + self.accel_z**2)))
                self.pitch_deg = math.degrees(math.atan2(-self.accel_x, math.sqrt(self.accel_y**2 + self.accel_z**2)))
                
                # Update scroll based on attitude
                self._update_scroll_from_attitude()
            
            # Extract button states
            self.button_a_pressed = bool(button_state & 0x01)
            self.button_b_pressed = bool(button_state & 0x02)
            
            return True
            
        except struct.error as e:
            print(f"IMU packet parsing error: {e}")
            return False
    
    def _update_scroll_from_attitude(self):
        """Update scroll position based on attitude with dt compensation"""
        current_time = time.time()
        dt = current_time - self.last_update_time
        
        # Handle initialization and long pauses
        if not self.is_initialized or dt > 0.1:
            self.last_update_time = current_time
            self.is_initialized = True
            return
        
        self.last_update_time = current_time
        
        # Check stability using angular velocity
        angular_change = math.sqrt((self.roll_deg - self.prev_roll_deg)**2 + 
                                  (self.pitch_deg - self.prev_pitch_deg)**2)
        self.stability_window.append(angular_change)
        
        # Device is stable if recent angular changes are small
        is_stable = len(self.stability_window) >= 3 and max(self.stability_window) < 5.0
        
        # Calculate total tilt magnitude
        total_tilt = math.sqrt(self.roll_deg**2 + self.pitch_deg**2)
        in_dead_zone = total_tilt < self.dead_zone_threshold
        
        # Calculate scroll input (normalized -1 to 1)
        if not in_dead_zone:
            # Normalize tilt to input range
            max_tilt = 45.0  # Maximum expected tilt angle
            self.scroll_input_x = max(-1.0, min(1.0, self.pitch_deg / max_tilt))
            self.scroll_input_y = max(-1.0, min(1.0, -self.roll_deg / max_tilt))
            self.scroll_input_magnitude = min(1.0, total_tilt / max_tilt)
        else:
            self.scroll_input_x = 0.0
            self.scroll_input_y = 0.0
            self.scroll_input_magnitude = 0.0
        
        # Apply forces based on tilt (with dt compensation)
        if is_stable and not in_dead_zone:
            # Calculate tilt-based forces
            tilt_factor = (total_tilt - self.dead_zone_threshold) / 30.0
            tilt_factor = max(0.0, min(1.0, tilt_factor))
            
            # Forces proportional to tilt with dt compensation
            force_x = self.pitch_deg * tilt_factor * self.tilt_force_scale
            force_y = -self.roll_deg * tilt_factor * self.tilt_force_scale
            
            # Adaptive decay based on dt (compensate for variable frame time)
            dt_factor = dt * self.sample_rate  # Normalize dt to expected frame time
            velocity_decay = self.velocity_decay_base ** dt_factor
            
            # Update velocity with dt-compensated physics
            self.scroll_velocity_x = self.scroll_velocity_x * velocity_decay + force_x * dt
            self.scroll_velocity_y = self.scroll_velocity_y * velocity_decay + force_y * dt
        else:
            # Stronger decay when not applying forces (also dt-compensated)
            dt_factor = dt * self.sample_rate
            strong_decay = 0.7 ** dt_factor
            self.scroll_velocity_x *= strong_decay
            self.scroll_velocity_y *= strong_decay
        
        # Limit maximum velocity
        velocity_magnitude = math.sqrt(self.scroll_velocity_x**2 + self.scroll_velocity_y**2)
        if velocity_magnitude > self.max_scroll_velocity:
            scale = self.max_scroll_velocity / velocity_magnitude
            self.scroll_velocity_x *= scale
            self.scroll_velocity_y *= scale
        
        # Update position with dt compensation
        self.scroll_position_x += self.scroll_velocity_x * dt
        self.scroll_position_y += self.scroll_velocity_y * dt
        
        # Apply boundaries
        self.scroll_position_x = max(-self.scroll_limit, min(self.scroll_limit, self.scroll_position_x))
        self.scroll_position_y = max(-self.scroll_limit, min(self.scroll_limit, self.scroll_position_y))
    
    def get_scroll_command(self):
        """Get current scroll command for ESP32"""
        return {
            'scroll_x': int(self.scroll_position_x),
            'scroll_y': int(self.scroll_position_y),
            'rotation': 0
        }
    
    def get_scroll_direction_display(self):
        """Get visual representation of scroll input direction and strength"""
        if self.scroll_input_magnitude < 0.01:
            return "CENTER"
        
        # Calculate angle in degrees
        angle = math.degrees(math.atan2(self.scroll_input_y, self.scroll_input_x))
        if angle < 0:
            angle += 360
        
        # Determine primary direction
        directions = [
            (0, "→"), (45, "↗"), (90, "↑"), (135, "↖"),
            (180, "←"), (225, "↙"), (270, "↓"), (315, "↘")
        ]
        
        # Find closest direction
        min_diff = 360
        direction_symbol = "?"
        for dir_angle, symbol in directions:
            diff = abs(angle - dir_angle)
            if diff > 180:
                diff = 360 - diff
            if diff < min_diff:
                min_diff = diff
                direction_symbol = symbol
        
        # Create strength indicator
        strength_bars = int(self.scroll_input_magnitude * 5)
        strength_display = "█" * strength_bars + "░" * (5 - strength_bars)
        
        return f"{direction_symbol} {strength_display}"
    
    def get_debug_info(self):
        """Get debug information for display"""
        total_tilt = math.sqrt(self.roll_deg**2 + self.pitch_deg**2)
        is_stable = len(self.stability_window) >= 3 and max(self.stability_window) < 5.0
        in_dead_zone = total_tilt < self.dead_zone_threshold
        
        # Calculate actual dt for display
        current_time = time.time()
        actual_dt = current_time - self.last_update_time if self.is_initialized else 0.0
        
        return {
            'roll_deg': self.roll_deg,
            'pitch_deg': self.pitch_deg,
            'total_tilt': total_tilt,
            'is_stable': is_stable,
            'in_dead_zone': in_dead_zone,
            'scroll_pos_x': self.scroll_position_x,
            'scroll_pos_y': self.scroll_position_y,
            'scroll_vel_x': self.scroll_velocity_x,
            'scroll_vel_y': self.scroll_velocity_y,
            'scroll_input_x': self.scroll_input_x,
            'scroll_input_y': self.scroll_input_y,
            'scroll_input_magnitude': self.scroll_input_magnitude,
            'scroll_direction': self.get_scroll_direction_display(),
            'actual_dt': actual_dt * 1000,  # Convert to ms
            'button_a': self.button_a_pressed,
            'button_b': self.button_b_pressed
        }

class RealisticFishModel:
    """Fish movement model based on real video analysis data"""
    
    def __init__(self, fish_id, initial_position=None, training_data=None, movement_radius=720):
        self.fish_id = fish_id
        self.position = np.array(initial_position if initial_position else (SCREEN_CENTER_X, SCREEN_CENTER_Y), dtype=float)
        self.velocity = np.array([0.0, 0.0])
        self.acceleration = np.array([0.0, 0.0])
        self.angle = 0.0
        self.curvature = 0.0

        # Movement boundary (default: scroll limit 720px radius)
        self.movement_radius = movement_radius

        # Time-stretch coefficient for relaxed movement
        self.K = 0.2
        
        # Data-driven physics parameters
        self.base_noise_std = 5.182
        self.base_acceleration_std = 7.690  
        self.base_velocity_decay = 0.85
        self.base_max_velocity = 8.482
        self.base_turning_rate = 22.411
        
        # Apply time-stretch scaling
        self.noise_std = self.base_noise_std * self.K
        self.acceleration_std = self.base_acceleration_std * (self.K ** 2)
        self.velocity_decay = 1.0 - (1.0 - self.base_velocity_decay) * self.K
        self.max_velocity = self.base_max_velocity * self.K
        self.turning_rate = self.base_turning_rate * self.K
        
        # Training data from fish analysis
        self.training_data = training_data
        if training_data:
            self._estimate_parameters_from_data(training_data)
    
    def _estimate_parameters_from_data(self, data):
        """Estimate goldfish parameters using statistical analysis of video data"""
        if len(data) < 3:
            return
        
        positions = [(d['center'][0], d['center'][1]) for d in data if d['center']]
        
        if len(positions) >= 3:
            positions = np.array(positions)
            velocities = np.diff(positions, axis=0)
            velocity_magnitudes = np.linalg.norm(velocities, axis=1)
            accelerations = np.diff(velocities, axis=0)
            acceleration_magnitudes = np.linalg.norm(accelerations, axis=1)
            
            # Extract base parameters then apply time scaling
            base_noise_std = np.std(velocity_magnitudes)
            base_max_velocity = np.mean(velocity_magnitudes)
            base_acceleration_std = np.std(acceleration_magnitudes)
            
            self.noise_std = base_noise_std * self.K
            self.max_velocity = base_max_velocity * self.K  
            self.acceleration_std = base_acceleration_std * (self.K ** 2)
        
        # Ensure reasonable bounds
        self.noise_std = max(0.1, min(20.0, self.noise_std))
        self.max_velocity = max(0.5, min(50.0, self.max_velocity))
        self.acceleration_std = max(0.1, min(30.0, self.acceleration_std))
        self.turning_rate = max(0.5, min(180.0, self.turning_rate))
    
    def update_position(self, dt=1.0):
        """Update fish position using data-driven movement model"""
        # Data-driven acceleration changes
        acceleration_change = np.random.normal(0, self.acceleration_std, 2)
        self.acceleration = self.acceleration * 0.8 + acceleration_change
        
        # Update velocity with acceleration and decay
        self.velocity = self.velocity * self.velocity_decay + self.acceleration * dt
        
        # Limit maximum velocity
        velocity_magnitude = np.linalg.norm(self.velocity)
        if velocity_magnitude > self.max_velocity:
            self.velocity = self.velocity / velocity_magnitude * self.max_velocity
        
        # Add position noise
        position_noise = np.random.normal(0, self.noise_std * 0.5, 2)
        
        # Calculate new position
        new_position = self.position + self.velocity * dt + position_noise
        
        # Handle circular boundary with realistic reflection
        if not self._is_point_in_circle(new_position[0], new_position[1]):
            dx = new_position[0] - SCREEN_CENTER_X
            dy = new_position[1] - SCREEN_CENTER_Y
            distance = np.sqrt(dx*dx + dy*dy)

            if distance > self.movement_radius:
                # Reflect velocity away from boundary
                normal_x = dx / distance
                normal_y = dy / distance

                dot_product = self.velocity[0] * normal_x + self.velocity[1] * normal_y
                self.velocity[0] -= 2 * dot_product * normal_x
                self.velocity[1] -= 2 * dot_product * normal_y

                # Place fish just inside boundary
                new_position[0] = SCREEN_CENTER_X + normal_x * (self.movement_radius - 5)
                new_position[1] = SCREEN_CENTER_Y + normal_y * (self.movement_radius - 5)
        
        self.position = new_position
        
        # Update angle based on velocity direction
        if np.linalg.norm(self.velocity) > 0.1:
            target_angle = math.degrees(math.atan2(self.velocity[1], self.velocity[0]))
            
            angle_diff = target_angle - self.angle
            # Handle angle wraparound
            if angle_diff > 180:
                angle_diff -= 360
            elif angle_diff < -180:
                angle_diff += 360
            
            # Limit turning rate
            max_turn = self.turning_rate
            angle_diff = max(-max_turn, min(max_turn, angle_diff))
            self.angle = (self.angle + angle_diff) % 360
    
    def _is_point_in_circle(self, x, y):
        """Check if point is within circular boundary"""
        dx = x - SCREEN_CENTER_X
        dy = y - SCREEN_CENTER_Y
        return (dx * dx + dy * dy) <= (self.movement_radius * self.movement_radius)

class FishDisplayWithIMU:
    def __init__(self, port, baudrate=500000):
        self.ser = serial.Serial(port, baudrate, timeout=0.5)  # Non-blocking read
        self.fish_models = []
        self.fish_data_cache = None
        self.imu_processor = IMUDataProcessor()
        self.motion_detector = MotionDetector()
        self.running = True

        # Game state
        self.current_health = DEFAULT_HEALTH
        self.game_over = False
        self.fish_caught = 0
        self.total_fish_spawned = 0

        # Startup stability delay
        self.start_time = time.time()
        self.stability_delay = 1.0  # 1 second delay for stability

        print(f"Connected to {port} at {baudrate} baud")
        print(f"Fish catching game initialized - Health: {self.current_health}")

        # Load training data from fish analysis
        self._load_fish_analysis_data()

        # Buffer for incoming data
        self.receive_buffer = bytearray()

        # Initialize sound system
        self._initialize_sound_system()

        # Send initial health value to ESP32
        self._send_initial_health()

    def _send_initial_health(self):
        """Send initial health value to ESP32 for display initialization"""
        try:
            # Wait a bit for serial connection to stabilize
            time.sleep(0.1)
            packet = self.create_game_event_packet(1, self.current_health)  # sound_flag=1 (trial/init)
            self.ser.write(packet)
            print(f"Initial health sent to ESP32: {self.current_health} poi")
        except Exception as e:
            print(f"Failed to send initial health: {e}")

    def _initialize_sound_system(self):
        """Initialize sound system for game events"""
        try:
            # Sound file paths
            self.sound_files = {
                'trial': './sound/trial.mp3',
                'success': './sound/success.mp3',
                'game_over': './sound/game_over.mp3',
                'game_clear': './sound/game_clear.mp3'
            }

            # Verify sound files exist
            missing_files = []
            for event, file_path in self.sound_files.items():
                if not os.path.exists(file_path):
                    missing_files.append(f"{event}: {file_path}")

            if missing_files:
                print("Warning: Sound files not found:")
                for missing in missing_files:
                    print(f"  - {missing}")
            else:
                print("All sound files found")

            # Test audio player availability
            players = ['ffplay', 'mpg123', 'cvlc']
            available_players = []
            for player in players:
                try:
                    subprocess.run([player, '--version'], check=False,
                                 stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                 timeout=2)
                    available_players.append(player)
                except (subprocess.TimeoutExpired, FileNotFoundError):
                    pass

            if available_players:
                print(f"Available audio players: {', '.join(available_players)}")
                print("Sound system initialized successfully")
            else:
                print("Warning: No audio players found. Sound will be disabled.")

        except Exception as e:
            print(f"Warning: Failed to initialize sound system: {e}")
            self.sound_files = {}

    def _play_sound_async(self, event_type):
        """Play sound asynchronously in separate thread"""
        def play_sound():
            try:
                if event_type in self.sound_files:
                    file_path = self.sound_files[event_type]
                    if os.path.exists(file_path):
                        # Try multiple MP3 players for Linux compatibility
                        players = [
                            ['mpg123', '-q', file_path],
                            ['ffplay', '-nodisp', '-autoexit', '-v', 'quiet', file_path],
                            ['cvlc', '--play-and-exit', '--intf', 'dummy', file_path]
                        ]

                        for player_cmd in players:
                            try:
                                result = subprocess.run(player_cmd, check=False,
                                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                               timeout=5)
                                if result.returncode == 0:
                                    break
                            except (subprocess.TimeoutExpired, FileNotFoundError):
                                continue
                        print(f"Played sound: {event_type}")
                    else:
                        print(f"Sound file not found: {file_path}")
            except Exception as e:
                print(f"Error playing sound {event_type}: {e}")

        # Play sound in separate thread to avoid blocking
        sound_thread = threading.Thread(target=play_sound, daemon=True)
        sound_thread.start()

    def _cleanup_sound_system(self):
        """Cleanup sound system resources"""
        try:
            # No specific cleanup needed for subprocess-based audio
            print("Sound system cleanup completed")
        except Exception as e:
            print(f"Error during sound system cleanup: {e}")

    def _load_fish_analysis_data(self):
        """Load fish behavior data from video analysis cache"""
        cache_path = "FishVideoModeling/fish_body_cache.pkl"
        if os.path.exists(cache_path):
            try:
                with open(cache_path, 'rb') as f:
                    cache_data = pickle.load(f)
                    self.fish_data_cache = cache_data['fish_data']
                    print(f"Loaded fish analysis data: {len(self.fish_data_cache)} frames")
            except Exception as e:
                print(f"Warning: Could not load fish analysis data: {e}")
                print("Using default movement parameters")
    
    def initialize_fish(self, count=3):
        """Initialize fish with realistic movement models"""
        self.fish_models = []
        
        for i in range(count):
            # Generate random starting position within circle
            angle = random.uniform(0, 2 * math.pi)
            radius = random.uniform(50, INIT_FISH_RADIUS)
            x = SCREEN_CENTER_X + radius * math.cos(angle)
            y = SCREEN_CENTER_Y + radius * math.sin(angle)
            
            fish_model = RealisticFishModel(
                fish_id=i,
                initial_position=(x, y),
                training_data=self.fish_data_cache
            )

            # Randomly assign fish type: 0=Red (Type A), 1=Black (Type B)
            fish_model.fish_type = random.choice([0, 1])

            self.fish_models.append(fish_model)
            fish_type_name = "Red" if fish_model.fish_type == 0 else "Black"
            print(f"Fish {i}: {fish_type_name} goldfish initialized at ({x:.0f},{y:.0f})")
    
    def create_fish_packet(self, fish_data):
        """Create protocol ID 0x02 packet for fish data"""
        timestamp = int(time.time() * 1000) & 0xFFFFFFFF
        
        packet = bytearray()
        packet.append(0x02)  # Protocol ID
        packet.extend(struct.pack('<I', timestamp))
        packet.append(len(fish_data))
        
        for fish in fish_data:
            packet.append(fish['sprite_id'])
            packet.append(fish.get('fish_type', 0))  # 0=Red (0x01), 1=Black (0x02)
            packet.append(0x00)  # Reserved byte
            packet.extend(struct.pack('<i', fish['x']))
            packet.extend(struct.pack('<i', fish['y']))
            packet.extend(struct.pack('<h', fish['direction']))
            
        return bytes(packet)
    
    def create_scroll_packet(self, scroll_x, scroll_y, rotation=0):
        """Create protocol ID 0x01 packet for scroll commands"""
        timestamp = int(time.time() * 1000) & 0xFFFFFFFF

        packet = bytearray()
        packet.append(0x01)  # Protocol ID
        # packet.extend(b'\x00\x00')  # Padding
        packet.extend(struct.pack('<I', timestamp))
        packet.extend(struct.pack('<i', scroll_x))
        packet.extend(struct.pack('<i', scroll_y))
        packet.extend(struct.pack('<h', rotation))

        return bytes(packet)

    def create_game_event_packet(self, sound_flag, health_value):
        """Create protocol ID 0x03 packet for game events"""
        timestamp = int(time.time() * 1000) & 0xFFFFFFFF

        packet = bytearray()
        packet.append(0x03)  # Protocol ID
        packet.extend(struct.pack('<I', timestamp))
        packet.append(sound_flag)   # 1=trial, 2=success, 3=game_clear, 4=game_over
        packet.append(health_value) # 0-3 health points
        packet.append(0)            # Reserved byte
        
        return bytes(packet)


        
    def process_incoming_data(self):
        """Process incoming IMU data from ESP32"""
        try:
            # Read available data
            data = self.ser.read(32)  # Read up to 32 bytes
            if data:
                self.receive_buffer.extend(data)

                # Process complete packets
                while len(self.receive_buffer) >= 18:  # IMU packet size
                    if self.receive_buffer[0] == 0x01:  # IMU data packet
                        packet_data = bytes(self.receive_buffer[:18])
                        if self.imu_processor.process_imu_packet(packet_data):
                            # Process motion detection for fish catching
                            self._process_motion_detection()
                            # Successfully processed, remove from buffer
                            del self.receive_buffer[:18]
                        else:
                            # Invalid packet, shift buffer
                            del self.receive_buffer[0]
                    else:
                        # Unknown packet, shift buffer
                        del self.receive_buffer[0]

        except serial.SerialException as e:
            print(f"Serial error: {e}")

    def _process_motion_detection(self):
        """Process motion detection for fish catching"""
        if self.game_over:
            return

        # Skip motion detection during stability delay period
        current_time = time.time()
        if current_time - self.start_time < self.stability_delay:
            return

        # Get current acceleration data
        accel_x = self.imu_processor.accel_x
        accel_y = self.imu_processor.accel_y
        accel_z = self.imu_processor.accel_z

        # Update motion detector
        catching_motion_detected = self.motion_detector.update_acceleration(accel_x, accel_y, accel_z)

        if catching_motion_detected:
            self._process_catch_attempt()

    def _process_catch_attempt(self):
        """Process a fish catching attempt"""
        # Get current scroll position to determine absolute target position
        scroll_cmd = self.imu_processor.get_scroll_command()
        target_world_x = SCREEN_CENTER_X + CATCH_TARGET_X + scroll_cmd['scroll_x']
        target_world_y = SCREEN_CENTER_Y + CATCH_TARGET_Y + scroll_cmd['scroll_y']

        # Check if any fish are within catching radius
        caught_fish = []
        min_distance = np.inf
        min_index = -1
        for i, fish_model in enumerate(self.fish_models):
            fish_x, fish_y = fish_model.position
            distance = math.sqrt((fish_x - target_world_x)**2 + (fish_y - target_world_y)**2)
            
            if min_distance > distance:
                min_distance = distance
                min_index = i
            if distance <= CATCH_RADIUS:
                caught_fish.append(i)

        print(f"nearlest fish:{self.fish_models[min_index].position} scroll pos:{scroll_cmd['scroll_x'], scroll_cmd['scroll_y']}    dinstance:{min_distance}")

        if caught_fish:
            # Success! Remove caught fish
            for fish_index in reversed(caught_fish):  # Remove in reverse order to maintain indices
                self.fish_models.pop(fish_index)
                self.fish_caught += 1

            print(f"Fish caught! Total: {self.fish_caught}")

            # Play success sound
            self._play_sound_async('success')

            # Send success event (health unchanged)
            self._send_game_event(2, self.current_health)  # sound_flag=2 (success)

            # Check for game clear (all fish caught)
            if len(self.fish_models) == 0:
                print("Game Clear! All fish caught!")
                # Play game clear sound
                self._play_sound_async('game_clear')
                self._send_game_event(3, self.current_health)  # sound_flag=3 (game_clear)
                self.game_over = True

        else:
            # Miss! Reduce health
            self.current_health -= (1 if self.current_health > 0 else 0)
            print(f"Missed! Health: {self.current_health}/3 poi remaining")

            # Play trial sound (attempt made)
            self._play_sound_async('trial')

            # Send health update to ESP32
            self._send_game_event(1, self.current_health)  # sound_flag=1 (trial)

            if self.current_health <= 0:
                # Game Over
                print("Game Over! All poi used.")
                # Play game over sound
                self._play_sound_async('game_over')
                self._send_game_event(4, 0)  # sound_flag=4 (game_over), health=0
                self.game_over = True

    def _send_game_event(self, sound_flag, health_value):
        """Send game event to ESP32"""
        try:
            packet = self.create_game_event_packet(sound_flag, health_value)
            self.ser.write(packet)
            print(f"Game event sent: sound={sound_flag}, health={health_value}")
        except Exception as e:
            print(f"Failed to send game event: {e}")
    
    def update_fish_positions(self):
        """Update all fish positions using realistic models"""
        for fish_model in self.fish_models:
            fish_model.update_position()
    
    def get_fish_data(self):
        """Get current fish data in protocol format"""
        fish_data = []
        for i, fish_model in enumerate(self.fish_models):
            fish = {
                'sprite_id': i,
                'fish_type': fish_model.fish_type,  # 0=Red, 1=Black
                'x': int(fish_model.position[0]),
                'y': int(fish_model.position[1]),
                'direction': int(fish_model.angle) % 360
            }
            fish_data.append(fish)
        
        return fish_data
    
    def send_fish_data(self):
        """Send current fish positions via serial"""
        fish_data = self.get_fish_data()
        # if fish_data:
        packet = self.create_fish_packet(fish_data)
        self.ser.write(packet)
    
    def send_scroll_command(self):
        """Send scroll command based on current IMU tilt"""
        scroll_cmd = self.imu_processor.get_scroll_command()
        packet = self.create_scroll_packet(
            scroll_cmd['scroll_x'],
            scroll_cmd['scroll_y'],
            scroll_cmd['rotation']
        )
        self.ser.write(packet)
    
    def print_status(self):
        """Print current system status with attitude, motion detection, and game state"""
        debug_info = self.imu_processor.get_debug_info()
        motion_info = self.motion_detector.get_debug_info()

        # Status indicators
        stable_status = "STABLE" if debug_info['is_stable'] else "MOVING"
        zone_status = "ACTIVE" if not debug_info['in_dead_zone'] else "DEAD_ZONE"
        motion_status = "MOTION" if motion_info['motion_active'] else "STILL"

        # Tilt information
        tilt_info = (f"Tilt: R={debug_info['roll_deg']:+6.1f}° P={debug_info['pitch_deg']:+6.1f}° "
                     f"Total={debug_info['total_tilt']:5.1f}°")

        # Motion detection information
        motion_detect = (f"Motion: Mag={motion_info['magnitude']:.2f}G Y={motion_info['y_accel']:+.2f}G "
                        f"Thresh=[{motion_info['threshold_min']:.1f}-{motion_info['threshold_max']:.1f}]")

        # Game state information
        game_status = "GAME_OVER" if self.game_over else "PLAYING"
        elapsed_time = time.time() - self.start_time
        stability_status = "STABILIZING" if elapsed_time < self.stability_delay else "ACTIVE"
        game_info = (f"Game: {game_status} Health={self.current_health}/3 "
                    f"Caught={self.fish_caught} Fish={len(self.fish_models)} [{stability_status}]")

        # Scroll state (abbreviated for space)
        scroll_state = (f"Scroll: ({debug_info['scroll_pos_x']:+4.0f},{debug_info['scroll_pos_y']:+4.0f}) "
                       f"Vel=({debug_info['scroll_vel_x']:+4.1f},{debug_info['scroll_vel_y']:+4.1f})")

        # Combine all information
        print(f"{tilt_info} | {stable_status:6s} {zone_status:9s} {motion_status:6s} | {motion_detect} | {game_info} | {scroll_state}")
    
    def run_simulation(self, duration_seconds=300, num_fish=4, update_rate_hz=10):
        """Run the integrated fish display with IMU scrolling and fish catching game"""
        print(f"Starting Fish Catching Game with IMU scrolling")
        print(f"Duration: {duration_seconds}s, Fish: {num_fish}, Rate: {update_rate_hz}Hz")
        print(f"Health: {self.current_health} hearts")
        print("")
        print("Game Instructions:")
        print("1. Tilt device to scroll and find fish")
        print(f"2. Position catch target at ({CATCH_TARGET_X}, {CATCH_TARGET_Y}) relative to center")
        print("3. Make horizontal catching motion when fish is in target area")
        print("4. Catch all fish to win, or lose health by missing")
        print("-" * 140)
        
        self.initialize_fish(num_fish)
        
        start_time = time.time()
        update_interval = 1.0 / update_rate_hz
        last_status_time = 0
        
        try:
            while time.time() - start_time < duration_seconds and self.running:
                loop_start_time = time.time()
                
                # Process incoming IMU data
                self.process_incoming_data()
                
                # Update fish positions
                self.update_fish_positions()
                
                # Send fish data
                self.send_fish_data()
                
                # Send scroll command
                self.send_scroll_command()
                
                # Print status every 2 seconds
                if time.time() - last_status_time > 2.0:
                    self.print_status()
                    last_status_time = time.time()
                
                # Maintain update rate
                elapsed = time.time() - loop_start_time
                sleep_time = max(0, update_interval - elapsed)
                time.sleep(sleep_time)
                
        except KeyboardInterrupt:
            print("\nSimulation stopped by user")

        self.running = False
        print("Simulation completed")

        # Cleanup sound system
        self._cleanup_sound_system()

        self.ser.close()

def find_usb_serial_ports():
    """実際に接続されているUSBシリアルデバイスを検出"""
    ports = serial.tools.list_ports.comports()
    usb_ports = []
    
    for port in ports:
        # USBデバイスのみをフィルタ（hwidにUSBまたはVID:PIDが含まれる）
        if 'USB' in port.hwid or 'VID:PID' in port.hwid or 'ACM' in port.device:
            # さらに接続テストを行う
            if test_port_connection(port.device):
                usb_ports.append({
                    'device': port.device,
                    'description': port.description,
                    'hwid': port.hwid,
                    'vid': port.vid,
                    'pid': port.pid
                })
    
    return usb_ports

def test_port_connection(port_path, timeout=0.1):
    """ポートに実際に接続できるかテスト"""
    try:
        # 接続テストのみ（即座にクローズ）
        ser = serial.Serial(port_path, 115200, timeout=timeout)
        ser.close()
        return True
    except (serial.SerialException, PermissionError, OSError):
        return False

def wait_for_usb_serial_port(retry_interval=3, timeout=None):
    """
    USBシリアルポートが見つかるまで待機（ループ）
    
    Args:
        retry_interval: 再検索までの待機秒数
        timeout: タイムアウト秒数（Noneの場合は無制限）
    
    Returns:
        検出されたポートのデバイスパス、または None
    """
    start_time = time.time()
    attempt = 0
    
    print("USBシリアルポートを検索中...")
    print("ESP32などのデバイスを接続してください (Ctrl+C で中断)")
    print("-" * 60)
    
    while True:
        attempt += 1
        ports = find_usb_serial_ports()
        
        if ports:
            # ttyACMポートを優先
            ttyacm_ports = [p for p in ports if 'ttyACM' in p['device']]
            ttyusb_ports = [p for p in ports if 'ttyUSB' in p['device']]
            
            # 優先順位: ttyACM > ttyUSB > その他
            if ttyacm_ports:
                selected = ttyacm_ports[0]
            elif ttyusb_ports:
                selected = ttyusb_ports[0]
            else:
                selected = ports[0]
            
            print(f"\n✓ USBデバイス検出: {selected['device']}")
            print(f"  説明: {selected['description']}")
            if selected['vid'] and selected['pid']:
                print(f"  VID:PID = {selected['vid']:04X}:{selected['pid']:04X}")
            print()
            
            return selected['device']
        
        # タイムアウトチェック
        if timeout and (time.time() - start_time) > timeout:
            print(f"\nタイムアウト: {timeout}秒経過しました")
            return None
        
        # 待機メッセージ
        elapsed = int(time.time() - start_time)
        print(f"[試行 {attempt}] USBデバイスなし ({elapsed}秒経過) - {retry_interval}秒後に再検索...", end='\r')
        
        try:
            time.sleep(retry_interval)
        except KeyboardInterrupt:
            print("\n\n検索を中断しました")
            return None
        
def main():

    while True:
        try:

            if len(sys.argv) != 2:
                print("Manual Usage: python3 fish_sender_with_imu_scroll.py <serial_port>")
                print("Example: python3 fish_sender_with_imu_scroll.py /dev/ttyUSB0")
                print()

                port = wait_for_usb_serial_port()

                print("Auto Selected:", port)

            else:
                port = sys.argv[1]

            system = FishDisplayWithIMU(port)
            system.run_simulation(duration_seconds=3600, num_fish=4, update_rate_hz=100)
        
        except Exception as e:
            print(e)
            print("reloading the system...")

        except KeyboardInterrupt:
            exit(0)

        


if __name__ == "__main__":
    main()