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

# Display parameters matching ESP32 code
SCREEN_WIDTH = 480
SCREEN_HEIGHT = 480
SCREEN_CENTER_X = SCREEN_WIDTH // 2
SCREEN_CENTER_Y = SCREEN_HEIGHT // 2
CIRCLE_RADIUS = 240

# Tilt detection and scrolling parameters
TILT_THRESHOLD = 15  # Minimum tilt (G) to start scrolling
SCROLL_SMOOTHING_FACTOR = 0.8  # Low-pass filter for smooth scrolling
MAX_SCROLL_VELOCITY = 50  # Maximum scroll speed (pixels/second)
TILT_SCALE_FACTOR = 200  # Scaling from tilt angle to scroll velocity

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
        self.b, self.a = signal.butter(1, [low, high], btype='band')
        
        # Filter state
        self.zi = signal.lfilter_zi(self.b, self.a)
        
    def apply(self, x):
        y, self.zi = signal.lfilter(self.b, self.a, [x], zi=self.zi)
        return y[0]

class IMUDataProcessor:
    """Processes IMU data from ESP32 and generates scroll commands with proper attitude estimation"""
    
    def __init__(self, sample_rate=100.0):
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
        
        # Bandpass filters for attitude stability detection
        self.roll_filter = BandpassFilter(sample_rate, lowcut=1.0, highcut=10.0)
        self.pitch_filter = BandpassFilter(sample_rate, lowcut=1.0, highcut=10.0)
        
        # Filtered attitude values (for stillness detection)
        self.roll_filtered = 0.0
        self.pitch_filtered = 0.0
        
        # Scroll state with physics-based inertial modeling
        self.scroll_velocity_x = 0.0
        self.scroll_velocity_y = 0.0
        self.scroll_position_x = 0.0
        self.scroll_position_y = 0.0
        
        # Physics parameters
        self.stillness_threshold = 12.5   # degrees (filtered attitude magnitude)
        self.dead_zone_threshold = 15.0  # degrees (no forces below this tilt)
        self.tilt_force_scale = 100.0    # Force scaling factor (increased for better response)
        self.velocity_decay = 0.85       # 1st order LPF decay factor for inertia
        self.max_scroll_velocity = 50   # pixels/second (increased for better response)
        
        # Scroll area limitation (1440x1440 = 3x screen size)
        self.max_scroll_area = 1440      # Maximum scroll area (pixels)
        self.scroll_limit = self.max_scroll_area // 2  # ±720 pixels from center
        
        self.last_update_time = time.time()
        self.button_a_pressed = False
        self.button_b_pressed = False
        
        # Moving average for accelerometer noise reduction
        self.accel_history_size = 3
        self.accel_x_history = deque(maxlen=self.accel_history_size)
        self.accel_y_history = deque(maxlen=self.accel_history_size)
        self.accel_z_history = deque(maxlen=self.accel_history_size)
        
    def process_imu_packet(self, packet_data):
        """Process received IMU data packet from ESP32 with proper attitude estimation"""
        # if len(packet_data) < 18:
        #     return False

        try:
            # Unpack IMU data packet (little-endian)
            protocol_id, timestamp, accel_x_raw, accel_y_raw, accel_z_raw, \
            gyro_roll_raw, gyro_pitch_raw, gyro_yaw_raw, button_state = struct.unpack('<BIhhhhhhB', packet_data)
            
            if protocol_id != 0x01:
                return False
                
            # Step 1: Parse accelerometer and gyroscope data
            # Convert raw values to physical units
            # ESP32 scaling: ±4G range, 16-bit: ±32767, scale = 8191.75
            #xy軸が通常の定義の逆向き
            accel_x = accel_y_raw / 8191.75
            accel_y = accel_x_raw / 8191.75
            accel_z = accel_z_raw / 8191.75
            
            # ESP32 scaling: ±2000deg/s range, 16-bit: ±32767, scale = 16.3835
            gyro_roll = gyro_roll_raw / 16.3835
            gyro_pitch = gyro_pitch_raw / 16.3835
            gyro_yaw = gyro_yaw_raw / 16.3835
            
            # Store raw sensor data
            self.accel_x = accel_x
            self.accel_y = accel_y
            self.accel_z = accel_z
            self.gyro_roll = gyro_roll
            self.gyro_pitch = gyro_pitch
            self.gyro_yaw = gyro_yaw
            
            # Apply moving average filter to reduce noise
            self.accel_x_history.append(accel_x)
            self.accel_y_history.append(accel_y)
            self.accel_z_history.append(accel_z)
            
            # Step 2: Estimate steady-state attitude (roll, pitch) from accelerometer
            # Note: Low-pass filter removed as it's already done on ESP32 side
            if len(self.accel_x_history) >= self.accel_history_size:
                # Use current accelerometer data directly (ESP32 already filtered)
                ax = accel_x
                ay = accel_y
                az = accel_z
                
                # Calculate roll and pitch from accelerometer (assuming gravity dominates)
                # Roll: rotation around X-axis (device forward/back tilt)
                # Pitch: rotation around Y-axis (device left/right tilt)
                self.roll_deg = math.degrees(math.atan2(ay, math.sqrt(ax**2 + az**2)))
                self.pitch_deg = math.degrees(math.atan2(-ax, math.sqrt(ay**2 + az**2)))
                
                # Step 3: Apply bandpass filter to attitude for stillness detection
                self.roll_filtered = self.roll_filter.apply(self.roll_deg)
                self.pitch_filtered = self.pitch_filter.apply(self.pitch_deg)
                
                # Step 4: Update scroll based on attitude and stillness
                self._update_scroll_from_attitude()
            
            # Extract button states
            self.button_a_pressed = bool(button_state & 0x01)
            self.button_b_pressed = bool(button_state & 0x02)
            
            return True
            
        except struct.error as e:
            print(f"IMU packet parsing error: {e}")
            return False
    
    def _update_scroll_from_attitude(self):
        """Update scroll position based on attitude estimation and stillness detection"""
        current_time = time.time()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time
        
        # if dt > 0.1:  # Skip if time delta is too large (first call or long pause)
        #     return
            
        # Step 3: Check if device is in steady state (stillness detection)
        # Use bandpass-filtered attitude magnitude to detect stability
        filtered_attitude_magnitude = math.sqrt(self.roll_filtered**2 + self.pitch_filtered**2)
        is_still = filtered_attitude_magnitude < self.stillness_threshold
        
        # Check if tilt is within dead zone (< 15 degrees total tilt)
        total_tilt_magnitude = math.sqrt(self.roll_deg**2 + self.pitch_deg**2)
        in_dead_zone = total_tilt_magnitude < self.dead_zone_threshold
        
        # Step 4: Physics-based scrolling with inertial modeling
        if is_still and not in_dead_zone:
            # Device is steady and tilted enough - apply tilt forces
            # Convert attitude angles to scroll forces
            # Roll (forward/back tilt) → Y scroll (up/down on screen)
            # Pitch (left/right tilt) → X scroll (left/right on screen)
            
            # Use stronger force calculation for better responsiveness
            force_x = math.sin(math.radians(self.pitch_deg)) * self.tilt_force_scale
            force_y = math.sin(math.radians(self.roll_deg)) * self.tilt_force_scale
            
            # Apply forces to velocity (acceleration = force, assuming unit mass)
            acceleration_x = force_x
            acceleration_y = force_y
            
            # Update velocity with acceleration and reduced decay for steady forces
            self.scroll_velocity_x = self.scroll_velocity_x * self.velocity_decay + acceleration_x * dt
            self.scroll_velocity_y = self.scroll_velocity_y * self.velocity_decay + acceleration_y * dt
            
        else:
            # Device is moving OR within dead zone - apply stronger decay
            stronger_decay = 0.7  # More aggressive decay when not applying forces
            self.scroll_velocity_x *= stronger_decay
            self.scroll_velocity_y *= stronger_decay
        
        # Limit maximum scroll velocity
        velocity_magnitude = math.sqrt(self.scroll_velocity_x**2 + self.scroll_velocity_y**2)
        if velocity_magnitude > self.max_scroll_velocity:
            scale = self.max_scroll_velocity / velocity_magnitude
            self.scroll_velocity_x *= scale
            self.scroll_velocity_y *= scale
        
        # Integrate velocity to get position
        self.scroll_position_x += self.scroll_velocity_x * dt
        self.scroll_position_y += self.scroll_velocity_y * dt
        
        # Apply scroll area limitation (1440x1440 maximum area)
        self.scroll_position_x = max(-self.scroll_limit, min(self.scroll_limit, self.scroll_position_x))
        self.scroll_position_y = max(-self.scroll_limit, min(self.scroll_limit, self.scroll_position_y))
    
    def get_scroll_command(self):
        """Get current scroll command for ESP32"""
        return {
            'scroll_x': int(self.scroll_position_x),
            'scroll_y': int(self.scroll_position_y),
            'rotation': 0  # Not implemented yet
        }
    
    def get_debug_info(self):
        """Get debug information for display"""
        filtered_magnitude = math.sqrt(self.roll_filtered**2 + self.pitch_filtered**2)
        is_still = filtered_magnitude < self.stillness_threshold
        total_tilt_magnitude = math.sqrt(self.roll_deg**2 + self.pitch_deg**2)
        in_dead_zone = total_tilt_magnitude < self.dead_zone_threshold
        
        return {
            'roll_deg': self.roll_deg,
            'pitch_deg': self.pitch_deg,
            'total_tilt_magnitude': total_tilt_magnitude,
            'roll_filtered': self.roll_filtered,
            'pitch_filtered': self.pitch_filtered,
            'filtered_magnitude': filtered_magnitude,
            'is_still': is_still,
            'in_dead_zone': in_dead_zone,
            'scroll_pos_x': self.scroll_position_x,
            'scroll_pos_y': self.scroll_position_y,
            'scroll_vel_x': self.scroll_velocity_x,
            'scroll_vel_y': self.scroll_velocity_y,
            'accel_x': self.accel_x,
            'accel_y': self.accel_y,
            'accel_z': self.accel_z,
            'button_a': self.button_a_pressed,
            'button_b': self.button_b_pressed
        }

class RealisticFishModel:
    """Fish movement model based on real video analysis data"""
    
    def __init__(self, fish_id, initial_position=None, training_data=None):
        self.fish_id = fish_id
        self.position = np.array(initial_position if initial_position else (SCREEN_CENTER_X, SCREEN_CENTER_Y), dtype=float)
        self.velocity = np.array([0.0, 0.0])
        self.acceleration = np.array([0.0, 0.0])
        self.angle = 0.0
        self.curvature = 0.0

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
            
            if distance > CIRCLE_RADIUS:
                # Reflect velocity away from boundary
                normal_x = dx / distance
                normal_y = dy / distance
                
                dot_product = self.velocity[0] * normal_x + self.velocity[1] * normal_y
                self.velocity[0] -= 2 * dot_product * normal_x
                self.velocity[1] -= 2 * dot_product * normal_y
                
                # Place fish just inside boundary
                new_position[0] = SCREEN_CENTER_X + normal_x * (CIRCLE_RADIUS - 5)
                new_position[1] = SCREEN_CENTER_Y + normal_y * (CIRCLE_RADIUS - 5)
        
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
        return (dx * dx + dy * dy) <= (CIRCLE_RADIUS * CIRCLE_RADIUS)

class FishDisplayWithIMU:
    def __init__(self, port, baudrate=115200):
        self.ser = serial.Serial(port, baudrate, timeout=0.01)  # Non-blocking read
        self.fish_models = []
        self.fish_data_cache = None
        self.imu_processor = IMUDataProcessor()
        self.running = True
        
        print(f"Connected to {port} at {baudrate} baud")
        
        # Load training data from fish analysis
        self._load_fish_analysis_data()
        
        # Buffer for incoming data
        self.receive_buffer = bytearray()
        
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
            radius = random.uniform(50, CIRCLE_RADIUS - 50)
            x = SCREEN_CENTER_X + radius * math.cos(angle)
            y = SCREEN_CENTER_Y + radius * math.sin(angle)
            
            fish_model = RealisticFishModel(
                fish_id=i,
                initial_position=(x, y),
                training_data=self.fish_data_cache
            )
            
            self.fish_models.append(fish_model)
            print(f"Fish {i}: initialized at ({x:.0f},{y:.0f})")
    
    def create_fish_packet(self, fish_data):
        """Create protocol ID 0x02 packet for fish data"""
        timestamp = int(time.time() * 1000) & 0xFFFFFFFF
        
        packet = bytearray()
        packet.append(0x02)  # Protocol ID
        packet.extend(struct.pack('<I', timestamp))
        packet.append(len(fish_data))
        
        for fish in fish_data:
            packet.append(fish['sprite_id'])
            packet.extend(b'\x00\x00')  # Reserved bytes
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
    
    def process_incoming_data(self):
        """Process incoming IMU data from ESP32"""
        try:
            # Read available data
            data = self.ser.read(32)  # Read up to 64 bytes
            if data:
                self.receive_buffer.extend(data)
                
                # Process complete packets
                while len(self.receive_buffer) >= 18:  # IMU packet size
                    if self.receive_buffer[0] == 0x01:  # IMU data packet
                        packet_data = bytes(self.receive_buffer[:18])
                        if self.imu_processor.process_imu_packet(packet_data):
                            # Successfully processed, remove from buffer
                            del self.receive_buffer[:19]
                        else:
                            # Invalid packet, shift buffer
                            del self.receive_buffer[0]
                    else:
                        # Unknown packet, shift buffer
                        del self.receive_buffer[0]
                        
        except serial.SerialException as e:
            print(f"Serial error: {e}")
    
    def update_fish_positions(self):
        """Update all fish positions using realistic models"""
        for fish_model in self.fish_models:
            fish_model.update_position()
    
    def get_fish_data(self):
        """Get current fish data in protocol format"""
        fish_data = []
        for i, fish_model in enumerate(self.fish_models):
            fish = {
                'sprite_id': i % 3,  # Vary sprite types
                'x': int(fish_model.position[0]),
                'y': int(fish_model.position[1]),
                'direction': int(fish_model.angle) % 360
            }
            fish_data.append(fish)
        
        return fish_data
    
    def send_fish_data(self):
        """Send current fish positions via serial"""
        fish_data = self.get_fish_data()
        if fish_data:
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
        """Print current system status with attitude information"""
        debug_info = self.imu_processor.get_debug_info()
        still_status = "STILL" if debug_info['is_still'] else "MOVING"
        dead_zone_status = "DEAD_ZONE" if debug_info['in_dead_zone'] else "ACTIVE"
        
        print(f"Attitude: Roll={debug_info['roll_deg']:.1f}° Pitch={debug_info['pitch_deg']:.1f}° "
              f"Total={debug_info['total_tilt_magnitude']:.1f}° | "
              f"Status: {still_status} {dead_zone_status} | "
              f"Scroll: ({debug_info['scroll_pos_x']:.0f},{debug_info['scroll_pos_y']:.0f}) | "
              f"Vel: ({debug_info['scroll_vel_x']:.1f},{debug_info['scroll_vel_y']:.1f}) | "
              f"Filt_Mag={debug_info['filtered_magnitude']:.1f}° | "
              f"Accel: ({debug_info['accel_x']:.2f},{debug_info['accel_y']:.2f},{debug_info['accel_z']:.2f})G")
    
    def run_simulation(self, duration_seconds=300, num_fish=4, update_rate_hz=10):
        """Run the integrated fish display and IMU scroll system"""
        print(f"Starting integrated fish display with IMU scrolling")
        print(f"Duration: {duration_seconds}s, Fish: {num_fish}, Rate: {update_rate_hz}Hz")
        print("Tilt the device to scroll the fish display")
        
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
                # print(elapsed)
                sleep_time = max(0, update_interval - elapsed)
                time.sleep(sleep_time)
                
        except KeyboardInterrupt:
            print("\nSimulation stopped by user")
        
        self.running = False
        print("Simulation completed")
        self.ser.close()

def main():
    if len(sys.argv) != 2:
        print("Usage: python3 fish_sender_with_imu_scroll.py <serial_port>")
        print("Example: python3 fish_sender_with_imu_scroll.py /dev/ttyUSB0")
        print()
        print("Integrated fish display system with IMU-based scrolling")
        print("Receives IMU data from ESP32 and sends fish + scroll commands")
        sys.exit(1)
    
    port = sys.argv[1]
    system = FishDisplayWithIMU(port)
    system.run_simulation(duration_seconds=300, num_fish=2,  update_rate_hz=100)

if __name__ == "__main__":
    main()