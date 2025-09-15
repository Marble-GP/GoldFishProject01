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
    def __init__(self, port, baudrate=500000):
        self.ser = serial.Serial(port, baudrate, timeout=0.5)  # Non-blocking read
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
            data = self.ser.read(32)  # Read up to 32 bytes
            if data:
                self.receive_buffer.extend(data)
                
                # Process complete packets
                while len(self.receive_buffer) >= 18:  # IMU packet size
                    if self.receive_buffer[0] == 0x01:  # IMU data packet
                        packet_data = bytes(self.receive_buffer[:18])
                        if self.imu_processor.process_imu_packet(packet_data):
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
        """Print current system status with attitude and scroll input visualization"""
        debug_info = self.imu_processor.get_debug_info()
        
        # Status indicators
        stable_status = "STABLE" if debug_info['is_stable'] else "MOVING"
        zone_status = "ACTIVE" if not debug_info['in_dead_zone'] else "DEAD_ZONE"
        
        # Tilt information
        tilt_info = (f"Tilt: R={debug_info['roll_deg']:+6.1f}° P={debug_info['pitch_deg']:+6.1f}° "
                     f"Total={debug_info['total_tilt']:5.1f}°")
        
        # Scroll input visualization
        scroll_input = (f"Input: {debug_info['scroll_direction']} "
                       f"({debug_info['scroll_input_x']:+.2f}, {debug_info['scroll_input_y']:+.2f}) "
                       f"Mag={debug_info['scroll_input_magnitude']:.2f}")
        
        # Scroll state
        scroll_state = (f"Pos=({debug_info['scroll_pos_x']:+6.0f},{debug_info['scroll_pos_y']:+6.0f}) "
                       f"Vel=({debug_info['scroll_vel_x']:+5.1f},{debug_info['scroll_vel_y']:+5.1f})")
        
        # Timing information
        timing_info = f"dt={debug_info['actual_dt']:.1f}ms"
        
        # Combine all information
        print(f"{tilt_info} | {stable_status:6s} {zone_status:9s} | {scroll_input} | {scroll_state} | {timing_info}")
    
    def run_simulation(self, duration_seconds=300, num_fish=4, update_rate_hz=10):
        """Run the integrated fish display and IMU scroll system"""
        print(f"Starting integrated fish display with IMU scrolling")
        print(f"Duration: {duration_seconds}s, Fish: {num_fish}, Rate: {update_rate_hz}Hz")
        print("Tilt the device to scroll the fish display")
        print("-" * 120)
        
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
    system.run_simulation(duration_seconds=300, num_fish=2, update_rate_hz=100)

if __name__ == "__main__":
    main()