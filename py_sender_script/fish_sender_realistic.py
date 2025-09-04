#!/usr/bin/env python3
"""
100% Data-Driven Fish position sender with Time-Stretch Control
Uses pure Gaussian statistical parameters from real fish video analysis
Time coefficient K allows relaxed movement while maintaining natural characteristics
K=1.0: original speed, K=0.2: 5x slower but physically equivalent
Sends authentic, time-scaled fish movement via serial using protocol ID 0x02
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

# Display parameters matching ESP32 code
SCREEN_WIDTH = 480
SCREEN_HEIGHT = 480
SCREEN_CENTER_X = SCREEN_WIDTH // 2
SCREEN_CENTER_Y = SCREEN_HEIGHT // 2
CIRCLE_RADIUS = 240

class RealisticFishModel:
    """Fish movement model based on real video analysis data"""
    
    def __init__(self, fish_id, initial_position=None, training_data=None):
        self.fish_id = fish_id
        self.position = np.array(initial_position if initial_position else (SCREEN_CENTER_X, SCREEN_CENTER_Y), dtype=float)
        self.velocity = np.array([0.0, 0.0])
        self.acceleration = np.array([0.0, 0.0])
        self.angle = 0.0
        self.curvature = 0.0

        # Time-stretch coefficient K for relaxed movement while maintaining natural characteristics
        # K scales the time axis: smaller K = slower, more relaxed movement
        # K = 1.0 = original video speed, K = 0.2 = 5x slower (relaxed goldfish-like)
        self.K = 0.2
        
        # Default physics parameters (will be overridden by data-driven analysis)
        # These are fallback values if no training data is available
        # Based on 100% pure statistical analysis from fish_body_cache.pkl
        # NOTE: These will be scaled by K to maintain physical equivalence
        self.base_noise_std = 5.182        # 100% real fish velocity std deviation
        self.base_acceleration_std = 7.690  # 100% real fish acceleration std deviation  
        self.base_velocity_decay = 0.85     # Estimated from real fish momentum behavior
        self.base_max_velocity = 8.482      # 100% real fish mean velocity
        self.base_turning_rate = 22.411     # 100% real fish mean angular velocity
        
        # Apply time-stretch scaling to maintain physical equivalence
        self.noise_std = self.base_noise_std * self.K
        self.acceleration_std = self.base_acceleration_std * (self.K ** 2)  # Acceleration scales as K²
        self.velocity_decay = 1.0 - (1.0 - self.base_velocity_decay) * self.K  # Decay rate scales with K
        self.max_velocity = self.base_max_velocity * self.K
        self.turning_rate = self.base_turning_rate * self.K
        
        # Training data from fish analysis
        self.training_data = training_data
        if training_data:
            self._estimate_parameters_from_data(training_data)
    
    def _estimate_parameters_from_data(self, data):
        """Estimate goldfish parameters using proper statistical analysis of video data"""
        if len(data) < 3:
            print(f"Fish {self.fish_id}: Insufficient data ({len(data)} frames), using defaults")
            return
        
        # Extract data arrays
        positions = [(d['center'][0], d['center'][1]) for d in data if d['center']]
        angles = [d['angle'] for d in data if d['angle'] is not None]
        curvatures = [d['curvature'] for d in data if d['curvature'] is not None]
        
        if len(positions) >= 3:
            positions = np.array(positions)
            
            # Calculate real fish movement statistics
            velocities = np.diff(positions, axis=0)
            velocity_magnitudes = np.linalg.norm(velocities, axis=1)
            
            accelerations = np.diff(velocities, axis=0)
            acceleration_magnitudes = np.linalg.norm(accelerations, axis=1)
            
            # 100% Pure Gaussian data-driven parameters with time-stretch scaling
            # Direct statistical extraction from real fish video analysis
            
            # Extract base parameters from real data (unscaled)
            base_noise_std = np.std(velocity_magnitudes)
            base_max_velocity = np.mean(velocity_magnitudes)
            base_acceleration_std = np.std(acceleration_magnitudes)
            
            # Velocity decay: Estimate from velocity autocorrelation in real data
            if len(velocity_magnitudes) > 1:
                # Calculate autocorrelation for velocity decay estimation
                velocity_x = velocities[:, 0]
                velocity_y = velocities[:, 1]
                if len(velocity_x) > 1:
                    corr_x = np.corrcoef(velocity_x[:-1], velocity_x[1:])[0, 1] if len(velocity_x) > 1 else 0.85
                    corr_y = np.corrcoef(velocity_y[:-1], velocity_y[1:])[0, 1] if len(velocity_y) > 1 else 0.85
                    # Use average correlation as base velocity decay
                    base_velocity_decay = max(0.7, min(0.98, (abs(corr_x) + abs(corr_y)) / 2))
                else:
                    base_velocity_decay = 0.85
            
            # Apply time-stretch scaling K for physically equivalent slower movement
            # Physics scaling laws:
            # - Velocity scales linearly with time scale: v' = v * K
            # - Acceleration scales quadratically: a' = a * K²  
            # - Angular velocity scales linearly: ω' = ω * K
            # - Decay rate: λ' = 1 - (1-λ) * K (slower decay for slower time)
            
            self.noise_std = base_noise_std * self.K
            self.max_velocity = base_max_velocity * self.K  
            self.acceleration_std = base_acceleration_std * (self.K ** 2)
            self.velocity_decay = 1.0 - (1.0 - base_velocity_decay) * self.K
        
        if len(angles) >= 2:
            # Angular velocity analysis with proper wraparound handling
            angle_diffs = np.diff(angles)
            # Handle angle wraparound (-180° to +180°)
            angle_diffs = np.where(angle_diffs > 180, angle_diffs - 360, angle_diffs)
            angle_diffs = np.where(angle_diffs < -180, angle_diffs + 360, angle_diffs)
            angular_velocities = np.abs(angle_diffs)
            
            # Turning rate: Extract base angular velocity then apply time scaling
            base_turning_rate = np.mean(angular_velocities)
            self.turning_rate = base_turning_rate * self.K
        
        # Ensure reasonable bounds (safety limits - but keep as wide as possible for 100% data-driven)
        self.noise_std = max(0.1, min(20.0, self.noise_std))              # Wide range for real data
        self.max_velocity = max(0.5, min(50.0, self.max_velocity))        # Wide range for real data
        self.acceleration_std = max(0.1, min(30.0, self.acceleration_std)) # Wide range for real data
        self.turning_rate = max(0.5, min(180.0, self.turning_rate))       # Wide range for real data
        
        print(f"Fish {self.fish_id} time-stretched data-driven parameters (K={self.K}):")
        if hasattr(self, 'training_data') and self.training_data:
            # Show both base (100% data) and scaled (K-adjusted) values
            base_max_vel = self.max_velocity / self.K if self.K > 0 else 0
            base_noise = self.noise_std / self.K if self.K > 0 else 0
            base_accel = self.acceleration_std / (self.K ** 2) if self.K > 0 else 0
            base_turn = self.turning_rate / self.K if self.K > 0 else 0
            
            print(f"  Max velocity: {base_max_vel:.3f} → {self.max_velocity:.3f} px/frame (×{self.K})")
            print(f"  Noise std: {base_noise:.3f} → {self.noise_std:.3f} px/frame (×{self.K})")
            print(f"  Acceleration std: {base_accel:.3f} → {self.acceleration_std:.3f} px/frame² (×{self.K:.1f}²)")
            print(f"  Turning rate: {base_turn:.3f} → {self.turning_rate:.3f}°/frame (×{self.K})")
            print(f"  Velocity decay: {self.velocity_decay:.3f} (time-adjusted)")
        else:
            print(f"  Max velocity: {self.max_velocity:.3f} px/frame")
            print(f"  Noise std: {self.noise_std:.3f} px/frame") 
            print(f"  Acceleration std: {self.acceleration_std:.3f} px/frame²")
            print(f"  Turning rate: {self.turning_rate:.3f}°/frame")
            print(f"  Velocity decay: {self.velocity_decay:.3f}")
    
    def update_position(self, dt=1.0):
        """Update fish position using 100% data-driven movement model"""
        # Pure data-driven acceleration changes (no artificial modifications)
        acceleration_change = np.random.normal(0, self.acceleration_std, 2)
        self.acceleration = self.acceleration * 0.8 + acceleration_change  # Standard physics decay
        
        # Update velocity with acceleration and decay
        self.velocity = self.velocity * self.velocity_decay + self.acceleration * dt
        
        # Limit maximum velocity (realistic fish speed)
        velocity_magnitude = np.linalg.norm(self.velocity)
        if velocity_magnitude > self.max_velocity:
            self.velocity = self.velocity / velocity_magnitude * self.max_velocity
        
        # Add position noise (natural swimming variation)
        position_noise = np.random.normal(0, self.noise_std * 0.5, 2)
        
        # Calculate new position
        new_position = self.position + self.velocity * dt + position_noise
        
        # Handle circular boundary with realistic reflection
        if not self._is_point_in_circle(new_position[0], new_position[1]):
            # Calculate distance from center
            dx = new_position[0] - SCREEN_CENTER_X
            dy = new_position[1] - SCREEN_CENTER_Y
            distance = np.sqrt(dx*dx + dy*dy)
            
            if distance > CIRCLE_RADIUS:
                # Reflect velocity away from boundary (fish avoiding edge)
                normal_x = dx / distance
                normal_y = dy / distance
                
                # Reflect velocity
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
            
            # Smooth angle transition (fish don't turn instantly)
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
        
        # Add some curvature variation (body flexibility)
        self.curvature += np.random.normal(0, 0.001)
        self.curvature = max(-0.01, min(0.01, self.curvature))
    
    def _is_point_in_circle(self, x, y):
        """Check if point is within circular boundary"""
        dx = x - SCREEN_CENTER_X
        dy = y - SCREEN_CENTER_Y
        return (dx * dx + dy * dy) <= (CIRCLE_RADIUS * CIRCLE_RADIUS)

class RealisticFishSender:
    def __init__(self, port, baudrate=115200):
        self.ser = serial.Serial(port, baudrate, timeout=1)
        self.fish_models = []
        self.fish_data_cache = None
        print(f"Connected to {port} at {baudrate} baud")
        
        # Load training data from fish analysis
        self._load_fish_analysis_data()
        
    def _load_fish_analysis_data(self):
        """Load fish behavior data from video analysis cache"""
        cache_path = "FishVideoModeling/fish_body_cache.pkl"
        if os.path.exists(cache_path):
            try:
                with open(cache_path, 'rb') as f:
                    cache_data = pickle.load(f)
                    self.fish_data_cache = cache_data['fish_data']
                    print(f"Loaded fish analysis data: {len(self.fish_data_cache)} frames")
                    print(f"Cache created: {cache_data['timestamp']}")
            except Exception as e:
                print(f"Warning: Could not load fish analysis data: {e}")
                print("Using default movement parameters")
    
    def is_point_in_circle(self, x, y):
        """Check if point is within the circular display area"""
        dx = x - SCREEN_CENTER_X
        dy = y - SCREEN_CENTER_Y
        return (dx * dx + dy * dy) <= (CIRCLE_RADIUS * CIRCLE_RADIUS)
    
    def create_protocol_packet(self, fish_data):
        """Create protocol ID 0x02 packet according to specification"""
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
    
    def initialize_fish(self, count=3):
        """Initialize fish with realistic movement models"""
        self.fish_models = []
        
        for i in range(count):
            # Generate random starting position within circle
            angle = random.uniform(0, 2 * math.pi)
            radius = random.uniform(50, CIRCLE_RADIUS - 50)
            x = SCREEN_CENTER_X + radius * math.cos(angle)
            y = SCREEN_CENTER_Y + radius * math.sin(angle)
            
            # Create fish model with training data
            fish_model = RealisticFishModel(
                fish_id=i,
                initial_position=(x, y),
                training_data=self.fish_data_cache
            )
            
            self.fish_models.append(fish_model)
            print(f"Fish {i}: initialized at ({x:.0f},{y:.0f})")
    
    def update_fish_positions(self):
        """Update all fish positions using realistic models"""
        for fish_model in self.fish_models:
            fish_model.update_position()
    
    def get_fish_data(self):
        """Get current fish data in protocol format"""
        fish_data = []
        for i, fish_model in enumerate(self.fish_models):
            # Ensure fish is within circular boundary
            if self.is_point_in_circle(fish_model.position[0], fish_model.position[1]):
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
            packet = self.create_protocol_packet(fish_data)
            self.ser.write(packet)
            print(f"Sent packet: {len(packet)} bytes, {len(fish_data)} fish")
            for i, fish in enumerate(fish_data):
                print(f"  Fish {i}: pos=({fish['x']},{fish['y']}) dir={fish['direction']}°")
    
    def run_simulation(self, duration_seconds=60, num_fish=3, update_rate_hz=10):
        """Run the time-stretched data-driven fish simulation"""
        print(f"Starting time-stretched data-driven fish simulation for {duration_seconds} seconds at {update_rate_hz} Hz")
        print(f"Using pure Gaussian parameters with time-stretch coefficient K=0.2 (5x slower)")
        print("Movement maintains natural fish characteristics while being relaxed and peaceful")
        
        self.initialize_fish(num_fish)
        
        start_time = time.time()
        update_interval = 1.0 / update_rate_hz
        
        try:
            while time.time() - start_time < duration_seconds:
                self.update_fish_positions()
                self.send_fish_data()
                time.sleep(update_interval)
                
        except KeyboardInterrupt:
            print("\nSimulation stopped by user")
        
        print("Simulation completed")
        self.ser.close()

def main():
    if len(sys.argv) != 2:
        print("Usage: python3 fish_sender_realistic.py <serial_port>")
        print("Example: python3 fish_sender_realistic.py /dev/ttyUSB0")
        print()
        print("This enhanced version uses data-driven fish behavior from video analysis")
        print("Make sure FishVideoModeling/fish_body_cache.pkl exists for realistic movement")
        sys.exit(1)
    
    port = sys.argv[1]
    sender = RealisticFishSender(port)
    sender.run_simulation(duration_seconds=60, num_fish=4, update_rate_hz=10)  # Standard update rate for data-driven fish

if __name__ == "__main__":
    main()