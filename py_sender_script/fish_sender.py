#!/usr/bin/env python3
"""
Fish position sender for ESP32 circular LCD display
Sends random walk data via serial using protocol ID 0x02
"""

import serial
import time
import random
import struct
import sys

# Display parameters matching ESP32 code
SCREEN_WIDTH = 480
SCREEN_HEIGHT = 480
SCREEN_CENTER_X = SCREEN_WIDTH // 2
SCREEN_CENTER_Y = SCREEN_HEIGHT // 2
CIRCLE_RADIUS = 240

class FishSender:
    def __init__(self, port, baudrate=115200):
        self.ser = serial.Serial(port, baudrate, timeout=1)
        self.fish_positions = []
        print(f"Connected to {port} at {baudrate} baud")
        
    def is_point_in_circle(self, x, y):
        """Check if point is within the circular display area"""
        dx = x - SCREEN_CENTER_X
        dy = y - SCREEN_CENTER_Y
        return (dx * dx + dy * dy) <= (CIRCLE_RADIUS * CIRCLE_RADIUS)
    
    def create_protocol_packet(self, fish_data):
        """Create protocol ID 0x02 packet according to specification"""
        # Calculate timestamp in milliseconds
        timestamp = int(time.time() * 1000) & 0xFFFFFFFF  # 32-bit unsigned
        
        # Packet structure:
        # Byte 0: Protocol ID (0x02)
        # Bytes 1-4: Timestamp (4 bytes, little endian)
        # Byte 5: Fish count
        # For each fish (13 bytes):
        #   Byte 0: Sprite ID
        #   Bytes 1-2: Reserved
        #   Bytes 3-6: X position (4 bytes, little endian)
        #   Bytes 7-10: Y position (4 bytes, little endian)  
        #   Bytes 11-12: Direction (2 bytes, little endian)
        
        packet = bytearray()
        packet.append(0x02)  # Protocol ID
        packet.extend(struct.pack('<I', timestamp))  # Timestamp (little endian)
        packet.append(len(fish_data))  # Fish count
        
        for fish in fish_data:
            packet.append(fish['sprite_id'])  # Sprite ID
            packet.extend(b'\x00\x00')  # Reserved bytes
            packet.extend(struct.pack('<i', fish['x']))  # X position
            packet.extend(struct.pack('<i', fish['y']))  # Y position
            packet.extend(struct.pack('<h', fish['direction']))  # Direction
            
        return bytes(packet)
    
    def initialize_fish(self, count=3):
        """Initialize fish with random positions within circle"""
        self.fish_positions = []
        for i in range(count):
            # Generate random position within circle
            while True:
                x = random.randint(50, SCREEN_WIDTH - 50)
                y = random.randint(50, SCREEN_HEIGHT - 50)
                if self.is_point_in_circle(x, y):
                    break
            
            fish = {
                'sprite_id': i % 2,  # Alternate between sprite 0 and 1
                'x': x,
                'y': y,
                'direction': random.randint(0, 359),
                'velocity_x': random.uniform(-2.0, 2.0),
                'velocity_y': random.uniform(-2.0, 2.0)
            }
            self.fish_positions.append(fish)
            print(f"Fish {i}: position=({x},{y}), sprite={fish['sprite_id']}")
    
    def update_fish_positions(self):
        """Update fish positions with random walk movement"""
        for fish in self.fish_positions:
            # Add some randomness to velocity (brownian motion)
            fish['velocity_x'] += random.uniform(-0.5, 0.5)
            fish['velocity_y'] += random.uniform(-0.5, 0.5)
            
            # Limit velocity
            max_vel = 3.0
            fish['velocity_x'] = max(-max_vel, min(max_vel, fish['velocity_x']))
            fish['velocity_y'] = max(-max_vel, min(max_vel, fish['velocity_y']))
            
            # Update position
            new_x = fish['x'] + fish['velocity_x']
            new_y = fish['y'] + fish['velocity_y']
            
            # Bounce off circle edges
            if not self.is_point_in_circle(int(new_x), int(new_y)):
                # Calculate reflection
                dx = new_x - SCREEN_CENTER_X
                dy = new_y - SCREEN_CENTER_Y
                
                # Reflect velocity
                dot_product = fish['velocity_x'] * dx + fish['velocity_y'] * dy
                fish['velocity_x'] -= 2 * dot_product * dx / (dx*dx + dy*dy)
                fish['velocity_y'] -= 2 * dot_product * dy / (dx*dx + dy*dy)
                
                # Keep old position if outside circle
                new_x = fish['x']
                new_y = fish['y']
            
            fish['x'] = int(new_x)
            fish['y'] = int(new_y)
            
            # Update direction based on velocity
            import math
            if fish['velocity_x'] != 0 or fish['velocity_y'] != 0:
                fish['direction'] = int(math.degrees(math.atan2(fish['velocity_y'], fish['velocity_x']))) % 360
    
    def send_fish_data(self):
        """Send current fish positions via serial"""
        packet = self.create_protocol_packet(self.fish_positions)
        self.ser.write(packet)
        print(f"Sent packet: {len(packet)} bytes, {len(self.fish_positions)} fish")
        for i, fish in enumerate(self.fish_positions):
            print(f"  Fish {i}: pos=({fish['x']},{fish['y']}) dir={fish['direction']}°")
    
    def run_simulation(self, duration_seconds=60, num_fish=3, update_rate_hz=10):
        """Run the fish simulation"""
        print(f"Starting fish simulation for {duration_seconds} seconds at {update_rate_hz} Hz")
        
        self.initialize_fish(num_fish)  # Start with 3 fish
        
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
        print("Usage: python3 fish_sender.py <serial_port>")
        print("Example: python3 fish_sender.py /dev/ttyUSB0")
        sys.exit(1)
    
    port = sys.argv[1]
    sender = FishSender(port)
    sender.run_simulation(duration_seconds=30, num_fish=6, update_rate_hz=5)

if __name__ == "__main__":
    main()