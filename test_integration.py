#!/usr/bin/env python3
"""
Quick test script for the integrated fish display with IMU scrolling system
Tests basic communication and protocol handling
"""

import sys
import time

def main():
    if len(sys.argv) != 2:
        print("Usage: python3 test_integration.py <serial_port>")
        print("Example: python3 test_integration.py /dev/ttyUSB0")
        print()
        print("This script tests the integrated system communication")
        sys.exit(1)
    
    port = sys.argv[1]
    
    try:
        # Import the integrated system
        from fish_sender_with_imu_scroll import FishDisplayWithIMU
        
        print("=== Fish Display with IMU Scrolling Test ===")
        print(f"Connecting to {port}...")
        
        # Create system instance
        system = FishDisplayWithIMU(port)
        
        print("System initialized successfully!")
        print("Starting test simulation for 30 seconds...")
        print("Tilt the ESP32 device to test scrolling functionality")
        print("Press Ctrl+C to stop")
        
        # Run a short test simulation
        system.run_simulation(duration_seconds=30, num_fish=3, update_rate_hz=10)
        
        print("Test completed successfully!")
        
    except ImportError as e:
        print(f"Import error: {e}")
        print("Make sure the required Python modules are installed")
        sys.exit(1)
    except Exception as e:
        print(f"Test failed: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()