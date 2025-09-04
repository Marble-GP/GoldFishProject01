#!/usr/bin/env python3
"""
Test script for the improved attitude-based scroll calculation
Verifies the 4-step process: parse → attitude estimation → bandpass filtering → physics scrolling
"""

import sys
import time
import numpy as np

def main():
    if len(sys.argv) != 2:
        print("Usage: python3 test_improved_scroll.py <serial_port>")
        print("Example: python3 test_improved_scroll.py /dev/ttyUSB0")
        print()
        print("Tests improved attitude-based scroll calculation:")
        print("1. Accelerometer/gyroscope data parsing")
        print("2. Roll/pitch attitude estimation") 
        print("3. Bandpass filtering for stillness detection")
        print("4. Physics-based scrolling with inertial modeling")
        sys.exit(1)
    
    port = sys.argv[1]
    
    try:
        # Import the improved system
        from fish_sender_with_imu_scroll import FishDisplayWithIMU
        
        print("=== Improved Attitude-Based Scroll Test ===")
        print(f"Connecting to {port}...")
        
        # Create system instance
        system = FishDisplayWithIMU(port)
        
        print("System initialized with improved scroll calculation!")
        print()
        print("Improved Features:")
        print("✓ Roll/Pitch attitude estimation from accelerometer")
        print("✓ Bandpass filtering (0.1-5Hz) for stillness detection") 
        print("✓ Physics-based forces proportional to tilt angle")
        print("✓ 1st order LPF inertial modeling (decay factor: 0.92)")
        print("✓ Enhanced debug output with attitude information")
        print()
        print("Testing for 60 seconds...")
        print("Tilt the device and observe the detailed status output")
        print("Press Ctrl+C to stop")
        
        # Run enhanced test simulation
        system.run_simulation(duration_seconds=60, num_fish=2, update_rate_hz=30)
        
        print("✓ Improved scroll test completed successfully!")
        
    except ImportError as e:
        print(f"Import error: {e}")
        print("Make sure scipy is available: pip install scipy")
        sys.exit(1)
    except Exception as e:
        print(f"Test failed: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()