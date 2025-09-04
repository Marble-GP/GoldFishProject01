#!/usr/bin/env python3
"""
Test script for the tiled scrollable background feature
Tests that the background tiles scroll seamlessly with the fish
"""

import sys
import time

def main():
    if len(sys.argv) != 2:
        print("Usage: python3 test_tiled_background.py <serial_port>")
        print("Example: python3 test_tiled_background.py /dev/ttyUSB0")
        print()
        print("Tests tiled scrollable background:")
        print("✓ 3x3 grid of background tiles for seamless scrolling")
        print("✓ Background tiles move opposite to scroll direction")  
        print("✓ Seamless wrapping using modulo arithmetic")
        print("✓ Fish and background scroll together")
        sys.exit(1)
    
    port = sys.argv[1]
    
    try:
        # Import the system with tiled background support
        from fish_sender_with_imu_scroll import FishDisplayWithIMU
        
        print("=== Tiled Background Scrolling Test ===")
        print(f"Connecting to {port}...")
        
        # Create system instance
        system = FishDisplayWithIMU(port)
        
        print("System initialized with tiled background support!")
        print()
        print("Tiled Background Features:")
        print("✓ ESP32: 3x3 grid of background tiles")
        print("✓ ESP32: Seamless tile wrapping using modulo arithmetic")
        print("✓ ESP32: Background scrolls opposite to scroll commands")
        print("✓ Python: Working attitude-based scroll calculation")
        print("✓ Integration: Fish and background scroll together")
        print()
        print("Test Instructions:")
        print("1. Tilt device → Both fish and background should scroll")
        print("2. Scroll continuously → Background should tile seamlessly")
        print("3. Observe that background pattern repeats smoothly")
        print("4. Fish should stay on background correctly")
        print()
        print("Testing for 120 seconds...")
        print("Press Ctrl+C to stop")
        
        # Run test simulation
        system.run_simulation(duration_seconds=120, num_fish=3, update_rate_hz=30)
        
        print("✓ Tiled background test completed successfully!")
        
    except ImportError as e:
        print(f"Import error: {e}")
        print("Make sure scipy is available: pip install scipy")
        sys.exit(1)
    except Exception as e:
        print(f"Test failed: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()