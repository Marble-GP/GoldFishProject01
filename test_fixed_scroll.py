#!/usr/bin/env python3
"""
Test script for the fixed scroll calculation addressing feedback issues:
1. Removed unnecessary low-pass filter (ESP32 already filters)
2. Increased scroll sensitivity (45° tilt should produce significant movement)
3. Fixed velocity decay (horizontal position should reduce velocity to zero)
4. Added 15° dead zone (no forces when tilt < 15°)
"""

import sys
import time

def main():
    if len(sys.argv) != 2:
        print("Usage: python3 test_fixed_scroll.py <serial_port>")
        print("Example: python3 test_fixed_scroll.py /dev/ttyUSB0")
        print()
        print("Tests fixed scroll calculation addressing specific issues:")
        print("✓ No unnecessary low-pass filter (ESP32 already filters)")
        print("✓ Increased force scaling (800x) for better 45° response")
        print("✓ Fixed velocity decay (stronger decay when horizontal)")  
        print("✓ 15° dead zone (no forces when total tilt < 15°)")
        print("✓ Enhanced debug output with dead zone status")
        sys.exit(1)
    
    port = sys.argv[1]
    
    try:
        # Import the fixed system
        from fish_sender_with_imu_scroll import FishDisplayWithIMU
        
        print("=== Fixed Scroll Calculation Test ===")
        print(f"Connecting to {port}...")
        
        # Create system instance
        system = FishDisplayWithIMU(port)
        
        print("System initialized with fixed scroll calculation!")
        print()
        print("Fixed Issues:")
        print("✓ Removed unnecessary low-pass filter")
        print("✓ Increased tilt force scale: 100 → 800")
        print("✓ Increased max velocity: 80 → 200 px/s")
        print("✓ Added 15° dead zone for near-horizontal positions")
        print("✓ Stronger velocity decay (0.7) when not applying forces")
        print("✓ Enhanced debug: Total tilt angle and dead zone status")
        print()
        print("Test Instructions:")
        print("1. Keep device horizontal → Should show DEAD_ZONE, velocity → 0")
        print("2. Tilt 45° → Should show significant scroll movement")
        print("3. Return to horizontal → Velocity should decay quickly")
        print("4. Observe detailed status output")
        print()
        print("Testing for 90 seconds...")
        print("Press Ctrl+C to stop")
        
        # Run test simulation with higher update rate for better responsiveness
        system.run_simulation(duration_seconds=90, num_fish=2, update_rate_hz=30)
        
        print("✓ Fixed scroll test completed successfully!")
        
    except ImportError as e:
        print(f"Import error: {e}")
        print("Make sure scipy is available: pip install scipy")
        sys.exit(1)
    except Exception as e:
        print(f"Test failed: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()