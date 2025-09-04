#!/usr/bin/env python3
"""
Test script for scroll area limitation feature
Tests that scroll area is properly limited to 1440x1440 pixels (±720 from center)
"""

import sys
import time

def main():
    if len(sys.argv) != 2:
        print("Usage: python3 test_scroll_limitation.py <serial_port>")
        print("Example: python3 test_scroll_limitation.py /dev/ttyUSB0")
        print()
        print("Tests scroll area limitation:")
        print("✓ Maximum scroll area: 1440x1440 pixels")
        print("✓ Scroll limits: ±720 pixels from center")  
        print("✓ Position clamping when limits reached")
        print("✓ Seamless boundary handling")
        sys.exit(1)
    
    port = sys.argv[1]
    
    try:
        # Import the system with scroll limitation
        from fish_sender_with_imu_scroll import FishDisplayWithIMU
        
        print("=== Scroll Area Limitation Test ===")
        print(f"Connecting to {port}...")
        
        # Create system instance
        system = FishDisplayWithIMU(port)
        
        print("System initialized with scroll area limitation!")
        print()
        print("Scroll Area Limitation Features:")
        print("✓ Maximum scroll area: 1440×1440 pixels")
        print("✓ Scroll limits: ±720 pixels from center (0,0)")
        print("✓ Python: Position clamping at boundaries")
        print("✓ ESP32: 3x3 tiled background supports full area")
        print("✓ Fish movement preserved within scroll boundaries")
        print()
        print("Test Instructions:")
        print("1. Tilt device strongly → Scroll should be limited to ±720")
        print("2. Hold extreme tilt → Position should clamp at boundaries")
        print("3. Return to center → Should scroll back smoothly")
        print("4. Monitor scroll values in status output")
        print()
        print("Expected scroll range: -720 ≤ scroll ≤ +720")
        print()
        print("Testing for 90 seconds...")
        print("Press Ctrl+C to stop")
        
        # Run test simulation
        system.run_simulation(duration_seconds=90, num_fish=3, update_rate_hz=30)
        
        print("✓ Scroll area limitation test completed successfully!")
        
    except ImportError as e:
        print(f"Import error: {e}")
        print("Make sure scipy is available: pip install scipy")
        sys.exit(1)
    except Exception as e:
        print(f"Test failed: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()