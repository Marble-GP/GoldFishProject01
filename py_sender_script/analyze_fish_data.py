#!/usr/bin/env python3
"""
Analyze fish_body_cache.pkl to extract real movement parameters
"""

import pickle
import numpy as np
import matplotlib.pyplot as plt
from scipy import stats

def analyze_fish_data():
    """Extract movement parameters from real fish analysis data"""
    
    # Load the fish analysis data
    try:
        with open('FishVideoModeling/fish_body_cache.pkl', 'rb') as f:
            cache_data = pickle.load(f)
            fish_data = cache_data['fish_data']
            print(f"Loaded {len(fish_data)} frames of fish analysis data")
            print(f"Cache created: {cache_data['timestamp']}")
    except Exception as e:
        print(f"Error loading fish data: {e}")
        return None
    
    # Extract key data
    positions = []
    angles = []
    curvatures = []
    major_axes = []
    minor_axes = []
    
    for frame in fish_data:
        if frame['center'] and frame['angle'] is not None:
            positions.append(frame['center'])
            angles.append(frame['angle'])
            
        if frame['curvature'] is not None:
            curvatures.append(frame['curvature'])
            
        if frame['major_axis_length'] and frame['minor_axis_length']:
            major_axes.append(frame['major_axis_length'])
            minor_axes.append(frame['minor_axis_length'])
    
    positions = np.array(positions)
    angles = np.array(angles)
    curvatures = np.array(curvatures)
    
    print(f"\nData extracted:")
    print(f"  Positions: {len(positions)} frames")
    print(f"  Angles: {len(angles)} frames")
    print(f"  Curvatures: {len(curvatures)} frames")
    
    # Calculate movement statistics
    if len(positions) >= 3:
        # Velocity analysis
        velocities = np.diff(positions, axis=0)
        velocity_magnitudes = np.linalg.norm(velocities, axis=1)
        
        # Acceleration analysis
        accelerations = np.diff(velocities, axis=0)
        acceleration_magnitudes = np.linalg.norm(accelerations, axis=1)
        
        # Angular velocity analysis
        angle_diffs = np.diff(angles)
        # Handle angle wraparound
        angle_diffs = np.where(angle_diffs > 180, angle_diffs - 360, angle_diffs)
        angle_diffs = np.where(angle_diffs < -180, angle_diffs + 360, angle_diffs)
        angular_velocities = np.abs(angle_diffs)
        
        print("\n" + "="*60)
        print("REAL FISH MOVEMENT STATISTICS FROM VIDEO ANALYSIS")
        print("="*60)
        
        print(f"\n🎯 VELOCITY CHARACTERISTICS:")
        print(f"  Mean velocity: {np.mean(velocity_magnitudes):.3f} pixels/frame")
        print(f"  Std velocity:  {np.std(velocity_magnitudes):.3f}")
        print(f"  Min velocity:  {np.min(velocity_magnitudes):.3f}")
        print(f"  Max velocity:  {np.max(velocity_magnitudes):.3f}")
        print(f"  Median velocity: {np.median(velocity_magnitudes):.3f}")
        print(f"  95th percentile: {np.percentile(velocity_magnitudes, 95):.3f}")
        
        print(f"\n⚡ ACCELERATION CHARACTERISTICS:")
        print(f"  Mean acceleration: {np.mean(acceleration_magnitudes):.3f} pixels/frame²")
        print(f"  Std acceleration:  {np.std(acceleration_magnitudes):.3f}")
        print(f"  Min acceleration:  {np.min(acceleration_magnitudes):.3f}")
        print(f"  Max acceleration:  {np.max(acceleration_magnitudes):.3f}")
        print(f"  Median acceleration: {np.median(acceleration_magnitudes):.3f}")
        
        print(f"\n🔄 ANGULAR VELOCITY CHARACTERISTICS:")
        print(f"  Mean angular velocity: {np.mean(angular_velocities):.3f}°/frame")
        print(f"  Std angular velocity:  {np.std(angular_velocities):.3f}°")
        print(f"  Min angular velocity:  {np.min(angular_velocities):.3f}°")
        print(f"  Max angular velocity:  {np.max(angular_velocities):.3f}°")
        print(f"  Median angular velocity: {np.median(angular_velocities):.3f}°")
        
        if len(curvatures) > 0:
            print(f"\n🌊 CURVATURE CHARACTERISTICS:")
            print(f"  Mean curvature: {np.mean(curvatures):.6f}")
            print(f"  Std curvature:  {np.std(curvatures):.6f}")
            print(f"  Min curvature:  {np.min(curvatures):.6f}")
            print(f"  Max curvature:  {np.max(curvatures):.6f}")
        
        # Calculate optimal parameters for goldfish simulation
        print(f"\n" + "="*60)
        print("RECOMMENDED GOLDFISH PARAMETERS (Data-Driven)")
        print("="*60)
        
        # Use statistical measures for parameter estimation
        noise_std = np.std(velocity_magnitudes) * 0.5  # 50% of velocity variation
        max_velocity = np.percentile(velocity_magnitudes, 80) * 0.4  # 40% of 80th percentile
        acceleration_std = np.std(acceleration_magnitudes) * 0.3  # 30% of acceleration variation
        turning_rate = np.percentile(angular_velocities, 75) * 0.5  # 50% of 75th percentile
        
        print(f"\n📊 GOLDFISH-SCALED PARAMETERS:")
        print(f"  noise_std = {noise_std:.3f}  # Based on velocity std")
        print(f"  max_velocity = {max_velocity:.3f}  # Based on 80th percentile")
        print(f"  acceleration_std = {acceleration_std:.3f}  # Based on acceleration std")
        print(f"  turning_rate = {turning_rate:.3f}  # Based on 75th percentile")
        print(f"  velocity_decay = 0.94  # Conservative momentum retention")
        
        # Velocity distribution analysis
        print(f"\n📈 VELOCITY DISTRIBUTION ANALYSIS:")
        print(f"  Distribution appears: ", end="")
        # Test for normality
        _, p_value = stats.normaltest(velocity_magnitudes)
        if p_value > 0.05:
            print("Normal (Gaussian)")
        else:
            print("Non-normal (may be log-normal or other)")
        
        print(f"  Normality test p-value: {p_value:.6f}")
        
        # Return the calculated parameters
        return {
            'noise_std': noise_std,
            'max_velocity': max_velocity,
            'acceleration_std': acceleration_std,
            'turning_rate': turning_rate,
            'velocity_decay': 0.94,
            'raw_data': {
                'velocity_magnitudes': velocity_magnitudes,
                'acceleration_magnitudes': acceleration_magnitudes,
                'angular_velocities': angular_velocities,
                'curvatures': curvatures
            }
        }
    
    return None

if __name__ == "__main__":
    print("Fish Movement Data Analysis")
    print("="*60)
    
    params = analyze_fish_data()
    
    if params:
        print(f"\n✅ Successfully extracted data-driven parameters!")
        print(f"   These should replace the hardcoded constants in fish_sender_realistic.py")
    else:
        print(f"\n❌ Failed to extract parameters from fish data")