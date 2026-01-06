#!/usr/bin/env python3
"""
Test SITL (Software-in-the-Loop) - Compare Python vs C firmware
Run the same sensor inputs through both implementations and compare outputs
"""

import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../..'))

import numpy as np
from target.c_firmware_board import CFirmwareLibrary
from miniflight.estimate import MahonyEstimator
from miniflight.control import StabilityController
from common.types import SensorReadings, ImuSample
from common.math import GRAVITY

def test_mahony_comparison():
    """Compare Python and C Mahony estimator outputs"""
    print("=" * 70)
    print("TEST: Mahony Estimator (Python vs C)")
    print("=" * 70)
    
    # Initialize both estimators
    py_est = MahonyEstimator()
    c_lib = CFirmwareLibrary()
    c_lib.init()
    
    dt = 0.01  # 100Hz
    
    # Synthetic sensor data: quadcopter at rest, slightly tilted
    accel = (0.1, 0.2, GRAVITY)  # Slight tilt (specific force pointing up)
    gyro = (0.0, 0.0, 0.0)  # No rotation
    altitude = 1.0
    
    # Run initialization (20 samples)
    print("\n1. Initialization phase (20 samples)...")
    for i in range(20):
        # Python
        readings = SensorReadings(
            imu=ImuSample(accel=accel, gyro=gyro, timestamp=i * dt),
            altitude=altitude,
            timestamp=i * dt
        )
        py_state = py_est.update(readings, dt)
        
        # C
        c_output = c_lib.update(accel, gyro, altitude, dt)
        c_state = c_lib.get_state()
        
        if i == 19:
            print(f"   Python initialized: {py_est.initialized}")
            print(f"   C initialized: {c_lib.is_initialized()}")
    
    # Compare orientation quaternions
    print("\n2. Orientation comparison:")
    print(f"   Python quat: {py_state.orientation.q}")
    print(f"   C quat:      [{c_state.orientation.q[0]:.6f}, {c_state.orientation.q[1]:.6f}, {c_state.orientation.q[2]:.6f}, {c_state.orientation.q[3]:.6f}]")
    
    # Calculate difference
    diff = np.abs(py_state.orientation.q - c_state.orientation.q)
    max_diff = np.max(diff)
    print(f"   Max difference: {max_diff:.9f}")
    
    if max_diff < 1e-5:
        print("   ✅ PASS: Quaternions match within tolerance")
    else:
        print(f"   ❌ FAIL: Difference too large (>{1e-5})")
    
    # Convert to Euler angles
    py_roll, py_pitch, py_yaw = py_state.orientation.to_euler()
    c_roll, c_pitch, c_yaw = c_state.orientation.to_euler()
    
    print("\n3. Euler angles (radians):")
    print(f"   Python: roll={py_roll:.6f}, pitch={py_pitch:.6f}, yaw={py_yaw:.6f}")
    print(f"   C:      roll={c_roll:.6f}, pitch={c_pitch:.6f}, yaw={c_yaw:.6f}")
    print(f"   Diff:   roll={abs(py_roll-c_roll):.9f}, pitch={abs(py_pitch-c_pitch):.9f}, yaw={abs(py_yaw-c_yaw):.9f}")
    
    print()
    return max_diff < 1e-5


def test_control_comparison():
    """Compare Python and C controller outputs"""
    print("=" * 70)
    print("TEST: Controller (Python vs C)")
    print("=" * 70)
    
    # Initialize both
    py_ctrl = StabilityController()
    c_lib = CFirmwareLibrary()
    c_lib.init()
    
    dt = 0.01
    
    # Run full pipeline with sensor data
    accel = (0.0, 0.0, GRAVITY)
    gyro = (0.0, 0.0, 0.0)
    altitude = 0.8  # Below setpoint (1.0)
    
    # Initialize estimators first
    print("\n1. Initializing estimators...")
    for i in range(20):
        readings = SensorReadings(
            imu=ImuSample(accel=accel, gyro=gyro, timestamp=i * dt),
            altitude=altitude,
            timestamp=i * dt
        )
        c_lib.update(accel, gyro, altitude, dt)
    
    # Now run control loop
    print("\n2. Running control loop (10 iterations)...")
    for i in range(10):
        # Python
        readings = SensorReadings(
            imu=ImuSample(accel=accel, gyro=gyro, timestamp=(20+i) * dt),
            altitude=altitude,
            timestamp=(20+i) * dt
        )
        from miniflight.estimate import MahonyEstimator
        py_est = MahonyEstimator()
        # Cheat: use C state for fair comparison
        c_state = c_lib.get_state()
        py_output = py_ctrl.update(c_state, dt)
        
        # C
        c_output = c_lib.update(accel, gyro, altitude, dt)
        
        if i == 9:  # Last iteration
            print(f"\n3. Control outputs (iteration {i+1}):")
            print(f"   Python: thrust={py_output[0]:.6f}, roll={py_output[1]:.6f}, pitch={py_output[2]:.6f}, yaw={py_output[3]:.6f}")
            print(f"   C:      thrust={c_output[0]:.6f}, roll={c_output[1]:.6f}, pitch={c_output[2]:.6f}, yaw={c_output[3]:.6f}")
            
            diff = [abs(py_output[i] - c_output[i]) for i in range(4)]
            max_diff = max(diff)
            print(f"   Diff:   thrust={diff[0]:.9f}, roll={diff[1]:.9f}, pitch={diff[2]:.9f}, yaw={diff[3]:.9f}")
            print(f"   Max difference: {max_diff:.9f}")
            
            if max_diff < 1e-5:
                print("   ✅ PASS: Control outputs match within tolerance")
                result = True
            else:
                print(f"   ❌ FAIL: Difference too large (>{1e-5})")
                result = False
    
    print()
    return result


def test_full_loop():
    """Run full control loop and print statistics"""
    print("=" * 70)
    print("TEST: Full Control Loop Performance")
    print("=" * 70)
    
    c_lib = CFirmwareLibrary()
    c_lib.init()
    
    dt = 0.01
    
    # Simulated sensor data
    accel = (0.05, -0.03, GRAVITY)
    gyro = (1.0, -0.5, 0.2)  # Some rotation
    altitude = 1.2
    
    print("\nRunning 1000 iterations...")
    import time
    start = time.time()
    
    for i in range(1000):
        output = c_lib.update(accel, gyro, altitude, dt)
    
    elapsed = time.time() - start
    avg_time = elapsed / 1000 * 1000  # ms
    
    print(f"\n Results:")
    print(f"   Total time: {elapsed:.3f} seconds")
    print(f"   Average per iteration: {avg_time:.6f} ms")
    print(f"   Max achievable rate: {1000.0 / avg_time:.0f} Hz")
    
    state = c_lib.get_state()
    print(f"\n Final state:")
    print(f"   Position: [{state.position.v[0]:.3f}, {state.position.v[1]:.3f}, {state.position.v[2]:.3f}]")
    print(f"   Orientation (quat): [{state.orientation.q[0]:.3f}, {state.orientation.q[1]:.3f}, {state.orientation.q[2]:.3f}, {state.orientation.q[3]:.3f}]")
    
    print()


if __name__ == "__main__":
    print("\n" + "=" * 70)
    print("SITL TESTING: Python vs C Firmware Comparison")
    print("=" * 70)
    print()
    
    try:
        # Run tests
        test1 = test_mahony_comparison()
        test2 = test_control_comparison()
        test_full_loop()
        
        # Summary
        print("=" * 70)
        print("SUMMARY")
        print("=" * 70)
        print(f"Mahony Estimator: {'✅ PASS' if test1 else '❌ FAIL'}")
        print(f"Controller:       {'✅ PASS' if test2 else '❌ FAIL'}")
        print()
        
        if test1 and test2:
            print("🎉 ALL TESTS PASSED! C firmware matches Python implementation!")
            sys.exit(0)
        else:
            print("⚠️  SOME TESTS FAILED. Check output above.")
            sys.exit(1)
    
    except FileNotFoundError as e:
        print(f"\n❌ ERROR: {e}")
        print("\nMake sure to build the C library first:")
        print("  cd target/stm32")
        print("  make -f Makefile.lib")
        sys.exit(1)
    except Exception as e:
        print(f"\n❌ ERROR: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

