"""
C Firmware Board - Run C firmware in Python simulator (SITL)
Software-in-the-Loop testing: Exact same C code that runs on hardware
"""

from __future__ import annotations

import ctypes
import os
import platform
from typing import Dict, Optional, Sequence, Tuple

import numpy as np

from common.math import Quaternion, Vector3D
from common.types import ImuSample, SensorReadings, StateEstimate
from miniflight.board import Board
from common.interface import Sensor, Actuator

# Import simulator for physics
from target.simulator import SimWorld


class CFirmwareLibrary:
    """Wrapper for C firmware shared library"""
    
    def __init__(self):
        # Find the shared library
        lib_dir = os.path.join(os.path.dirname(__file__), 'stm32', 'build_lib')
        
        # Determine library name based on platform
        system = platform.system()
        if system == 'Darwin':  # macOS
            lib_name = 'libminiflight.dylib'
        elif system == 'Linux':
            lib_name = 'libminiflight.so'
        elif system == 'Windows':
            lib_name = 'libminiflight.dll'
        else:
            raise RuntimeError(f"Unsupported platform: {system}")
        
        lib_path = os.path.join(lib_dir, lib_name)
        
        if not os.path.exists(lib_path):
            raise FileNotFoundError(
                f"C firmware library not found: {lib_path}\n"
                f"Build it with:\n"
                f"  cd target/stm32\n"
                f"  make -f Makefile.lib"
            )
        
        # Load the library
        self.lib = ctypes.CDLL(lib_path)
        
        # Define function signatures
        self.lib.firmware_lib_init.argtypes = []
        self.lib.firmware_lib_init.restype = None
        
        self.lib.firmware_lib_reset.argtypes = []
        self.lib.firmware_lib_reset.restype = None
        
        self.lib.firmware_lib_update.argtypes = [
            ctypes.c_float,  # accel_x
            ctypes.c_float,  # accel_y
            ctypes.c_float,  # accel_z
            ctypes.c_float,  # gyro_x
            ctypes.c_float,  # gyro_y
            ctypes.c_float,  # gyro_z
            ctypes.c_float,  # altitude
            ctypes.c_float,  # dt
            ctypes.POINTER(ctypes.c_float)  # output[4]
        ]
        self.lib.firmware_lib_update.restype = None
        
        self.lib.firmware_lib_get_state.argtypes = [
            ctypes.POINTER(ctypes.c_float),  # position[3]
            ctypes.POINTER(ctypes.c_float),  # velocity[3]
            ctypes.POINTER(ctypes.c_float),  # orientation[4]
            ctypes.POINTER(ctypes.c_float),  # angular_velocity[3]
        ]
        self.lib.firmware_lib_get_state.restype = None
        
        self.lib.firmware_lib_set_altitude.argtypes = [ctypes.c_float]
        self.lib.firmware_lib_set_altitude.restype = None
        
        self.lib.firmware_lib_set_attitude.argtypes = [
            ctypes.c_float,  # roll
            ctypes.c_float,  # pitch
            ctypes.c_float,  # yaw
        ]
        self.lib.firmware_lib_set_attitude.restype = None
        
        self.lib.firmware_lib_is_initialized.argtypes = []
        self.lib.firmware_lib_is_initialized.restype = ctypes.c_int
        
        self.lib.firmware_lib_init_progress.argtypes = []
        self.lib.firmware_lib_init_progress.restype = ctypes.c_float
    
    def init(self):
        self.lib.firmware_lib_init()
    
    def reset(self):
        self.lib.firmware_lib_reset()
    
    def update(self, accel: tuple, gyro: tuple, altitude: float, dt: float) -> tuple:
        """Run one control loop iteration"""
        output = (ctypes.c_float * 4)()
        self.lib.firmware_lib_update(
            ctypes.c_float(accel[0]),
            ctypes.c_float(accel[1]),
            ctypes.c_float(accel[2]),
            ctypes.c_float(gyro[0]),
            ctypes.c_float(gyro[1]),
            ctypes.c_float(gyro[2]),
            ctypes.c_float(altitude),
            ctypes.c_float(dt),
            output
        )
        return tuple(output)
    
    def get_state(self) -> StateEstimate:
        """Get current state estimate from C firmware"""
        position = (ctypes.c_float * 3)()
        velocity = (ctypes.c_float * 3)()
        orientation = (ctypes.c_float * 4)()
        angular_velocity = (ctypes.c_float * 3)()
        
        self.lib.firmware_lib_get_state(position, velocity, orientation, angular_velocity)
        
        return StateEstimate(
            position=Vector3D(*position),
            velocity=Vector3D(*velocity),
            orientation=Quaternion(*orientation),
            angular_velocity=Vector3D(*angular_velocity),
        )
    
    def set_altitude(self, z: float):
        self.lib.firmware_lib_set_altitude(ctypes.c_float(z))
    
    def set_attitude(self, roll: float, pitch: float, yaw: float):
        self.lib.firmware_lib_set_attitude(
            ctypes.c_float(roll),
            ctypes.c_float(pitch),
            ctypes.c_float(yaw)
        )
    
    def is_initialized(self) -> bool:
        return bool(self.lib.firmware_lib_is_initialized())
    
    def init_progress(self) -> float:
        return self.lib.firmware_lib_init_progress()


class CFirmwareBoard(Board):
    """
    Board implementation that runs C firmware in Python simulator
    SITL (Software-in-the-Loop) - validates C implementation
    """
    
    def __init__(self, dt: float = 0.01):
        self.dt = dt
        
        # Physics simulator (same as Python firmware)
        self._world = SimWorld(dt)
        
        # C firmware library
        self._firmware = CFirmwareLibrary()
        self._firmware.init()
        
        # Storage for C firmware output
        self._last_control_output = [0.0, 0.0, 0.0, 0.0]
        
        # Track setpoints for pilot commands
        self._z_setpoint = 1.0
        self._yaw_setpoint = 0.0
        
        print("✅ C Firmware Board initialized (SITL mode)")
        print(f"   Library loaded: target/stm32/build_lib/")
        print(f"   🎯 C firmware handles ALL control (Python bypassed)")
        print(f"   ⚡ Clean implementation - only C runs!")
    
    def read_sensors(self) -> SensorReadings:
        """Read sensors from physics sim"""
        imu_sample = self._world.imu_sample()
        readings = SensorReadings(
            imu=imu_sample,
            timestamp=imu_sample.timestamp,
            altitude=self._world.altitude()
        )
        
        # Run C firmware update with sensor data
        output = self._firmware.update(
            accel=readings.imu.accel,
            gyro=readings.imu.gyro,
            altitude=readings.altitude,
            dt=self.dt
        )
        
        # Store control output for write_actuators
        self._last_control_output = output
        
        return readings
    
    def write_actuators(self, commands: Sequence[float]):
        """
        Write commands to physics sim
        
        With the clean implementation, 'commands' comes from state_control()
        which now correctly returns the C firmware output (no Python estimator/controller).
        So we just use the commands directly.
        """
        self._world.apply_motor_commands(list(commands))
    
    def motor_geometry(self):
        return self._world.motor_geometry()
    
    def close(self):
        self._world.close()
    
    def get_input_state(self) -> Dict[str, bool]:
        """Get keyboard state from renderer"""
        return self._world.get_input_state()
    
    # Control setpoint updates (route to C firmware)
    
    def update_altitude_setpoint(self, delta: float):
        """Update altitude setpoint (called from pilot command)"""
        self._z_setpoint += delta
        self._z_setpoint = max(0.0, min(20.0, self._z_setpoint))
        self._firmware.set_altitude(self._z_setpoint)
    
    def update_yaw_rate(self, yaw_rate: float, dt: float):
        """Update yaw setpoint from rate command"""
        from common.math import wrap_angle
        self._yaw_setpoint = wrap_angle(self._yaw_setpoint + yaw_rate * dt)
    
    def update_attitude_setpoint(self, roll: float, pitch: float):
        """Update attitude setpoint (called from pilot command)"""
        self._firmware.set_attitude(roll, pitch, self._yaw_setpoint)
    
    # C firmware-specific methods
    
    def get_c_state_estimate(self) -> StateEstimate:
        """Get state estimate directly from C firmware"""
        return self._firmware.get_state()
    
    def set_c_altitude(self, z: float):
        """Set altitude setpoint in C firmware"""
        self._firmware.set_altitude(z)
    
    def set_c_attitude(self, roll: float, pitch: float, yaw: float):
        """Set attitude setpoints in C firmware"""
        self._firmware.set_attitude(roll, pitch, yaw)
    
    def c_is_initialized(self) -> bool:
        """Check if C firmware estimator is initialized"""
        return self._firmware.is_initialized()
    
    def c_init_progress(self) -> float:
        """Get C firmware initialization progress"""
        return self._firmware.init_progress()
    
    def run_c_firmware_step(self, readings: SensorReadings) -> tuple:
        """
        Run one iteration of C firmware control loop
        Returns: [thrust, roll_cmd, pitch_cmd, yaw_cmd]
        """
        output = self._firmware.update(
            accel=readings.imu.accel,
            gyro=readings.imu.gyro,
            altitude=readings.altitude,
            dt=self.dt
        )
        return output


# For backward compatibility, expose as SimBoard-like interface
__all__ = ["CFirmwareBoard"]

