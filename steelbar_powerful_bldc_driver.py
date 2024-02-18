# SPDX-FileCopyrightText: 2017 Scott Shawcroft, written for Adafruit Industries
# SPDX-FileCopyrightText: Copyright (c) 2024 Emily Trau for Steel Bar Robotics
#
# SPDX-License-Identifier: MIT
"""
`steelbar_powerful_bldc_driver`
================================================================================


.. todo:: Describe what the library does.


* Author(s): Emily Trau

Implementation Notes
--------------------

**Hardware:**

.. todo:: Add links to any specific hardware product page(s), or category page(s).
  Use unordered list & hyperlink rST inline format: "* `Link Text <url>`_"

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://circuitpython.org/downloads

.. todo:: Uncomment or remove the Bus Device and/or the Register library dependencies
  based on the library's use of either.

# * Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
# * Adafruit's Register library: https://github.com/adafruit/Adafruit_CircuitPython_Register
"""

# imports
import struct
from micropython import const
from busio import I2C
from adafruit_bus_device import i2c_device

__version__ = "0.0.0+auto.0"
__repo__ = "https://github.com/SteelBarRobotics/SteelBar_CircuitPython_powerful_bldc_driver.git"


_DEFAULT_I2C_ADDR = const(0x19)

OPERATING_MODE_TRAPEZOIDAL = const(1)
OPERATING_MODE_SINUSOIDAL = const(2)
OPERATING_MODE_FIELD_ORIENTED_CONTROL = const(3)
OPERATING_MODE_CALIBRATION = const(15)

SENSOR_TYPE_SIN_COS_ENCODER = const(1)
SENSOR_TYPE_INCREMENTAL_ENCODER = const(2)
SENSOR_TYPE_SENSORLESS = const(3)

COMMAND_MODE_VOLTAGE = const(1)
COMMAND_MODE_TORQUE = const(2)
COMMAND_MODE_SPEED = const(12)
COMMAND_MODE_POSITION = const(13)
COMMAND_MODE_CALIBRATION = const(15)


class PowerfulBLDCDriver:
    def __init__(self, i2c_bus: I2C, address: int = _DEFAULT_I2C_ADDR) -> None:
        self._i2c_device = i2c_device.I2CDevice(i2c_bus, address)
        self._address = address
        self._send_buffer = bytearray(9)
    
    def _send_register(self, register: int) -> None:
        self._send_buffer[0] = register
        with self._i2c_device as device:
            device.write(self._send_buffer, 0, 1)
    
    def _send_8bit_value(self, register: int, data: int) -> None:
        self._send_buffer[0] = register
        self._send_buffer[1] = data
        with self._i2c_device as device:
            device.write(self._send_buffer, 0, 2)
    
    def _send_16bit_value(self, register: int, data: int) -> None:
        self._send_buffer[0] = register
        self._send_buffer[1] = data & 0xFF
        self._send_buffer[2] = (data >> 8) & 0xFF
        with self._i2c_device as device:
            device.write(self._send_buffer, 0, 3)
    
    def _send_32bit_value(self, register: int, data: int) -> None:
        self._send_buffer[0] = register
        self._send_buffer[1] = data & 0xFF
        self._send_buffer[2] = (data >> 8) & 0xFF
        self._send_buffer[3] = (data >> 16) & 0xFF
        self._send_buffer[4] = (data >> 24) & 0xFF
        with self._i2c_device as device:
            device.write(self._send_buffer, 0, 5)
    
    def _send_float_value(self, register: int, data: float) -> None:
        self._send_buffer[0] = register
        struct.pack_into("<f", self._send_buffer, 1, data)
        with self._i2c_device as device:
            device.write(self._send_buffer, 0, 5)
    
    def _send_two_float_values(self, register: int, data1: float, data2: float) -> None:
        self._send_buffer[0] = register
        struct.pack_into("<ff", self._send_buffer, 1, data1, data2)
        with self._i2c_device as device:
            device.write(self._send_buffer, 0, 9)
    
    def set_iq_pid_constants(self, kp: float, ki: float) -> None:
        self._send_two_float_values(0x40, kp, ki)
    
    def set_id_pid_constants(self, kp: float, ki: float) -> None:
        self._send_two_float_values(0x41, kp, ki)
    
    def set_speed_pid_constants(self, kp: float, ki: float) -> None:
        self._send_two_float_values(0x42, kp, ki)
    
    def configure_operating_mode_and_sensor(self, operating_mode: int, sensor_type: int) -> None:
        self._send_8bit_value(0x20, operating_mode + (sensor_type << 4))
    
    def configure_command_mode(self, command_mode: int) -> None:
        self._send_8bit_value(0x21, command_mode)
    
    def set_voltage(self, voltage: int) -> None:
        self._send_32bit_value(0x10, voltage)
    
    def set_torque(self, torque: int) -> None:
        self._send_32bit_value(0x11, torque)
    
    def set_speed(self, speed: int) -> None:
        self._send_32bit_value(0x12, speed)
    
    def set_current_limit_foc(self, current: int) -> None:
        self._send_32bit_value(0x33, current)
    
    def clear_faults(self) -> None:
        self._send_register(0x01)
    
    def set_ELECANGLEOFFSET(self, value: int) -> None:
        self._send_32bit_value(0x30, value)
    
    def set_EAOPERSPEED(self, value: int) -> None:
        self._send_32bit_value(0x31, value)
    
    def set_SINCOSCENTRE(self, value: int) -> None:
        self._send_32bit_value(0x32, value)
    