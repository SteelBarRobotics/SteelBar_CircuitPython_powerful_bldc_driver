# SPDX-FileCopyrightText: 2017 Scott Shawcroft, written for Adafruit Industries
# SPDX-FileCopyrightText: Copyright (c) 2024 Emily Trau for Steel Bar Robotics
#
# SPDX-License-Identifier: MIT
"""
`steelbar_powerful_bldc_driver`
================================================================================


.. todo:: Describe what the library does.


* Author(s): Emily Trau, Andrew Chen

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
    self._send_buffer = bytearray(64)
  def _pack_float(self, offset:int, data: float) -> None:
    struct.pack_into("<f", self._send_buffer, offset, data);
  def _pack_8bit(self, offset:int, data: int) -> None:
    self._send_buffer[offset] = data
  def _pack_16bit(self, offset:int, data: int) -> None:
    self._send_buffer[offset] = data & 0xFF
    self._send_buffer[offset+1] = (data >> 8) & 0xFF
  def _pack_32bit(self, offset:int, data: int) -> None:
    self._send_buffer[offset] = data & 0xFF
    self._send_buffer[offset+1] = (data >> 8) & 0xFF
    self._send_buffer[offset+2] = (data >> 16) & 0xFF
    self._send_buffer[offset+3] = (data >> 24) & 0xFF
    
  def set_iq_pid_constants(self, kp: float, ki: float, kd: float) -> None:
    self._send_buffer[0] = 0x40
    self._pack_float(1, kp)
    self._pack_float(5, ki)
    self._pack_float(9, kd)
    self._i2c_device.write(self._send_buffer, end=13)
  def set_id_pid_constants(self, kp: float, ki: float, kd: float) -> None:
    self._send_buffer[0] = 0x41
    self._pack_float(1, kp)
    self._pack_float(5, ki)
    self._pack_float(9, kd)
    self._i2c_device.write(self._send_buffer, end=13)
  def set_speed_pid_constants(self, kp: float, ki: float, kd: float) -> None:
    self._send_buffer[0] = 0x42
    self._pack_float(1, kp)
    self._pack_float(5, ki)
    self._pack_float(9, kd)
    self._i2c_device.write(self._send_buffer, end=13)
  def configure_operating_mode_and_sensor(self, operating_mode: int, sensor_type: int) -> None:
    self._send_buffer[0] = 0x20
    self._send_buffer[1] = operating_mode + (sensor_type << 4)
    self._i2c_device.write(self._send_buffer, end=2)
  def configure_command_mode(self, command_mode: int) -> None:
    self._send_buffer[0] = 0x21
    self._send_buffer[1] = command_mode
    self._i2c_device.write(self._send_buffer, end=2)
  def set_voltage(self, voltage: int) -> None:
    self._send_buffer[0] = 0x10
    self._pack_32bit(voltage)
    self._i2c_device.write(self._send_buffer, end=5)
  def set_torque(self, torque: int) -> None:
    self._send_buffer[0] = 0x11
    self._pack_32bit(torque)
    self._i2c_device.write(self._send_buffer, end=5)
  def set_speed(self, speed: int) -> None:
    self._send_buffer[0] = 0x12
    self._pack_32bit(speed)
    self._i2c_device.write(self._send_buffer, end=5)
  def set_current_limit_foc(self, current: int) -> None:
    self._send_buffer[0] = 0x33
    self._pack_32bit(current)
    self._i2c_device.write(self._send_buffer, end=5)
  def clear_faults(self) -> None:
    self._send_buffer[0] = 0x01
    self._i2c_device.write(self._send_buffer, end=1)
  def set_ELECANGLEOFFSET(self, value: int) -> None:
    self._send_buffer[0] = 0x30
    self._pack_32bit(value)
    self._i2c_device.write(self._send_buffer, end=5)
  def set_EAOPERSPEED(self, value: int) -> None:
    self._send_buffer[0] = 0x31
    self._pack_32bit(value)
    self._i2c_device.write(self._send_buffer, end=5)
  def set_SINCOSCENTRE(self, value: int) -> None:
    self._send_buffer[0] = 0x32
    self._pack_32bit(value)
    self._i2c_device.write(self._send_buffer, end=5)
