# Copyright 2025 Marc Duclusaud & Grégoire Passault

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:

#     http://www.apache.org/licenses/LICENSE-2.0

import os
import time
import numpy as np
from dynamixel_sdk import *

# =============================================================================
# Control table – Protocol V1 (MX / AX series)
# =============================================================================
# Torque enable
ADDR_TORQUE_ENABLE = 24
# P Gain
ADDR_P_GAIN = 28
# Goal position
ADDR_GOAL_POSITION = 30
# Present position (2 bytes)
ADDR_PRESENT_POSITION = 36
# Present speed (2 bytes)
ADDR_PRESENT_SPEED = 38
# Present load (2 bytes)
ADDR_PRESENT_LOAD = 40
# Present voltage (1 byte)
ADDR_PRESENT_VOLTAGE = 42
# Present temperature (1 byte)
ADDR_PRESENT_TEMPERATURE = 43

# =============================================================================
# Control table – XL-320 (Protocol V2 only)
# =============================================================================
# --- EEPROM ---
XL320_ADDR_CW_ANGLE_LIMIT = 6       # 2 bytes
XL320_ADDR_CCW_ANGLE_LIMIT = 8      # 2 bytes
XL320_ADDR_CONTROL_MODE = 11        # 1 byte  (1=Wheel, 2=Joint)
XL320_ADDR_MAX_TORQUE = 15          # 2 bytes
# --- RAM ---
XL320_ADDR_TORQUE_ENABLE = 24       # 1 byte
XL320_ADDR_LED = 25                 # 1 byte  (RGB bitmask: R=1, G=2, B=4)
XL320_ADDR_D_GAIN = 27              # 1 byte
XL320_ADDR_I_GAIN = 28              # 1 byte
XL320_ADDR_P_GAIN = 29              # 1 byte
XL320_ADDR_GOAL_POSITION = 30       # 2 bytes  (0–1023, 0.29°/step, range 0–300°)
XL320_ADDR_MOVING_SPEED = 32        # 2 bytes  (0–1023 CCW, 1024–2047 CW, 0.111 rpm/step)
XL320_ADDR_TORQUE_LIMIT = 35        # 2 bytes
XL320_ADDR_PRESENT_POSITION = 37    # 2 bytes
XL320_ADDR_PRESENT_SPEED = 39       # 2 bytes
XL320_ADDR_PRESENT_LOAD = 41        # 2 bytes
XL320_ADDR_PRESENT_VOLTAGE = 45     # 1 byte
XL320_ADDR_PRESENT_TEMPERATURE = 46 # 1 byte
XL320_ADDR_MOVING = 49              # 1 byte  (0=stopped, 1=moving)
XL320_ADDR_HW_ERROR_STATUS = 50     # 1 byte

# XL-320 position resolution
XL320_RESOLUTION = 1023             # 10-bit
XL320_RANGE_DEG = 300.0             # degrees
XL320_CENTER = 512                  # raw value for 0 rad (150°)

# =============================================================================
# Control table – XH-540 (Protocol V2, shared by the X430-generation X-series)
# =============================================================================
# https://docs.robotis.com/en/docs/dxl/model_reference/x_series/xh_series/xh540-w270
# --- EEPROM ---
XH540_ADDR_OPERATING_MODE = 11         # 1 byte
XH540_ADDR_PWM_LIMIT = 36              # 2 bytes  (raw counts = 100% duty, default 885)
XH540_ADDR_CURRENT_LIMIT = 38          # 2 bytes
XH540_ADDR_VELOCITY_LIMIT = 44         # 4 bytes
XH540_ADDR_MAX_POSITION_LIMIT = 48     # 4 bytes
XH540_ADDR_MIN_POSITION_LIMIT = 52     # 4 bytes
# Indirect Address 1..N: pointer slots (2 bytes each) -- writing a source
# register address here mirrors that byte into Indirect Data (see below) on
# every subsequent read. Lives in EEPROM: torque must be OFF to write it.
XH540_ADDR_INDIRECT_ADDRESS_1 = 168    # 2 bytes per slot
# --- RAM ---
XH540_ADDR_TORQUE_ENABLE = 64          # 1 byte
XH540_ADDR_LED = 65                    # 1 byte
XH540_ADDR_HARDWARE_ERROR_STATUS = 70  # 1 byte
XH540_ADDR_POSITION_D_GAIN = 80        # 2 bytes
XH540_ADDR_POSITION_I_GAIN = 82        # 2 bytes
XH540_ADDR_POSITION_P_GAIN = 84        # 2 bytes  (KPP = register / 128, per manual)
XH540_ADDR_GOAL_PWM = 100              # 2 bytes
XH540_ADDR_GOAL_CURRENT = 102          # 2 bytes
XH540_ADDR_GOAL_VELOCITY = 104         # 4 bytes
XH540_ADDR_PROFILE_ACCELERATION = 108  # 4 bytes
XH540_ADDR_PROFILE_VELOCITY = 112      # 4 bytes
XH540_ADDR_GOAL_POSITION = 116         # 4 bytes
XH540_ADDR_MOVING = 122                # 1 byte
XH540_ADDR_PRESENT_PWM = 124           # 2 bytes  (signed, 0.113 %/count)
XH540_ADDR_PRESENT_CURRENT = 126       # 2 bytes
XH540_ADDR_PRESENT_VELOCITY = 128      # 4 bytes  (signed, 0.229 rpm/count)
XH540_ADDR_PRESENT_POSITION = 132      # 4 bytes  (signed, multi-turn capable)
XH540_ADDR_PRESENT_INPUT_VOLTAGE = 144 # 2 bytes  (0.1 V/count)
XH540_ADDR_PRESENT_TEMPERATURE = 146   # 1 byte
# Indirect Data 1..N: where the mirrored bytes actually show up (1 byte
# each). Ordinary RAM, so it's read like any other register.
XH540_ADDR_INDIRECT_DATA_1 = 224       # 1 byte per slot

# XH-540 position resolution
XH540_RESOLUTION = 4096             # 12-bit (0.088°/count)
# Operating Mode (addr 11) on this hardware is Position Control Mode (3),
# NOT Extended Position Control Mode -- confirmed by reading it back live
# (returned 3, and Present Position sat around 2048, not 0). That means Goal/
# Present Position is single-turn UNSIGNED 0..4095, not the signed multi-turn
# range the rest of this file used to assume. rad<->raw conversion below is
# ported from ROBOTIS's ROS 2 dynamixel_hardware_interface package, which
# targets this same mode: raw 0..4095 maps onto -pi..+pi.

# ---------------------------------------------------------------------------
# Indirect Address block for status read-back: wires several non-contiguous
# RAM registers into one contiguous block so they can be fetched (or written)
# in a single bus transaction, instead of one packet per register. Same
# technique/registers as dynamixel_hardware_interface's
# control_table.hpp::status_indirect, with Present PWM added (needed to
# identify KP_DIVISOR -- see bam/dynamixel/configure_kp_divisor.py) and
# Present Current dropped (not needed here).
# ---------------------------------------------------------------------------
XH540_INDIRECT_STATUS_FIELDS = [
    # (name, source address, length in bytes)
    ("hardware_error_status", XH540_ADDR_HARDWARE_ERROR_STATUS, 1),
    ("goal_position", XH540_ADDR_GOAL_POSITION, 4),
    ("present_pwm", XH540_ADDR_PRESENT_PWM, 2),
    ("present_velocity", XH540_ADDR_PRESENT_VELOCITY, 4),
    ("present_position", XH540_ADDR_PRESENT_POSITION, 4),
    ("present_input_voltage", XH540_ADDR_PRESENT_INPUT_VOLTAGE, 2),
    ("present_temperature", XH540_ADDR_PRESENT_TEMPERATURE, 1),
]


def _build_indirect_layout(fields):
    """Flatten a [(name, address, length), ...] spec into the per-byte
    source-address list to program into Indirect Address, plus each field's
    byte offset inside the resulting Indirect Data block."""
    source_addresses = []
    offsets = {}
    for name, address, length in fields:
        offsets[name] = len(source_addresses)
        source_addresses.extend(address + i for i in range(length))
    return source_addresses, offsets


XH540_INDIRECT_STATUS_ADDRESSES, XH540_INDIRECT_STATUS_OFFSETS = _build_indirect_layout(
    XH540_INDIRECT_STATUS_FIELDS
)
XH540_INDIRECT_STATUS_LENGTH = len(XH540_INDIRECT_STATUS_ADDRESSES)

# ---------------------------------------------------------------------------
# Indirect Address block for commands (write side): Goal Position / Torque
# Enable / Position P Gain wired into one contiguous block, so every control
# step can Sync Write all of them as ONE packet -- Sync Write needs a single
# fixed (address, length) shared across the group, which is exactly what an
# Indirect Data block gives us (and, unlike Bulk Write, generalizes to many
# servo IDs at once if this class is ever used for more than one motor).
# Placed in the slots right after the status block (XH-540 has 28 Indirect
# Address/Data slots total; status uses 18, this uses 7 -> 25 of 28).
# ---------------------------------------------------------------------------
XH540_INDIRECT_COMMAND_FIELDS = [
    ("goal_position", XH540_ADDR_GOAL_POSITION, 4),
    ("torque_enable", XH540_ADDR_TORQUE_ENABLE, 1),
    ("p_gain", XH540_ADDR_POSITION_P_GAIN, 2),
]
XH540_INDIRECT_COMMAND_ADDRESSES, XH540_INDIRECT_COMMAND_OFFSETS = _build_indirect_layout(
    XH540_INDIRECT_COMMAND_FIELDS
)
XH540_INDIRECT_COMMAND_LENGTH = len(XH540_INDIRECT_COMMAND_ADDRESSES)

# Slot index (0-based) where the command block starts, right after status
_XH540_COMMAND_SLOT_OFFSET = XH540_INDIRECT_STATUS_LENGTH
XH540_ADDR_INDIRECT_ADDRESS_COMMANDS = XH540_ADDR_INDIRECT_ADDRESS_1 + 2 * _XH540_COMMAND_SLOT_OFFSET
XH540_ADDR_INDIRECT_DATA_COMMANDS = XH540_ADDR_INDIRECT_DATA_1 + _XH540_COMMAND_SLOT_OFFSET

if _XH540_COMMAND_SLOT_OFFSET + XH540_INDIRECT_COMMAND_LENGTH > 28:
    raise RuntimeError("XH-540 only has 28 Indirect Address/Data slots")


class DynamixelActuatorV1:
    def __init__(self, port: str, id: int = 1):
        self.id = id

        result = os.system(f"setserial {port} low_latency")
        if result != 0:
            raise Exception("Failed to set low latency mode (you can try: sudo apt install setserial)")

        self.portHandler = PortHandler(port)
        self.packetHandler = PacketHandler(1.0)

        self.portHandler.openPort()
        self.portHandler.setBaudRate(1000000)

    def set_p_gain(self, gain: int):
        # Set P gain
        self.packetHandler.write2ByteTxOnly(
            self.portHandler, self.id, ADDR_P_GAIN, gain
        )

    def set_torque(self, enable: bool):
        # Enable torque
        self.packetHandler.write1ByteTxOnly(
            self.portHandler, self.id, ADDR_TORQUE_ENABLE, 1 if enable else 0
        )

    def set_goal_position(self, position: float):
        # Position is a 12-bit value
        position = int(4096 * (position / (2 * np.pi) + 0.5))

        # Set goal position
        self.packetHandler.write2ByteTxOnly(
            self.portHandler, self.id, ADDR_GOAL_POSITION, position
        )

    def read_data(self):
        # Reading position, speed, load, voltage and temperature
        data, result, error = self.packetHandler.readTxRx(
            self.portHandler, self.id, ADDR_PRESENT_POSITION, 8
        )

        # Position is a 12-bit value
        position = (data[1] << 8) | data[0]
        position = 2 * np.pi * ((position / 4096) - 0.5)

        # Speed is a 10-bit value, units are 0.11 rpm per step
        speed = (data[3] << 8) | data[2]
        if speed > 1024:
            speed = -(speed - 1024)
        speed = speed * 0.11 * 2 * np.pi / 60.0

        # Applied "load"
        load = (data[5] << 8) | data[4]
        if load > 1024:
            load = -(load - 1024)

        # Voltage is a byte value, units are 0.1 V
        volts = data[6] / 10.0

        # Temperature are °C
        temp = data[7]

        return {
            "position": position,
            "speed": speed,
            "load": load,
            "input_volts": volts,
            "temp": temp,
        }


class DynamixelXL320:
    """
    Controller for the XL-320 servo motor.

    The XL-320 uses **Protocol 2.0 only** and has a 10-bit position encoder
    covering a 300° range (0 to 1023 raw, centre = 512 ≡ 0 rad).

    Args:
        port: Serial port, e.g. ``"/dev/ttyUSB0"``.
        id:   DYNAMIXEL ID (default 1).
    """

    def __init__(self, port: str, id: int = 1):
        self.id = id

        result = os.system(f"setserial {port} low_latency")
        if result != 0:
            raise Exception(
                "Failed to set low latency mode "
                "(you can try: sudo apt install setserial)"
            )

        self.portHandler = PortHandler(port)
        self.packetHandler = PacketHandler(2.0)  # Protocol 2.0 mandatory

        self.portHandler.openPort()
        self.portHandler.setBaudRate(1000000)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _rad_to_raw(position: float) -> int:
        """Convert radians to XL-320 raw position (0–1023, centre = 0 rad)."""
        raw = int(position * (XL320_RESOLUTION / (XL320_RANGE_DEG * np.pi / 180.0)) + XL320_CENTER)
        return int(np.clip(raw, 0, XL320_RESOLUTION))

    @staticmethod
    def _raw_to_rad(raw: int) -> float:
        """Convert XL-320 raw position to radians."""
        return (raw - XL320_CENTER) * (XL320_RANGE_DEG * np.pi / 180.0) / XL320_RESOLUTION

    # ------------------------------------------------------------------
    # Write commands
    # ------------------------------------------------------------------

    def set_torque(self, enable: bool):
        """Enable or disable motor torque."""
        self.packetHandler.write1ByteTxOnly(
            self.portHandler, self.id, XL320_ADDR_TORQUE_ENABLE, 1 if enable else 0
        )

    def set_p_gain(self, gain: int):
        """Set the proportional (P) gain (0–254)."""
        self.packetHandler.write1ByteTxOnly(
            self.portHandler, self.id, XL320_ADDR_P_GAIN, gain
        )

    def set_i_gain(self, gain: int):
        """Set the integral (I) gain (0–254)."""
        self.packetHandler.write1ByteTxOnly(
            self.portHandler, self.id, XL320_ADDR_I_GAIN, gain
        )

    def set_d_gain(self, gain: int):
        """Set the derivative (D) gain (0–254)."""
        self.packetHandler.write1ByteTxOnly(
            self.portHandler, self.id, XL320_ADDR_D_GAIN, gain
        )

    def set_pid_gains(self, p: int, i: int, d: int):
        """Set P, I and D gains in one call (values 0–254 each)."""
        self.set_p_gain(p)
        self.set_i_gain(i)
        self.set_d_gain(d)

    def set_goal_position(self, position: float):
        """
        Set goal position.

        Args:
            position: Target angle in radians. Acceptable range:
                      ``[-150°, +150°]`` → ``[-2.618, +2.618]`` rad.
        """
        raw = self._rad_to_raw(position)
        self.packetHandler.write2ByteTxOnly(
            self.portHandler, self.id, XL320_ADDR_GOAL_POSITION, raw
        )

    def set_moving_speed(self, speed_rpm: float):
        """
        Set moving speed in Joint Mode.

        Args:
            speed_rpm: Speed in RPM (0 = maximum, 114 = approx. max).
                       The sign is ignored; always positive in joint mode.
        """
        raw = int(abs(speed_rpm) / 0.111)
        raw = int(np.clip(raw, 0, 1023))
        self.packetHandler.write2ByteTxOnly(
            self.portHandler, self.id, XL320_ADDR_MOVING_SPEED, raw
        )

    def set_led(self, color: int):
        """
        Set the LED colour using a RGB bitmask.

        Args:
            color: Bitmask – Red=1, Green=2, Blue=4 (combinations allowed, 0=off).
        """
        self.packetHandler.write1ByteTxOnly(
            self.portHandler, self.id, XL320_ADDR_LED, color & 0x07
        )

    # ------------------------------------------------------------------
    # Read feedback
    # ------------------------------------------------------------------

    def read_data(self) -> dict:
        """
        Read present position, speed, load, input voltage and temperature.

        Returns a dict with keys:
        ``position`` (rad), ``speed`` (rad/s), ``load`` (signed, 0.1 % units),
        ``input_volts`` (V), ``temp`` (°C).

        Note: addresses 37–42 are contiguous; 43–44 are unused; 45–46 are
        voltage and temperature.  Two separate reads are performed.
        """
        # Read position (2 B), speed (2 B), load (2 B) → 6 bytes from addr 37
        data_psl, result, error = self.packetHandler.readTxRx(
            self.portHandler, self.id, XL320_ADDR_PRESENT_POSITION, 6
        )
        
        # Present position – 10-bit, 0.29°/step, centre = 512
        position_raw = (data_psl[1] << 8) | data_psl[0]
        position = self._raw_to_rad(position_raw)

        # Present speed – 11-bit value, bit 10 = direction (0=CCW, 1=CW)
        # Unit: 0.111 rpm/step
        speed_raw = (data_psl[3] << 8) | data_psl[2]
        if speed_raw > 1023:
            speed = -(speed_raw - 1024)
        else:
            speed = speed_raw
        speed = speed * 0.111 * 2 * np.pi / 60.0  # rad/s

        # Present load – 11-bit, bit 10 = direction, unit 0.1 %
        load_raw = (data_psl[5] << 8) | data_psl[4]
        if load_raw > 1023:
            load = -(load_raw - 1024)
        else:
            load = load_raw

        # Read voltage (1 B) and temperature (1 B) → 2 bytes from addr 45
        data_vt, result, error = self.packetHandler.readTxRx(
            self.portHandler, self.id, XL320_ADDR_PRESENT_VOLTAGE, 2
        )

        # Voltage: unit is 0.1 V
        volts = data_vt[0] / 10.0

        # Temperature in °C
        temp = data_vt[1]

        return {
            "position": position,
            "speed": speed,
            "load": load,
            "input_volts": volts,
            "temp": temp,
        }


class DynamixelXH540:
    """
    Controller for the XH-540 servo motor (Protocol 2.0), also valid for the
    other X430-generation X-series servos sharing the same control table
    (XH430, XM430, XM540, XL430, ...).

    12-bit encoder (4096 counts/rev). Operating Mode is Position Control
    Mode (single-turn, raw 0..4095, unsigned) on this hardware, NOT the
    signed multi-turn range Extended Position Control Mode would give --
    raw 0..4095 maps onto -pi..+pi (see _rad_to_raw/_raw_to_rad), not
    centred on raw 0. Unlike the XL-320, the duty-cycle scale (``load``)
    depends on the servo's configured ``PWM Limit`` (EEPROM address 36),
    which is read once at connection time and cached.

    Args:
        port: Serial port, e.g. ``"/dev/ttyUSB0"``.
        id:   DYNAMIXEL ID (default 1).
    """

    def __init__(self, port: str, id: int = 1):
        self.id = id

        result = os.system(f"setserial {port} low_latency")
        if result != 0:
            raise Exception(
                "Failed to set low latency mode "
                "(you can try: sudo apt install setserial)"
            )

        self.portHandler = PortHandler(port)
        self.packetHandler = PacketHandler(2.0)  # Protocol 2.0 mandatory

        if not self.portHandler.openPort():
            raise RuntimeError(f"Failed to open serial port {port}")
        if not self.portHandler.setBaudRate(1000000):
            raise RuntimeError(f"Failed to set baud rate on {port}")

        # Raw PWM count corresponding to 100% duty; needed to normalize
        # Present PWM into a -1..1 duty cycle (see read_data()).
        pwm_limit_bytes = self._read_bytes(XH540_ADDR_PWM_LIMIT, 2, "PWM Limit")
        self.pwm_limit = pwm_limit_bytes[0] | (pwm_limit_bytes[1] << 8)

        self._configure_indirect_status()
        self._configure_indirect_commands()

        # write_targets() always sends the full command block (Sync Write
        # needs a fixed length); these hold the last commanded value for
        # whichever register a given call doesn't pass.
        self._last_p_gain = 0
        self._last_torque_enable = False

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _check_comm(self, result: int, error: int, context: str):
        """Raise a clear error on a failed read/write instead of silently
        continuing with garbage data (e.g. an empty/zero-filled reply) --
        that's what previously turned a "servo not responding" situation
        into a confusing downstream IndexError/ZeroDivisionError.

        ERRBIT_ALERT rides along on every packet while the servo's Hardware
        Error Status is nonzero (e.g. a live Input Voltage Error) -- it isn't
        specific to this read/write, so it's masked off here and doesn't
        raise on its own. A genuine per-instruction error (bad address, CRC,
        wrong length, ...) still does.
        """
        if result != COMM_SUCCESS:
            raise RuntimeError(
                f"{context} (id={self.id}): {self.packetHandler.getTxRxResult(result)}"
            )
        real_error = error & ~ERRBIT_ALERT
        if real_error != 0:
            raise RuntimeError(
                f"{context} (id={self.id}): {self.packetHandler.getRxPacketError(real_error)}"
            )

    def _drain(self):
        """Discard stale bytes sitting in the serial receive buffer.

        PortHandler.clearPort() only flushes pending *writes* (pyserial's
        Serial.flush() is output-only, despite the name) -- it never clears
        the input buffer. A leftover tail from an earlier mangled exchange
        can sit there and get prepended to the next reply, which would
        explain a short read failing the exact same way on every retry.
        """
        n = self.portHandler.getBytesAvailable()
        if n > 0:
            self.portHandler.readPort(n)

    def _read_bytes(self, address: int, length: int, context: str, retries: int = 5) -> list:
        """readTxRx() with a length check and a few retries.

        A CRC-valid reply (result=COMM_SUCCESS, error=0) can still carry
        fewer bytes than requested if the packet got mangled by line noise
        in transit -- _check_comm() alone doesn't catch that, since from the
        SDK's point of view the read "succeeded". Treat a short reply the
        same as a comm failure, drain any stale buffered bytes, and retry a
        few times before giving up.
        """
        last_result, last_error, last_len = None, None, 0
        for _ in range(retries):
            self._drain()  # start each attempt from a clean buffer, not just after a failure
            data, result, error = self.packetHandler.readTxRx(self.portHandler, self.id, address, length)
            real_error = error & ~ERRBIT_ALERT  # ALERT alone (live Hardware Error Status) isn't a read failure
            if result == COMM_SUCCESS and real_error == 0 and len(data) == length:
                return data
            last_result, last_error, last_len = result, error, len(data)
            time.sleep(0.002)

        self._check_comm(last_result, last_error, f"Failed to read {context}")
        raise RuntimeError(
            f"Failed to read {context} (id={self.id}): got {last_len}/{length} bytes after {retries} attempts"
        )

    @staticmethod
    def _rad_to_raw(position: float) -> int:
        """Convert radians to XH-540 raw position (Position Control Mode,
        unsigned 0..4095). Ported as-is from the ROBOTIS ROS 2
        dynamixel_hardware_interface package (MotorSetting::radianToTick,
        motor_setting.cpp) -- clamps to +-pi, then maps that onto 0..4095."""
        clamped = float(np.clip(position, -np.pi, np.pi))
        return int(round((clamped + np.pi) * ((XH540_RESOLUTION - 1) / (2 * np.pi))))

    @staticmethod
    def _raw_to_rad(raw: int) -> float:
        """Convert XH-540 raw position to radians -- inverse of _rad_to_raw,
        ported from the same package's MotorStatus::positionRawToRadian
        (motor_status.cpp)."""
        return (raw / (XH540_RESOLUTION - 1)) * (2 * np.pi) - np.pi

    # ------------------------------------------------------------------
    # Write commands
    #
    # Only 3 registers ever need writing for identification: Torque Enable
    # (event-driven, rare), Position P Gain (once per run -- the swept
    # kp_firmware), and Goal Position (every control step -- the actual
    # excitation signal). Each has its own single-packet setter below;
    # write_targets() additionally sends any combination of them as ONE
    # Sync Write packet, through the 'commands' Indirect Data block (see
    # _configure_indirect_commands) so the same call generalizes to many
    # servo IDs at once, unlike Bulk Write which is inherently per-ID.
    # ------------------------------------------------------------------

    def set_torque(self, enable: bool):
        """Enable or disable motor torque."""
        self.packetHandler.write1ByteTxOnly(
            self.portHandler, self.id, XH540_ADDR_TORQUE_ENABLE, 1 if enable else 0
        )

    def set_p_gain(self, gain: int):
        """Set the Position P Gain (0–16383, internally KPP = gain / 128)."""
        self.packetHandler.write2ByteTxOnly(
            self.portHandler, self.id, XH540_ADDR_POSITION_P_GAIN, gain
        )

    def _write_checked(self, write_fn, address: int, value: int, context: str):
        """write2/4ByteTxRx() with a buffer drain first -- a stale leftover
        reply sitting in the input buffer from an earlier exchange can get
        misread as this write's ack, the same failure mode _read_bytes()
        guards against on the read side (see its docstring)."""
        self._drain()
        result, error = write_fn(self.portHandler, self.id, address, value)
        self._check_comm(result, error, context)

    def set_i_gain(self, gain: int):
        """Set the Position I Gain (0–16383). TxRx (checked) since this is a
        one-time setup call, not a per-step write -- worth confirming it
        actually landed rather than assuming a TxOnly write succeeded."""
        self._write_checked(
            self.packetHandler.write2ByteTxRx, XH540_ADDR_POSITION_I_GAIN, gain,
            "Failed to set Position I Gain",
        )

    def set_d_gain(self, gain: int):
        """Set the Position D Gain (0–16383). TxRx (checked), see set_i_gain.

        The firmware's actual PWM output is a full PID, not pure P:
        roughly ``raw_pwm = Kp*error/128 + Ki*integral - Kd*velocity/16``.
        For identification this must be 0 (along with set_i_gain(0)) so
        duty = kp*error/KP_DIVISOR holds while the motor is moving --
        otherwise the velocity-dependent D term contaminates every sample
        taken during a moving excitation signal (triangle wave, ramp, ...),
        and does so differently at each Kp/speed, which is exactly the kind
        of Kp-dependent bias configure_kp_divisor.py was seeing.
        """
        self._write_checked(
            self.packetHandler.write2ByteTxRx, XH540_ADDR_POSITION_D_GAIN, gain,
            "Failed to set Position D Gain",
        )

    def read_pid_gains(self) -> tuple[int, int, int]:
        """Read back (p_gain, i_gain, d_gain) directly from the servo, to
        confirm what's actually configured rather than trusting our own
        last write. Uses _read_bytes() (drain + length check + retries) --
        NOT a bare read2ByteTxRx per register, which was the actual bug
        behind I/D reading back as bogus/duplicated values: a stale reply
        left over from the previous register's read was getting picked up
        as this one's, with nothing to catch a short/wrong-length response.
        """
        def u16(data: list) -> int:
            return data[0] | (data[1] << 8)

        p = u16(self._read_bytes(XH540_ADDR_POSITION_P_GAIN, 2, "Position P Gain"))
        i = u16(self._read_bytes(XH540_ADDR_POSITION_I_GAIN, 2, "Position I Gain"))
        d = u16(self._read_bytes(XH540_ADDR_POSITION_D_GAIN, 2, "Position D Gain"))
        return p, i, d

    def set_profile(self, velocity: int = 0, acceleration: int = 0):
        """Set Profile Velocity / Profile Acceleration (raw units, default 0).

        In Position Control Mode these normally make the firmware smooth a
        new Goal Position into a trapezoidal trajectory, and drive the PID
        against that internal setpoint rather than the raw Goal Position we
        wrote. 0 disables this (treated as "infinite" velocity/acceleration)
        so a Goal Position write becomes the PID's setpoint immediately --
        required for identification: with profiling on, the position error
        we compute from our commanded goal_position is not what the
        firmware's controller actually sees, which silently breaks the
        duty = kp*error/KP_DIVISOR relationship (see configure_kp_divisor.py).
        """
        self._write_checked(
            self.packetHandler.write4ByteTxRx, XH540_ADDR_PROFILE_VELOCITY, velocity,
            "Failed to set Profile Velocity",
        )
        self._write_checked(
            self.packetHandler.write4ByteTxRx, XH540_ADDR_PROFILE_ACCELERATION, acceleration,
            "Failed to set Profile Acceleration",
        )

    def set_goal_position(self, position: float):
        """
        Set goal position.

        Args:
            position: Target angle in radians.
        """
        raw = self._rad_to_raw(position)
        self.packetHandler.write4ByteTxOnly(
            self.portHandler, self.id, XH540_ADDR_GOAL_POSITION, raw
        )

    def write_targets(
        self,
        goal_position: float,
        p_gain: int | None = None,
        torque_enable: bool | None = None,
    ):
        """Sync-write Goal Position (+ Position P Gain, Torque Enable) to the
        servo as ONE packet. ``p_gain``/``torque_enable``, when omitted,
        keep whatever value was last commanded (Sync Write always writes the
        full fixed-length command block, so there is no "leave untouched"
        the way Bulk Write's addParam-per-register allowed).
        """
        if p_gain is not None:
            self._last_p_gain = p_gain
        if torque_enable is not None:
            self._last_torque_enable = torque_enable

        raw = self._rad_to_raw(goal_position) & 0xFFFFFFFF
        data = [
            raw & 0xFF, (raw >> 8) & 0xFF, (raw >> 16) & 0xFF, (raw >> 24) & 0xFF,
            1 if self._last_torque_enable else 0,
            self._last_p_gain & 0xFF, (self._last_p_gain >> 8) & 0xFF,
        ]

        group = GroupSyncWrite(
            self.portHandler, self.packetHandler,
            XH540_ADDR_INDIRECT_DATA_COMMANDS, XH540_INDIRECT_COMMAND_LENGTH,
        )
        group.addParam(self.id, data)
        if group.txPacket() != COMM_SUCCESS:
            raise RuntimeError("write_targets(): Sync Write failed")

    # ------------------------------------------------------------------
    # Indirect Address / Indirect Data
    # ------------------------------------------------------------------

    def _configure_indirect_status(self):
        """Program XH540_INDIRECT_STATUS_FIELDS into the Indirect Address
        table, so read_status()/read_data() fetch all of them in a single
        bus transaction instead of one packet per register.

        Indirect Address lives in EEPROM, so torque must be disabled while
        writing it (harmless here: called once from __init__, before torque
        is ever enabled).
        """
        self.set_torque(False)
        for i, source_address in enumerate(XH540_INDIRECT_STATUS_ADDRESSES):
            self._write_checked(
                self.packetHandler.write2ByteTxRx,
                XH540_ADDR_INDIRECT_ADDRESS_1 + 2 * i,
                source_address,
                f"Failed to map status slot {i} (addr {XH540_ADDR_INDIRECT_ADDRESS_1 + 2 * i} -> {source_address})",
            )
            time.sleep(0.005)  # let the EEPROM write settle before the next one

    def _configure_indirect_commands(self):
        """Program XH540_INDIRECT_COMMAND_FIELDS into the Indirect Address
        table (right after the status slots), so write_targets() can Sync
        Write Goal Position/Torque Enable/Position P Gain as one packet.

        Same EEPROM caveat as _configure_indirect_status(): torque off while
        writing it, harmless here (still __init__, before torque is used).
        """
        self.set_torque(False)
        for i, source_address in enumerate(XH540_INDIRECT_COMMAND_ADDRESSES):
            self._write_checked(
                self.packetHandler.write2ByteTxRx,
                XH540_ADDR_INDIRECT_ADDRESS_COMMANDS + 2 * i,
                source_address,
                f"Failed to map command slot {i} (addr {XH540_ADDR_INDIRECT_ADDRESS_COMMANDS + 2 * i} -> {source_address})",
            )
            time.sleep(0.005)  # let the EEPROM write settle before the next one

    # ------------------------------------------------------------------
    # Read feedback
    #
    # Only Present Position and Present Velocity actually feed the CMA-ES
    # objective (bam.simulate.Simulator.rollout_log re-derives the control
    # signal from the model, so goal_position/torque_enable are the values
    # WE commanded, never read back). Present PWM, Present Input Voltage,
    # Present Temperature and Hardware Error Status aren't used by the fit
    # itself: PWM is what configure_kp_divisor.py needs for KP_DIVISOR, and
    # the rest are safety/diagnostics. All of it comes back in one packet
    # via Indirect Data, so there's no real cost to reading a bit extra.
    # ------------------------------------------------------------------

    def read_status(self) -> dict:
        """
        Read the whole status block mapped by _configure_indirect_status()
        in a single readTxRx() call.

        Returns a dict with keys:
        ``hardware_error_status`` (raw byte), ``goal_position_readback``
        (rad, as echoed by the servo), ``position`` (rad), ``speed`` (rad/s),
        ``load`` (duty cycle, -1..1), ``input_volts`` (V), ``temp`` (°C).
        """
        data = self._read_bytes(
            XH540_ADDR_INDIRECT_DATA_1, XH540_INDIRECT_STATUS_LENGTH, "status block"
        )
        off = XH540_INDIRECT_STATUS_OFFSETS

        def u32(offset: int) -> int:
            raw = data[offset] | (data[offset + 1] << 8) | (data[offset + 2] << 16) | (data[offset + 3] << 24)
            return raw - 2**32 if raw > 2**31 - 1 else raw

        def u16(offset: int) -> int:
            raw = data[offset] | (data[offset + 1] << 8)
            return raw - 2**16 if raw > 2**15 - 1 else raw

        hardware_error_status = data[off["hardware_error_status"]]
        goal_position_readback = self._raw_to_rad(u32(off["goal_position"]))

        pwm_raw = u16(off["present_pwm"])
        load = float(np.clip(pwm_raw / self.pwm_limit, -1.0, 1.0))

        speed = u32(off["present_velocity"]) * 0.229 * 2 * np.pi / 60.0  # rad/s
        position = self._raw_to_rad(u32(off["present_position"]))

        v_off = off["present_input_voltage"]
        volts = (data[v_off] | (data[v_off + 1] << 8)) / 10.0  # unit is 0.1 V
        temp = data[off["present_temperature"]]  # °C

        return {
            "hardware_error_status": hardware_error_status,
            "goal_position_readback": goal_position_readback,
            "position": position,
            "speed": speed,
            "load": load,
            "input_volts": volts,
            "temp": temp,
        }

    def read_data(self) -> dict:
        """
        Read present position, speed, load, input voltage and temperature.

        Returns a dict with keys:
        ``position`` (rad), ``speed`` (rad/s), ``load`` (duty cycle, -1..1),
        ``input_volts`` (V), ``temp`` (°C).

        Thin wrapper over read_status() kept for parity with
        DynamixelActuatorV1/DynamixelXL320's read_data() signature (used by
        record.py). Fields required by the CMA-ES fit itself are only
        ``position`` and ``speed`` -- see bam/simulate.py.
        """
        status = self.read_status()
        return {
            "position": status["position"],
            "speed": status["speed"],
            "load": status["load"],
            "input_volts": status["input_volts"],
            "temp": status["temp"],
        }
