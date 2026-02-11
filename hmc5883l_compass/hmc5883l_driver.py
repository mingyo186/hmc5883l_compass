# Copyright 2025 The hmc5883l_compass Authors
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
"""HMC5883L I2C Driver - 3-axis digital compass / magnetometer."""

import random
import struct
import time


class FakeHMC5883LDriver:
    """Fake driver that generates random magnetic field data without I2C hardware."""

    def __init__(self, **kwargs):
        pass

    def read_all(self):
        """Return (mx, my, mz) in Tesla."""
        mx = random.gauss(25.0e-6, 1.0e-6)
        my = random.gauss(0.0, 1.0e-6)
        mz = random.gauss(-45.0e-6, 1.0e-6)
        return mx, my, mz

    def id_string(self) -> str:
        return 'H43'

    def close(self):
        pass


class HMC5883LDriver:
    """Low-level I2C driver for Honeywell HMC5883L."""

    # ── Register Map ──────────────────────────────────────────────
    REG_CONFIG_A = 0x00
    REG_CONFIG_B = 0x01
    REG_MODE = 0x02
    REG_DATA_X_H = 0x03   # 6 bytes: XH, XL, ZH, ZL, YH, YL
    REG_STATUS = 0x09
    REG_ID_A = 0x0A   # 'H'
    REG_ID_B = 0x0B   # '4'
    REG_ID_C = 0x0C   # '3'

    # ── Gain Table ───────────────────────────────────────────────
    #  gain setting -> (register value, LSB/Gauss)
    GAIN_TABLE = {
        0: (0x00, 1370.0),    # ±0.88 Ga
        1: (0x20, 1090.0),    # ±1.3  Ga (default)
        2: (0x40,  820.0),    # ±1.9  Ga
        3: (0x60,  660.0),    # ±2.5  Ga
        4: (0x80,  440.0),    # ±4.0  Ga
        5: (0xA0,  390.0),    # ±4.7  Ga
        6: (0xC0,  330.0),    # ±5.6  Ga
        7: (0xE0,  230.0),    # ±8.1  Ga
    }

    GAUSS_TO_TESLA = 1.0e-4   # 1 Gauss = 0.0001 Tesla

    def __init__(self, bus: int = 1, address: int = 0x1E,
                 gain: int = 1, data_rate: int = 4,
                 samples_averaged: int = 3):
        from smbus2 import SMBus

        self.address = address
        self.bus = SMBus(bus)
        self._scale = self.GAIN_TABLE[gain][1]

        self._init_device(gain, data_rate, samples_averaged)

    # ── Initialisation ───────────────────────────────────────────
    def _init_device(self, gain: int, data_rate: int,
                     samples_averaged: int):
        # Config Register A: MA[6:5]=samples, DO[4:2]=rate, MS[1:0]=normal
        config_a = ((samples_averaged & 0x03) << 5) | \
                   ((data_rate & 0x07) << 2) | 0x00
        self.bus.write_byte_data(self.address, self.REG_CONFIG_A, config_a)

        # Config Register B: GN[7:5]=gain
        config_b = self.GAIN_TABLE[gain][0]
        self.bus.write_byte_data(self.address, self.REG_CONFIG_B, config_b)

        # Mode Register: continuous measurement mode
        self.bus.write_byte_data(self.address, self.REG_MODE, 0x00)
        time.sleep(0.01)

    # ── Burst read ───────────────────────────────────────────────
    def read_all(self):
        """Read magnetic field in one burst and return (mx, my, mz) in Tesla."""
        buf = bytes(self.bus.read_i2c_block_data(
            self.address, self.REG_DATA_X_H, 6))

        # HMC5883L data order: X, Z, Y (big-endian signed 16-bit)
        raw_x = struct.unpack_from('>h', buf, 0)[0]
        raw_z = struct.unpack_from('>h', buf, 2)[0]
        raw_y = struct.unpack_from('>h', buf, 4)[0]

        mx = raw_x / self._scale * self.GAUSS_TO_TESLA
        my = raw_y / self._scale * self.GAUSS_TO_TESLA
        mz = raw_z / self._scale * self.GAUSS_TO_TESLA

        return mx, my, mz

    def id_string(self) -> str:
        """Read identification registers (should return 'H43')."""
        a = self.bus.read_byte_data(self.address, self.REG_ID_A)
        b = self.bus.read_byte_data(self.address, self.REG_ID_B)
        c = self.bus.read_byte_data(self.address, self.REG_ID_C)
        return chr(a) + chr(b) + chr(c)

    def close(self):
        self.bus.close()
