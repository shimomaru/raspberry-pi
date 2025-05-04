import smbus2
import time

class QMC5883L:
    def __init__(self, bus=1, address=0x0D):
        self.bus = smbus2.SMBus(bus)
        self.address = address
        self.init_sensor()

    def init_sensor(self):
        # Configure QMC5883L
        self.bus.write_byte_data(self.address, 0x0B, 0x01)  # Set period register
        self.bus.write_byte_data(self.address, 0x09, 0x1D)  # Continuous mode, 200Hz, 8G, OSR = 512

    def read_raw(self):
        data = self.bus.read_i2c_block_data(self.address, 0x00, 6)
        x = self._twos_complement(data[1] << 8 | data[0], 16)
        y = self._twos_complement(data[3] << 8 | data[2], 16)
        z = self._twos_complement(data[5] << 8 | data[4], 16)
        return [x, y, z]

    def _twos_complement(self, val, bits):
        if val & (1 << (bits - 1)):
            val -= 1 << bits
        return val

