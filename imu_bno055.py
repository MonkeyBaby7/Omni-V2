# imu_bno055.py
# ------------------------------------------------------------
# Very small BNO055 driver: we only need "heading" (yaw).
#
# It talks over I2C and returns heading in degrees (0..360).
# ------------------------------------------------------------

from machine import I2C
import time

_BNO055_CHIP_ID      = 0x00
_BNO055_OPR_MODE     = 0x3D
_BNO055_PWR_MODE     = 0x3E
_BNO055_PAGE_ID      = 0x07
_BNO055_SYS_TRIGGER  = 0x3F
_BNO055_EULER_H_LSB  = 0x1A

_MODE_CONFIG = 0x00
_MODE_NDOF   = 0x0C


class BNO055:
    def __init__(self, i2c: I2C, addr: int):
        self.i2c = i2c
        self.addr = addr

    def _write8(self, reg: int, val: int) -> None:
        self.i2c.writeto_mem(self.addr, reg, bytes([val & 0xFF]))

    def _read8(self, reg: int) -> int:
        return int.from_bytes(self.i2c.readfrom_mem(self.addr, reg, 1), "little")

    def _read16(self, reg: int) -> int:
        d = self.i2c.readfrom_mem(self.addr, reg, 2)
        return d[0] | (d[1] << 8)

    def begin(self) -> bool:
        """Put BNO055 into NDOF fusion mode so we can read heading."""
        if self._read8(_BNO055_CHIP_ID) != 0xA0:
            return False

        # Enter config mode before changing settings
        self._write8(_BNO055_OPR_MODE, _MODE_CONFIG)
        time.sleep(0.03)

        self._write8(_BNO055_PAGE_ID, 0x00)
        time.sleep(0.01)

        self._write8(_BNO055_PWR_MODE, 0x00)      # normal power
        time.sleep(0.01)

        self._write8(_BNO055_SYS_TRIGGER, 0x00)   # normal operation
        time.sleep(0.01)

        # Switch to fusion mode
        self._write8(_BNO055_OPR_MODE, _MODE_NDOF)
        time.sleep(0.03)

        return True

    def heading_deg(self) -> float:
        """
        Heading (yaw) in degrees.
        BNO055 stores heading as 1/16 degree units.
        """
        raw = self._read16(_BNO055_EULER_H_LSB)
        return (raw / 16.0) % 360.0


def find_bno055(i2c: I2C):
    """
    BNO055 is usually at 0x28 or 0x29.
    Return a working BNO055 object or None.
    """
    for addr in (0x28, 0x29):
        try:
            imu = BNO055(i2c, addr)
            if imu.begin():
                return imu
        except OSError:
            pass
    return None
