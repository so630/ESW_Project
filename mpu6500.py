import time
from smbus2 import SMBus


MPU_ADDR = 0x68

# ============================================================
# Registers
# ============================================================

WHO_AM_I = 0x75

PWR_MGMT_1 = 0x6B
PWR_MGMT_2 = 0x6C

CONFIG = 0x1A
SMPLRT_DIV = 0x19

GYRO_CONFIG = 0x1B
ACCEL_CONFIG = 0x1C

INT_PIN_CFG = 0x37
INT_ENABLE = 0x38
INT_STATUS = 0x3A

ACCEL_XOUT_H = 0x3B


class MPU6500:

    def __init__(self, bus=1, address=MPU_ADDR):

        self.bus = SMBus(bus)
        self.address = address


    # ========================================================
    # I2C helpers
    # ========================================================

    def write_byte(self, register, value):

        self.bus.write_byte_data(
            self.address,
            register,
            value
        )


    def read_byte(self, register):

        return self.bus.read_byte_data(
            self.address,
            register
        )


    def read_bytes(self, register, length):

        return self.bus.read_i2c_block_data(
            self.address,
            register,
            length
        )


    # ========================================================
    # I2C helper with retry
    #
    # Added because I2C writes can transiently fail
    # (Errno 5, Input/output error) if something else on
    # the system (e.g. camera subsystem init) disturbs the
    # bus or power rail at the exact moment of the write.
    # ========================================================

    def _write_byte_retry(self, register, value, attempts=5, delay=0.3):

        last_err = None

        for attempt in range(1, attempts + 1):

            try:

                self.write_byte(register, value)

                return

            except OSError as e:

                last_err = e

                print(
                    f"  I2C write to 0x{register:02X} failed "
                    f"(attempt {attempt}/{attempts}): {e}"
                )

                time.sleep(delay)

        raise RuntimeError(
            f"MPU6500 not responding after {attempts} attempts: {last_err}"
        )


    # ========================================================
    # Initialization
    # ========================================================

    def initialize(self):

        # ----------------------------------------------------
        # Reset MPU-6500
        # ----------------------------------------------------

        self._write_byte_retry(
            PWR_MGMT_1,
            0x80
        )

        time.sleep(0.1)


        # ----------------------------------------------------
        # Wake up
        #
        # CLKSEL = 1
        # Use PLL as clock source
        # ----------------------------------------------------

        self._write_byte_retry(
            PWR_MGMT_1,
            0x01
        )

        time.sleep(0.1)


        # ----------------------------------------------------
        # Gyroscope DLPF
        #
        # CONFIG = 3
        # Gyro DLPF enabled
        # Internal gyro sample rate = 1 kHz
        # ----------------------------------------------------

        self._write_byte_retry(
            CONFIG,
            0x03
        )


        # ----------------------------------------------------
        # Sample rate
        #
        # 1000 / (1 + 9)
        # = 100 Hz
        # ----------------------------------------------------

        self._write_byte_retry(
            SMPLRT_DIV,
            9
        )


        # ----------------------------------------------------
        # Gyroscope
        #
        # FS_SEL = 0
        # ±250 degrees/sec
        # ----------------------------------------------------

        self._write_byte_retry(
            GYRO_CONFIG,
            0x00
        )


        # ----------------------------------------------------
        # Accelerometer
        #
        # AFS_SEL = 0
        # ±2g
        # ----------------------------------------------------

        self._write_byte_retry(
            ACCEL_CONFIG,
            0x00
        )


        # ----------------------------------------------------
        # Interrupt configuration
        #
        # INT_PIN_CFG = 0x20
        #
        # LATCH_INT_EN = 1
        # Interrupt remains HIGH until INT_STATUS is read.
        #
        # This is important because the Raspberry Pi GPIO
        # callback needs to reliably detect the event.
        # ----------------------------------------------------

        self._write_byte_retry(
            INT_PIN_CFG,
            0x20
        )


        # ----------------------------------------------------
        # Enable DATA_READY interrupt
        # ----------------------------------------------------

        self._write_byte_retry(
            INT_ENABLE,
            0x01
        )


        time.sleep(0.1)


    # ========================================================
    # WHO_AM_I
    # ========================================================

    def who_am_i(self):

        return self.read_byte(
            WHO_AM_I
        )


    # ========================================================
    # Interrupt status
    # ========================================================

    def read_int_status(self):

        return self.read_byte(
            INT_STATUS
        )


    # ========================================================
    # Convert signed 16-bit value
    # ========================================================

    @staticmethod
    def int16(high, low):

        value = (
            (high << 8) |
            low
        )

        if value & 0x8000:

            value -= 65536

        return value


    # ========================================================
    # Read accelerometer + temperature + gyroscope
    # ========================================================

    def read(self):

        data = self.read_bytes(
            ACCEL_XOUT_H,
            14
        )


        # ----------------------------------------------------
        # Accelerometer
        # ----------------------------------------------------

        ax = self.int16(
            data[0],
            data[1]
        )

        ay = self.int16(
            data[2],
            data[3]
        )

        az = self.int16(
            data[4],
            data[5]
        )


        # ----------------------------------------------------
        # Temperature
        # ----------------------------------------------------

        temperature = self.int16(
            data[6],
            data[7]
        )


        # ----------------------------------------------------
        # Gyroscope
        # ----------------------------------------------------

        gx = self.int16(
            data[8],
            data[9]
        )

        gy = self.int16(
            data[10],
            data[11]
        )

        gz = self.int16(
            data[12],
            data[13]
        )


        return {

            "ax": ax,
            "ay": ay,
            "az": az,

            "temperature": temperature,

            "gx": gx,
            "gy": gy,
            "gz": gz
        }


    # ========================================================
    # Close
    # ========================================================

    def close(self):

        self.bus.close()