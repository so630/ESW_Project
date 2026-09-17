import time

from mpu6500 import MPU6500


# -----------------------------------------
# Initialize MPU
# -----------------------------------------

imu = MPU6500()

imu.initialize()


# -----------------------------------------
# Print device information
# -----------------------------------------

print()
print("==============================")
print("MPU-6500 INTERRUPT STATUS TEST")
print("==============================")
print()

who_am_i = imu.who_am_i()

print(
    f"WHO_AM_I   = 0x{who_am_i:02X}"
)

print()


# -----------------------------------------
# Read back configuration registers
# -----------------------------------------

print("Register configuration:")
print("-----------------------")


registers = [
    ("SMPLRT_DIV", 0x19),
    ("CONFIG", 0x1A),
    ("GYRO_CONFIG", 0x1B),
    ("ACCEL_CONFIG", 0x1C),
    ("INT_PIN_CFG", 0x37),
    ("INT_ENABLE", 0x38),
    ("INT_STATUS", 0x3A),
]


for name, address in registers:

    value = imu.bus.read_byte_data(
        imu.address,
        address
    )

    print(
        f"{name:12s} = 0x{value:02X}"
    )


print()
print("==============================")
print("Monitoring INT_STATUS")
print("==============================")
print()
print("Press Ctrl+C to stop.")
print()


# -----------------------------------------
# Monitor INT_STATUS
# -----------------------------------------

try:

    while True:

        status = imu.read_int_status()

        data_ready = bool(
            status & 0x01
        )

        print(
            f"INT_STATUS = 0x{status:02X}   "
            f"DATA_READY = {data_ready}"
        )

        time.sleep(0.1)


except KeyboardInterrupt:

    print()
    print("Stopping...")


finally:

    imu.close()