import time

from mpu6500 import MPU6500


imu = MPU6500()

imu.initialize()

print(
    "WHO_AM_I:",
    hex(imu.who_am_i())
)

print("MPU-6500 initialized")
print("Reading sensor...")
print("Press Ctrl+C to stop")


try:

    while True:

        data = imu.read()

        print(
            f"ACC: "
            f"{data['ax']:6d} "
            f"{data['ay']:6d} "
            f"{data['az']:6d}   "

            f"GYRO: "
            f"{data['gx']:6d} "
            f"{data['gy']:6d} "
            f"{data['gz']:6d}"
        )

        time.sleep(0.1)


except KeyboardInterrupt:

    pass


finally:

    imu.close()