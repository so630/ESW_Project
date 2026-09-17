import time
import lgpio

from mpu6500 import MPU6500


GPIO = 17

imu = MPU6500()
imu.initialize()

chip = lgpio.gpiochip_open(0)

lgpio.gpio_claim_input(
    chip,
    GPIO,
    lgpio.SET_PULL_DOWN
)

print()
print("==============================")
print("LATCHED INTERRUPT TEST")
print("==============================")
print()

time.sleep(1)

level = lgpio.gpio_read(chip, GPIO)

print(f"GPIO17 level: {level}")

print()
print("Reading INT_STATUS...")
print()

status = imu.read_int_status()

print(
    f"INT_STATUS = 0x{status:02X}"
)

print(
    f"DATA_READY = {bool(status & 0x01)}"
)

print()
print("GPIO17 after reading INT_STATUS:")

level = lgpio.gpio_read(chip, GPIO)

print(f"GPIO17 level: {level}")

print()

time.sleep(1)

print("Checking again:")

level = lgpio.gpio_read(chip, GPIO)

print(f"GPIO17 level: {level}")

print()

lgpio.gpiochip_close(chip)
imu.close()