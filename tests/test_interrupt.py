import time
import lgpio

from mpu6500 import MPU6500


# Raspberry Pi GPIO connected to MPU-6500 INT
IMU_INT_GPIO = 17


# -----------------------------
# Initialize MPU-6500
# -----------------------------

imu = MPU6500()
imu.initialize()


# -----------------------------
# Open GPIO chip
# -----------------------------

chip = lgpio.gpiochip_open(0)


# GPIO17 is an input.
# MPU INT is configured as active-high.
lgpio.gpio_claim_input(
    chip,
    IMU_INT_GPIO,
    lgpio.SET_PULL_DOWN
)


# -----------------------------
# Variables
# -----------------------------

count = 0
timestamps = []


# -----------------------------
# Interrupt callback
# -----------------------------

def imu_interrupt(chip, gpio, level, timestamp):

    global count

    # We only care about rising edges
    if level != 1:
        return

    count += 1

    # lgpio gives this timestamp in nanoseconds
    timestamps.append(timestamp)


# Register callback for rising edge
callback = lgpio.callback(
    chip,
    IMU_INT_GPIO,
    lgpio.RISING_EDGE,
    imu_interrupt
)


# -----------------------------
# Run test
# -----------------------------

print()
print("==============================")
print("MPU-6500 INTERRUPT TEST")
print("==============================")
print()
print("GPIO:", IMU_INT_GPIO)
print("Running for 10 seconds...")
print()


start = time.monotonic()

try:

    while time.monotonic() - start < 10:
        time.sleep(0.1)

finally:

    callback.cancel()
    lgpio.gpiochip_close(chip)
    imu.close()


elapsed = time.monotonic() - start


# -----------------------------
# Results
# -----------------------------

print()
print("==============================")
print("RESULTS")
print("==============================")

print(
    f"Interrupts: {count}"
)

print(
    f"Elapsed:    {elapsed:.3f} s"
)

print(
    f"Rate:       {count / elapsed:.3f} Hz"
)


# -----------------------------
# Calculate period
# -----------------------------

if len(timestamps) > 1:

    intervals = []

    for i in range(1, len(timestamps)):

        dt = (
            timestamps[i] -
            timestamps[i - 1]
        ) / 1e9

        intervals.append(dt)


    mean_period = (
        sum(intervals) /
        len(intervals)
    )


    mean_rate = 1.0 / mean_period


    print(
        f"Mean period: "
        f"{mean_period * 1000:.6f} ms"
    )

    print(
        f"Mean rate:   "
        f"{mean_rate:.3f} Hz"
    )

    print(
        f"Min period:  "
        f"{min(intervals) * 1000:.6f} ms"
    )

    print(
        f"Max period:  "
        f"{max(intervals) * 1000:.6f} ms"
    )

else:

    print()
    print("Not enough interrupts received.")