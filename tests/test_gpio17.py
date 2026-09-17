import time
import lgpio


GPIO = 17


chip = lgpio.gpiochip_open(0)


lgpio.gpio_claim_input(
    chip,
    GPIO,
    lgpio.SET_PULL_DOWN
)


count = 0
timestamps = []


def callback(
    chip,
    gpio,
    level,
    timestamp
):

    global count

    if level == 1:

        count += 1

        timestamps.append(timestamp)

        print(
            f"RISING EDGE #{count} "
            f"timestamp={timestamp}"
        )


cb = lgpio.callback(
    chip,
    GPIO,
    lgpio.RISING_EDGE,
    callback
)


print()
print("==============================")
print("GPIO17 EDGE TEST")
print("==============================")
print()
print("Watching GPIO17 for 10 seconds...")
print()


start = time.monotonic()


try:

    while time.monotonic() - start < 10:

        time.sleep(0.1)


except KeyboardInterrupt:

    pass


finally:

    cb.cancel()

    lgpio.gpiochip_close(chip)


print()
print("==============================")
print("RESULT")
print("==============================")
print()

print(
    f"Rising edges detected: {count}"
)