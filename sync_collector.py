import csv
import time
import threading
import traceback
from pathlib import Path

import lgpio

from imu_calibration import CalibratedMPU as MPU6500
from gps_reader import GPSReader


# ============================================================
# Configuration
# ============================================================

IMU_INT_GPIO = 17

CAMERA_FPS = 30

# GPS fixes take real time to acquire (30-60+ seconds cold
# start, outdoors, with clear sky view). A 5-second window is
# fine for proving IMU/camera sync but will very likely show
# zero GPS velocity samples. Set this longer for an actual
# outdoor GPS demo.
DURATION_SECONDS = 60

GPS_PORT = "/dev/serial0"
GPS_BAUDRATE = 9600

OUTPUT_DIR = Path("sync_data")


# ============================================================
# Shared state
# ============================================================

running = True

imu_samples = []

imu_lock = threading.Lock()

# Counts every time the IMU callback is invoked, regardless of
# whether the sample was accepted. Used to distinguish
# "callback never fired" from "callback fired but errored".
call_count = 0


# ============================================================
# IMU
# ============================================================

imu = MPU6500()

gpio_chip = None
imu_callback = None


def imu_interrupt(
    chip,
    gpio,
    level,
    timestamp
):

    global running
    global call_count

    call_count += 1

    try:

        # Ignore interrupts after collection stops
        if not running:
            return


        # Only respond to rising edge
        if level != 1:
            return


        # ----------------------------------------------------
        # Timestamp the DATA_READY interrupt
        #
        # monotonic_ns() is used because it provides a
        # monotonic clock suitable for comparing timestamps
        # across the IMU, camera, and GPS streams.
        # ----------------------------------------------------

        timestamp_ns = time.monotonic_ns()


        # ----------------------------------------------------
        # Read IMU
        # ----------------------------------------------------

        data = imu.read()


        # ----------------------------------------------------
        # Clear the latched interrupt
        #
        # Reading INT_STATUS causes GPIO17 to return LOW.
        # ----------------------------------------------------

        imu.read_int_status()


        # ----------------------------------------------------
        # Store sample
        # ----------------------------------------------------

        sample = {

            "timestamp_ns": timestamp_ns,

            "ax": data["ax"],
            "ay": data["ay"],
            "az": data["az"],

            "gx": data["gx"],
            "gy": data["gy"],
            "gz": data["gz"],

            "temperature": data["temperature"]
        }


        with imu_lock:

            imu_samples.append(
                sample
            )

    except Exception:

        # lgpio invokes this callback from a C thread via
        # ctypes. Exceptions raised here do NOT propagate to
        # the main thread and would otherwise be swallowed
        # silently, making IMU dropouts invisible. Print them
        # explicitly instead.
        print("Exception inside imu_interrupt:")
        traceback.print_exc()


# ============================================================
# Camera setup
#
# picamera2 is imported here, inside the function, rather
# than at module load time. Importing/constructing Picamera2
# touches camera subsystem resources immediately, and doing
# that before the IMU has finished I2C initialization has
# been observed to disturb the I2C bus enough to cause
# "OSError: [Errno 5] Input/output error" during
# imu.initialize(). Deferring the import ensures the IMU is
# fully initialized first.
# ============================================================

def setup_camera():

    from picamera2 import Picamera2

    camera = Picamera2()


    config = camera.create_video_configuration(

        main={
            "size": (
                1280,
                720
            ),

            "format": "RGB888"
        },

        controls={
            "FrameRate": CAMERA_FPS
        }
    )


    camera.configure(
        config
    )


    return camera


# ============================================================
# Main
# ============================================================

def main():

    global running
    global gpio_chip
    global imu_callback


    # ========================================================
    # Output directory
    # ========================================================

    OUTPUT_DIR.mkdir(
        parents=True,
        exist_ok=True
    )


    imu_csv_path = (
        OUTPUT_DIR /
        "imu.csv"
    )


    camera_timestamp_path = (
        OUTPUT_DIR /
        "camera_timestamps.csv"
    )


    gps_csv_path = (
        OUTPUT_DIR /
        "gps.csv"
    )


    video_path = (
        OUTPUT_DIR /
        "camera.h264"
    )


    # ========================================================
    # Start GPS reader FIRST
    #
    # GPS fix acquisition can take 30-60+ seconds. Starting
    # it before IMU/GPIO/camera setup gives it the maximum
    # possible warm-up time before the collection window
    # begins. GPS runs over UART, not I2C, so it does not
    # interfere with the MPU-6500 init sequence below.
    # ========================================================

    print(
        "Starting GPS reader..."
    )


    gps = GPSReader(
        port=GPS_PORT,
        baudrate=GPS_BAUDRATE
    )


    try:

        gps.start()

    except Exception as e:

        print(
            f"WARNING: could not open GPS serial port "
            f"({GPS_PORT}): {e}"
        )

        print(
            "Continuing without GPS. Check wiring and that "
            "/dev/serial0 is enabled (see setup notes)."
        )

        gps = None


    # ========================================================
    # Initialize MPU
    #
    # This happens BEFORE anything camera-related is
    # imported or constructed (see setup_camera() comment
    # above), and before GPIO is touched.
    # ========================================================

    print(
        "Initializing MPU-6500..."
    )


    imu.initialize()


    print(
        f"WHO_AM_I = "
        f"0x{imu.who_am_i():02X}"
    )


    # ========================================================
    # Initialize GPIO
    # ========================================================

    print(
        "Initializing GPIO17..."
    )


    gpio_chip = lgpio.gpiochip_open(
        0
    )


    # --------------------------------------------------------
    # Claim the line as an ALERT source (not a plain input).
    #
    # lgpio.callback() requires the line to have been claimed
    # via gpio_claim_alert(), which requests it as an edge-
    # event source at the kernel level.
    # --------------------------------------------------------

    lgpio.gpio_claim_alert(

        gpio_chip,

        IMU_INT_GPIO,

        lgpio.RISING_EDGE,

        lgpio.SET_PULL_DOWN
    )


    # ========================================================
    # Register callback BEFORE clearing interrupt
    # ========================================================

    print(
        "Registering GPIO17 interrupt callback..."
    )


    imu_callback = lgpio.callback(

        gpio_chip,

        IMU_INT_GPIO,

        lgpio.RISING_EDGE,

        imu_interrupt
    )


    # ========================================================
    # Clear any interrupt generated during initialization
    # ========================================================

    print(
        "Clearing pending IMU interrupt..."
    )


    status = imu.read_int_status()


    print(
        f"Initial INT_STATUS = "
        f"0x{status:02X}"
    )


    # ========================================================
    # Initialize camera
    #
    # Deliberately done AFTER the IMU and GPIO/interrupt are
    # fully set up, so any bus/power disturbance from camera
    # subsystem init cannot interfere with IMU initialization.
    # ========================================================

    print(
        "Initializing camera..."
    )


    camera = setup_camera()


    from picamera2.encoders import H264Encoder
    from picamera2.outputs import FfmpegOutput


    # ========================================================
    # Camera timestamp CSV
    # ========================================================

    camera_file = open(

        camera_timestamp_path,

        "w",

        newline=""
    )


    camera_writer = csv.writer(
        camera_file
    )


    camera_writer.writerow([

        "frame_number",

        "timestamp_ns"
    ])


    # ========================================================
    # Video encoder
    # ========================================================

    encoder = H264Encoder(

        bitrate=10_000_000
    )


    output = FfmpegOutput(

        str(video_path)
    )


    # ========================================================
    # Start
    # ========================================================

    print()
    print(
        "=============================="
    )
    print(
        "SYNCHRONIZED DATA COLLECTION"
    )
    print(
        "=============================="
    )
    print()


    print(
        f"Camera: {CAMERA_FPS} FPS"
    )


    print(
        "IMU:    ~100 Hz"
    )


    print(
        "GPS:    ~1 Hz (fix-dependent)"
    )


    print(
        f"Duration: "
        f"{DURATION_SECONDS} seconds"
    )


    print()


    # ========================================================
    # Start recording
    # ========================================================

    camera.start_recording(

        encoder,

        output
    )


    # Allow camera to stabilize

    time.sleep(
        1
    )


    # ========================================================
    # Start collection timer
    # ========================================================

    start_time = (
        time.monotonic_ns()
    )


    frame_count = 0


    print(
        "Recording..."
    )

    print()


    # ========================================================
    # Collection loop
    # ========================================================

    try:

        while True:

            now = (
                time.monotonic_ns()
            )


            elapsed = (

                now -
                start_time

            ) / 1e9


            if elapsed >= DURATION_SECONDS:

                break


            # ------------------------------------------------
            # Capture frame
            # ------------------------------------------------

            camera.capture_array()


            # ------------------------------------------------
            # Timestamp frame arrival
            # ------------------------------------------------

            timestamp_ns = (
                time.monotonic_ns()
            )


            frame_count += 1


            # ------------------------------------------------
            # Save camera timestamp
            # ------------------------------------------------

            camera_writer.writerow([

                frame_count,

                timestamp_ns
            ])


            camera_file.flush()


    except KeyboardInterrupt:

        print()
        print(
            "Stopping early..."
        )


    finally:

        running = False


        # ----------------------------------------------------
        # Stop camera
        # ----------------------------------------------------

        print(
            "Stopping camera..."
        )


        camera.stop_recording()

        camera.close()


        # ----------------------------------------------------
        # Stop GPIO callback
        # ----------------------------------------------------

        print(
            "Stopping IMU interrupt..."
        )


        if imu_callback is not None:

            imu_callback.cancel()


        # ----------------------------------------------------
        # Close GPIO
        # ----------------------------------------------------

        if gpio_chip is not None:

            lgpio.gpiochip_close(
                gpio_chip
            )


        # ----------------------------------------------------
        # Close IMU
        # ----------------------------------------------------

        imu.close()


        # ----------------------------------------------------
        # Stop GPS
        # ----------------------------------------------------

        print(
            "Stopping GPS reader..."
        )


        if gps is not None:

            gps.stop()


        # ----------------------------------------------------
        # Close camera timestamp file
        # ----------------------------------------------------

        camera_file.close()


    # ========================================================
    # Write IMU CSV
    # ========================================================

    print(
        "Writing IMU CSV..."
    )


    with open(

        imu_csv_path,

        "w",

        newline=""
    ) as f:

        writer = csv.writer(
            f
        )


        writer.writerow([

            "timestamp_ns",

            "ax",
            "ay",
            "az",

            "gx",
            "gy",
            "gz",

            "temperature"
        ])


        with imu_lock:

            for sample in imu_samples:

                writer.writerow([

                    sample[
                        "timestamp_ns"
                    ],

                    sample["ax"],
                    sample["ay"],
                    sample["az"],

                    sample["gx"],
                    sample["gy"],
                    sample["gz"],

                    sample[
                        "temperature"
                    ]
                ])


    # ========================================================
    # Write GPS CSV
    # ========================================================

    gps_samples = (
        gps.get_samples()
        if gps is not None
        else []
    )


    print(
        "Writing GPS CSV..."
    )


    with open(

        gps_csv_path,

        "w",

        newline=""
    ) as f:

        writer = csv.writer(
            f
        )


        writer.writerow([

            "timestamp_ns",

            "sentence_type",

            "fix_valid",

            "latitude",
            "longitude",

            "speed_knots",
            "speed_kmh",

            "num_satellites"
        ])


        for sample in gps_samples:

            writer.writerow([

                sample["timestamp_ns"],

                sample["sentence_type"],

                sample["fix_valid"],

                sample["latitude"],
                sample["longitude"],

                sample["speed_knots"],
                sample["speed_kmh"],

                sample["num_satellites"]
            ])


    # ========================================================
    # Statistics
    # ========================================================

    total_imu = len(
        imu_samples
    )

    total_gps = len(
        gps_samples
    )

    gps_with_speed = sum(

        1
        for s in gps_samples

        if s["speed_kmh"] is not None
    )


    print()
    print(
        "=============================="
    )
    print(
        "COLLECTION COMPLETE"
    )
    print(
        "=============================="
    )
    print()


    print(
        f"Camera frames: "
        f"{frame_count}"
    )


    print(
        f"IMU samples:   "
        f"{total_imu}"
    )


    print(
        f"Callback invocations: "
        f"{call_count}"
    )


    print(
        f"GPS sentences parsed: "
        f"{total_gps} "
        f"({gps_with_speed} with a speed value)"
    )


    if gps is not None:

        print(

            f"GPS raw lines read:   "
            f"{gps.lines_read} "
            f"(parsed: {gps.sentences_parsed})"
        )


        if total_gps == 0:

            print(

                "  NOTE: no NMEA sentences were parsed. "
                "Check wiring (TX -> Pi RXD, pin 10), that "
                "/dev/serial0 is enabled, and that the "
                "module has power (its LED should be "
                "blinking once it has a fix)."
            )

        elif gps_with_speed == 0:

            print(

                "  NOTE: sentences were parsed but none had "
                "a speed value yet -- this usually means no "
                "GPS fix has been acquired. Try again "
                "outdoors with a clear view of the sky, and "
                "allow more time (cold start can take "
                "30-60+ seconds)."
            )


    # ========================================================
    # Calculate IMU frequency
    # ========================================================

    if total_imu > 1:

        first = imu_samples[0][
            "timestamp_ns"
        ]


        last = imu_samples[-1][
            "timestamp_ns"
        ]


        duration = (

            last -
            first

        ) / 1e9


        rate = (

            total_imu - 1
        ) / duration


        print(

            f"Measured IMU rate: "
            f"{rate:.3f} Hz"
        )


    print()


    print(
        f"Video: "
        f"{video_path}"
    )


    print(
        f"IMU: "
        f"{imu_csv_path}"
    )


    print(
        f"Camera timestamps: "
        f"{camera_timestamp_path}"
    )


    print(
        f"GPS: "
        f"{gps_csv_path}"
    )


# ============================================================
# Entry point
# ============================================================

if __name__ == "__main__":

    main()