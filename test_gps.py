import time

from gps_reader import GPSReader


# ============================================================
# Configuration
# ============================================================

GPS_PORT = "/dev/serial0"
GPS_BAUDRATE = 9600

RUN_TIME = 60


# ============================================================
# Main
# ============================================================

def main():

    print()
    print("==============================")
    print("GPS CONNECTION TEST")
    print("==============================")
    print()

    print(f"Port:     {GPS_PORT}")
    print(f"Baudrate: {GPS_BAUDRATE}")
    print(f"Runtime:  {RUN_TIME} seconds")
    print()

    print("Starting GPS reader...")
    print()


    gps = GPSReader(
        port=GPS_PORT,
        baudrate=GPS_BAUDRATE,
        timeout=1.0
    )


    try:

        gps.start()

        print("GPS reader started.")
        print()
        print("Waiting for GPS data...")
        print("")


        start_time = time.monotonic()

        last_lines = 0
        last_parsed = 0
        last_fix = False


        while time.monotonic() - start_time < RUN_TIME:

            time.sleep(1)


            # =================================================
            # Check whether ANY serial data is arriving
            # =================================================

            if gps.lines_read == 0:

                print(
                    "STATUS: NO GPS DATA"
                )

            else:

                if last_lines == 0:

                    print(
                        "STATUS: GPS DATA RECEIVED"
                    )


            last_lines = gps.lines_read


            # =================================================
            # Check satellite fix
            # =================================================

            samples = gps.get_samples()


            current_fix = False

            satellite_count = None
            latitude = None
            longitude = None
            speed_kmh = None


            # Find most recent useful information

            for sample in reversed(samples):

                if (
                    sample["fix_valid"]
                    is True
                ):

                    current_fix = True

                    break


            # Get latest satellite count

            for sample in reversed(samples):

                if sample["num_satellites"] is not None:

                    satellite_count = (
                        sample["num_satellites"]
                    )

                    break


            # Get latest position

            for sample in reversed(samples):

                if sample["latitude"] is not None:

                    latitude = sample["latitude"]
                    longitude = sample["longitude"]

                    break


            # Get latest speed

            for sample in reversed(samples):

                if sample["speed_kmh"] is not None:

                    speed_kmh = sample["speed_kmh"]

                    break


            # =================================================
            # Fix acquired
            # =================================================

            if current_fix and not last_fix:

                print()
                print(
                    "================================"
                )
                print(
                    "SATELLITE FIX ACQUIRED!"
                )
                print(
                    "================================"
                )
                print()


            last_fix = current_fix


            # =================================================
            # Print status
            # =================================================

            print(
                f"Lines received:    "
                f"{gps.lines_read}"
            )

            print(
                f"Sentences parsed:  "
                f"{gps.sentences_parsed}"
            )

            print(
                f"Samples stored:    "
                f"{len(samples)}"
            )


            if current_fix:

                print(
                    "GPS FIX:            YES"
                )

            else:

                print(
                    "GPS FIX:            NO"
                )


            if satellite_count is not None:

                print(
                    f"Satellites:         "
                    f"{satellite_count}"
                )

            else:

                print(
                    "Satellites:         N/A"
                )


            if latitude is not None:

                print(
                    f"Latitude:           "
                    f"{latitude}"
                )

                print(
                    f"Longitude:          "
                    f"{longitude}"
                )


            if speed_kmh is not None:

                print(
                    f"Speed:              "
                    f"{speed_kmh:.2f} km/h"
                )

            else:

                print(
                    "Speed:              N/A"
                )


            print(
                "--------------------------------"
            )


    except KeyboardInterrupt:

        print()
        print("Stopped by user.")


    finally:

        print()
        print("Stopping GPS reader...")

        gps.stop()

        print("GPS reader stopped.")

        print()
        print("==============================")
        print("FINAL RESULT")
        print("==============================")
        print()

        print(
            f"Lines received:   "
            f"{gps.lines_read}"
        )

        print(
            f"Sentences parsed: "
            f"{gps.sentences_parsed}"
        )

        print(
            f"Samples stored:   "
            f"{len(gps.get_samples())}"
        )

        print()

        if gps.lines_read == 0:

            print(
                "RESULT: GPS MODULE NOT RESPONDING"
            )

            print(
                "Check power, TX/RX wiring, "
                "serial configuration and baudrate."
            )

        elif not gps.fix_acquired:

            print(
                "RESULT: GPS MODULE CONNECTED"
            )

            print(
                "RESULT: NO SATELLITE FIX"
            )

            print(
                "Move the antenna outdoors / "
                "to an area with a clear view of the sky."
            )

        else:

            print(
                "RESULT: GPS MODULE CONNECTED"
            )

            print(
                "RESULT: SATELLITE FIX ACQUIRED"
            )

        print()


if __name__ == "__main__":

    main()