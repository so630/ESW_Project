import threading
import time

import serial
import pynmea2


# ============================================================
# GPSReader
#
# Reads NMEA sentences from a serial GPS module (e.g. the
# GY-GPS6MV2 / NEO-6M) on a background thread and records
# ground-speed velocity samples, timestamped with
# time.monotonic_ns() so they share the same clock as the
# IMU interrupt callback and the camera capture loop.
#
# We deliberately timestamp using OUR clock at the moment the
# line is received, not the GPS's own UTC time field. The
# GPS's UTC timestamp has only 1-second (or sometimes
# fractional but still coarse) resolution and is on a
# completely different clock than monotonic_ns(), so it can't
# be used to align against the IMU/camera streams. What we
# care about here is "when, on our shared timeline, did we
# learn this velocity value" -- not "what time was it in UTC".
# ============================================================


class GPSReader:

    def __init__(self, port="/dev/serial0", baudrate=9600, timeout=1.0):

        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout

        self.serial_conn = None

        self.thread = None
        self.running = False

        self.samples = []
        self.lock = threading.Lock()

        # Counts every line read, valid or not, so you can
        # tell "no GPS data at all" apart from "data arriving
        # but no fix yet" when debugging.
        self.lines_read = 0
        self.sentences_parsed = 0


    # ========================================================
    # Start / stop
    # ========================================================

    def start(self):

        self.serial_conn = serial.Serial(

            self.port,

            baudrate=self.baudrate,

            timeout=self.timeout
        )

        self.running = True

        self.thread = threading.Thread(

            target=self._read_loop,

            daemon=True
        )

        self.thread.start()


    def stop(self):

        self.running = False

        if self.thread is not None:

            self.thread.join(timeout=2)

        if self.serial_conn is not None:

            self.serial_conn.close()


    # ========================================================
    # Background read loop
    # ========================================================

    def _read_loop(self):

        while self.running:

            try:

                raw_line = self.serial_conn.readline()

                # Timestamp as close as possible to when the
                # data actually arrived, before any parsing
                # work happens.
                timestamp_ns = time.monotonic_ns()


                if not raw_line:

                    # readline() timed out with no data.
                    continue


                self.lines_read += 1


                try:

                    line = raw_line.decode(
                        "ascii",
                        errors="replace"
                    ).strip()

                except Exception:

                    continue


                if not line.startswith("$"):

                    continue


                try:

                    msg = pynmea2.parse(line)

                except pynmea2.ParseError:

                    continue


                self.sentences_parsed += 1


                self._handle_message(
                    msg,
                    timestamp_ns
                )


            except serial.SerialException as e:

                print(f"GPS serial error: {e}")

                time.sleep(0.5)


            except Exception:

                import traceback

                traceback.print_exc()


    # ========================================================
    # Parse relevant NMEA sentence types
    # ========================================================

    def _handle_message(self, msg, timestamp_ns):

        sentence_type = getattr(
            msg,
            "sentence_type",
            None
        )


        sample = {

            "timestamp_ns": timestamp_ns,

            "sentence_type": sentence_type,

            "fix_valid": None,

            "latitude": None,
            "longitude": None,

            "speed_knots": None,
            "speed_kmh": None,

            "num_satellites": None
        }


        # ----------------------------------------------------
        # RMC: has fix status + speed over ground (knots)
        # ----------------------------------------------------

        if sentence_type == "RMC":

            sample["fix_valid"] = (
                msg.status == "A"
            )

            sample["latitude"] = msg.latitude
            sample["longitude"] = msg.longitude

            if msg.spd_over_grnd is not None:

                knots = float(
                    msg.spd_over_grnd
                )

                sample["speed_knots"] = knots

                sample["speed_kmh"] = (
                    knots * 1.852
                )


        # ----------------------------------------------------
        # VTG: dedicated ground-speed sentence, both units
        # ----------------------------------------------------

        elif sentence_type == "VTG":

            if msg.spd_over_grnd_kmph is not None:

                sample["speed_kmh"] = float(
                    msg.spd_over_grnd_kmph
                )

            if msg.spd_over_grnd_kts is not None:

                sample["speed_knots"] = float(
                    msg.spd_over_grnd_kts
                )


        # ----------------------------------------------------
        # GGA: position + satellite count (no speed)
        # ----------------------------------------------------

        elif sentence_type == "GGA":

            sample["latitude"] = msg.latitude
            sample["longitude"] = msg.longitude

            try:

                sample["num_satellites"] = int(
                    msg.num_sats
                )

            except (TypeError, ValueError):

                pass


        else:

            # Ignore sentence types we don't care about
            # (GSA, GSV, GLL, etc).
            return


        with self.lock:

            self.samples.append(
                sample
            )


    # ========================================================
    # Accessors
    # ========================================================

    def get_samples(self):

        with self.lock:

            return list(
                self.samples
            )