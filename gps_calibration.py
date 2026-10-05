"""GPS-only startup calibration and per-reading speed correction.

No IMU code; no dependency on gps_reader.py.

Existing collector integration: replace ONLY its GPS import with
    from gps_calibration import CalibratedGPSReader as GPSReader
Its existing gps.start() calibrates once, then automatically applies
apply_gps_calibration() to every incoming RMC measurement.

Explicit API:
    gps = CalibratedGPSReader()
    cal = initialize_gps_calibration(gps)  # once before collection; starts reader
    samples = gps.get_samples()          # each sample already corrected
    gps.stop()

If you already have decoded readings, call apply_gps_calibration(sample, cal)
on each one. Supply fix_valid and quality_ok booleans and speed_kmh, or
speed_kmh_raw. It returns a new dict and preserves raw speed separately.

This measures stationary speed noise and applies a deadband; it cannot remove
absolute GPS position error. Real slow movement can be suppressed: retain raw
speed. timestamp_ns is Pi monotonic line-arrival time, not a precisely aligned
sensor measurement time. GPS UTC is retained separately.

Run standalone: python3 gps_calibration.py
Wait for startup calibration, print 10 seconds of measurements, then exit.
"""

from datetime import datetime, timezone
import json
import math
from pathlib import Path
import statistics
import threading
import time

import pynmea2
import serial

CAL_PATH = Path(__file__).resolve().with_name("gps_calibration.json")


def finite_float(value):
    try:
        number = float(value)
        return number if math.isfinite(number) else None
    except (ValueError, TypeError):
        return None


def percentile(values, fraction):
    ordered = sorted(values)
    index = (len(ordered) - 1) * fraction
    low = int(index)
    high = min(low + 1, len(ordered) - 1)
    return ordered[low] + (ordered[high] - ordered[low]) * (index - low)


class GPSCalibrationError(RuntimeError):
    pass


def apply_gps_calibration(sample, calibration):
    """Apply stored speed threshold to ONE measurement without recalibrating.

    Coordinates/timestamps are preserved. Invalid fix => unavailable speed.
    If quality is poor/unknown, retain valid raw speed instead of zeroing it.
    Repeated application is safe because the original raw speed is retained.
    """
    result = dict(sample)
    raw = finite_float(sample.get("speed_kmh_raw", sample.get("speed_kmh")))
    if not sample.get("fix_valid", False) or raw is None or raw < 0:
        raw = None
    threshold = None
    if calibration is not None:
        threshold = finite_float(calibration.get("stationary_threshold_kmh"))
        if threshold is None or not 0 <= threshold <= 1.5:
            raise GPSCalibrationError("Invalid GPS stationary threshold")
    filtered = raw
    if (raw is not None and threshold is not None
            and sample.get("quality_ok", False) and raw <= threshold):
        filtered = 0.0
    result.update(speed_kmh_raw=raw,
                  speed_knots_raw=raw / 1.852 if raw is not None else None,
                  speed_kmh=filtered,
                  speed_knots=filtered / 1.852 if filtered is not None else None,
                  stationary_threshold_kmh=threshold)
    return result


def initialize_gps_calibration(gps):
    """Calibrate once, start the GPS reader, and return its coefficients.

    Call instead of gps.start(), never in addition to it. Each subsequent
    measurement calls apply_gps_calibration internally using these coefficients.
    """
    gps.start()
    return dict(gps.calibration)


class CalibratedGPSReader:
    def __init__(self, port="/dev/serial0", baudrate=9600, timeout=0.5,
                 fix_timeout=120, settle_seconds=10, static_seconds=60,
                 calibration_file=CAL_PATH):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_conn = None
        self.thread = None
        self.running = False
        self.samples = []
        self.lock = threading.Lock()
        self.lines_read = 0
        self.sentences_parsed = 0
        if fix_timeout <= 0 or settle_seconds < 0 or static_seconds < 30:
            raise ValueError("Use positive fix_timeout and at least 30 s static window")
        self.fix_timeout = fix_timeout
        self.settle_seconds = settle_seconds
        self.static_seconds = static_seconds
        self.calibration_file = Path(calibration_file)
        self.calibration = None
        self._gga = None
        self._last_epoch = None
        self._stop_event = threading.Event()

    def _next_message(self):
        raw = self.serial_conn.readline()
        timestamp_ns = time.monotonic_ns()
        if not raw:
            return None, timestamp_ns
        self.lines_read += 1
        try:
            message = pynmea2.parse(raw.decode("ascii").strip(), check=True)
        except (UnicodeError, pynmea2.ParseError, ValueError):
            return None, timestamp_ns
        self.sentences_parsed += 1
        return message, timestamp_ns

    def _decode(self, msg, timestamp_ns):
        typ = getattr(msg, "sentence_type", "")
        if typ == "GGA":
            quality = finite_float(msg.gps_qual)
            sats = finite_float(msg.num_sats)
            hdop = finite_float(msg.horizontal_dil)
            self._gga = {"timestamp_ns": timestamp_ns,
                         "valid": quality is not None and quality > 0,
                         "num_satellites": int(sats) if sats is not None else None,
                         "hdop": hdop}
            return None
        # One speed observation per epoch; do not duplicate it with VTG.
        if typ != "RMC":
            return None
        valid = msg.status == "A" and getattr(msg, "mode_indicator", "") != "N"
        try:
            utc = datetime.combine(msg.datestamp, msg.timestamp,
                                   tzinfo=timezone.utc).isoformat()
        except (ValueError, TypeError, AttributeError):
            utc = None
        if utc is not None and utc == self._last_epoch:
            return None
        self._last_epoch = utc
        lat = lon = None
        if valid:
            try:
                lat, lon = finite_float(msg.latitude), finite_float(msg.longitude)
            except (ValueError, TypeError):
                valid = False
            if lat is None or lon is None or not -90 <= lat <= 90 or not -180 <= lon <= 180:
                valid = False
        knots = finite_float(msg.spd_over_grnd) if valid else None
        if knots is not None and knots < 0:
            knots = None
        raw_speed = knots * 1.852 if knots is not None else None
        gga = self._gga
        if gga is None or not 0 <= timestamp_ns - gga["timestamp_ns"] <= 2_500_000_000:
            gga = {}
        quality_ok = bool(valid and utc is not None and gga.get("valid")
                          and gga.get("num_satellites") is not None
                          and gga["num_satellites"] >= 4
                          and gga.get("hdop") is not None and 0 < gga["hdop"] <= 3)
        sample = {"timestamp_ns": timestamp_ns, "sentence_type": "RMC",
                  "fix_valid": valid, "latitude": lat if valid else None,
                  "longitude": lon if valid else None,
                  "speed_kmh_raw": raw_speed,
                  "num_satellites": gga.get("num_satellites"),
                  "hdop": gga.get("hdop"), "gps_utc": utc,
                  "quality_ok": quality_ok}
        return apply_gps_calibration(sample, self.calibration)

    def _next_sample(self):
        msg, timestamp_ns = self._next_message()
        return self._decode(msg, timestamp_ns) if msg is not None else None

    def _calibrate(self):
        print("GPS: keep receiver STILL outdoors with a clear sky view.", flush=True)
        print(f"GPS: waiting up to {self.fix_timeout:g} s for a valid RMC + GGA fix...", flush=True)
        deadline = time.monotonic() + self.fix_timeout
        next_report = time.monotonic() + 10
        while True:
            sample = self._next_sample()
            if sample and sample["quality_ok"] and sample["speed_kmh_raw"] is not None:
                break
            if time.monotonic() >= deadline:
                raise GPSCalibrationError("No good GPS fix; check sky view, UART and baud rate")
            if time.monotonic() >= next_report:
                print("GPS: still waiting for satellite fix...", flush=True)
                next_report += 10
        print(f"GPS: fix acquired; settling for {self.settle_seconds:g} s...", flush=True)
        deadline = time.monotonic() + self.settle_seconds
        while time.monotonic() < deadline:
            self._next_sample()  # continuously drain serial stream
        print(f"GPS: measuring stationary noise for {self.static_seconds:g} s...", flush=True)
        window_start = time.monotonic()
        next_report = window_start + 10
        fixes, valid_epochs = [], 0
        while time.monotonic() - window_start < self.static_seconds:
            sample = self._next_sample()
            if sample is not None:
                valid_epochs += 1
                if sample["quality_ok"] and sample["speed_kmh_raw"] is not None:
                    fixes.append(sample)
            if time.monotonic() >= next_report:
                print(f"GPS: {time.monotonic()-window_start:.0f}/{self.static_seconds:g} s; "
                      f"{len(fixes)} good fixes", flush=True)
                next_report += 10
        if len(fixes) < 20 or len(fixes) < 0.8 * valid_epochs:
            raise GPSCalibrationError("Too few good fixes or poor/intermittent quality; retry outdoors")
        speeds = [s["speed_kmh_raw"] for s in fixes]
        mean = statistics.fmean(speeds)
        std = statistics.pstdev(speeds)
        threshold = max(0.5, mean + 3 * std)
        # Reject noisy/moving windows rather than masking motion with a huge deadband.
        if mean > 1.0 or threshold > 1.5:
            raise GPSCalibrationError("Excessive stationary speed/noise; hold still and retry")
        lat0 = statistics.fmean(s["latitude"] for s in fixes)
        # Local unwrapping also works if a short window crosses +/-180 longitude.
        anchor = fixes[0]["longitude"]
        lons = [anchor + (s["longitude"] - anchor + 180) % 360 - 180 for s in fixes]
        lon0 = statistics.fmean(lons)
        radii = [math.hypot((s["latitude"] - lat0) * 111320,
                           (lon - lon0) * 111320 * math.cos(math.radians(lat0)))
                 for s, lon in zip(fixes, lons)]
        spread95 = percentile(radii, 0.95)
        if spread95 > 15:
            raise GPSCalibrationError("Stationary position scatter >15 m; moving or poor signal")
        intervals = [(b["timestamp_ns"] - a["timestamp_ns"]) / 1e9
                     for a, b in zip(fixes, fixes[1:])]
        if not intervals or min(intervals) <= 0 or max(intervals) > 3:
            raise GPSCalibrationError("GPS arrival stream has long gaps or invalid timing")
        cal = {"version": 1, "created_utc": datetime.now(timezone.utc).isoformat(),
               "port": self.port, "baud": self.baudrate, "n_fixes": len(fixes),
               "static_seconds": self.static_seconds,
               "reference_latitude": lat0, "reference_longitude": (lon0 + 180) % 360 - 180,
               "scatter95_about_mean_m": spread95,
               "zero_speed_mean_kmh": mean, "zero_speed_std_kmh": std,
               "stationary_threshold_kmh": threshold,
               "arrival_rate_hz": 1 / statistics.median(intervals),
               "arrival_interval_std_ms": statistics.pstdev(intervals) * 1000,
               "timestamp_basis": "pi_monotonic_line_arrival",
               "position_bias_corrected": False}
        self.calibration_file.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.calibration_file.with_suffix(".json.tmp")
        temporary.write_text(json.dumps(cal, indent=2, allow_nan=False) + "\n", encoding="utf-8")
        temporary.replace(self.calibration_file)
        self.calibration = cal
        print(f"GPS: ready; stationary speed threshold {threshold:.2f} km/h; "
              f"scatter95 {spread95:.1f} m. Raw speeds will also be saved.", flush=True)

    def start(self):
        if self.running:
            raise RuntimeError("GPS reader already started")
        self.calibration = None
        self._gga = None
        self._last_epoch = None
        self._stop_event.clear()
        with self.lock:
            self.samples.clear()
        self.serial_conn = serial.Serial(self.port, baudrate=self.baudrate,
                                         timeout=self.timeout, exclusive=True)
        try:
            self.serial_conn.reset_input_buffer()
            self._calibrate()
            self.serial_conn.reset_input_buffer()
            self._gga = None
            self._last_epoch = None
            self.running = True
            self.thread = threading.Thread(target=self._read_loop, daemon=True)
            self.thread.start()
        except BaseException:
            self.serial_conn.close()
            self.serial_conn = None
            raise

    def _read_loop(self):
        while not self._stop_event.is_set():
            try:
                sample = self._next_sample()
                if sample is not None:
                    with self.lock:
                        self.samples.append(sample)
            except (serial.SerialException, OSError) as exc:
                print(f"GPS serial error: {exc}", flush=True)
                self._stop_event.set()
                self.running = False
            except (ValueError, TypeError, AttributeError):
                # A malformed message does not kill the reader thread.
                continue

    def stop(self):
        self._stop_event.set()
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=self.timeout + 2)
        if self.serial_conn is not None:
            self.serial_conn.close()
            self.serial_conn = None


    def get_samples(self):
        """Return a snapshot of readings; each has already been corrected."""
        with self.lock:
            return list(self.samples)


def main():
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/serial0")
    parser.add_argument("--baud", type=int, default=9600)
    parser.add_argument("--static", type=float, default=60)
    parser.add_argument("--fix-timeout", type=float, default=120)
    parser.add_argument("--duration", type=float, default=10,
                        help="Seconds to display readings AFTER calibration")
    args = parser.parse_args()
    gps = CalibratedGPSReader(port=args.port, baudrate=args.baud,
                              static_seconds=args.static,
                              fix_timeout=args.fix_timeout)
    try:
        initialize_gps_calibration(gps)
        print("GPS calibration initialized. Each new measurement is corrected.")
        deadline = time.monotonic() + args.duration
        displayed = 0
        while time.monotonic() < deadline and gps.running:
            readings = gps.get_samples()
            for sample in readings[displayed:]:
                print(json.dumps(sample, allow_nan=False), flush=True)
            displayed = len(readings)
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    except (GPSCalibrationError, serial.SerialException, OSError) as exc:
        parser.exit(1, f"GPS initialization failed: {exc}\n")
    finally:
        gps.stop()


if __name__ == "__main__":
    main()
