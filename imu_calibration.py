"""Calibration wrapper for so630/ESW_Project's existing mpu6500.py.

Collector integration (only change its import):
    from imu_calibration import CalibratedMPU as MPU6500

initialize() calibrates BEFORE GPIO callbacks are registered; read() then
automatically corrects every sample. Existing keys/units are preserved:
accelerometer and gyro are fractional ADC counts; temperature stays raw.
Use read_si() for m/s^2, deg/s and degrees Celsius instead.

First use: keep +Z pointing vertically upward on a LEVEL, stationary surface.
For better accel calibration: python imu_calibration.py --six-face
Thereafter startup loads accel calibration and recalibrates gyro while still
in ANY orientation. Calibration must have sole access to this IMU/I2C device.

This estimates constant bias and (six-face mode) diagonal accel sensitivity.
It does not remove noise, temperature-dependent drift, cross-axis errors,
gyro scale errors or gravity. No zero-error guarantee is possible.
"""

import argparse
import json
import math
from pathlib import Path
import statistics
import time

from mpu6500 import MPU6500

ACCEL_KEYS = ("ax", "ay", "az")
GYRO_KEYS = ("gx", "gy", "gz")
KEYS = ACCEL_KEYS + GYRO_KEYS
ACCEL_LSB_G = 16384.0  # existing driver config: +/-2 g
GYRO_LSB_DPS = 131.0   # existing driver config: +/-250 deg/s
DEFAULT_PROFILE = Path(__file__).resolve().with_name("imu_accel_calibration.json")
POSES = {"x+": (1, 0, 0), "x-": (-1, 0, 0),
         "y+": (0, 1, 0), "y-": (0, -1, 0),
         "z+": (0, 0, 1), "z-": (0, 0, -1)}


class CalibrationError(RuntimeError):
    pass


def correct_imu_reading(raw, accel_bias, accel_scale, gyro_bias):
    """Pure function: subtract biases and apply accel gains to ONE reading.

    All biases use ADC counts. accel_scale is dimensionless. Output uses
    the same count units as input, so the collector's CSV schema still fits.
    No deadband: small real motion must not be forced to zero.
    """
    result = dict(raw)
    for i, key in enumerate(ACCEL_KEYS):
        result[key] = (raw[key] - accel_bias[i]) * accel_scale[i]
    for i, key in enumerate(GYRO_KEYS):
        result[key] = raw[key] - gyro_bias[i]
    return result


class CalibratedMPU(MPU6500):
    def __init__(self, bus=1, address=0x68, calibration_file=DEFAULT_PROFILE,
                 startup_pose="z+", warmup_seconds=10, samples=500):
        if startup_pose not in POSES:
            raise ValueError("startup_pose must be x+, x-, y+, y-, z+ or z-")
        if samples < 100 or warmup_seconds < 0:
            raise ValueError("Use at least 100 samples and nonnegative warmup")
        super().__init__(bus=bus, address=address)
        self.calibration_file = Path(calibration_file)
        self.startup_pose = startup_pose
        self.warmup_seconds = warmup_seconds
        self.samples = samples
        self.accel_bias = [0.0] * 3
        self.accel_scale = [1.0] * 3
        self.gyro_bias = [0.0] * 3
        self.calibrated = False
        self.sensor_id = None

    def initialize(self, calibrate=True):
        """Initialize the old driver, identify chip, then calibrate once."""
        self.calibrated = False
        super().initialize()
        self.sensor_id = self.who_am_i()
        if self.sensor_id not in (0x68, 0x70):
            raise CalibrationError(
                f"Unexpected WHO_AM_I 0x{self.sensor_id:02x}; expected "
                "MPU6050 (0x68) or MPU6500 (0x70)")
        # Calibration coefficients below assume the original driver's ranges.
        if self.read_byte(0x1C) & 0x18 or self.read_byte(0x1B) & 0x18:
            raise CalibrationError("Expected +/-2 g and +/-250 deg/s ranges")
        # MPU6500 also has a separate accelerometer DLPF register.
        if self.sensor_id == 0x70:
            self._write_byte_retry(0x1D, 0x03)
        if calibrate:
            self.calibrate_startup()

    def read_raw(self):
        return super().read()

    def read(self):
        if not self.calibrated:
            raise CalibrationError("Call initialize() successfully before read()")
        return correct_imu_reading(self.read_raw(), self.accel_bias,
                                   self.accel_scale, self.gyro_bias)

    def read_si(self):
        """Separate opt-in output format; does not change read() or CSV units."""
        data = self.read()
        for key in ACCEL_KEYS:
            data[key] = data[key] / ACCEL_LSB_G * 9.80665
        for key in GYRO_KEYS:
            data[key] /= GYRO_LSB_DPS
        if self.sensor_id == 0x68:
            data["temperature"] = data["temperature"] / 340.0 + 36.53
        else:
            data["temperature"] = data["temperature"] / 333.87 + 21.0
        return data

    def _capture_stationary(self):
        """Poll DATA_READY, accepting only fresh sensor frames (not repeats).

        Thresholds are practical motion checks, not proof of stationarity.
        Slow steady rotation can still look like gyro bias: user MUST hold still.
        """
        values = {key: [] for key in KEYS}
        self.read_int_status()  # discard old ready event
        deadline = time.monotonic() + max(15.0, self.samples * 0.04)
        while len(values["ax"]) < self.samples:
            if time.monotonic() > deadline:
                raise CalibrationError("Timed out waiting for fresh IMU data")
            if not self.read_int_status() & 1:
                time.sleep(0.001)
                continue
            raw = self.read_raw()
            if any(abs(raw[key]) >= 32760 for key in KEYS):
                raise CalibrationError("Sensor saturated during calibration")
            for key in KEYS:
                values[key].append(raw[key])
        means = {key: statistics.fmean(values[key]) for key in KEYS}
        for key in ACCEL_KEYS:
            if statistics.stdev(values[key]) / ACCEL_LSB_G > 0.02:
                raise CalibrationError("Acceleration varied: keep the IMU still")
        for key in GYRO_KEYS:
            if statistics.stdev(values[key]) / GYRO_LSB_DPS > 0.5:
                raise CalibrationError("Rotation varied: keep the IMU still")
            if abs(means[key]) / GYRO_LSB_DPS > 5:
                raise CalibrationError("Rotation or excessive gyro bias detected")
        magnitude = math.sqrt(sum(means[k] ** 2 for k in ACCEL_KEYS)) / ACCEL_LSB_G
        if not 0.8 < magnitude < 1.2:
            raise CalibrationError("Expected about 1 g while stationary")
        return means

    def _check_pose(self, means, pose):
        # Only coarse detection: accurate leveling remains the user's job.
        expected = POSES[pose]
        for i, key in enumerate(ACCEL_KEYS):
            if abs(means[key] / ACCEL_LSB_G - expected[i]) > 0.15:
                raise CalibrationError(
                    f"Wrong pose for {pose}; point that signed axis vertically up")

    def _warmup(self):
        print(f"Keep IMU STILL. Warming up for {self.warmup_seconds:g} seconds...")
        time.sleep(self.warmup_seconds)

    def calibrate_startup(self):
        """Loaded six-face accel profile OR known-pose one-point accel bias.

        Gyroscope zero offset is always re-estimated at this startup.
        Call BEFORE registering GPIO callbacks or starting reader threads.
        """
        self.calibrated = False
        has_profile = self.calibration_file.exists()
        if has_profile:
            self.load_accel_profile()
            print("Loaded six-face accel profile. Hold still in any orientation.")
        else:
            print(f"No six-face profile: hold {self.startup_pose} vertically UP; "
                  "align other axes level. Do not move during startup.")
        self._warmup()
        means = self._capture_stationary()
        if not has_profile:
            self._check_pose(means, self.startup_pose)
            expected = POSES[self.startup_pose]
            self.accel_bias = [means[key] - expected[i] * ACCEL_LSB_G
                               for i, key in enumerate(ACCEL_KEYS)]
            self.accel_scale = [1.0] * 3
        self.gyro_bias = [means[key] for key in GYRO_KEYS]
        self.calibrated = True
        self.read_int_status()  # clear latch before collector GPIO setup
        print("Calibration complete.")
        print("Accel bias (counts):", self.accel_bias)
        print("Accel gains:", self.accel_scale)
        print("Gyro bias (deg/s):", [v / GYRO_LSB_DPS for v in self.gyro_bias])

    def calibrate_six_faces(self):
        """Interactive +/-1 g measurements; estimate accel offset AND gain.

        b[i] = (positive[i] + negative[i])/2
        gain[i] = 2*16384/(positive[i] - negative[i])
        Only accelerometer coefficients are persisted. Gyro recalibrates at boot.
        Gyro sensitivity needs a known-rate reference, not stationary poses.
        """
        self.calibrated = False
        self._warmup()
        faces = {}
        for pose in POSES:
            while True:
                input(f"Point {pose} vertically UP, hold completely still, "
                      "then press Enter: ")
                time.sleep(1)
                try:
                    means = self._capture_stationary()
                    self._check_pose(means, pose)
                    faces[pose] = means
                    break
                except CalibrationError as exc:
                    print(f"Rejected: {exc}. Repeat this face.")
        biases, gains = [], []
        for axis, key in zip("xyz", ACCEL_KEYS):
            positive = faces[axis + "+"][key]
            negative = faces[axis + "-"][key]
            span = positive - negative
            if not 1.6 * ACCEL_LSB_G < span < 2.4 * ACCEL_LSB_G:
                raise CalibrationError(f"Invalid span for {axis}; repeat calibration")
            biases.append((positive + negative) / 2.0)
            gains.append(2.0 * ACCEL_LSB_G / span)
        # Check corrected six-face vectors against expected +/- unit gravity.
        for pose, means in faces.items():
            corrected = [(means[k] - biases[i]) * gains[i] / ACCEL_LSB_G
                         for i, k in enumerate(ACCEL_KEYS)]
            residual = math.sqrt(sum((corrected[i] - POSES[pose][i]) ** 2
                                     for i in range(3)))
            if residual > 0.06:
                raise CalibrationError(
                    f"Six-face residual too large at {pose}; level axes more accurately")
        self.accel_bias, self.accel_scale = biases, gains
        self.gyro_bias = [statistics.fmean(faces[p][k] for p in POSES)
                          for k in GYRO_KEYS]
        self.save_accel_profile()
        self.calibrated = True
        self.read_int_status()
        print(f"Saved six-face accel calibration: {self.calibration_file}")

    def save_accel_profile(self):
        data = {"version": 1, "method": "six-face", "sensor_id": self.sensor_id,
                "address": self.address, "accel_lsb_per_g": ACCEL_LSB_G,
                "accel_bias_counts": self.accel_bias, "accel_scale": self.accel_scale}
        self.calibration_file.parent.mkdir(parents=True, exist_ok=True)
        temp_path = self.calibration_file.with_suffix(".json.tmp")
        temp_path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
        temp_path.replace(self.calibration_file)

    def load_accel_profile(self):
        try:
            data = json.loads(self.calibration_file.read_text(encoding="utf-8"))
            if (data["version"] != 1 or data["method"] != "six-face"
                    or data["sensor_id"] != self.sensor_id
                    or data["address"] != self.address
                    or data["accel_lsb_per_g"] != ACCEL_LSB_G):
                raise ValueError("Profile chip/address/range/method does not match")
            biases = list(map(float, data["accel_bias_counts"]))
            gains = list(map(float, data["accel_scale"]))
            if len(biases) != 3 or len(gains) != 3:
                raise ValueError("Expected three coefficients per vector")
            if not all(math.isfinite(v) for v in biases + gains):
                raise ValueError("Non-finite coefficient")
            if not all(0.8 < v < 1.25 for v in gains):
                raise ValueError("Implausible accelerometer gain")
            if not all(abs(v) < 0.3 * ACCEL_LSB_G for v in biases):
                raise ValueError("Implausible accelerometer bias")
            self.accel_bias, self.accel_scale = biases, gains
        except (OSError, ValueError, KeyError, TypeError) as exc:
            raise CalibrationError(f"Invalid calibration profile: {exc}") from exc


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--six-face", action="store_true")
    parser.add_argument("--bus", type=int, default=1)
    parser.add_argument("--address", type=lambda s: int(s, 0), default=0x68)
    parser.add_argument("--calibration-file", type=Path, default=DEFAULT_PROFILE)
    parser.add_argument("--pose", choices=POSES, default="z+")
    parser.add_argument("--samples", type=int, default=500)
    parser.add_argument("--warmup", type=float, default=10)
    args = parser.parse_args()
    imu = CalibratedMPU(bus=args.bus, address=args.address,
                        calibration_file=args.calibration_file,
                        startup_pose=args.pose, warmup_seconds=args.warmup,
                        samples=args.samples)
    try:
        imu.initialize(calibrate=not args.six_face)
        if args.six_face:
            imu.calibrate_six_faces()
        print("Corrected acceleration in m/s^2, gyro in deg/s, temperature in C")
        while True:
            if imu.read_int_status() & 1:
                print(imu.read_si())
                time.sleep(0.1)
            else:
                time.sleep(0.001)
    except KeyboardInterrupt:
        pass
    except (CalibrationError, OSError) as exc:
        parser.exit(1, f"IMU calibration failed: {exc}\n")
    finally:
        imu.close()


if __name__ == "__main__":
    main()
