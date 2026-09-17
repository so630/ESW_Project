"""
verify_sync.py

Checks whether the IMU and camera timestamp streams produced by
sync_collector.py are actually well-synchronized, rather than just
trusting that "it ran without errors."

Usage:
    python3 verify_sync.py sync_data/

Produces:
    - Printed statistics (gaps, overlap, nearest-neighbor alignment)
    - sync_check.png : a plot of both timelines + alignment error
"""

import sys
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def load_data(data_dir):

    imu_path = data_dir / "imu.csv"
    cam_path = data_dir / "camera_timestamps.csv"

    if not imu_path.exists():
        raise FileNotFoundError(f"Missing {imu_path}")

    if not cam_path.exists():
        raise FileNotFoundError(f"Missing {cam_path}")

    imu = pd.read_csv(imu_path)
    cam = pd.read_csv(cam_path)

    return imu, cam


def check_overlap(imu, cam):

    imu_start, imu_end = imu["timestamp_ns"].iloc[0], imu["timestamp_ns"].iloc[-1]
    cam_start, cam_end = cam["timestamp_ns"].iloc[0], cam["timestamp_ns"].iloc[-1]

    print("=== Timeline overlap ===")
    print(f"IMU:    {imu_start/1e9:.3f}s -> {imu_end/1e9:.3f}s  "
          f"(duration {(imu_end - imu_start)/1e9:.3f}s)")
    print(f"Camera: {cam_start/1e9:.3f}s -> {cam_end/1e9:.3f}s  "
          f"(duration {(cam_end - cam_start)/1e9:.3f}s)")

    start_gap_ms = (cam_start - imu_start) / 1e6
    end_gap_ms = (imu_end - cam_end) / 1e6

    print(f"Camera starts {start_gap_ms:+.1f} ms relative to IMU start")
    print(f"IMU ends {end_gap_ms:+.1f} ms after camera ends")

    if abs(start_gap_ms) > 2000 or abs(end_gap_ms) > 2000:
        print("  WARNING: start/end offset is larger than expected "
              "(> 2s) -- streams may not be from the same run, or one "
              "started/stopped very late.")
    print()


def check_gaps(name, timestamps_ns, expected_period_ms):

    deltas_ms = np.diff(timestamps_ns) / 1e6

    print(f"=== {name} sample spacing ===")
    print(f"Expected period: ~{expected_period_ms:.2f} ms")
    print(f"Mean delta:      {deltas_ms.mean():.3f} ms")
    print(f"Median delta:    {np.median(deltas_ms):.3f} ms")
    print(f"Max delta:       {deltas_ms.max():.3f} ms")
    print(f"Min delta:       {deltas_ms.min():.3f} ms")

    # Flag deltas more than 2x the expected period as likely dropped samples
    threshold = expected_period_ms * 2
    n_gaps = int((deltas_ms > threshold).sum())

    if n_gaps > 0:
        worst = deltas_ms.max()
        print(f"  WARNING: {n_gaps} gap(s) exceed {threshold:.1f} ms "
              f"(worst: {worst:.1f} ms) -- likely dropped/delayed samples.")
    else:
        print("  No abnormal gaps detected.")
    print()

    return deltas_ms


def check_nearest_neighbor(imu, cam, imu_period_ms):

    imu_ts = imu["timestamp_ns"].to_numpy()
    cam_ts = cam["timestamp_ns"].to_numpy()

    # For each camera frame, find index of nearest IMU timestamp
    idx = np.searchsorted(imu_ts, cam_ts)
    idx = np.clip(idx, 1, len(imu_ts) - 1)

    left = imu_ts[idx - 1]
    right = imu_ts[idx]

    left_diff = np.abs(cam_ts - left)
    right_diff = np.abs(cam_ts - right)

    nearest_diff_ns = np.minimum(left_diff, right_diff)
    nearest_diff_ms = nearest_diff_ns / 1e6

    print("=== Frame -> nearest IMU sample alignment ===")
    print(f"Mean alignment error:   {nearest_diff_ms.mean():.3f} ms")
    print(f"Max alignment error:    {nearest_diff_ms.max():.3f} ms")
    print(f"95th percentile error:  {np.percentile(nearest_diff_ms, 95):.3f} ms")

    # A frame should always have an IMU sample within roughly one IMU
    # period of it. If not, that frame has no good IMU data near it.
    threshold = imu_period_ms
    n_bad = int((nearest_diff_ms > threshold).sum())

    if n_bad > 0:
        print(f"  WARNING: {n_bad} frame(s) have no IMU sample within "
              f"{threshold:.1f} ms -- those frames are not well "
              f"time-matched to IMU data.")
    else:
        print(f"  Every frame has an IMU sample within {threshold:.1f} ms. "
              f"Streams are well aligned.")
    print()

    return nearest_diff_ms


def plot_results(imu, cam, imu_deltas_ms, cam_deltas_ms, nearest_diff_ms, out_path):

    fig, axes = plt.subplots(3, 1, figsize=(10, 9))

    t0 = min(imu["timestamp_ns"].iloc[0], cam["timestamp_ns"].iloc[0])

    # Timeline: event markers for both streams
    ax = axes[0]
    imu_t = (imu["timestamp_ns"] - t0) / 1e9
    cam_t = (cam["timestamp_ns"] - t0) / 1e9
    ax.eventplot(imu_t, lineoffsets=1, linelengths=0.8, color="tab:blue", label="IMU samples")
    ax.eventplot(cam_t, lineoffsets=0, linelengths=0.8, color="tab:orange", label="Camera frames")
    ax.set_yticks([0, 1])
    ax.set_yticklabels(["Camera", "IMU"])
    ax.set_xlabel("Time (s)")
    ax.set_title("Event timeline (zoom in to inspect visually)")

    # Sample spacing over time
    ax = axes[1]
    ax.plot(imu_t[1:], imu_deltas_ms, ".", color="tab:blue", label="IMU delta (ms)", markersize=3)
    ax.plot(cam_t[1:], cam_deltas_ms, ".", color="tab:orange", label="Camera delta (ms)", markersize=4)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Delta (ms)")
    ax.set_title("Sample spacing over time (look for spikes = dropped samples)")
    ax.legend()

    # Nearest-neighbor alignment error per frame
    ax = axes[2]
    ax.plot(cam_t, nearest_diff_ms, ".-", color="tab:green", markersize=4)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Alignment error (ms)")
    ax.set_title("Frame -> nearest IMU sample time difference")

    fig.tight_layout()
    fig.savefig(out_path, dpi=130)
    print(f"Saved plot to {out_path}")


def main():

    data_dir = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("sync_data")

    imu, cam = load_data(data_dir)

    check_overlap(imu, cam)

    imu_deltas_ms = check_gaps("IMU", imu["timestamp_ns"].to_numpy(), expected_period_ms=10.0)
    cam_deltas_ms = check_gaps("Camera", cam["timestamp_ns"].to_numpy(), expected_period_ms=1000.0 / 30.0)

    nearest_diff_ms = check_nearest_neighbor(imu, cam, imu_period_ms=10.0)

    plot_results(imu, cam, imu_deltas_ms, cam_deltas_ms, nearest_diff_ms,
                 out_path=data_dir / "sync_check.png")


if __name__ == "__main__":
    main()