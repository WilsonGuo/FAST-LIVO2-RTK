#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Offline rosbag timestamp synchronization analyzer.

Analyze topics:
  /receiver_pvt
  /livox/lidar
  /left_camera/image

For your bag:
  /receiver_pvt      : gnss_comm/GnssPVTSolnMsg, about 1 Hz
  /livox/lidar       : livox_ros_driver/CustomMsg, about 10 Hz
  /left_camera/image : sensor_msgs/Image, about 10 Hz

This script analyzes:
  1. Frequency of each topic from header.stamp
  2. Frequency of each topic from bag receive time
  3. header.stamp - bag_time
  4. lidar - image nearest timestamp difference
  5. rtk - lidar nearest timestamp difference
  6. rtk - image nearest timestamp difference
  7. RTK GNSS week/tow time compared with header.stamp and bag time

Usage:
  python3 analyze_bag_time_sync.py \
    --bag /media/wilson/A81D-2413/2026-07-02-11-49-14.bag

Optional:
  python3 analyze_bag_time_sync.py \
    --bag /media/wilson/A81D-2413/2026-06-24-15-28-09.bag \
    --csv /tmp/bag_time_sync.csv
"""

import argparse
import bisect
import csv
import math
import os
import statistics

import rosbag


GPS_EPOCH_UNIX_TIME = 315964800.0
SECONDS_PER_WEEK = 604800.0
LEAP_SECONDS = 18.0


def gps_week_tow_to_unix(week, tow):
    return float(week) * SECONDS_PER_WEEK + float(tow) + GPS_EPOCH_UNIX_TIME - LEAP_SECONDS


def stamp_to_sec(stamp):
    return stamp.secs + stamp.nsecs * 1e-9


def get_header_stamp(msg):
    try:
        return stamp_to_sec(msg.header.stamp)
    except Exception:
        return None


def get_rtk_gnss_stamp(msg):
    """
    For gnss_comm/GnssPVTSolnMsg.
    Expected fields:
      msg.time.week
      msg.time.tow
    """
    try:
        return gps_week_tow_to_unix(msg.time.week, msg.time.tow)
    except Exception:
        return None


def stats(values):
    values = [v for v in values if v is not None and math.isfinite(v)]
    if not values:
        return None

    return {
        "count": len(values),
        "mean": statistics.mean(values),
        "std": statistics.pstdev(values) if len(values) > 1 else 0.0,
        "median": statistics.median(values),
        "min": min(values),
        "max": max(values),
    }


def print_stats_seconds(name, values, unit="ms"):
    s = stats(values)
    if s is None:
        print(f"{name}: no data")
        return

    if unit == "ms":
        k = 1000.0
        suffix = "ms"
    else:
        k = 1.0
        suffix = "s"

    print(
        f"{name}: count={s['count']}, "
        f"mean={s['mean'] * k:+.3f} {suffix}, "
        f"std={s['std'] * k:.3f} {suffix}, "
        f"median={s['median'] * k:+.3f} {suffix}, "
        f"min={s['min'] * k:+.3f} {suffix}, "
        f"max={s['max'] * k:+.3f} {suffix}"
    )


def print_rate(name, times):
    times = [t for t in times if t is not None and math.isfinite(t)]
    if len(times) < 2:
        print(f"{name}: no enough data")
        return

    times = sorted(times)
    dts = [times[i] - times[i - 1] for i in range(1, len(times))]
    dts = [dt for dt in dts if dt > 0]

    if not dts:
        print(f"{name}: invalid timestamps")
        return

    mean_dt = statistics.mean(dts)
    rate = 1.0 / mean_dt if mean_dt > 0 else 0.0
    std_dt = statistics.pstdev(dts) if len(dts) > 1 else 0.0

    print(
        f"{name}: count={len(times)}, "
        f"rate={rate:.4f} Hz, "
        f"mean_dt={mean_dt * 1000.0:.3f} ms, "
        f"std_dt={std_dt * 1000.0:.3f} ms, "
        f"min_dt={min(dts) * 1000.0:.3f} ms, "
        f"max_dt={max(dts) * 1000.0:.3f} ms"
    )


def nearest_diff(query_t, ref_times):
    """
    Return query_t - nearest_ref_time.
    Positive means query topic timestamp is later than nearest reference timestamp.
    """
    if query_t is None or not ref_times:
        return None

    idx = bisect.bisect_left(ref_times, query_t)

    if idx <= 0:
        nearest = ref_times[0]
    elif idx >= len(ref_times):
        nearest = ref_times[-1]
    else:
        before = ref_times[idx - 1]
        after = ref_times[idx]
        nearest = before if abs(query_t - before) <= abs(query_t - after) else after

    return query_t - nearest


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True, help="Path to rosbag file")
    parser.add_argument("--rtk_topic", default="/receiver_pvt")
    parser.add_argument("--lidar_topic", default="/livox/lidar")
    parser.add_argument("--image_topic", default="/left_camera/image")
    parser.add_argument("--csv", default="", help="Optional CSV output path")
    args = parser.parse_args()

    if not os.path.exists(args.bag):
        raise FileNotFoundError(args.bag)

    topics = [args.rtk_topic, args.lidar_topic, args.image_topic]

    lidar_header_times = []
    lidar_bag_times = []

    image_header_times = []
    image_bag_times = []

    rtk_header_times = []
    rtk_bag_times = []
    rtk_gnss_times = []

    # Delay-like differences
    lidar_header_minus_bag = []
    image_header_minus_bag = []
    rtk_header_minus_bag = []
    rtk_gnss_minus_bag = []
    rtk_gnss_minus_header = []

    print(f"Reading bag: {args.bag}")
    print(f"Topics: {topics}")

    with rosbag.Bag(args.bag, "r") as bag:
        for topic, msg, bag_time in bag.read_messages(topics=topics):
            t_bag = bag_time.to_sec()
            t_header = get_header_stamp(msg)

            if topic == args.lidar_topic:
                lidar_bag_times.append(t_bag)
                if t_header is not None:
                    lidar_header_times.append(t_header)
                    lidar_header_minus_bag.append(t_header - t_bag)

            elif topic == args.image_topic:
                image_bag_times.append(t_bag)
                if t_header is not None:
                    image_header_times.append(t_header)
                    image_header_minus_bag.append(t_header - t_bag)

            elif topic == args.rtk_topic:
                rtk_bag_times.append(t_bag)

                if t_header is not None:
                    rtk_header_times.append(t_header)
                    rtk_header_minus_bag.append(t_header - t_bag)

                t_gnss = get_rtk_gnss_stamp(msg)
                if t_gnss is not None and math.isfinite(t_gnss):
                    rtk_gnss_times.append(t_gnss)
                    rtk_gnss_minus_bag.append(t_gnss - t_bag)

                    if t_header is not None:
                        rtk_gnss_minus_header.append(t_gnss - t_header)

    lidar_header_times = sorted(lidar_header_times)
    image_header_times = sorted(image_header_times)
    rtk_header_times = sorted(rtk_header_times)
    rtk_gnss_times = sorted(rtk_gnss_times)

    print("\n" + "=" * 100)
    print("1. Topic rate from header.stamp")
    print("=" * 100)
    print_rate(args.lidar_topic, lidar_header_times)
    print_rate(args.image_topic, image_header_times)
    print_rate(args.rtk_topic + " header", rtk_header_times)
    print_rate(args.rtk_topic + " GNSS", rtk_gnss_times)

    print("\n" + "=" * 100)
    print("2. Topic rate from rosbag record time")
    print("=" * 100)
    print_rate(args.lidar_topic + " bag_time", lidar_bag_times)
    print_rate(args.image_topic + " bag_time", image_bag_times)
    print_rate(args.rtk_topic + " bag_time", rtk_bag_times)

    print("\n" + "=" * 100)
    print("3. header.stamp - rosbag_time")
    print("=" * 100)
    print("Positive means header.stamp is later than bag record time.")
    print_stats_seconds("lidar header - bag_time", lidar_header_minus_bag)
    print_stats_seconds("image header - bag_time", image_header_minus_bag)
    print_stats_seconds("rtk header - bag_time", rtk_header_minus_bag)
    print_stats_seconds("rtk GNSS   - bag_time", rtk_gnss_minus_bag)
    print_stats_seconds("rtk GNSS   - rtk header", rtk_gnss_minus_header)

    # Use RTK GNSS time first. If not available, use RTK header time.
    rtk_time_for_sync = rtk_gnss_times if rtk_gnss_times else rtk_header_times

    lidar_minus_image = []
    for t_lidar in lidar_header_times:
        d = nearest_diff(t_lidar, image_header_times)
        if d is not None:
            lidar_minus_image.append(d)

    MAX_NEAREST_DT = 0.05  # 50 ms for 10 Hz lidar/image

    rtk_minus_lidar = []
    rtk_minus_lidar_outliers = []

    for t_rtk in rtk_time_for_sync:
        d = nearest_diff(t_rtk, lidar_header_times)
        if d is not None:
            if abs(d) <= MAX_NEAREST_DT:
                rtk_minus_lidar.append(d)
            else:
                rtk_minus_lidar_outliers.append(d)

    rtk_minus_image = []
    rtk_minus_image_outliers = []

    for t_rtk in rtk_time_for_sync:
        d = nearest_diff(t_rtk, image_header_times)
        if d is not None:
            if abs(d) <= MAX_NEAREST_DT:
                rtk_minus_image.append(d)
            else:
                rtk_minus_image_outliers.append(d)

    print("\n" + "=" * 100)
    print("4. Nearest timestamp difference based on header/GNSS time")
    print("=" * 100)
    print("Sign convention:")
    print("  lidar - image > 0 means lidar timestamp is later than nearest image.")
    print("  rtk   - lidar > 0 means RTK timestamp is later than nearest lidar.")
    print("  rtk   - image > 0 means RTK timestamp is later than nearest image.")
    print_stats_seconds("lidar - image", lidar_minus_image)
    print_stats_seconds("rtk   - lidar", rtk_minus_lidar)
    print_stats_seconds("rtk   - image", rtk_minus_image)

    print("\n" + "=" * 100)
    print("5. Basic timestamp range")
    print("=" * 100)
    if lidar_header_times:
        print(f"lidar header range: {lidar_header_times[0]:.9f} -> {lidar_header_times[-1]:.9f}")
    if image_header_times:
        print(f"image header range: {image_header_times[0]:.9f} -> {image_header_times[-1]:.9f}")
    if rtk_time_for_sync:
        print(f"rtk sync range    : {rtk_time_for_sync[0]:.9f} -> {rtk_time_for_sync[-1]:.9f}")

    print("\n" + "=" * 100)
    print("6. Judgment hints")
    print("=" * 100)
    print("For your bag, lidar and image are both about 10 Hz.")
    print("If lidar-image mean is close to 0 ms and std is small, camera-lidar timestamp sync is good.")
    print("RTK is about 1 Hz, so rtk-lidar nearest difference should normally fall within about ±50 ms.")
    print("If rtk-lidar or rtk-image has a stable bias larger than 100 ms, check gps_time_offset or timestamp source.")
    print("If rtk GNSS - bag_time is very large, host system time and GNSS time are not in the same time base.")
    print("=" * 100)

    if args.csv:
        print(f"\nWriting CSV: {args.csv}")

        with open(args.csv, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "metric",
                "value_sec",
                "value_ms",
            ])

            for v in lidar_minus_image:
                writer.writerow(["lidar_minus_image", f"{v:.9f}", f"{v * 1000.0:.6f}"])

            for v in rtk_minus_lidar:
                writer.writerow(["rtk_minus_lidar", f"{v:.9f}", f"{v * 1000.0:.6f}"])

            for v in rtk_minus_image:
                writer.writerow(["rtk_minus_image", f"{v:.9f}", f"{v * 1000.0:.6f}"])

            for v in lidar_header_minus_bag:
                writer.writerow(["lidar_header_minus_bag", f"{v:.9f}", f"{v * 1000.0:.6f}"])

            for v in image_header_minus_bag:
                writer.writerow(["image_header_minus_bag", f"{v:.9f}", f"{v * 1000.0:.6f}"])

            for v in rtk_gnss_minus_bag:
                writer.writerow(["rtk_gnss_minus_bag", f"{v:.9f}", f"{v * 1000.0:.6f}"])

        print("CSV saved.")


if __name__ == "__main__":
    main()