#!/usr/bin/env python3
"""
Create the FusionCore demo bag from NCLT CSV source data.

Extracts the first --duration seconds of the NCLT 2012-01-08 sequence and
writes a self-contained MCAP bag with raw sensor topics:
  /imu/data         sensor_msgs/Imu
  /odom/wheels      nav_msgs/Odometry
  /gnss/fix         sensor_msgs/NavSatFix
  /clock            rosgraph_msgs/Clock

Requirements: ROS 2 (Jazzy or Humble) sourced. No extra pip packages needed.
  source /opt/ros/jazzy/setup.bash
  python3 demo/make_demo_bag.py

Usage:
  python3 demo/make_demo_bag.py \\
    --data_dir benchmarks/nclt/2012-01-08 \\
    --out      demo/nclt_demo_120s.mcap \\
    --duration 120
"""

import argparse
import csv
import math
import os
import sys
from pathlib import Path

# ROS serialization -- available when ROS is sourced
try:
    import rclpy
    from rclpy.serialization import serialize_message
    from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
    from builtin_interfaces.msg import Time
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Vector3, Quaternion, Twist, TwistWithCovariance
    from geometry_msgs.msg import Pose, PoseWithCovariance, Point
    from std_msgs.msg import Header
except ImportError as e:
    sys.exit(
        f"Import error: {e}\n"
        "Source your ROS installation first:\n"
        "  source /opt/ros/jazzy/setup.bash   # or humble\n"
        "Then re-run this script."
    )

WHEEL_TRACK_M = 0.522   # NCLT Segway RMP400 track width


def utime_to_ros_time(utime_us: int) -> Time:
    ns   = utime_us * 1000
    t    = Time()
    t.sec    = int(ns // 1_000_000_000)
    t.nanosec = int(ns % 1_000_000_000)
    return t


def utime_to_ns(utime_us: int) -> int:
    return utime_us * 1000


def read_imu(path: Path, t_start_us: int, t_end_us: int):
    with open(path) as f:
        for line in f:
            s = line.strip()
            if not s or s.startswith('#'):
                continue
            row = s.split(',')
            t = int(row[0])
            if t < t_start_us:
                continue
            if t > t_end_us:
                break
            # ms25: accel cols 4,5,6 and gyro cols 7,8,9 in NED body frame
            # NED -> ENU: ay = -ay_ned, az = -az_ned, wy = -wy_ned, wz = -wz_ned
            ax =  float(row[4])
            ay = -float(row[5])
            az = -float(row[6])
            wx =  float(row[7])
            wy = -float(row[8])
            wz = -float(row[9])
            yield t, wx, wy, wz, ax, ay, az


def read_wheels(path: Path, t_start_us: int, t_end_us: int):
    with open(path) as f:
        for line in f:
            s = line.strip()
            if not s or s.startswith('#'):
                continue
            row = s.split(',')
            t = int(row[0])
            if t < t_start_us:
                continue
            if t > t_end_us:
                break
            vl = float(row[1])
            vr = float(row[2])
            vx = (vl + vr) / 2.0
            wz = (vr - vl) / WHEEL_TRACK_M
            yield t, vx, wz


def read_gps(path: Path, t_start_us: int, t_end_us: int):
    with open(path) as f:
        for line in f:
            s = line.strip()
            if not s or s.startswith('#'):
                continue
            row = s.split(',')
            t = int(row[0])
            if t < t_start_us:
                continue
            if t > t_end_us:
                break
            mode = int(row[1])
            if mode < 2:
                continue
            lat_rad = float(row[3])
            lon_rad = float(row[4])
            alt_m   = float(row[5])
            hdop    = float(row[6]) if len(row) > 6 and row[6].strip() else 1.5
            yield t, math.degrees(lat_rad), math.degrees(lon_rad), alt_m, hdop, mode


def make_bag(data_dir: Path, out_path: Path, duration_s: float):
    raw = data_dir / 'raw files'
    imu_csv    = raw / 'ms25.csv'
    wheels_csv = raw / 'wheels.csv'
    gps_csv    = raw / 'gps.csv'

    for p in (imu_csv, wheels_csv, gps_csv):
        if not p.exists():
            sys.exit(f"Missing: {p}\nMake sure --data_dir points to the NCLT sequence folder.")

    # find first IMU timestamp
    t_start_us = None
    with open(imu_csv) as f:
        for line in f:
            s = line.strip()
            if s and not s.startswith('#'):
                t_start_us = int(s.split(',')[0])
                break
    if t_start_us is None:
        sys.exit("Could not read first timestamp from ms25.csv")

    t_end_us = t_start_us + int(duration_s * 1e6)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    # remove existing bag at that path
    if out_path.exists():
        out_path.unlink()
    bag_dir = str(out_path.parent / out_path.stem)

    writer = SequentialWriter()
    writer.open(
        StorageOptions(uri=bag_dir, storage_id='mcap'),
        ConverterOptions('', ''),
    )

    topic_id = 0

    def add_topic(name, msg_type):
        nonlocal topic_id
        meta = TopicMetadata(
            id=topic_id,
            name=name,
            type=msg_type,
            serialization_format='cdr',
        )
        topic_id += 1
        writer.create_topic(meta)

    add_topic('/imu/data',    'sensor_msgs/msg/Imu')
    add_topic('/odom/wheels', 'nav_msgs/msg/Odometry')
    add_topic('/gnss/fix',    'sensor_msgs/msg/NavSatFix')
    add_topic('/clock',       'rosgraph_msgs/msg/Clock')

    # merge all events sorted by timestamp
    events = []
    for row in read_imu(imu_csv, t_start_us, t_end_us):
        events.append(('imu', row))
    for row in read_wheels(wheels_csv, t_start_us, t_end_us):
        events.append(('wheels', row))
    for row in read_gps(gps_csv, t_start_us, t_end_us):
        events.append(('gps', row))
    events.sort(key=lambda e: e[1][0])

    last_clock_us  = t_start_us
    CLOCK_INTERVAL = 10_000   # 100 Hz

    n_imu = n_wheels = n_gps = 0

    for kind, row in events:
        t_us = row[0]

        # emit clock up to this event
        while last_clock_us + CLOCK_INTERVAL <= t_us:
            last_clock_us += CLOCK_INTERVAL
            clk = Clock()
            clk.clock = utime_to_ros_time(last_clock_us)
            writer.write('/clock', serialize_message(clk), utime_to_ns(last_clock_us))

        stamp = utime_to_ros_time(t_us)

        if kind == 'imu':
            _, wx, wy, wz, ax, ay, az = row
            msg            = Imu()
            msg.header     = Header()
            msg.header.stamp    = stamp
            msg.header.frame_id = 'imu_link'
            msg.orientation             = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            msg.orientation_covariance  = [-1.0] + [0.0] * 8
            msg.angular_velocity        = Vector3(x=wx, y=wy, z=wz)
            c3 = [0.0] * 9
            c3[0] = c3[4] = c3[8] = 0.0025
            msg.angular_velocity_covariance    = c3
            msg.linear_acceleration            = Vector3(x=ax, y=ay, z=az)
            c3b = [0.0] * 9
            c3b[0] = c3b[4] = 0.01
            c3b[8] = 0.01
            msg.linear_acceleration_covariance = c3b
            writer.write('/imu/data', serialize_message(msg), utime_to_ns(t_us))
            n_imu += 1

        elif kind == 'wheels':
            _, vx, wz = row
            msg            = Odometry()
            msg.header     = Header()
            msg.header.stamp    = stamp
            msg.header.frame_id = 'odom'
            msg.child_frame_id  = 'base_link'
            cov36 = [0.0] * 36
            cov36[0]  = 0.0025
            cov36[35] = 0.0004
            msg.twist = TwistWithCovariance(
                twist=Twist(
                    linear=Vector3(x=vx, y=0.0, z=0.0),
                    angular=Vector3(x=0.0, y=0.0, z=wz),
                ),
                covariance=cov36,
            )
            msg.pose = PoseWithCovariance(
                pose=Pose(
                    position=Point(x=0.0, y=0.0, z=0.0),
                    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                ),
                covariance=[0.0] * 36,
            )
            writer.write('/odom/wheels', serialize_message(msg), utime_to_ns(t_us))
            n_wheels += 1

        elif kind == 'gps':
            _, lat, lon, alt, hdop, mode = row
            msg            = NavSatFix()
            msg.header     = Header()
            msg.header.stamp    = stamp
            msg.header.frame_id = 'gnss_link'
            msg.status         = NavSatStatus()
            msg.status.status  = NavSatStatus.STATUS_FIX if mode >= 3 else NavSatStatus.STATUS_NO_FIX
            msg.status.service = NavSatStatus.SERVICE_GPS
            msg.latitude   = lat
            msg.longitude  = lon
            msg.altitude   = alt
            h2 = (hdop * 1.5) ** 2
            msg.position_covariance = [h2, 0.0, 0.0,
                                        0.0, h2, 0.0,
                                        0.0, 0.0, h2 * 4.0]
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            writer.write('/gnss/fix', serialize_message(msg), utime_to_ns(t_us))
            n_gps += 1

    del writer   # flushes and closes

    # rosbag2 writes to a directory; rename the .mcap inside to the requested path
    bag_mcap = Path(bag_dir) / (Path(bag_dir).name + '_0.mcap')
    if bag_mcap.exists():
        import shutil
        shutil.move(str(bag_mcap), str(out_path))
        # copy metadata.yaml alongside
        meta_src = Path(bag_dir) / 'metadata.yaml'
        meta_dst = out_path.parent / 'metadata.yaml'
        if meta_src.exists():
            shutil.copy(str(meta_src), str(meta_dst))
        shutil.rmtree(bag_dir, ignore_errors=True)

    size_kb = out_path.stat().st_size // 1024 if out_path.exists() else '?'
    print(f"Written {n_imu} IMU  |  {n_wheels} wheel  |  {n_gps} GPS  messages")
    print(f"Output: {out_path}  ({size_kb} KB)")


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument('--data_dir', default='benchmarks/nclt/2012-01-08',
                        help='NCLT sequence directory containing "raw files/"')
    parser.add_argument('--out',      default='demo/nclt_demo_120s.mcap',
                        help='Output MCAP file path')
    parser.add_argument('--duration', type=float, default=120.0,
                        help='Seconds to extract (default: 120)')
    args = parser.parse_args()

    rclpy.init(args=[])
    make_bag(Path(args.data_dir), Path(args.out), args.duration)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
