#!/usr/bin/env python3
"""Pretends to be the phone: sends sensor packets that slowly turn the head left and right.

Useful to test the SteamVR driver without the phone. Usage:
    python send_imu.py [pc_ip] [--port 9944] [--seconds 30]
"""
import argparse
import math
import socket
import struct
import time

RATE_HZ = 400
GRAVITY = 9.81


def sensor_packet(seq, timestamp_ns, gyro, accel, mag=(0.0, 0.0, 0.0)):
    return b"HMD1" + struct.pack("<B3xIq9f", 1, seq, timestamp_ns, *gyro, *accel, *mag)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("host", nargs="?", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9944)
    parser.add_argument("--seconds", type=float, default=30.0)
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    start = time.monotonic()
    seq = 0
    while time.monotonic() - start < args.seconds:
        t = time.monotonic() - start
        seq += 1
        # Phone upright in the headset: gravity on the phone's x axis; turning the head is rotation around x.
        yaw_rate = 0.8 * math.cos(t)
        packet = sensor_packet(seq, time.monotonic_ns(), (yaw_rate, 0.0, 0.0), (GRAVITY, 0.0, 0.0))
        sock.sendto(packet, (args.host, args.port))
        time.sleep(1.0 / RATE_HZ)


if __name__ == "__main__":
    main()
