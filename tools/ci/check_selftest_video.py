#!/usr/bin/env python3
"""Checks the video the camera app's encoder self-test saved decodes as the PC decodes it, into
the picture it was made from: grey, with a red square at the top left.

    check_selftest_video.py selftest.h264
"""
import sys

import av


def main(path):
    with open(path, "rb") as f:
        data = f.read()
    decoder = av.CodecContext.create("h264", "r")
    frames = []
    for packet in decoder.parse(data):
        frames += decoder.decode(packet)
    frames += decoder.decode(None)
    if not frames:
        print(f"::error::No frame decodes from the encoder's {len(data)} bytes")
        return 1
    print(f"{len(frames)} frames of {frames[0].width}x{frames[0].height} decoded from {len(data)} bytes")
    image = frames[-1].to_ndarray(format="rgb24")
    red, grey = image[60, 80], image[240, 320]
    print(f"Top left {tuple(int(c) for c in red)}, middle {tuple(int(c) for c in grey)}")
    if len(frames) < 50 or (frames[0].width, frames[0].height) != (640, 480):
        print("::error::Expected about 60 frames of 640x480")
        return 1
    if not (red[0] > 150 and red[1] < 100 and red[2] < 100) or any(abs(int(c) - 128) > 25 for c in grey):
        print("::error::The picture changed on the way: the encoder got its planes wrong")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1]))
