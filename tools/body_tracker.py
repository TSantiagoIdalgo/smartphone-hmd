#!/usr/bin/env python3
"""Body tracking on the PC: finds the pose of the person the body camera phones see.

The phones (the "HMD Cámara" app) send their camera's video here. For each phone this decodes it,
finds the person's 33 pose points with MediaPipe's pose model, run on several CPU cores with
LiteRT, and sends them back to the phone, which draws them on its screen. It also finds a lit PS
Move's bulb near the hands, for calibrating the cameras, and hands each frame's points to the
headset driver on this PC, which turns the cameras' points into the body in 3D. Every 2 s it
prints per phone: frames per second, how often it sees a person, the model's time per frame, the
delay from the camera to the finished pose, lost frames, and how sure it is of the hips, knees and
ankles.

    pip install -r tools/requirements-body.txt
    python tools/body_tracker.py                   # watch
    python tools/body_tracker.py --model heavy     # more exact model, slower
    python tools/body_tracker.py --threads 8       # more cores per camera
    python tools/body_tracker.py --record a.jsonl  # also save every pose, to replay later
    python tools/body_tracker.py --snapshots fotos # a picture of each camera every 2 s
    python tools/body_tracker.py --ball-color magenta  # only a bulb of the Move's colour

The first run downloads the pose model next to this script. On Windows, allow Python through the
firewall on private networks when asked.
"""
import argparse
import hashlib
import json
import math
import os
import socket
import struct
import sys
import threading
import time
import urllib.request
import zipfile
from pathlib import Path

os.environ.setdefault("GLOG_minloglevel", "2")  # MediaPipe's start-up chatter

try:
    import av
    import numpy as np
except ImportError:  # the packets work without them; main() says what to install
    av = np = None
try:
    import cv2
except ImportError:  # only the LiteRT pose and the Move's ball need it
    cv2 = None

PORT = 9946
MAGIC = b"HMD1"
TYPE_DISCOVER = 4
TYPE_PC_HERE = 5
TYPE_BODY = 7
TYPE_TIME_PING = 8
TYPE_TIME_PONG = 9
TYPE_VIDEO = 10
TYPE_KEYFRAME_REQUEST = 11
# Here to the headset driver: one camera frame's pose, laid out as TYPE_BODY but with the frame's
# age (ms from the camera taking it to this packet leaving) where the inference time goes, so the
# driver puts it on its own clock without sharing ours. See driver/src/body_protocol.h.
TYPE_BODY_FRAME = 12
DRIVER_PORT = 9947
FLAG_POSE = 1
FLAG_EXPOSURE_LOCKED = 2
FLAG_BALL = 4
FLAG_KEYFRAME = 8
MAX_PAYLOAD = 1400

# Little-endian, laid out as in android/camera/.../BodyProtocol.kt.
HEADER = struct.Struct("<4sB3xIq")  # magic, type, seq, timestamp ns
BODY = struct.Struct("<IHHfBBxxfff")  # device, width, height, inference ms, flags, count, ball x/y/r
LANDMARK = struct.Struct("<5f")  # x, y, z, visibility, presence
PONG = struct.Struct("<qI")  # PC time echoed back, device id
VIDEO = struct.Struct("<IHHHHBBxx")  # device, width, height, chunk, chunk count, quarter turns, flags
HIPS, KNEES, ANKLES = (23, 24), (25, 26), (27, 28)

MODEL_DIR = Path(__file__).resolve().parent / "models"
MODEL_URL = "https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_{0}/float16/1/pose_landmarker_{0}.task"
MODEL_SHA256 = {
    "lite": "59929e1d1ee95287735ddd833b19cf4ac46d29bc7afddbbf6753c459690d574a",
    "full": "5134a3aad27a58b93da0088d431f366da362b44e3ccfbe3462b3827a839011b1",
    "heavy": "64437af838a65d18e5ba7a0d39b465540069bc8aae8308de3e318aad31fcbc7b",
}
# A frame this far behind the newest means the phone's app started again and counts from 1.
RESTART_GAP = 300
# Often enough that a whole picture lost on the way, the frames' largest, costs a few frames, not ten.
KEYFRAME_REQUEST_EVERY_NS = 100_000_000
SNAPSHOT_EVERY_NS = 2_000_000_000


def header(kind, seq=0, timestamp_ns=0):
    return HEADER.pack(MAGIC, kind, seq, timestamp_ns)


def parse_header(data):
    """(type, seq, timestamp ns), or None if the packet is not ours."""
    if len(data) < HEADER.size:
        return None
    magic, kind, seq, timestamp = HEADER.unpack_from(data)
    if magic != MAGIC:
        return None
    return kind, seq, timestamp


def pack_body(seq, capture_ns, device, width, height, inference_ms, flags, landmarks, ball=(0.0, 0.0, 0.0),
              kind=TYPE_BODY):
    """A body packet: one frame's pose, from here to the phone (or, as TYPE_BODY_FRAME, the driver)."""
    data = header(kind, seq, capture_ns)
    data += BODY.pack(device, width, height, inference_ms, flags, len(landmarks), *ball)
    for point in landmarks:
        data += LANDMARK.pack(*point)
    return data


def parse_body(data, kind=TYPE_BODY):
    """A dict with the frame's fields, or None if the packet is short or not a body packet."""
    head = parse_header(data)
    if head is None or head[0] != kind or len(data) < HEADER.size + BODY.size:
        return None
    _, seq, capture_ns = head
    device, width, height, inference_ms, flags, count, ball_x, ball_y, ball_r = BODY.unpack_from(data, HEADER.size)
    start = HEADER.size + BODY.size
    if len(data) < start + count * LANDMARK.size:
        return None
    landmarks = [LANDMARK.unpack_from(data, start + i * LANDMARK.size) for i in range(count)]
    return {
        "seq": seq, "capture_ns": capture_ns, "device": device, "width": width, "height": height,
        "inference_ms": inference_ms, "flags": flags, "ball": (ball_x, ball_y, ball_r), "landmarks": landmarks,
    }


def pack_video(seq, capture_ns, device, width, height, rotation, flags, data):
    """One encoded frame split into video packets, as the phone sends it; for tests and fake phones."""
    count = max(1, -(-len(data) // MAX_PAYLOAD))
    return [
        header(TYPE_VIDEO, seq, capture_ns)
        + VIDEO.pack(device, width, height, i, count, rotation, flags)
        + data[i * MAX_PAYLOAD:(i + 1) * MAX_PAYLOAD]
        for i in range(count)
    ]


def parse_video(data):
    """A dict with one video packet's fields and its piece of the frame, or None."""
    head = parse_header(data)
    if head is None or head[0] != TYPE_VIDEO or len(data) < HEADER.size + VIDEO.size:
        return None
    _, seq, capture_ns = head
    device, width, height, chunk, count, rotation, flags = VIDEO.unpack_from(data, HEADER.size)
    if chunk >= count:
        return None
    return {
        "seq": seq, "capture_ns": capture_ns, "device": device, "width": width, "height": height,
        "chunk": chunk, "chunk_count": count, "rotation": rotation & 3, "flags": flags,
        "payload": data[HEADER.size + VIDEO.size:],
    }


def parse_pong(data):
    """(ping seq, phone ns, PC ns echoed, device), or None."""
    head = parse_header(data)
    if head is None or head[0] != TYPE_TIME_PONG or len(data) < HEADER.size + PONG.size:
        return None
    _, seq, phone_ns = head
    pc_ns, device = PONG.unpack_from(data, HEADER.size)
    return seq, phone_ns, pc_ns, device


class Clock:
    """Phone-to-PC clock offset from ping round trips: the quickest recent trip is the most exact."""

    def __init__(self, keep=20):
        self.samples = []  # (round trip ns, offset ns)
        self.keep = keep

    def add(self, pc_sent_ns, phone_ns, pc_back_ns):
        trip = pc_back_ns - pc_sent_ns
        if trip < 0:
            return
        offset = (pc_sent_ns + pc_back_ns) // 2 - phone_ns
        self.samples = (self.samples + [(trip, offset)])[-self.keep:]

    def best(self):
        """(offset ns, round trip ns) to add to a phone time to get PC time, or None before any ping."""
        if not self.samples:
            return None
        trip, offset = min(self.samples)
        return offset, trip


class FrameAssembler:
    """Joins the packets of each frame. A packet of a newer frame drops an unfinished one: UDP
    does not resend, so its missing pieces are not coming."""

    def __init__(self):
        self.newest = None
        self.parts = []
        self.missing = 0

    def add(self, chunk):
        """The whole frame (the last packet's fields, with "data") once all its pieces arrived."""
        seq = chunk["seq"]
        if self.newest is not None and seq < self.newest:
            if self.newest - seq < RESTART_GAP:
                return None  # late, or a repeat of a frame already done
            self.newest = None
        if seq != self.newest:
            self.newest = seq
            self.parts = [None] * chunk["chunk_count"]
            self.missing = len(self.parts)
        index = chunk["chunk"]
        if index >= len(self.parts) or self.parts[index] is not None:
            return None
        self.parts[index] = chunk["payload"]
        self.missing -= 1
        if self.missing:
            return None
        frame = {k: v for k, v in chunk.items() if k not in ("payload", "chunk", "chunk_count")}
        frame["data"] = b"".join(self.parts)
        self.parts = []  # so repeats of its packets are ignored
        return frame


class Stats:
    """One camera's numbers since the last report. The pose thread adds, the main thread reports."""

    def __init__(self):
        self.lock = threading.Lock()
        self.reset()

    def reset(self):
        self.frames = 0
        self.bytes = 0
        self.lost = 0
        self.discarded = 0  # frames that came while waiting for a whole picture after a loss
        self.poses = 0
        self.persons = 0
        self.balls = 0
        self.skipped = 0
        self.inference = 0.0
        self.arrivals = []
        self.delays = []
        self.locked = False
        self.waiting_keyframe = False
        self.visibility = {name: [] for name in ("cadera", "rodillas", "tobillos")}

    def report(self, name, seconds, clock):
        with self.lock:
            if self.frames == 0:
                text = f"{name}: sin video"
                if self.waiting_keyframe:
                    text += " entero (esperando un cuadro completo)"
                self.reset()
                return text
            parts = [f"{self.frames / seconds:.1f} cuadros/s", f"{self.bytes * 8 / seconds / 1e6:.1f} Mb/s"]
            if self.poses:
                parts.append(f"pose {self.poses / seconds:.1f}/s")
                parts.append(f"persona en {100 * self.persons / self.poses:.0f}%")
                parts.append(f"modelo {self.inference / self.poses:.0f} ms")
            else:
                parts.append("sin poses todavía")
            if self.arrivals and self.delays:
                arrivals, delays = sorted(self.arrivals), sorted(self.delays)
                parts.append(f"llega a los {arrivals[len(arrivals) // 2]:.0f} ms")
                parts.append(f"pose lista a los {delays[len(delays) // 2]:.0f} ms (máx {delays[-1]:.0f})")
            parts.append(f"perdidos {self.lost}")
            if self.discarded:
                parts.append(f"descartados {self.discarded} (esperando un cuadro completo)")
            if self.skipped:
                parts.append(f"saltados {self.skipped} (el modelo seguía con el anterior)")
            if self.persons:
                parts.append(" ".join(f"{k} {100 * sum(v) / len(v):.0f}%" for k, v in self.visibility.items()))
            if self.balls:
                parts.append(f"bola del Move en {100 * self.balls / self.poses:.0f}%")
            sync = clock.best()
            if sync is not None:
                parts.append(f"reloj ±{sync[1] / 2e6:.1f} ms")
            parts.append("exposición fija" if self.locked else "exposición automática")
            self.reset()
        return f"{name}: " + ", ".join(parts)


class PoseWorker:
    """Runs the pose model of one camera on its own thread, always on the newest frame: if the CPU
    falls behind, older frames are skipped rather than queued, so the delay does not grow."""

    def __init__(self, make_pose, run, stats):
        self.make_pose = make_pose
        self.run = run
        self.stats = stats
        self.cond = threading.Condition()
        self.job = None
        self.stopped = False
        self.thread = threading.Thread(target=self.loop, name="pose", daemon=True)
        self.thread.start()

    def submit(self, job):
        with self.cond:
            if self.job is not None:
                with self.stats.lock:
                    self.stats.skipped += 1
            self.job = job
            self.cond.notify()

    def stop(self):
        with self.cond:
            self.stopped = True
            self.cond.notify()
        self.thread.join()

    def loop(self):
        pose = self.make_pose()
        try:
            while True:
                with self.cond:
                    while self.job is None and not self.stopped:
                        self.cond.wait()
                    if self.stopped:
                        return
                    job, self.job = self.job, None
                self.run(pose, job)
        finally:
            pose.close()


class Camera:
    def __init__(self, device, address):
        self.device = device
        self.address = address
        self.name = f"Cámara {device & 0xFFFF:04X} ({address[0]})"
        self.clock = Clock()
        self.assembler = FrameAssembler()
        self.stats = Stats()
        self.decoder = None
        self.last_seq = None
        self.waiting_keyframe = True
        self.next_keyframe_request_ns = 0
        self.worker = None
        self.pose = None  # when the pose runs on the main thread
        self.last_pose_ms = 0
        self.next_snapshot_ns = 0
        self.snapshots = 0

    def decode(self, data):
        """(worked, the decoded frame or None)."""
        if self.decoder is None:
            self.decoder = av.CodecContext.create("h264", "r")
            self.decoder.thread_type = "SLICE"  # frame threads would hold each frame back
        try:
            frames = self.decoder.decode(av.Packet(data))
        except av.error.FFmpegError:
            return False, None
        return True, frames[-1] if frames else None


class Tracker:
    """Everything but the socket: answers the phones, decodes their video, runs the pose and sends
    it back, and prints the stats.

    send(data, address) sends a packet; out(text) prints a line; make_pose() opens a pose model
    (an object with detect(rgb image, ms) -> points and close()); now() gives monotonic ns. With
    threaded=False the pose runs on the caller's thread, for tests. send_driver(data), if given,
    hands each frame's pose to the headset driver once the phone's clock is known.
    """

    def __init__(self, send, make_pose, out=print, record=None, now=time.perf_counter_ns,
                 threaded=True, report_every_ns=2_000_000_000, send_driver=None, snapshots=None, ball_hue=None):
        self.send = send
        self.send_driver = send_driver
        self.snapshots = snapshots  # a folder for a picture of each camera every few seconds
        self.ball_hue = ball_hue  # of the Move's bulb, in degrees; None: any colour
        self.make_pose = make_pose
        self.out = out
        self.record = record
        self.record_lock = threading.Lock()
        self.now = now
        self.threaded = threaded
        self.report_every_ns = report_every_ns
        self.cameras = {}
        self.ping_seq = 0
        self.next_ping_ns = None
        self.next_report_ns = None

    def tick(self, now_ns):
        """Pings each phone once a second and prints the stats every report_every_ns."""
        if self.next_ping_ns is None:
            self.next_ping_ns = now_ns
            self.next_report_ns = now_ns + self.report_every_ns
        if now_ns >= self.next_ping_ns:
            self.next_ping_ns = now_ns + 1_000_000_000
            for camera in self.cameras.values():
                self.ping_seq += 1
                self.send(header(TYPE_TIME_PING, self.ping_seq, now_ns), camera.address)
        if now_ns >= self.next_report_ns:
            seconds = (self.report_every_ns + now_ns - self.next_report_ns) / 1e9
            for camera in self.cameras.values():
                self.out(camera.stats.report(camera.name, seconds, camera.clock))
            if not self.cameras:
                self.out("Todavía no llega nada. ¿La app está abierta y en la misma red o con el cable?")
            self.next_report_ns = now_ns + self.report_every_ns

    def handle(self, data, address, arrived_ns):
        head = parse_header(data)
        if head is None:
            return
        kind = head[0]
        if kind == TYPE_DISCOVER:
            self.send(header(TYPE_PC_HERE), address)
        elif kind == TYPE_VIDEO:
            chunk = parse_video(data)
            if chunk is not None:
                self.video(chunk, address, arrived_ns)
        elif kind == TYPE_TIME_PONG:
            pong = parse_pong(data)
            camera = self.cameras.get(pong[3]) if pong else None
            if camera is not None:
                camera.clock.add(pong[2], pong[1], arrived_ns)

    def close(self):
        for camera in self.cameras.values():
            self.drop(camera)
        self.cameras.clear()

    def drop(self, camera):
        if camera.worker is not None:
            camera.worker.stop()
        if camera.pose is not None:
            camera.pose.close()

    def video(self, chunk, address, arrived_ns):
        camera = self.cameras.get(chunk["device"])
        if camera is None or camera.address != address:
            # A new phone, or the same one after its app started again.
            if camera is not None:
                self.drop(camera)
            camera = self.cameras[chunk["device"]] = Camera(chunk["device"], address)
            self.out(f"Nueva cámara {chunk['device'] & 0xFFFF:04X} desde {address[0]}, "
                     f"video {chunk['width']}x{chunk['height']}")
        frame = camera.assembler.add(chunk)
        if frame is None:
            return
        keyframe = bool(frame["flags"] & FLAG_KEYFRAME)
        with camera.stats.lock:
            camera.stats.frames += 1
            camera.stats.bytes += len(frame["data"])
            camera.stats.locked = bool(frame["flags"] & FLAG_EXPOSURE_LOCKED)
            if camera.last_seq is not None and frame["seq"] > camera.last_seq + 1:
                camera.stats.lost += frame["seq"] - camera.last_seq - 1
                camera.waiting_keyframe = True
            if keyframe:
                camera.waiting_keyframe = False
            camera.stats.waiting_keyframe = camera.waiting_keyframe
        camera.last_seq = frame["seq"]
        # After a loss the frames that follow only make sense on top of the lost one: skip them
        # until a whole picture arrives.
        if camera.waiting_keyframe:
            self.discard(camera, arrived_ns)
            return
        worked, image = camera.decode(frame["data"])
        if not worked:
            camera.waiting_keyframe = True
            self.discard(camera, arrived_ns)
            return
        if image is None:
            return
        job = (frame, image, arrived_ns)
        if not self.threaded:
            if camera.pose is None:
                camera.pose = self.make_pose()
            self.run_pose(camera, camera.pose, job)
            return
        if camera.worker is None:
            camera.worker = PoseWorker(self.make_pose, lambda pose, j, c=camera: self.run_pose(c, pose, j), camera.stats)
        camera.worker.submit(job)

    def discard(self, camera, now_ns):
        with camera.stats.lock:
            camera.stats.discarded += 1
        self.request_keyframe(camera, now_ns)

    def request_keyframe(self, camera, now_ns):
        if now_ns >= camera.next_keyframe_request_ns:
            camera.next_keyframe_request_ns = now_ns + KEYFRAME_REQUEST_EVERY_NS
            self.send(header(TYPE_KEYFRAME_REQUEST), camera.address)

    def run_pose(self, camera, pose, job):
        frame, image, arrived_ns = job
        rgb = image.to_ndarray(format="rgb24")
        if frame["rotation"]:
            # The phone sends the sensor's picture and how far to turn it clockwise to stand upright.
            rgb = np.ascontiguousarray(np.rot90(rgb, -frame["rotation"]))
        height, width = rgb.shape[:2]
        # The model wants strictly increasing times in ms.
        ms = max(frame["capture_ns"] // 1_000_000, camera.last_pose_ms + 1)
        camera.last_pose_ms = ms
        start = self.now()
        points = pose.detect(rgb, ms)
        done = self.now()
        inference_ms = (done - start) / 1e6
        ball = find_ball(rgb, points, self.ball_hue)
        flags = (FLAG_POSE if points else 0) | (FLAG_BALL if ball else 0) | (frame["flags"] & FLAG_EXPOSURE_LOCKED)
        ball = ball or (0.0, 0.0, 0.0)
        packet = pack_body(frame["seq"], frame["capture_ns"], camera.device, width, height, inference_ms, flags, points,
                           ball)
        self.send(packet, camera.address)

        sync = camera.clock.best()
        if self.send_driver and sync is not None:
            captured = frame["capture_ns"] + sync[0]
            age_ms = (self.now() - captured) / 1e6
            self.send_driver(pack_body(frame["seq"], captured, camera.device, width, height, age_ms, flags, points,
                                       ball, kind=TYPE_BODY_FRAME))
        if self.snapshots and done >= camera.next_snapshot_ns:
            # Numbered, and marked when a bulb was found, to look for the ones it got wrong.
            camera.next_snapshot_ns = done + SNAPSHOT_EVERY_NS
            camera.snapshots += 1
            found = flags & FLAG_BALL
            name = f"camara_{camera.device & 0xFFFF:04X}_{camera.snapshots:04d}{'_bola' if found else ''}.jpg"
            save_snapshot(Path(self.snapshots) / name, rgb, points, ball if found else None)
        stats = camera.stats
        with stats.lock:
            stats.poses += 1
            stats.balls += bool(flags & FLAG_BALL)
            stats.inference += inference_ms
            if sync is not None:
                captured = frame["capture_ns"] + sync[0]
                stats.arrivals.append((arrived_ns - captured) / 1e6)
                stats.delays.append((done - captured) / 1e6)
            if points and len(points) > max(ANKLES):
                stats.persons += 1
                for name, pair in zip(stats.visibility, (HIPS, KNEES, ANKLES)):
                    stats.visibility[name].append(min(points[i][3] for i in pair))
        if self.record:
            line = parse_body(packet)
            line["pc_ns"] = done
            line["offset_ns"] = sync[0] if sync else None
            with self.record_lock:
                self.record.write(json.dumps(line) + "\n")


def ssd_anchors():
    """The centres of the pose detector's 2254 boxes, in the order of its output (x, y in 0..1)."""
    centres = []
    # 28x28 cells of 2 boxes, 14x14 of 2 and 7x7 of 6: the 5 layers, those of the same stride together.
    for cells, per_cell in ((28, 2), (14, 2), (7, 6)):
        for y in range(cells):
            for x in range(cells):
                centres += [((x + 0.5) / cells, (y + 0.5) / cells)] * per_cell
    return np.array(centres, dtype=np.float32)


def sigmoid(x):
    return 0.5 + 0.5 * np.tanh(0.5 * x)  # 1 / (1 + e^-x), without overflowing for large -x


def pose_square(centre, scale, width, height):
    """The square around a person that the landmark model looks at, from the centre of their hips
    and a point whose distance gives their size and whose direction which way they stand (pixels).
    As MediaPipe's rects: centre and size over the picture's width and height, rotation in radians."""
    (x0, y0), (x1, y1) = centre, scale
    size = 2 * math.hypot(x1 - x0, y1 - y0) * 1.25  # the margin the model was trained with
    rotation = math.pi / 2 - math.atan2(y0 - y1, x1 - x0)
    rotation -= 2 * math.pi * math.floor((rotation + math.pi) / (2 * math.pi))
    return x0 / width, y0 / height, size / width, size / height, rotation


def refine_from_heatmap(points, heatmap, kernel=7, min_confidence=0.5):
    """Moves each point (x, y over the model's picture) to the weighted centre of the model's
    heatmap around it, where the heatmap is sure enough. In place."""
    rows, cols, count = heatmap.shape
    col = (points[:, 0] * cols).astype(int)  # towards zero, as MediaPipe's C++ does
    row = (points[:, 1] * rows).astype(int)
    inside = (col >= 0) & (col < cols) & (row >= 0) & (row < rows)
    offsets = np.arange(kernel) - kernel // 2
    r, c = row[:, None] + offsets, col[:, None] + offsets
    # The window shrinks at the heatmap's edges.
    valid = ((r >= 0) & (r < rows))[:, :, None] & ((c >= 0) & (c < cols))[:, None, :]
    values = heatmap[np.clip(r, 0, rows - 1)[:, :, None], np.clip(c, 0, cols - 1)[:, None, :],
                     np.arange(count)[:, None, None]]
    confidence = sigmoid(values) * valid
    total = confidence.sum(axis=(1, 2))
    refine = inside & (confidence.max(axis=(1, 2)) >= min_confidence) & (total > 0)
    points[refine, 0] = (confidence * c[:, None, :]).sum(axis=(1, 2))[refine] / cols / total[refine]
    points[refine, 1] = (confidence * r[:, :, None]).sum(axis=(1, 2))[refine] / rows / total[refine]


class PoseSmoother:
    """MediaPipe's smoothing of one person's points from frame to frame: a low-pass filter on how
    visible each point is, and a one-euro filter on where it is, which smooths a point that stands
    still a lot and one that moves quickly little. Positions are filtered in pixels, against the
    person's size."""

    MIN_CUTOFF, BETA, SPEED_CUTOFF = 0.05, 80.0, 1.0
    VISIBILITY_ALPHA = 0.1

    def __init__(self):
        self.last_ns = -1
        self.frequency = 30.0
        self.raw = self.value = self.speed = self.visibility = None

    def alpha(self, cutoff):
        return 1 / (1 + self.frequency / (2 * math.pi * cutoff))

    def apply(self, points, timestamp_ms, width, height, person_size):
        """points: rows of x, y, z, visibility, presence as the model gives them; returns them smoothed."""
        out = points.copy()
        visibility = points[:, 3]
        if self.visibility is not None:
            visibility = self.VISIBILITY_ALPHA * visibility + (1 - self.VISIBILITY_ALPHA) * self.visibility
        out[:, 3] = self.visibility = visibility

        now_ns = timestamp_ms * 1_000_000
        if now_ns <= self.last_ns:
            return out
        if self.last_ns != 0 and now_ns != 0:
            self.frequency = 1e9 / (now_ns - self.last_ns)
        self.last_ns = now_ns
        scale = np.array([width, height, width], dtype=np.float64)
        values = points[:, :3] * scale
        if self.raw is None:
            self.value, self.speed = values, np.zeros_like(values)
        else:
            change = (values - self.raw) * self.frequency / person_size
            self.speed = self.speed + self.alpha(self.SPEED_CUTOFF) * (change - self.speed)
            self.value = self.value + self.alpha(self.MIN_CUTOFF + self.BETA * np.abs(self.speed)) * (values - self.value)
        self.raw = values
        out[:, :3] = self.value / scale
        return out


class LiteRtPose:
    """MediaPipe's pose landmarker for a video, one person and 33 points, done here step by step and
    run on LiteRT, which unlike MediaPipe's Python package can spread each model over several cores.

    The model file holds two models: a person detector, run only while no one is being followed,
    and the landmark model, run on a square around the person that the last frame's points
    predict. The steps between them follow MediaPipe's pose landmarker so the points come out the
    same; test_body_tracker.py compares the two.
    """

    DETECTOR_SIZE, LANDMARKS_SIZE = 224, 256
    MIN_DETECTION, MIN_PRESENCE, MIN_OVERLAP = 0.5, 0.5, 0.3

    def __init__(self, model_path, threads=4):
        from ai_edge_litert.interpreter import Interpreter

        self.cv2 = cv2
        with zipfile.ZipFile(model_path) as bundle:
            self.detector = Interpreter(model_content=bundle.read("pose_detector.tflite"), num_threads=threads)
            self.landmarker = Interpreter(model_content=bundle.read("pose_landmarks_detector.tflite"),
                                          num_threads=threads)
        for model in (self.detector, self.landmarker):
            model.allocate_tensors()

        def output(model, shape):  # found by shape, as the names are just Identity, Identity_1…
            return next(d["index"] for d in model.get_output_details() if tuple(d["shape"]) == shape)

        self.detector_in = self.detector.get_input_details()[0]["index"]
        self.boxes_out = output(self.detector, (1, 2254, 12))
        self.scores_out = output(self.detector, (1, 2254, 1))
        self.landmarker_in = self.landmarker.get_input_details()[0]["index"]
        self.points_out = output(self.landmarker, (1, 195))
        self.presence_out = output(self.landmarker, (1, 1))
        self.heatmap_out = output(self.landmarker, (1, 64, 64, 39))
        self.anchors = ssd_anchors()
        self.square = None  # where to look for the person in the next frame
        self.smoother = None
        self.picture_size = None

    def detect(self, rgb, timestamp_ms):
        """The points, (x, y, z, visibility, presence) each, or [] if there is no one."""
        height, width = rgb.shape[:2]
        if (width, height) != self.picture_size:
            self.picture_size = (width, height)
            self.smoother = None
        square = self.square or self.find_person(rgb)
        found = self.landmarks(rgb, square) if square else None
        if found is None:
            self.square = self.smoother = None
            return []
        points, self.square = found
        if self.smoother is None:
            self.smoother = PoseSmoother()
        person_size = (self.square[2] * width + self.square[3] * height) / 2
        points = self.smoother.apply(points, timestamp_ms, width, height, person_size)
        return [tuple(p) for p in points.tolist()]

    def close(self):
        self.detector = self.landmarker = None

    def crop(self, rgb, rect, size, border):
        """The rotated rect (centre x, y, width, height in pixels, rotation) as a size x size picture."""
        cx, cy, w, h, rotation = rect
        corners = self.cv2.boxPoints(((cx, cy), (w, h), math.degrees(rotation)))
        target = np.array([[0, size], [0, 0], [size, 0], [size, size]], dtype=np.float32)
        matrix = self.cv2.getPerspectiveTransform(corners, target)
        picture = self.cv2.warpPerspective(rgb, matrix, (size, size), flags=self.cv2.INTER_LINEAR, borderMode=border)
        return picture.astype(np.float32)

    def find_person(self, rgb):
        """The square around the most likely person in the whole picture, or None."""
        height, width = rgb.shape[:2]
        side = max(width, height)
        size = self.DETECTOR_SIZE
        # The whole picture, with black bars to make it square.
        picture = self.crop(rgb, (width / 2, height / 2, side, side, 0.0), size, self.cv2.BORDER_CONSTANT)
        self.detector.set_tensor(self.detector_in, (picture * (2 / 255) - 1)[None])
        self.detector.invoke()
        scores = sigmoid(np.clip(self.detector.get_tensor(self.scores_out)[0, :, 0], -100, 100))
        found = np.flatnonzero(scores >= self.MIN_DETECTION)
        if not found.size:
            return None
        found = found[np.argsort(-scores[found], kind="stable")]
        raw = self.detector.get_tensor(self.boxes_out)[0, found] / size
        anchors = self.anchors[found]
        centres, half = raw[:, 0:2] + anchors, raw[:, 2:4] / 2
        boxes = np.hstack([centres - half, centres + half])
        keypoints = raw[:, 4:12].reshape(-1, 4, 2) + anchors[:, None]
        # The best box, averaged with those that overlap it, weighted by their scores.
        low, high = np.maximum(boxes[0, :2], boxes[:, :2]), np.minimum(boxes[0, 2:], boxes[:, 2:])
        common = np.prod(np.clip(high - low, 0, None), axis=1)
        areas = np.prod(boxes[:, 2:] - boxes[:, :2], axis=1)
        near = common / (areas[0] + areas - common) > self.MIN_OVERLAP
        weights = scores[found][near]
        centre, scale = (keypoints[near, :2] * weights[:, None, None]).sum(axis=0) / weights.sum()
        # Keypoint 0 is the centre of the hips, 1 gives the size of the whole body; to pixels
        # of the picture, without the bars.
        bars = np.array([side - width, side - height]) / 2
        return pose_square(centre * side - bars, scale * side - bars, width, height)

    def landmarks(self, rgb, square):
        """(the points over the picture, the square to look at in the next frame), or None if the
        model sees no one there."""
        height, width = rgb.shape[:2]
        x_centre, y_centre, w, h, rotation = square
        if w * width < 1:
            return None
        size = self.LANDMARKS_SIZE
        picture = self.crop(rgb, (x_centre * width, y_centre * height, w * width, h * height, rotation), size,
                            self.cv2.BORDER_REPLICATE)
        self.landmarker.set_tensor(self.landmarker_in, (picture * (1 / 255))[None])
        self.landmarker.invoke()
        if self.landmarker.get_tensor(self.presence_out)[0, 0] <= self.MIN_PRESENCE:
            return None
        raw = self.landmarker.get_tensor(self.points_out).reshape(39, 5)
        points = raw[:, :3].astype(np.float64) / size
        refine_from_heatmap(points, self.landmarker.get_tensor(self.heatmap_out)[0])
        # From the square back to the whole picture.
        x, y = points[:, 0] - 0.5, points[:, 1] - 0.5
        cos, sin = math.cos(rotation), math.sin(rotation)
        points[:, 0] = (cos * x - sin * y) * w + x_centre
        points[:, 1] = (sin * x + cos * y) * h + y_centre
        points[:, 2] *= w
        # Points 33 and 34 are not part of the pose: like the detector's, they say where the
        # person will be next.
        pixels = points[33:35, :2] * (width, height)
        next_square = pose_square(pixels[0], pixels[1], width, height)
        visibility, presence = sigmoid(raw[:33, 3:4]), sigmoid(raw[:33, 4:5])
        return np.hstack([points[:33], visibility, presence]), next_square


WRISTS_AND_ELBOWS = ((15, 13), (16, 14))
# The colours PSMoveServiceEx lights a Move's bulb with, as hues in degrees: the first Move gets
# magenta, the next cyan, then yellow.
BALL_HUES = {"magenta": 300, "cyan": 180, "yellow": 60, "red": 0, "green": 120, "blue": 240}
BALL_HUE_TOLERANCE = 35
# The bulb is 4.5 cm across and the trunk, shoulders to hips, about 50 cm long. With its glow the
# bulb looks up to some three times its size, and half of it may be behind the fingers.
BALL_RADIUS_PER_TRUNK = (0.018, 0.14)


def find_ball(rgb, points, hue=None):
    """Where a lit PS Move bulb is, as (x, y, radius) over the picture's width and height (the
    radius over its width), or None. The driver calibrates the cameras with it. It is looked for
    near each wrist, within a forearm and a bit: the bulb glows brighter than anything else there,
    is coloured, of the bulb's size next to the body, and of its colour, the hue in degrees, if
    given."""
    if not points or cv2 is None:
        return None
    height, width = rgb.shape[:2]
    trunk = trunk_length(points, width, height)
    radius = (BALL_RADIUS_PER_TRUNK[0] * trunk, BALL_RADIUS_PER_TRUNK[1] * trunk) if trunk else None
    best = None
    for wrist, elbow in WRISTS_AND_ELBOWS:
        if points[wrist][3] < 0.3:
            continue
        wx, wy = points[wrist][0] * width, points[wrist][1] * height
        forearm = math.hypot(wx - points[elbow][0] * width, wy - points[elbow][1] * height)
        reach = min(max(1.3 * forearm, 0.08 * height), 0.3 * height)
        x0, y0 = int(max(wx - reach, 0)), int(max(wy - reach, 0))
        x1, y1 = int(min(wx + reach, width)), int(min(wy + reach, height))
        if x1 - x0 < 4 or y1 - y0 < 4:
            continue
        blob = bright_blob(rgb[y0:y1, x0:x1], radius, hue)
        if blob and (best is None or blob[3] > best[3]):
            best = (blob[0] + x0, blob[1] + y0, blob[2], blob[3])
    if best is None:
        return None
    return best[0] / width, best[1] / height, best[2] / width


def trunk_length(points, width, height):
    """From between the shoulders to between the hips, in pixels; None if they are not all seen."""
    if any(points[i][3] < 0.3 for i in (11, 12, 23, 24)):
        return None
    shoulders = (points[11][0] + points[12][0]) * width / 2, (points[11][1] + points[12][1]) * height / 2
    hips = (points[23][0] + points[24][0]) * width / 2, (points[23][1] + points[24][1]) * height / 2
    return math.hypot(shoulders[0] - hips[0], shoulders[1] - hips[1])


def bright_blob(patch, radius=None, hue=None):
    """The most glowing coloured blob in an RGB patch: (centre x, y, radius in pixels, score), or
    None. Glowing means at least two colours near their top (the bulb blows out the camera; a lit
    bulb is magenta, cyan or yellow, or white in its core), which a red shirt in good light is
    not; coloured means saturated, which a white lamp or wall is not and skin is too little.
    radius is (smallest, largest) in pixels, hue the wanted one in degrees."""
    channels = patch.astype(np.int16)
    top, bottom = channels.max(axis=2), channels.min(axis=2)
    coloured = (top >= 150) & (top - bottom >= 0.5 * top)
    hot = np.count_nonzero(channels >= 230, axis=2) >= 2
    count, labels, stats, centres = cv2.connectedComponentsWithStats((coloured | hot).astype(np.uint8),
                                                                     connectivity=8)
    hues = cv2.cvtColor(np.ascontiguousarray(patch), cv2.COLOR_RGB2HSV)[:, :, 0] * (math.pi / 90) if hue is not None \
        else None
    best = None
    for i in range(1, count):
        area = stats[i, cv2.CC_STAT_AREA]
        w, h = stats[i, cv2.CC_STAT_WIDTH], stats[i, cv2.CC_STAT_HEIGHT]
        if area < 5 or area < 0.35 * w * h or max(w, h) > 2.5 * min(w, h):
            continue  # a speck, or not round
        size = math.sqrt(area / math.pi)
        if radius and not radius[0] <= size <= radius[1]:
            continue  # far too big or too small for a bulb by this person
        inside = labels == i
        colour = coloured & inside
        glow = int(np.count_nonzero(hot & inside))
        if np.count_nonzero(colour) < 0.1 * area or glow < 2:
            continue  # a white light, or something coloured that does not glow
        if hues is not None:
            angles = hues[colour]
            mean = math.degrees(math.atan2(np.sin(angles).mean(), np.cos(angles).mean()))
            if abs((mean - hue + 180) % 360 - 180) > BALL_HUE_TOLERANCE:
                continue  # another colour than the Move's
        score = glow / area
        if best is None or score > best[3]:
            best = (centres[i][0] + 0.5, centres[i][1] + 0.5, size, score)
    return best


def save_snapshot(path, rgb, points, ball):
    """A JPEG of what the pose and ball finders saw: the points in green and the ball in a ring."""
    if cv2 is None:
        return
    picture = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    height, width = picture.shape[:2]
    for x, y, _, visibility, _ in points:
        if visibility >= 0.5:
            cv2.circle(picture, (int(x * width), int(y * height)), 3, (0, 255, 0), -1)
    if ball:
        cv2.circle(picture, (int(ball[0] * width), int(ball[1] * height)), int(ball[2] * width) + 6, (255, 255, 0), 2)
    path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(path), picture)


class MediaPipePose:
    """MediaPipe's pose landmarker for a video: one person, 33 points. The same as LiteRtPose but
    on one core, as MediaPipe's Python package has no setting for more."""

    def __init__(self, model_path):
        import mediapipe as mp
        from mediapipe.tasks.python import BaseOptions, vision

        self.mp = mp
        self.landmarker = vision.PoseLandmarker.create_from_options(vision.PoseLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=str(model_path)),
            running_mode=vision.RunningMode.VIDEO,
            num_poses=1,
        ))

    def detect(self, rgb, timestamp_ms):
        """The points, (x, y, z, visibility, presence) each, or [] if there is no one."""
        image = self.mp.Image(image_format=self.mp.ImageFormat.SRGB, data=rgb)
        result = self.landmarker.detect_for_video(image, timestamp_ms)
        if not result.pose_landmarks:
            return []
        return [(p.x, p.y, p.z, p.visibility or 0.0, p.presence or 0.0) for p in result.pose_landmarks[0]]

    def close(self):
        self.landmarker.close()


def ensure_model(name, folder=MODEL_DIR, out=print):
    """The model's file, downloaded the first time and checked against its known hash."""
    url = MODEL_URL.format(name)
    path = Path(folder) / url.rsplit("/", 1)[1]
    if path.exists() and hashlib.sha256(path.read_bytes()).hexdigest() == MODEL_SHA256[name]:
        return path
    out(f"Descargando el modelo de pose «{name}»…")
    with urllib.request.urlopen(url, timeout=60) as response:
        data = response.read()
    if hashlib.sha256(data).hexdigest() != MODEL_SHA256[name]:
        raise RuntimeError("El modelo descargado no es el esperado; prueba otra vez")
    path.parent.mkdir(parents=True, exist_ok=True)
    partial = path.with_suffix(".part")
    partial.write_bytes(data)
    partial.replace(path)
    return path


def default_threads():
    """A quarter of the PC's cores, from 2 to 8: with two cameras the model took 24 ms on 4 cores
    each where one camera took 14, and more cores bring that back down."""
    return max(2, min(8, (os.cpu_count() or 8) // 4))


def run(port, record_path, model, threads, engine, driver_port, snapshots, ball_color):
    model_path = ensure_model(model)
    if engine == "mediapipe":
        make_pose, cores = (lambda: MediaPipePose(model_path)), "1 núcleo, con MediaPipe"
    else:
        make_pose, cores = (lambda: LiteRtPose(model_path, threads)), f"{threads} núcleos por cámara"
    make_pose().close()  # fail here, clearly, rather than on the first frame
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # Room for a burst: a whole picture is tens of packets at once.
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
    sock.bind(("0.0.0.0", port))
    sock.settimeout(0.05)
    record = open(record_path, "w", encoding="utf-8") if record_path else None
    # Its own socket: on Windows, packets to a driver that is not running would make the phones'
    # socket report errors.
    driver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send_driver(data):
        try:
            driver.sendto(data, ("127.0.0.1", driver_port))
        except OSError:
            pass

    tracker = Tracker(sock.sendto, make_pose, record=record, send_driver=send_driver, snapshots=snapshots,
                      ball_hue=BALL_HUES.get(ball_color))
    ball = f"; bola del Move {ball_color}" if ball_color in BALL_HUES else ""
    print(f"Modelo «{model}» listo ({cores}{ball}). Esperando las cámaras en el puerto UDP {port}… "
          "(Ctrl+C para salir)")
    try:
        while True:
            tracker.tick(time.perf_counter_ns())
            try:
                data, address = sock.recvfrom(2048)
            except socket.timeout:
                continue
            except ConnectionResetError:
                continue  # Windows reports an earlier packet that found no one
            tracker.handle(data, address, time.perf_counter_ns())
    except KeyboardInterrupt:
        pass
    finally:
        tracker.close()
        if record:
            record.close()
        sock.close()
        driver.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument("--model", choices=sorted(MODEL_SHA256), default="full",
                        help="lite is quicker, heavy more exact; full (default) is in between")
    parser.add_argument("--threads", type=int, default=default_threads(),
                        help=f"cores for each camera's pose model (default {default_threads()}: a quarter of this "
                             "PC's, so two cameras leave half for the game, from 2 to 8)")
    parser.add_argument("--engine", choices=("litert", "mediapipe"), default="litert",
                        help="mediapipe runs the same model on one core (needs pip install mediapipe)")
    parser.add_argument("--record", metavar="FILE", help="save every pose as one JSON line")
    parser.add_argument("--driver-port", type=int, default=DRIVER_PORT,
                        help=f"UDP port of the headset driver on this PC (default {DRIVER_PORT})")
    parser.add_argument("--snapshots", metavar="FOLDER",
                        help="save what each camera sees, with the points and the Move's ball, every 2 s")
    parser.add_argument("--ball-color", choices=sorted(BALL_HUES) + ["any"], default="any",
                        help="the colour of the lit Move's bulb, so nothing else is taken for it (default any)")
    args = parser.parse_args()
    if args.threads < 1:
        parser.error("--threads must be 1 or more")
    try:
        if args.engine == "mediapipe":
            import mediapipe  # noqa: F401
        else:
            import ai_edge_litert  # noqa: F401
        missing = av is None or np is None or cv2 is None
    except ImportError:
        missing = True
    if missing:
        print("Faltan librerías. Instálalas con:  pip install -r tools/requirements-body.txt"
              + ("  y  pip install mediapipe" if args.engine == "mediapipe" else ""))
        return 1
    run(args.port, args.record, args.model, args.threads, args.engine, args.driver_port, args.snapshots,
        args.ball_color)
    return 0


if __name__ == "__main__":
    sys.exit(main())
