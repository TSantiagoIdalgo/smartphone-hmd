#!/usr/bin/env python3
"""Body tracking on the PC: finds the pose of the person the body camera phones see.

The phones (the "HMD Cámara" app) send their camera's video here. For each phone this decodes it,
finds the person's 33 pose points with MediaPipe on the CPU and sends them back to the phone, which
draws them on its screen. Every 2 s it prints per phone: frames per second, how often it sees a
person, the model's time per frame, the delay from the camera to the finished pose, lost frames,
and how sure it is of the hips, knees and ankles.

    pip install -r tools/requirements-body.txt
    python tools/body_tracker.py                   # watch
    python tools/body_tracker.py --model lite      # quicker model, less exact
    python tools/body_tracker.py --record a.jsonl  # also save every pose, to replay later

The first run downloads the pose model next to this script. On Windows, allow Python through the
firewall on private networks when asked.
"""
import argparse
import hashlib
import json
import os
import socket
import struct
import sys
import threading
import time
import urllib.request
from pathlib import Path

os.environ.setdefault("GLOG_minloglevel", "2")  # MediaPipe's start-up chatter

try:
    import av
    import numpy as np
except ImportError:  # the packets work without them; main() says what to install
    av = np = None

PORT = 9946
MAGIC = b"HMD1"
TYPE_DISCOVER = 4
TYPE_PC_HERE = 5
TYPE_BODY = 7
TYPE_TIME_PING = 8
TYPE_TIME_PONG = 9
TYPE_VIDEO = 10
TYPE_KEYFRAME_REQUEST = 11
FLAG_POSE = 1
FLAG_EXPOSURE_LOCKED = 2
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
KEYFRAME_REQUEST_EVERY_NS = 300_000_000


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


def pack_body(seq, capture_ns, device, width, height, inference_ms, flags, landmarks, ball=(0.0, 0.0, 0.0)):
    """A body packet: one frame's pose, from here to the phone."""
    data = header(TYPE_BODY, seq, capture_ns)
    data += BODY.pack(device, width, height, inference_ms, flags, len(landmarks), *ball)
    for point in landmarks:
        data += LANDMARK.pack(*point)
    return data


def parse_body(data):
    """A dict with the frame's fields, or None if the packet is short or not a body packet."""
    head = parse_header(data)
    if head is None or head[0] != TYPE_BODY or len(data) < HEADER.size + BODY.size:
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
        self.poses = 0
        self.persons = 0
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
            if self.skipped:
                parts.append(f"saltados {self.skipped} (la CPU no da abasto)")
            if self.persons:
                parts.append(" ".join(f"{k} {100 * sum(v) / len(v):.0f}%" for k, v in self.visibility.items()))
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
    threaded=False the pose runs on the caller's thread, for tests.
    """

    def __init__(self, send, make_pose, out=print, record=None, now=time.perf_counter_ns,
                 threaded=True, report_every_ns=2_000_000_000):
        self.send = send
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
            self.request_keyframe(camera, arrived_ns)
            return
        worked, image = camera.decode(frame["data"])
        if not worked:
            camera.waiting_keyframe = True
            self.request_keyframe(camera, arrived_ns)
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
        flags = (FLAG_POSE if points else 0) | (frame["flags"] & FLAG_EXPOSURE_LOCKED)
        packet = pack_body(frame["seq"], frame["capture_ns"], camera.device, width, height, inference_ms, flags, points)
        self.send(packet, camera.address)

        sync = camera.clock.best()
        stats = camera.stats
        with stats.lock:
            stats.poses += 1
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


class MediaPipePose:
    """MediaPipe's pose landmarker for a video: one person, 33 points."""

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


def run(port, record_path, model):
    model_path = ensure_model(model)
    MediaPipePose(model_path).close()  # fail here, clearly, rather than on the first frame
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # Room for a burst: a whole picture is tens of packets at once.
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
    sock.bind(("0.0.0.0", port))
    sock.settimeout(0.05)
    record = open(record_path, "w", encoding="utf-8") if record_path else None
    tracker = Tracker(sock.sendto, lambda: MediaPipePose(model_path), record=record)
    print(f"Modelo «{model}» listo. Esperando las cámaras en el puerto UDP {port}… (Ctrl+C para salir)")
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


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument("--model", choices=sorted(MODEL_SHA256), default="full",
                        help="lite is quicker, heavy more exact; full (default) is in between")
    parser.add_argument("--record", metavar="FILE", help="save every pose as one JSON line")
    args = parser.parse_args()
    try:
        import mediapipe  # noqa: F401
    except ImportError:
        mediapipe = None
    if av is None or np is None or mediapipe is None:
        print("Faltan librerías. Instálalas con:  pip install -r tools/requirements-body.txt")
        return 1
    run(args.port, args.record, args.model)
    return 0


if __name__ == "__main__":
    sys.exit(main())
