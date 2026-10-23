#!/usr/bin/env python3
"""Shows what the body camera phones (the "HMD Cámara" app) send, before the driver uses it.

Answers the phones' search for the PC, syncs each phone's clock with this PC's, and every 2 s
prints per phone: frames per second, how often it sees a person, the model's time per frame, the
delay from the camera to this PC, lost packets, and how sure it is of the hips, knees and ankles.

    python body_receiver.py                  # watch
    python body_receiver.py --record a.jsonl # also save every frame, to replay later

Close SteamVR's body tracking first if it ever listens on the same port (9946). On Windows, allow
Python through the firewall on private networks when asked.
"""
import argparse
import json
import socket
import struct
import sys
import time

PORT = 9946
MAGIC = b"HMD1"
TYPE_DISCOVER = 4
TYPE_PC_HERE = 5
TYPE_BODY = 7
TYPE_TIME_PING = 8
TYPE_TIME_PONG = 9
FLAG_POSE = 1
FLAG_EXPOSURE_LOCKED = 2

# Little-endian, laid out as in android/camera/.../BodyProtocol.kt.
HEADER = struct.Struct("<4sB3xIq")  # magic, type, seq, timestamp ns
BODY = struct.Struct("<IHHfBBxxfff")  # device, width, height, inference ms, flags, count, ball x/y/r
LANDMARK = struct.Struct("<5f")  # x, y, z, visibility, presence
PONG = struct.Struct("<qI")  # PC time echoed back, device id
HIPS, KNEES, ANKLES = (23, 24), (25, 26), (27, 28)


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
    """Builds a body packet like the phone does; for tests and fake phones."""
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


class Phone:
    def __init__(self, device, address):
        self.device = device
        self.address = address
        self.clock = Clock()
        self.last_seq = None
        self.reset()

    def reset(self):
        self.frames = 0
        self.poses = 0
        self.lost = 0
        self.inference = 0.0
        self.delays = []
        self.locked = False
        self.visibility = {name: [] for name in ("cadera", "rodillas", "tobillos")}

    def add(self, frame, arrived_ns):
        if self.last_seq is not None and frame["seq"] > self.last_seq + 1:
            self.lost += frame["seq"] - self.last_seq - 1
        self.last_seq = frame["seq"]
        self.frames += 1
        self.inference += frame["inference_ms"]
        self.locked = bool(frame["flags"] & FLAG_EXPOSURE_LOCKED)
        sync = self.clock.best()
        if sync is not None:
            self.delays.append((arrived_ns - (frame["capture_ns"] + sync[0])) / 1e6)
        points = frame["landmarks"]
        if frame["flags"] & FLAG_POSE and len(points) > max(ANKLES):
            self.poses += 1
            for name, pair in zip(self.visibility, (HIPS, KNEES, ANKLES)):
                self.visibility[name].append(min(points[i][3] for i in pair))

    def report(self, seconds):
        name = f"Cámara {self.device & 0xFFFF:04X} ({self.address[0]})"
        if self.frames == 0:
            return f"{name}: sin cuadros"
        parts = [
            f"{self.frames / seconds:.1f} cuadros/s",
            f"persona en {100 * self.poses / self.frames:.0f}%",
            f"modelo {self.inference / self.frames:.0f} ms",
        ]
        if self.delays:
            delays = sorted(self.delays)
            parts.append(f"retraso {delays[len(delays) // 2]:.0f} ms (máx {delays[-1]:.0f})")
        parts.append(f"perdidos {self.lost}")
        if self.poses:
            parts.append(" ".join(f"{k} {100 * sum(v) / len(v):.0f}%" for k, v in self.visibility.items()))
        sync = self.clock.best()
        if sync is not None:
            parts.append(f"reloj ±{sync[1] / 2e6:.1f} ms")
        parts.append("exposición fija" if self.locked else "exposición automática")
        return f"{name}: " + ", ".join(parts)


class Receiver:
    """Everything but the socket: answers the phones, keeps their stats and prints them.

    send(data, address) sends a packet; out(text) prints a line; times are monotonic ns.
    """

    def __init__(self, send, out=print, record=None, report_every_ns=2_000_000_000):
        self.send = send
        self.out = out
        self.record = record
        self.report_every_ns = report_every_ns
        self.phones = {}
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
            for phone in self.phones.values():
                self.ping_seq += 1
                self.send(header(TYPE_TIME_PING, self.ping_seq, now_ns), phone.address)
        if now_ns >= self.next_report_ns:
            seconds = (self.report_every_ns + now_ns - self.next_report_ns) / 1e9
            for phone in self.phones.values():
                self.out(phone.report(seconds))
                phone.reset()
            if not self.phones:
                self.out("Todavía no llega nada. ¿La app está abierta y en la misma red o con el cable?")
            self.next_report_ns = now_ns + self.report_every_ns

    def handle(self, data, address, arrived_ns):
        head = parse_header(data)
        if head is None:
            return
        kind = head[0]
        if kind == TYPE_DISCOVER:
            self.send(header(TYPE_PC_HERE), address)
        elif kind == TYPE_BODY:
            frame = parse_body(data)
            if frame is None:
                return
            phone = self.phones.get(frame["device"])
            if phone is None or phone.address != address:
                phone = self.phones[frame["device"]] = Phone(frame["device"], address)
                self.out(f"Nueva cámara {frame['device'] & 0xFFFF:04X} desde {address[0]}, "
                         f"imagen {frame['width']}x{frame['height']}")
            phone.add(frame, arrived_ns)
            if self.record:
                sync = phone.clock.best()
                frame["pc_ns"] = arrived_ns
                frame["offset_ns"] = sync[0] if sync else None
                self.record.write(json.dumps(frame) + "\n")
        elif kind == TYPE_TIME_PONG:
            pong = parse_pong(data)
            if pong and pong[3] in self.phones:
                self.phones[pong[3]].clock.add(pong[2], pong[1], arrived_ns)


def run(port, record_path):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", port))
    sock.settimeout(0.1)
    record = open(record_path, "w", encoding="utf-8") if record_path else None
    receiver = Receiver(sock.sendto, record=record)
    print(f"Esperando las cámaras en el puerto UDP {port}… (Ctrl+C para salir)")
    try:
        while True:
            receiver.tick(time.monotonic_ns())
            try:
                data, address = sock.recvfrom(2048)
            except socket.timeout:
                continue
            except ConnectionResetError:
                continue  # Windows reports an earlier ping that found no one
            receiver.handle(data, address, time.monotonic_ns())
    except KeyboardInterrupt:
        pass
    finally:
        if record:
            record.close()
        sock.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument("--record", metavar="FILE", help="save every frame as one JSON line")
    args = parser.parse_args()
    run(args.port, args.record)


if __name__ == "__main__":
    sys.exit(main())
