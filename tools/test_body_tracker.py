"""Tests for body_tracker.py: python -m unittest discover -s tools -p "test_*.py"

The video tests need PyAV and NumPy, and the last one MediaPipe and the internet (it downloads the
pose model and a photo of a person); each is skipped when what it needs is missing.
"""
import hashlib
import io
import json
import tempfile
import time
import unittest
import urllib.request
from fractions import Fraction
from pathlib import Path

import body_tracker as bt

try:
    import mediapipe
except ImportError:
    mediapipe = None

PHONE = ("192.168.1.20", 40000)
DEVICE = 0x1234ABCD
SECOND = 1_000_000_000
FRAME_NS = 33_333_333
PHOTO_URL = "https://storage.googleapis.com/mediapipe-assets/pose.jpg"
PHOTO_SHA256 = "c8a830ed683c0276d713dd5aeda28f415f10cd6291972084a40d0d8b934ed62b"
needs_video = unittest.skipIf(bt.av is None or bt.np is None or "libx264" not in bt.av.codecs_available,
                              "needs NumPy and PyAV with libx264, to make the phone's video")
needs_model = unittest.skipIf(bt.av is None or bt.np is None or mediapipe is None, "needs PyAV, NumPy and MediaPipe")


def standing_pose(visibility=0.9):
    return [(0.5, i / 33, 0.0, visibility, 0.99) for i in range(33)]


def encode(images, gop=30):
    """H.264 like the phone sends it: no B-frames, one packet per frame, headers on each keyframe."""
    encoder = bt.av.CodecContext.create("libx264", "w")
    encoder.height, encoder.width = images[0].shape[:2]
    encoder.pix_fmt = "yuv420p"
    encoder.time_base = Fraction(1, 1000)
    encoder.gop_size = gop
    encoder.options = {"preset": "ultrafast", "tune": "zerolatency", "profile": "baseline"}
    packets = []
    for i, image in enumerate(images):
        frame = bt.av.VideoFrame.from_ndarray(image, format="rgb24").reformat(format="yuv420p")
        frame.pts = i * 33
        packets += [(bytes(p), p.is_keyframe) for p in encoder.encode(frame)]
    packets += [(bytes(p), p.is_keyframe) for p in encoder.encode(None)]
    assert len(packets) == len(images)
    return packets


def marked_picture(height=640, width=480):
    """An upright grey picture with a red square at its top left."""
    image = bt.np.full((height, width, 3), 128, dtype=bt.np.uint8)
    image[:120, :120] = (220, 30, 30)
    return image


class FakePose:
    """Stands in for MediaPipe: remembers what it was shown and always sees a person."""

    def __init__(self):
        self.seen = []
        self.closed = False

    def detect(self, rgb, ms):
        self.seen.append((rgb.shape, tuple(int(c) for c in rgb[40, 40]), ms))
        return standing_pose()

    def close(self):
        self.closed = True


class FakePhone:
    """Sends video and answers pings like the app, on a clock 5 s behind the PC's."""

    def __init__(self, behind_ns=5 * SECOND):
        self.behind_ns = behind_ns

    def packets(self, seq, pc_capture_ns, encoded, rotation=1, flags=0, width=640, height=480):
        data, keyframe = encoded
        flags |= bt.FLAG_KEYFRAME if keyframe else 0
        return bt.pack_video(seq, pc_capture_ns - self.behind_ns, DEVICE, width, height, rotation, flags, data)

    def pong(self, ping, pc_now_ns):
        _, seq, pc_ns = bt.parse_header(ping)
        return bt.header(bt.TYPE_TIME_PONG, seq, pc_now_ns - self.behind_ns) + bt.PONG.pack(pc_ns, DEVICE)


class PacketTest(unittest.TestCase):
    def test_body_round_trip(self):
        points = [(0.1, 0.2, -0.3, 0.9, 0.95), (0.4, 0.5, 0.6, 0.7, 0.8)]
        data = bt.pack_body(7, 123, 9, 480, 640, 12.5, 3, points, (0.25, 0.5, 0.125))
        frame = bt.parse_body(data)
        self.assertEqual(len(data), 48 + 2 * 20)
        self.assertEqual((frame["seq"], frame["capture_ns"], frame["device"]), (7, 123, 9))
        self.assertEqual((frame["width"], frame["height"], frame["flags"]), (480, 640, 3))
        self.assertEqual(frame["ball"], (0.25, 0.5, 0.125))
        self.assertAlmostEqual(frame["landmarks"][1][2], 0.6, places=6)

    def test_rejects_short_and_foreign_packets(self):
        data = bt.pack_body(1, 0, 9, 480, 640, 0.0, 1, standing_pose())
        self.assertIsNone(bt.parse_body(data[:-1]))
        self.assertIsNone(bt.parse_body(b"XXXX" + data[4:]))
        self.assertIsNone(bt.parse_body(bt.header(bt.TYPE_DISCOVER)))
        self.assertIsNone(bt.parse_video(bt.pack_video(1, 0, 9, 640, 480, 0, 0, b"x")[0][:35]))

    def test_video_packet_layout(self):
        # The same bytes are expected from the phone in BodyProtocolTest.kt.
        packet, = bt.pack_video(5, 123456789012, 0x01020304, 640, 480, 1, 9, b"\x00\x00\x00\x01\x67")
        self.assertEqual(
            packet.hex(),
            "484d44310a00000005000000141a99be1c000000040302018002e00100000100010900000000000167",
        )

    def test_frame_split_into_packets_and_joined_in_any_order(self):
        data = bytes(range(256)) * 12  # 3072 bytes: 1400 + 1400 + 272
        packets = bt.pack_video(9, 1, 2, 640, 480, 3, bt.FLAG_KEYFRAME, data)
        self.assertEqual([len(p) for p in packets], [1436, 1436, 308])
        assembler = bt.FrameAssembler()
        self.assertIsNone(assembler.add(bt.parse_video(packets[2])))
        self.assertIsNone(assembler.add(bt.parse_video(packets[0])))
        frame = assembler.add(bt.parse_video(packets[1]))
        self.assertEqual(frame["data"], data)
        self.assertEqual((frame["seq"], frame["rotation"], frame["flags"]), (9, 3, bt.FLAG_KEYFRAME))
        self.assertIsNone(assembler.add(bt.parse_video(packets[1])), "a repeated packet is not a new frame")

    def test_a_newer_frame_drops_an_unfinished_one(self):
        assembler = bt.FrameAssembler()
        first = bt.pack_video(1, 0, 2, 640, 480, 0, 0, bytes(2000))
        second = bt.pack_video(2, 0, 2, 640, 480, 0, 0, bytes(10))
        self.assertIsNone(assembler.add(bt.parse_video(first[0])))
        self.assertEqual(assembler.add(bt.parse_video(second[0]))["seq"], 2)
        self.assertIsNone(assembler.add(bt.parse_video(first[1])), "too late: frame 1 is gone")

    def test_follows_an_app_that_started_again(self):
        assembler = bt.FrameAssembler()
        assembler.add(bt.parse_video(bt.pack_video(5000, 0, 2, 640, 480, 0, 0, b"a")[0]))
        frame = assembler.add(bt.parse_video(bt.pack_video(1, 0, 2, 640, 480, 0, 0, b"b")[0]))
        self.assertEqual(frame["seq"], 1)


class ClockTest(unittest.TestCase):
    def test_quickest_round_trip_wins(self):
        clock = bt.Clock()
        clock.add(100, 50, 140)  # 40 ns there and back, symmetric
        clock.add(200, 120, 300)  # slow way back
        self.assertEqual(clock.best(), (70, 40))


class TrackerTest(unittest.TestCase):
    def setUp(self):
        self.sent = []
        self.lines = []
        self.record = io.StringIO()
        self.poses = []
        self.pc_now = 0
        self.tracker = bt.Tracker(lambda data, address: self.sent.append((data, address)), self.make_pose,
                                  out=self.lines.append, record=self.record, now=lambda: self.pc_now,
                                  threaded=False)
        self.phone = FakePhone()

    def make_pose(self):
        self.poses.append(FakePose())
        return self.poses[-1]

    def sent_of_type(self, kind):
        return [data for data, address in self.sent if bt.parse_header(data)[0] == kind and address == PHONE]

    def stream(self, encoded, start_ns, first_seq=1, drop=lambda seq, chunk: False):
        """Sends each frame 25 ms after it was taken; its pose is done 30 ms after it was taken."""
        for i, frame in enumerate(encoded):
            seq = first_seq + i
            capture = start_ns + i * FRAME_NS
            self.pc_now = capture + 30_000_000
            for chunk, packet in enumerate(self.phone.packets(seq, capture, frame)):
                if not drop(seq, chunk):
                    self.tracker.handle(packet, PHONE, capture + 25_000_000)

    def test_answers_the_search_for_the_pc(self):
        self.tracker.handle(bt.header(bt.TYPE_DISCOVER), PHONE, 0)
        self.assertEqual(self.sent, [(bt.header(bt.TYPE_PC_HERE), PHONE)])

    @needs_video
    def test_sends_back_the_pose_of_the_upright_picture(self):
        upright = marked_picture()
        sensor = bt.np.ascontiguousarray(bt.np.rot90(upright, 1))  # the phone's picture, a quarter turn back
        self.stream(encode([sensor] * 5), 10 * SECOND)

        bodies = [bt.parse_body(data) for data in self.sent_of_type(bt.TYPE_BODY)]
        self.assertEqual([b["seq"] for b in bodies], [1, 2, 3, 4, 5])
        self.assertEqual((bodies[0]["width"], bodies[0]["height"]), (480, 640))
        self.assertEqual(bodies[0]["flags"], bt.FLAG_POSE)
        self.assertEqual(bodies[0]["capture_ns"], 5 * SECOND, "the phone's own time comes back")
        self.assertEqual(len(bodies[0]["landmarks"]), 33)
        shape, top_left, _ = self.poses[0].seen[0]
        self.assertEqual(shape, (640, 480, 3))
        self.assertGreater(top_left[0], 150, f"the red corner should be at the top left, got {top_left}")
        self.assertLess(top_left[2], 100)
        self.assertEqual(self.sent_of_type(bt.TYPE_KEYFRAME_REQUEST), [])

    @needs_video
    def test_a_lost_packet_waits_for_a_whole_picture(self):
        encoded = encode([marked_picture()] * 25, gop=10)
        self.stream(encoded, 0, drop=lambda seq, chunk: seq == 4)

        seqs = [bt.parse_body(data)["seq"] for data in self.sent_of_type(bt.TYPE_BODY)]
        self.assertEqual(seqs, [1, 2, 3] + list(range(11, 26)), "frames 5-10 build on the lost one")
        self.assertEqual(len(self.sent_of_type(bt.TYPE_KEYFRAME_REQUEST)), 1)
        self.tracker.tick(0)
        self.tracker.tick(2 * SECOND)
        self.assertIn("perdidos 1", self.lines[-1])

    @needs_video
    def test_a_phone_joining_late_is_asked_for_a_whole_picture(self):
        encoded = encode([marked_picture()] * 12, gop=10)
        self.stream(encoded[2:], 0)  # its first frame builds on frames it never sent here
        self.assertEqual(len(self.sent_of_type(bt.TYPE_KEYFRAME_REQUEST)), 1)
        self.assertEqual([bt.parse_body(d)["seq"] for d in self.sent_of_type(bt.TYPE_BODY)], [9, 10])

    @needs_video
    def test_syncs_the_clock_and_reports(self):
        encoded = encode([marked_picture()] * 31)
        now = 100 * SECOND
        self.tracker.tick(now)
        self.stream(encoded[:1], now)
        self.assertIn("Nueva cámara ABCD", self.lines[0])

        # A ping a second later, answered by the phone 2 ms after; it arrives 4 ms after sending.
        now += SECOND
        self.tracker.tick(now)
        ping, address = self.sent[-1]
        self.assertEqual((bt.parse_header(ping)[0], address), (bt.TYPE_TIME_PING, PHONE))
        self.tracker.handle(self.phone.pong(ping, now + 2_000_000), PHONE, now + 4_000_000)

        self.stream(encoded[1:], now + 10_000_000, first_seq=2)
        self.tracker.tick(now + SECOND)
        report = self.lines[-1]
        for part in ("persona en 100%", "llega a los 25 ms", "pose lista a los 30 ms", "perdidos 0",
                     "cadera 90% rodillas 90% tobillos 90%", "reloj ±2.0 ms", "exposición automática"):
            self.assertIn(part, report)

        recorded = [json.loads(line) for line in self.record.getvalue().splitlines()]
        self.assertEqual(len(recorded), 31)
        self.assertEqual(recorded[-1]["offset_ns"], 5 * SECOND)
        self.assertEqual(len(recorded[-1]["landmarks"]), 33)

    @needs_video
    def test_pose_thread_keeps_up_and_closes(self):
        tracker = bt.Tracker(lambda data, address: self.sent.append((data, address)), self.make_pose,
                             out=self.lines.append)
        for i, frame in enumerate(encode([marked_picture()] * 5)):
            for packet in self.phone.packets(i + 1, i * FRAME_NS, frame):
                tracker.handle(packet, PHONE, i * FRAME_NS)
            deadline = time.monotonic() + 5
            while len(self.sent_of_type(bt.TYPE_BODY)) <= i and time.monotonic() < deadline:
                time.sleep(0.01)
        tracker.close()
        self.assertEqual(len(self.sent_of_type(bt.TYPE_BODY)), 5)
        self.assertTrue(self.poses[0].closed)


@needs_model
class RealPoseTest(unittest.TestCase):
    """MediaPipe on a real photo of a person, which it must find standing upright."""

    @classmethod
    def setUpClass(cls):
        folder = Path(tempfile.mkdtemp())
        cls.model = bt.ensure_model("full", folder, out=lambda text: None)
        with urllib.request.urlopen(PHOTO_URL, timeout=60) as response:
            jpeg = response.read()
        assert hashlib.sha256(jpeg).hexdigest() == PHOTO_SHA256
        (folder / "pose.jpg").write_bytes(jpeg)
        with bt.av.open(str(folder / "pose.jpg")) as container:
            photo = next(container.decode(video=0)).to_ndarray(format="rgb24", width=640, height=426)
        cls.upright = bt.np.zeros((480, 640, 3), dtype=bt.np.uint8)
        cls.upright[27:453] = photo

    def assert_standing(self, points):
        nose, hips = points[0], (points[23], points[24])
        for hip in hips:
            self.assertGreater(hip[3], 0.5, "the hips are in view")
        self.assertLess(nose[1], min(h[1] for h in hips), "upright: the head is above the hips")

    def test_finds_the_person(self):
        pose = bt.MediaPipePose(self.model)
        try:
            points = pose.detect(self.upright, 0)
        finally:
            pose.close()
        self.assertEqual(len(points), 33)
        self.assert_standing(points)

    @needs_video
    def test_finds_the_person_sent_as_video(self):
        # Held upside down: the phone says to turn it half a turn.
        sensor = bt.np.ascontiguousarray(bt.np.rot90(self.upright, 2))
        sent = []
        tracker = bt.Tracker(lambda data, address: sent.append(data), lambda: bt.MediaPipePose(self.model),
                             out=lambda text: None, threaded=False)
        phone = FakePhone()
        for i, frame in enumerate(encode([sensor] * 10)):
            for packet in phone.packets(i + 1, i * FRAME_NS, frame, rotation=2):
                tracker.handle(packet, PHONE, i * FRAME_NS)
        tracker.close()

        body = bt.parse_body([d for d in sent if bt.parse_header(d)[0] == bt.TYPE_BODY][-1])
        self.assertEqual(body["flags"] & bt.FLAG_POSE, bt.FLAG_POSE)
        self.assert_standing(body["landmarks"])


if __name__ == "__main__":
    unittest.main()
