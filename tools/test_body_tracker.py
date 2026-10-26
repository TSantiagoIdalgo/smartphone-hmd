"""Tests for body_tracker.py: python -m unittest discover -s tools -p "test_*.py"

The video tests need PyAV and NumPy, and the last ones LiteRT, OpenCV and the internet (they
download the pose model and a photo of a person); the comparison with MediaPipe also needs
MediaPipe. Each is skipped when what it needs is missing.
"""
import hashlib
import io
import json
import math
import tempfile
import time
import unittest
import urllib.request
from fractions import Fraction
from pathlib import Path

import body_tracker as bt

try:
    import ai_edge_litert
    import cv2
except ImportError:
    ai_edge_litert = cv2 = None
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
needs_numpy = unittest.skipIf(bt.np is None, "needs NumPy")
needs_model = unittest.skipIf(bt.av is None or bt.np is None or ai_edge_litert is None,
                              "needs PyAV, NumPy, LiteRT and OpenCV")
needs_mediapipe = unittest.skipIf(mediapipe is None, "needs MediaPipe, to compare with")


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


def encode_jpeg(images):
    """Each picture as a JPEG, as an ESP32-CAM sends it: (bytes, keyframe) pairs."""
    encoder = bt.av.CodecContext.create("mjpeg", "w")
    encoder.height, encoder.width = images[0].shape[:2]
    encoder.pix_fmt = "yuvj420p"
    encoder.time_base = Fraction(1, 1000)
    packets = []
    for i, image in enumerate(images):
        frame = bt.av.VideoFrame.from_ndarray(image, format="rgb24")
        frame.pts = i
        packets += [(bytes(p), True) for p in encoder.encode(frame)]
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

    def test_driver_packet_layout(self):
        # The same bytes are expected by the driver in driver/tests/body_tests.cpp.
        data = bt.pack_body(3, 1234, 0x0A0B0C0D, 480, 640, 121.5, bt.FLAG_POSE | bt.FLAG_BALL,
                            [(0.25, 0.5, -0.1, 0.9, 0.95)], (0.5, 0.25, 0.02), kind=bt.TYPE_BODY_FRAME)
        self.assertEqual(
            data.hex(),
            "484d44310c00000003000000d2040000000000000d0c0b0ae00180020000f342050100000000003f0000803e0ad7a33c"
            "0000803e0000003fcdccccbd6666663f3333733f",
        )

    def test_driver_packet_with_bulbs(self):
        # The same bytes are expected by the driver in driver/tests/body_tests.cpp.
        bulbs = [(0.5, 0.25, 0.02), (0.75, 0.5, 0.03125)]
        data = bt.pack_body(3, 1234, 0x0A0B0C0D, 480, 640, 121.5, bt.FLAG_POSE | bt.FLAG_BALL,
                            [(0.25, 0.5, -0.1, 0.9, 0.95)], (0.5, 0.25, 0.02), kind=bt.TYPE_BODY_FRAME, bulbs=bulbs)
        self.assertEqual(
            data.hex(),
            "484d44310c00000003000000d2040000000000000d0c0b0ae00180020000f342250100000000003f0000803e0ad7a33c"
            "0000803e0000003fcdccccbd6666663f3333733f020000000000003f0000803e0ad7a33c0000403f0000003f0000003d",
        )
        frame = bt.parse_body(data, kind=bt.TYPE_BODY_FRAME)
        self.assertEqual(frame["flags"], bt.FLAG_POSE | bt.FLAG_BALL | bt.FLAG_BULBS)
        self.assertEqual([round(x, 6) for x in frame["bulbs"][1]], [0.75, 0.5, 0.03125])
        self.assertIsNone(bt.parse_body(data[:-1], kind=bt.TYPE_BODY_FRAME))

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


@needs_numpy
class PoseStepsTest(unittest.TestCase):
    """The steps around the models, on made-up numbers."""

    def test_one_anchor_per_detector_box(self):
        anchors = bt.ssd_anchors()
        self.assertEqual(anchors.shape, (2254, 2))
        bt.np.testing.assert_allclose(anchors[0], (0.5 / 28, 0.5 / 28))
        bt.np.testing.assert_allclose(anchors[-1], (6.5 / 7, 6.5 / 7))

    def test_square_around_a_person(self):
        # Standing: the size point is straight above the hips, so no turn.
        x, y, w, h, rotation = bt.pose_square((320, 300), (320, 200), 640, 480)
        self.assertEqual((x, y), (0.5, 300 / 480))
        self.assertAlmostEqual(w * 640, 250)  # twice the distance, and a quarter more
        self.assertAlmostEqual(h * 480, 250)
        self.assertAlmostEqual(rotation, 0)
        # Lying with the head to the right: a quarter turn.
        self.assertAlmostEqual(bt.pose_square((320, 300), (420, 300), 640, 480)[4], math.pi / 2)

    def test_heatmap_moves_a_point_to_its_peak(self):
        heatmap = bt.np.full((64, 64, 2), -20.0, dtype=bt.np.float32)
        heatmap[20, 30, 0] = 10.0  # sure, near the first point
        heatmap[40, 40, 1] = -1.0  # unsure: the second point stays
        points = bt.np.array([[31 / 64, 22 / 64, 0.0], [41 / 64, 41 / 64, 0.0]])
        bt.refine_from_heatmap(points, heatmap)
        self.assertAlmostEqual(points[0, 0], 30 / 64, places=4)
        self.assertAlmostEqual(points[0, 1], 20 / 64, places=4)
        self.assertEqual(tuple(points[1, :2]), (41 / 64, 41 / 64))

    def test_smoothing_steadies_a_still_point_and_follows_a_moving_one(self):
        smoother = bt.PoseSmoother()
        point = bt.np.array([[0.5, 0.5, 0.0, 0.9, 0.9]])
        self.assertEqual(smoother.apply(point, 1000, 640, 480, 200).tolist(), point.tolist(), "the first passes")
        jitter = point + [[0.002, 0, 0, -0.5, 0]]
        steady = smoother.apply(jitter, 1033, 640, 480, 200)
        self.assertLess(abs(steady[0, 0] - 0.5), 0.001, "a small wobble is mostly smoothed away")
        self.assertAlmostEqual(steady[0, 3], 0.85, msg="visibility moves a tenth of the way")
        jump = smoother.apply(point + [[0.2, 0, 0, 0, 0]], 1066, 640, 480, 200)
        self.assertGreater(jump[0, 0], 0.65, "a quick move comes through")


@unittest.skipIf(bt.np is None or cv2 is None, "needs NumPy and OpenCV")
class BallTest(unittest.TestCase):
    """Finding a lit Move's bulb near a hand."""

    def setUp(self):
        self.picture = bt.np.full((640, 480, 3), (110, 100, 90), dtype=bt.np.uint8)
        self.points = standing_pose()
        self.points[15] = (0.5, 0.5, 0.0, 0.9, 0.99)  # left wrist in the middle
        self.points[13] = (0.5, 0.6, 0.0, 0.9, 0.99)  # its elbow below

    def draw_bulb(self, x, y, colour=(255, 40, 230)):
        cv2.circle(self.picture, (x, y), 9, colour, -1, lineType=cv2.LINE_AA)
        cv2.circle(self.picture, (x, y), 5, (255, 255, 255), -1, lineType=cv2.LINE_AA)

    def test_finds_the_bulb_by_the_hand(self):
        self.draw_bulb(255, 290)
        x, y, radius = bt.find_ball(self.picture, self.points)
        self.assertAlmostEqual(x * 480, 255.5, delta=1.0)
        self.assertAlmostEqual(y * 640, 290.5, delta=1.0)
        self.assertAlmostEqual(radius * 480, 9, delta=2.5)

    def test_nothing_without_a_bulb_or_far_from_the_hands(self):
        self.assertIsNone(bt.find_ball(self.picture, self.points))
        self.draw_bulb(60, 600)  # by the feet
        self.assertIsNone(bt.find_ball(self.picture, self.points))
        self.assertIsNone(bt.find_ball(self.picture, []), "no person, no hands")

    def test_white_lights_and_skin_are_not_bulbs(self):
        cv2.circle(self.picture, (250, 300), 12, (255, 255, 255), -1)  # a lamp
        cv2.circle(self.picture, (230, 330), 14, (225, 170, 140), -1)  # a hand
        self.assertIsNone(bt.find_ball(self.picture, self.points))

    def test_a_red_shirt_in_good_light_is_not_a_bulb(self):
        # As bright as it gets in red alone: a bulb blows out two colours, or all three.
        cv2.circle(self.picture, (250, 300), 12, (245, 40, 30), -1)
        self.assertIsNone(bt.find_ball(self.picture, self.points))

    def test_nothing_too_big_or_too_small_for_a_bulb_by_this_person(self):
        # The trunk is 233 px long here: a bulb's radius comes out between about 4 and 33 px.
        cv2.circle(self.picture, (250, 300), 45, (255, 40, 230), -1)
        cv2.circle(self.picture, (250, 300), 30, (255, 255, 255), -1)
        self.assertIsNone(bt.find_ball(self.picture, self.points))
        self.setUp()
        cv2.circle(self.picture, (250, 300), 2, (255, 255, 255), -1)
        cv2.circle(self.picture, (253, 300), 1, (255, 40, 230), -1)
        self.assertIsNone(bt.find_ball(self.picture, self.points))

    def test_a_bulb_by_each_hand(self):
        self.points[16] = (0.3, 0.5, 0.0, 0.9, 0.99)  # the right wrist to the side
        self.points[14] = (0.3, 0.6, 0.0, 0.9, 0.99)
        self.draw_bulb(255, 290)
        self.draw_bulb(140, 285, colour=(40, 230, 255))
        bulbs = sorted(bt.find_balls(self.picture, self.points))
        self.assertEqual(len(bulbs), 2)
        self.assertAlmostEqual(bulbs[0][0] * 480, 140.5, delta=1.0)
        self.assertAlmostEqual(bulbs[1][0] * 480, 255.5, delta=1.0)
        # Both hands by one bulb: it once.
        self.setUp()
        self.points[16] = (0.52, 0.5, 0.0, 0.9, 0.99)
        self.points[14] = (0.52, 0.6, 0.0, 0.9, 0.99)
        self.draw_bulb(255, 290)
        self.assertEqual(len(bt.find_balls(self.picture, self.points)), 1)
        self.assertEqual(bt.find_balls(self.picture, []), [])

    def test_only_the_moves_colour_when_given(self):
        magenta, cyan = bt.BALL_HUES["magenta"], bt.BALL_HUES["cyan"]
        self.draw_bulb(255, 290)
        cv2.circle(self.picture, (220, 330), 10, (250, 240, 60), -1)  # a yellow light by the other side of the hand
        cv2.circle(self.picture, (220, 330), 5, (255, 255, 255), -1)
        x, y, _ = bt.find_ball(self.picture, self.points, magenta)
        self.assertAlmostEqual(x * 480, 255.5, delta=1.0)
        self.assertAlmostEqual(y * 640, 290.5, delta=1.0)
        self.assertIsNone(bt.find_ball(self.picture, self.points, cyan))
        self.setUp()
        self.draw_bulb(255, 290, colour=(40, 230, 255))
        self.assertIsNotNone(bt.find_ball(self.picture, self.points, cyan))
        self.assertIsNone(bt.find_ball(self.picture, self.points, magenta))


@unittest.skipIf(cv2 is None or bt.np is None, "needs OpenCV and NumPy")
class ViewerTest(unittest.TestCase):
    """The page that shows the cameras on the PC."""

    def setUp(self):
        self.viewer = bt.Viewer(port=0)  # any free port
        self.addCleanup(self.viewer.close)

    def get(self, path):
        return urllib.request.urlopen(self.viewer.url.rstrip("/") + path, timeout=5)

    def test_lists_the_cameras_on_its_page(self):
        self.assertIn("Cámaras del cuerpo", self.get("/").read().decode())
        self.assertEqual(json.loads(self.get("/cameras").read()), [])
        self.viewer.update("19BB", marked_picture(), standing_pose(), None)
        self.assertEqual(json.loads(self.get("/cameras").read()), ["19BB"])

    def test_streams_each_camera_live(self):
        self.viewer.update("19BB", marked_picture(), standing_pose(), (0.5, 0.5, 0.02))
        with self.get("/video/19BB") as stream:
            self.assertTrue(stream.headers["Content-Type"].startswith("multipart/x-mixed-replace"))
            self.assertEqual(stream.readline(), b"--frame\r\n")
            self.assertEqual(stream.readline(), b"Content-Type: image/jpeg\r\n")
            length = int(stream.readline().split(b":")[1])
            stream.readline()
            picture = cv2.imdecode(bt.np.frombuffer(stream.read(length), bt.np.uint8), cv2.IMREAD_COLOR)
        self.assertEqual(picture.shape, (640, 480, 3))
        self.assertGreater(picture[60, 60, 2], 150, "the red corner, in BGR")


class TrackerTest(unittest.TestCase):
    def setUp(self):
        self.sent = []
        self.lines = []
        self.record = io.StringIO()
        self.poses = []
        self.pc_now = 0
        self.to_driver = []
        self.tracker = bt.Tracker(lambda data, address: self.sent.append((data, address)), self.make_pose,
                                  out=self.lines.append, record=self.record, now=lambda: self.pc_now,
                                  threaded=False, send_driver=self.to_driver.append)
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
    def test_shows_each_camera_on_the_pc(self):
        shown = []

        class FakeViewer:
            def update(self, name, rgb, points, ball):
                shown.append((name, rgb.shape, len(points), ball))

        self.tracker.viewer = FakeViewer()
        sensor = bt.np.ascontiguousarray(bt.np.rot90(marked_picture(), 1))  # as the phone sends it
        self.stream(encode([sensor] * 3), 10 * SECOND)
        self.assertEqual(shown, [(f"{DEVICE & 0xFFFF:04X}", (640, 480, 3), 33, None)] * 3,
                         "upright, as the pose saw it")

    @needs_video
    def test_keeps_a_numbered_picture_every_two_seconds(self):
        with tempfile.TemporaryDirectory() as folder:
            self.tracker.snapshots = folder
            self.stream(encode([marked_picture()] * 3), 10 * SECOND)
            self.stream(encode([marked_picture()] * 3), 13 * SECOND, first_seq=4)
            self.assertEqual(sorted(p.name for p in Path(folder).iterdir()),
                             [f"camara_{DEVICE & 0xFFFF:04X}_0001.jpg", f"camara_{DEVICE & 0xFFFF:04X}_0002.jpg"])

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
    def test_takes_an_esp32_cams_jpeg_pictures(self):
        # Sideways like a phone, each frame a whole JPEG: no waiting for keyframes after a loss.
        sensor = bt.np.ascontiguousarray(bt.np.rot90(marked_picture(), 1))
        encoded = encode_jpeg([sensor] * 4)
        for i, frame in enumerate(encoded):
            capture = 10 * SECOND + i * FRAME_NS
            self.pc_now = capture + 30_000_000
            packets = self.phone.packets(i + 1, capture, frame, flags=bt.FLAG_JPEG)
            for chunk, packet in enumerate(packets):
                if not (i == 1 and chunk == 0):
                    self.tracker.handle(packet, PHONE, capture + 25_000_000)
        bodies = [bt.parse_body(data) for data in self.sent_of_type(bt.TYPE_BODY)]
        self.assertEqual([b["seq"] for b in bodies], [1, 3, 4])
        shape, top_left, _ = self.poses[0].seen[0]
        self.assertEqual(shape, (640, 480, 3))
        self.assertGreater(top_left[0], 150, f"the red corner should be at the top left, got {top_left}")
        self.assertEqual(self.sent_of_type(bt.TYPE_KEYFRAME_REQUEST), [])

    @needs_video
    def test_a_lost_packet_waits_for_a_whole_picture(self):
        encoded = encode([marked_picture()] * 25, gop=10)
        self.stream(encoded, 0, drop=lambda seq, chunk: seq == 4)

        seqs = [bt.parse_body(data)["seq"] for data in self.sent_of_type(bt.TYPE_BODY)]
        self.assertEqual(seqs, [1, 2, 3] + list(range(11, 26)), "frames 5-10 build on the lost one")
        # Asked at once, and again 100 ms later in case the whole picture went missing too.
        self.assertEqual(len(self.sent_of_type(bt.TYPE_KEYFRAME_REQUEST)), 2)
        self.tracker.tick(0)
        self.tracker.tick(2 * SECOND)
        self.assertIn("perdidos 1, descartados 6 (esperando un cuadro completo)", self.lines[-1])

    @needs_video
    def test_a_phone_joining_late_is_asked_for_a_whole_picture(self):
        encoded = encode([marked_picture()] * 12, gop=10)
        self.stream(encoded[2:], 0)  # its first frame builds on frames it never sent here
        self.assertEqual(len(self.sent_of_type(bt.TYPE_KEYFRAME_REQUEST)), 2)
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

        self.assertEqual(self.to_driver, [], "nothing for the driver before the phone's clock is known")
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

        # The driver gets each frame on the PC's clock, with how long ago the camera took it.
        frames = [bt.parse_body(data, kind=bt.TYPE_BODY_FRAME) for data in self.to_driver]
        self.assertEqual(len(frames), 30)
        self.assertEqual(frames[0]["seq"], 2)
        self.assertEqual(frames[0]["capture_ns"], now + 10_000_000, "the PC's time of the picture")
        self.assertAlmostEqual(frames[0]["inference_ms"], 30.0, places=3, msg="its age: taken 30 ms before")
        self.assertEqual(frames[0]["device"], DEVICE)
        self.assertEqual(len(frames[0]["landmarks"]), 33)

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
    """The pose model on a real photo of a person, which it must find standing upright."""

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
        pose = bt.LiteRtPose(self.model, threads=2)
        try:
            points = pose.detect(self.upright, 0)
            self.assertEqual(pose.detect(bt.np.zeros_like(self.upright), 33), [], "no one in a black picture")
        finally:
            pose.close()
        self.assertEqual(len(points), 33)
        self.assert_standing(points)

    @needs_mediapipe
    def test_same_points_as_mediapipe(self):
        # The person moves, turns and is gone for a moment, so the following, the detector and the
        # smoothing all take part.
        height, width = self.upright.shape[:2]
        frames = []
        for i in range(45):
            turn = cv2.getRotationMatrix2D((width / 2, height / 2), 20 * math.sin(i / 8), 1 + 0.1 * math.sin(i / 13))
            turn[:, 2] += (50 * math.sin(i / 6), 0)
            frames.append(cv2.warpAffine(self.upright, turn, (width, height)) if not 25 <= i < 30
                          else bt.np.zeros_like(self.upright))
        ours, theirs = bt.LiteRtPose(self.model, threads=2), bt.MediaPipePose(self.model)
        try:
            pairs = [(ours.detect(f, 1000 + i * 33), theirs.detect(f, 1000 + i * 33)) for i, f in enumerate(frames)]
        finally:
            ours.close()
            theirs.close()
        self.assertEqual([bool(a) for a, b in pairs], [bool(b) for a, b in pairs], "a person in the same frames")
        both = [(bt.np.array(a), bt.np.array(b)) for a, b in pairs if a]
        self.assertEqual(len(both), 40)
        apart = bt.np.array([bt.np.abs(a[:, :2] - b[:, :2]) for a, b in both])
        self.assertLess(apart.mean(), 0.002, "on average within a fifth of a percent of the picture")
        self.assertLess(apart.max(), 0.03)
        self.assertLess(max(bt.np.abs(a[:, 3] - b[:, 3]).max() for a, b in both), 0.05, "visibility")

    @needs_video
    def test_finds_the_person_sent_as_video(self):
        # Held upside down: the phone says to turn it half a turn.
        sensor = bt.np.ascontiguousarray(bt.np.rot90(self.upright, 2))
        sent = []
        tracker = bt.Tracker(lambda data, address: sent.append(data), lambda: bt.LiteRtPose(self.model, threads=2),
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
