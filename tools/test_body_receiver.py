"""Tests for body_receiver.py: python -m unittest discover -s tools -p "test_*.py" """
import json
import io
import unittest

import body_receiver as br

PHONE = ("192.168.1.20", 40000)
SECOND = 1_000_000_000


def standing_pose(visibility=0.9):
    return [(0.5, i / 33, 0.0, visibility, 0.99) for i in range(33)]


class FakePhone:
    """Answers pings like the app's PcLink, on a clock 5 s behind the PC's."""

    def __init__(self, device=0x1234ABCD, behind_ns=5 * SECOND):
        self.device = device
        self.behind_ns = behind_ns
        self.seq = 0

    def frame(self, pc_now_ns, landmarks, flags=br.FLAG_POSE):
        self.seq += 1
        return br.pack_body(self.seq, pc_now_ns - self.behind_ns, self.device, 480, 640, 18.0, flags, landmarks)

    def pong(self, ping, pc_now_ns):
        _, seq, pc_ns = br.parse_header(ping)
        return br.header(br.TYPE_TIME_PONG, seq, pc_now_ns - self.behind_ns) + br.PONG.pack(pc_ns, self.device)


class PacketTest(unittest.TestCase):
    def test_body_round_trip(self):
        points = [(0.1, 0.2, -0.3, 0.9, 0.95), (0.4, 0.5, 0.6, 0.7, 0.8)]
        data = br.pack_body(7, 123, 9, 480, 640, 12.5, 3, points, (0.25, 0.5, 0.125))
        frame = br.parse_body(data)
        self.assertEqual(len(data), 48 + 2 * 20)
        self.assertEqual((frame["seq"], frame["capture_ns"], frame["device"]), (7, 123, 9))
        self.assertEqual((frame["width"], frame["height"], frame["flags"]), (480, 640, 3))
        self.assertEqual(frame["ball"], (0.25, 0.5, 0.125))
        self.assertAlmostEqual(frame["landmarks"][1][2], 0.6, places=6)

    def test_rejects_short_and_foreign_packets(self):
        data = br.pack_body(1, 0, 9, 480, 640, 0.0, 1, standing_pose())
        self.assertIsNone(br.parse_body(data[:-1]))
        self.assertIsNone(br.parse_body(b"XXXX" + data[4:]))
        self.assertIsNone(br.parse_body(br.header(br.TYPE_DISCOVER)))


class ClockTest(unittest.TestCase):
    def test_quickest_round_trip_wins(self):
        clock = br.Clock()
        clock.add(100, 50, 140)  # 40 ns there and back, symmetric
        clock.add(200, 120, 300)  # slow way back
        self.assertEqual(clock.best(), (70, 40))


class ReceiverTest(unittest.TestCase):
    def setUp(self):
        self.sent = []
        self.lines = []
        self.record = io.StringIO()
        self.receiver = br.Receiver(lambda data, address: self.sent.append((data, address)),
                                    out=self.lines.append, record=self.record)

    def test_answers_the_search_for_the_pc(self):
        self.receiver.handle(br.header(br.TYPE_DISCOVER), PHONE, 0)
        self.assertEqual(self.sent, [(br.header(br.TYPE_PC_HERE), PHONE)])

    def test_syncs_the_clock_and_reports(self):
        phone = FakePhone()
        now = 100 * SECOND
        self.receiver.tick(now)
        self.receiver.handle(phone.frame(now, standing_pose()), PHONE, now + 10_000_000)
        self.assertIn("Nueva cámara ABCD", self.lines[-1])

        # A ping a second later, answered by the phone 2 ms after; it arrives 4 ms after sending.
        now += SECOND
        self.receiver.tick(now)
        ping, address = self.sent[-1]
        self.assertEqual((br.parse_header(ping)[0], address), (br.TYPE_TIME_PING, PHONE))
        self.receiver.handle(phone.pong(ping, now + 2_000_000), PHONE, now + 4_000_000)

        # 30 frames, each arriving 25 ms after the camera took it, one lost on the way.
        for i in range(31):
            capture = now + 10_000_000 + i * 33_000_000
            data = phone.frame(capture, standing_pose())
            if i != 15:
                self.receiver.handle(data, PHONE, capture + 25_000_000)
        self.receiver.tick(now + SECOND)
        report = self.lines[-1]
        self.assertIn("persona en 100%", report)
        self.assertIn("modelo 18 ms", report)
        self.assertIn("retraso 25 ms", report)
        self.assertIn("perdidos 1", report)
        self.assertIn("cadera 90% rodillas 90% tobillos 90%", report)
        self.assertIn("reloj ±2.0 ms", report)

        recorded = [json.loads(line) for line in self.record.getvalue().splitlines()]
        self.assertEqual(len(recorded), 31)
        self.assertEqual(recorded[-1]["offset_ns"], 5 * SECOND)
        self.assertEqual(len(recorded[-1]["landmarks"]), 33)

    def test_frames_without_a_person(self):
        phone = FakePhone()
        self.receiver.tick(0)
        for i in range(10):
            self.receiver.handle(phone.frame(i * 33_000_000, [], flags=0), PHONE, i * 33_000_000)
        self.receiver.tick(2 * SECOND)
        self.assertIn("persona en 0%", self.lines[-1])
        self.assertNotIn("cadera", self.lines[-1])


if __name__ == "__main__":
    unittest.main()
