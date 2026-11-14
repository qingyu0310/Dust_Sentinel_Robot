import importlib.util
from pathlib import Path
import unittest

SPEC = importlib.util.spec_from_file_location(
    "pitch_gravity_check", Path(__file__).resolve().parents[1] / "pitch_gravity_check.py")
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class FakeSerial:
    def __init__(self, response):
        self.response = response
        self.queue = []

    def write(self, data):
        self.queue.extend(self.response.splitlines(keepends=True))

    def flush(self):
        pass

    def readline(self):
        return self.queue.pop(0) if self.queue else b""


class GetFloatTests(unittest.TestCase):
    def test_plain_and_named_responses(self):
        for response in (b"0.500000\r\n", b"pitch_up_gscale = 0.500000\r\n",
                         b"pitch_up_gscale=5e-1\r\n",
                         b"\x1b[32mpitch_up_gscale = .5\x1b[0m\r\n"):
            with self.subTest(response=response):
                self.assertEqual(MODULE.Link(FakeSerial(response)).get_float("pitch_up_gscale"), 0.5)

    def test_unrelated_variable_is_ignored(self):
        response = b"pitch_up_kp = 6.000000\r\npitch_up_gscale = 0.500000\r\n"
        self.assertEqual(MODULE.Link(FakeSerial(response)).get_float("pitch_up_gscale"), 0.5)

    def test_heartbeat_ack_is_not_confused_with_other_command(self):
        class AckSerial(FakeSerial):
            def write(self, data):
                self.queue.append(b"ok\r\n")
        link = MODULE.Link(AckSerial(b""))
        link.active = True
        link.last_feed -= 1
        link.set("pitch_up_gscale", 0.5)
        self.assertEqual(link.pending, 0)
        self.assertEqual(link.feed, 2)
        self.assertEqual(link.feed_ack, 2)
        self.assertFalse(link.ack_commands)

    def test_mcu_heartbeat_reports_observed_value_and_age(self):
        ser = FakeSerial(b"")
        ser.queue.append(b"[inf] PHEART,12345,26,44,2\r\n")
        link = MODULE.Link(ser)
        _, data = link.receive()
        self.assertIsNone(data)
        self.assertEqual(link.mcu_feed, 26)
        self.assertEqual(link.mcu_feed_age, 44)


if __name__ == "__main__":
    unittest.main()
