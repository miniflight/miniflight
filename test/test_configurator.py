"""Safety and protocol tests for the read-only localhost configurator."""

import struct
import time
import unittest

from common.serve import SharedState
from config.serve import (
    LinkState,
    MSP_ATTITUDE,
    MSP_MOTOR,
    MSP_RAW_IMU,
    MSPParser,
    decode_payload,
    empty_snapshot,
    msp_request,
    publish_link,
    xor_checksum,
)


def response_frame(command, payload=b""):
    body = bytes((len(payload), command)) + payload
    return b"$M>" + body + bytes((xor_checksum(body),))


class ConfiguratorProtocolTests(unittest.TestCase):
    def test_read_request_has_valid_msp_v1_checksum(self):
        request = msp_request(MSP_ATTITUDE)

        self.assertEqual(request, b"$M<\x00ll")
        self.assertEqual(xor_checksum(request[3:-1]), request[-1])

    def test_write_command_is_rejected_by_allowlist(self):
        with self.assertRaisesRegex(ValueError, "read-only allowlist"):
            msp_request(205)  # MSP_ACC_CALIBRATION

    def test_parser_accepts_fragmented_frame_and_discards_noise(self):
        parser = MSPParser()
        payload = struct.pack("<hhh", 123, -45, 278)
        frame = response_frame(MSP_ATTITUDE, payload)

        parser.feed(b"noise" + frame[:4])
        self.assertIsNone(parser.pop())

        parser.feed(frame[4:])
        self.assertEqual(parser.pop(), (">", MSP_ATTITUDE, payload))

    def test_parser_discards_bad_checksum(self):
        parser = MSPParser()
        frame = bytearray(response_frame(MSP_ATTITUDE, b"\x01\x02"))
        frame[-1] ^= 0xFF
        parser.feed(frame)

        self.assertIsNone(parser.pop())

    def test_decode_attitude_uses_betaflight_wire_units(self):
        snapshot = empty_snapshot()
        decode_payload(
            MSP_ATTITUDE,
            struct.pack("<hhh", 123, -45, 278),
            snapshot,
            [],
        )

        self.assertEqual(
            snapshot["signals"]["attitude_deg"],
            [12.3, -4.5, 278.0],
        )

    def test_decode_raw_imu_preserves_signed_msp_values(self):
        snapshot = empty_snapshot()
        values = (-120, 40, 2050, -3, 2, 1, 0, 0, 0)
        decode_payload(MSP_RAW_IMU, struct.pack("<9h", *values), snapshot, [])

        self.assertEqual(snapshot["signals"]["accel_msp"], [-120, 40, 2050])
        self.assertEqual(snapshot["signals"]["gyro_msp"], [-3, 2, 1])

    def test_motor_decoder_hides_unused_zero_outputs(self):
        snapshot = empty_snapshot()
        values = (1000, 1001, 1002, 1003, 0, 0, 0, 0)
        decode_payload(MSP_MOTOR, struct.pack("<8H", *values), snapshot, [])

        self.assertEqual(
            snapshot["signals"]["motors"],
            [1000, 1001, 1002, 1003],
        )

    def test_empty_snapshot_reports_no_unobserved_machine_capabilities(self):
        snapshot = empty_snapshot()

        self.assertEqual(snapshot["machine"], {
            "sensors": [],
            "motor_count": None,
            "has_imu": False,
        })

    def test_offline_link_retains_last_good_sample_and_ages_it(self):
        shared = SharedState()
        snapshot = empty_snapshot(LinkState.LIVE, "Live MSP telemetry.")
        snapshot["connection"].update(
            port="/dev/cu.usbmodem-test",
            last_port="/dev/cu.usbmodem-test",
            last_frame_at_ms=round(time.time() * 1000) - 25,
            last_frame_age_ms=0,
        )
        snapshot["signals"]["attitude_deg"] = [1.0, 2.0, 3.0]
        shared.set_snapshot(snapshot)

        offline = publish_link(
            shared,
            LinkState.WAITING_USB,
            "Connect the flight controller by USB-C.",
            retain_last_sample=True,
        )

        self.assertEqual(offline["connection"]["state"], "waiting_usb")
        self.assertEqual(offline["connection"]["port"], None)
        self.assertEqual(offline["connection"]["last_port"], "/dev/cu.usbmodem-test")
        self.assertGreaterEqual(offline["connection"]["last_frame_age_ms"], 25)
        self.assertEqual(offline["signals"]["attitude_deg"], [1.0, 2.0, 3.0])

    def test_snapshot_update_time_refreshes_on_each_publish(self):
        shared = SharedState()
        shared.set_snapshot({"updated_at": 0.0})

        self.assertGreater(shared.get_snapshot()["updated_at"], 0.0)


if __name__ == "__main__":
    unittest.main()
