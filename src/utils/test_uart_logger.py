"""Host capture-format and MSP transport tests; no hardware required."""
import binascii
import struct
import unittest

from uart_logger import MSP, decode


def header():
    data = bytearray(64)
    data[:8] = b"UARTLOG\x01"
    struct.pack_into("<IBBBBI", data, 8, 115200, 51, 8, 0, 1, 100)
    struct.pack_into("<I", data, 60, binascii.crc_hqx(data[:60], 0))
    return bytes(data)


def record(payload=b"", kind=1, dropped=0):
    data = bytearray(64)
    struct.pack_into("<2sBBIII", data, 0, b"UL", kind, len(payload), 200, dropped, 0)
    data[16:16 + len(payload)] = payload
    crc = binascii.crc_hqx(data[16:], binascii.crc_hqx(data[:12], 0))
    struct.pack_into("<I", data, 12, crc)
    return bytes(data)


class CaptureTest(unittest.TestCase):
    def test_binary_roundtrip_and_erased_padding(self):
        raw = bytes(range(256))
        blocks = b"".join(record(raw[i:i + 48]) for i in range(0, len(raw), 48))
        result, metadata = decode(header() + blocks + record(kind=2, dropped=9) + b"\xff" * 128)
        self.assertEqual(raw, result)
        self.assertTrue(metadata["complete"])
        self.assertEqual(9, metadata["dropped"])

    def test_torn_tail_preserves_only_verified_prefix(self):
        for tail in (record(b"bad")[:20], b"\xff" * 64):
            raw, metadata = decode(header() + record(b"good") + tail)
            self.assertEqual(b"good", raw)
            self.assertFalse(metadata["complete"])

    def test_checksum_rejects_corruption(self):
        bad = bytearray(record(b"payload"))
        bad[16] ^= 1
        raw, metadata = decode(header() + bad + record(kind=2))
        self.assertEqual(b"", raw)
        self.assertFalse(metadata["complete"])
        with self.assertRaises(ValueError):
            decode(b"invalid")

    def test_empty_capture(self):
        raw, metadata = decode(header() + record(kind=2))
        self.assertEqual(b"", raw)
        self.assertTrue(metadata["complete"])


class FakeSerial:
    def __init__(self, response):
        self.response = bytearray(response)
        self.written = b""

    def write(self, data):
        self.written += data

    def read(self, size):
        # Exercise partial reads, including the multi-byte header and payload.
        count = min(size, 2)
        result = self.response[:count]
        del self.response[:count]
        return bytes(result)


def msp_reply(command, data):
    frame = bytes((len(data), command)) + data
    crc = 0
    for value in frame:
        crc ^= value
    return b"$M>" + frame + bytes((crc,))


class TransportTest(unittest.TestCase):
    def test_flash_read(self):
        port = FakeSerial(msp_reply(71, struct.pack("<IHB", 64, 3, 0) + b"abc"))
        self.assertEqual(b"abc", MSP(port).read_flash(64, 3))
        self.assertEqual(b"$M<\x06G", port.written[:5])

    def test_invalid_address_or_crc(self):
        port = FakeSerial(msp_reply(71, struct.pack("<IHB", 0, 3, 0) + b"abc"))
        with self.assertRaises(ValueError):
            MSP(port).read_flash(64, 3)
        reply = bytearray(msp_reply(70, b"abc"))
        reply[-1] ^= 1
        with self.assertRaises(ValueError):
            MSP(FakeSerial(reply)).request(70)


if __name__ == "__main__":
    unittest.main()
