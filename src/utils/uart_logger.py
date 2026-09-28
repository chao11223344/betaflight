#!/usr/bin/env python3
"""Download raw UART captures over MSP USB, or export their payload bytes."""

import argparse
import binascii
import json
from pathlib import Path
import struct
import sys
import time


BLOCK_SIZE = 64
MAGIC = b"UARTLOG\x01"


def decode(data):
    if len(data) < BLOCK_SIZE or data[:8] != MAGIC:
        raise ValueError("Not a UARTLOG version 1 capture")
    if struct.unpack_from("<I", data, 60)[0] != binascii.crc_hqx(data[:60], 0):
        raise ValueError("Invalid capture header checksum")
    baud, uart, bits, parity, stops, started = struct.unpack_from("<IBBBBI", data, 8)
    metadata = dict(baud=baud, uart=uart, data_bits=bits, parity=parity,
                    stop_bits=stops, started_us=started, complete=False,
                    dropped=0, records=[])
    payload = bytearray()
    for offset in range(BLOCK_SIZE, len(data), BLOCK_SIZE):
        block = data[offset:offset + BLOCK_SIZE]
        if block == b"\xff" * len(block):
            break
        if len(block) != BLOCK_SIZE:
            metadata["warning"] = f"Truncated block at {offset}"
            break
        marker, kind, length, timestamp, dropped, crc = struct.unpack_from("<2sBBIII", block)
        actual_crc = binascii.crc_hqx(block[16:], binascii.crc_hqx(block[:12], 0))
        if marker != b"UL" or kind not in (1, 2) or length > 48 or crc != actual_crc:
            metadata["warning"] = f"Invalid block at {offset}; export stops here"
            break
        if (kind == 2 and length != 0) or (kind == 1 and length == 0):
            metadata["warning"] = f"Invalid block length at {offset}"
            break
        metadata["dropped"] = dropped
        metadata["records"].append(dict(offset=len(payload), length=length,
                                          timestamp_us=timestamp, dropped=dropped))
        if kind == 2:
            metadata["complete"] = True
            break
        payload.extend(block[16:16 + length])
    metadata["bytes"] = len(payload)
    return bytes(payload), metadata


class MSP:
    def __init__(self, port):
        self.port = port

    def read_exact(self, count):
        result = bytearray()
        while len(result) < count:
            chunk = self.port.read(count - len(result))
            if not chunk:
                raise TimeoutError("MSP response timed out; close Configurator and exit CLI")
            result.extend(chunk)
        return bytes(result)

    def request(self, command, payload=b""):
        frame = bytes((len(payload), command)) + payload
        checksum = 0
        for value in frame:
            checksum ^= value
        self.port.write(b"$M<" + frame + bytes((checksum,)))
        deadline = time.monotonic() + 5
        prefix = b""
        while time.monotonic() < deadline:
            prefix = (prefix + self.read_exact(1))[-3:]
            if prefix not in (b"$M>", b"$M!"):
                continue
            size, received = self.read_exact(2)
            response = self.read_exact(size)
            check = size ^ received
            for value in response:
                check ^= value
            if self.read_exact(1)[0] != check:
                raise ValueError("MSP checksum mismatch")
            if received != command:
                continue
            if prefix == b"$M!":
                raise ValueError(f"Firmware rejected MSP command {command}")
            return response
        raise TimeoutError("No matching MSP response")

    def read_flash(self, offset, size):
        # Six-byte request selects uncompressed, variable-length replies.
        reply = self.request(71, struct.pack("<IH", offset, size))
        if len(reply) < 7:
            raise ValueError("Short Flash response")
        address, length, compression = struct.unpack_from("<IHB", reply)
        if address != offset or compression != 0 or length != size or len(reply) != 7 + length:
            raise ValueError("Unexpected Flash response address, size, or compression")
        return reply[7:]


def download(port_name, destination):
    try:
        import serial
    except ImportError as exc:
        raise RuntimeError("Download requires pyserial: python3 -m pip install pyserial") from exc
    with serial.Serial(port_name, 115200, timeout=3, write_timeout=3) as port:
        port.reset_input_buffer()
        client = MSP(port)
        # First read stops capture and drains the firmware queue. Query used
        # length only afterwards, otherwise the last buffered bytes are omitted.
        first = client.read_flash(0, BLOCK_SIZE)
        if first[:8] != MAGIC:
            raise ValueError("Flash does not contain a UARTLOG capture")
        summary = client.request(70)
        if len(summary) < 13:
            raise ValueError("Invalid Flash summary")
        flags, sectors, capacity, used = struct.unpack_from("<BIII", summary)
        if flags & 3 != 3 or not BLOCK_SIZE <= used <= capacity:
            raise ValueError("Flash unavailable or inconsistent used length")
        data = bytearray(first)
        while len(data) < used:
            data.extend(client.read_flash(len(data), min(192, used - len(data))))
        # Boot scanning may round used length upward. Decode stops at the end
        # record or erased area; retain the downloaded image for recovery.
        Path(destination).write_bytes(data)
        return len(data)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    dl = commands.add_parser("download")
    dl.add_argument("port")
    dl.add_argument("output", type=Path)
    export = commands.add_parser("export")
    export.add_argument("capture", type=Path)
    export.add_argument("output", type=Path)
    export.add_argument("--metadata", type=Path)
    args = parser.parse_args()
    try:
        if args.command == "download":
            if args.output.exists():
                raise ValueError("Output already exists")
            print(f"Downloaded {download(args.port, args.output)} bytes to {args.output}")
        else:
            metadata_path = args.metadata or args.output.with_suffix(args.output.suffix + ".json")
            if args.output.exists() or metadata_path.exists() or args.output.resolve() == metadata_path.resolve():
                raise ValueError("Output paths must be distinct and must not already exist")
            raw, metadata = decode(args.capture.read_bytes())
            args.output.write_bytes(raw)
            metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
            print(f"Exported {len(raw)} bytes; dropped={metadata['dropped']}; complete={metadata['complete']}")
            if not metadata["complete"]:
                print(metadata.get("warning", "No end record: power loss or unfinished capture"), file=sys.stderr)
                return 2
    except (OSError, ValueError, RuntimeError, TimeoutError) as exc:
        print(str(exc), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
