"""
Packet framing layer for Revopoint 3D scanners.
Reverse engineered from 3DCamera.dll (DeviceDataPacketParser.cpp).
"""

import struct
import time
from typing import Generator, Optional, Tuple

MAGIC = 0xAA55AA55
BASE_HEADER_SIZE = 24  # 0x18


def compute_checksum(header_bytes: bytes) -> int:
    """
    Computes single-byte XOR checksum over the header bytes.
    Matches the vectorized implementation at 0x18124a020 in 3DCamera.dll.
    """
    chk = 0
    for b in header_bytes:
        chk ^= b
    return chk


def pack_packet(
    packet_id: int,
    payload: bytes = b"",
    flags: int = 0,
    timestamp_ms: Optional[int] = None,
    extra_hdr: bytes = b"",
) -> bytes:
    """
    Packs a frame with the Revopoint 0xAA55AA55 protocol header.

    Header layout (24 bytes base + extra_hdr + 1 byte checksum):
      [0..3]   uint32 magic (0xAA55AA55)
      [4..5]   uint16 flags / sub_type
      [6..9]   uint32 payload_len
      [10..17] uint64 timestamp_ms
      [18..21] uint32 packet_id
      [22..23] uint16 extra_hdr_len
      [24..]   bytes  extra_hdr
      [24+N]   uint8  checksum (XOR of bytes 0..(24+N-1))
      followed by payload.
    """
    if timestamp_ms is None:
        timestamp_ms = int(time.time() * 1000)

    extra_len = len(extra_hdr)
    payload_len = len(payload)

    base_hdr = struct.pack(
        "<IH I Q I H",
        MAGIC,
        flags,
        payload_len,
        timestamp_ms,
        packet_id,
        extra_len,
    )
    full_hdr_no_chk = base_hdr + extra_hdr
    chk = compute_checksum(full_hdr_no_chk)

    return full_hdr_no_chk + bytes([chk]) + payload


class PacketParser:
    """
    Stream parser for incoming Revopoint packet streams.
    Handles buffer accumulation, magic byte synchronization, and checksum verification.
    """

    def __init__(self):
        self._buffer = bytearray()

    def feed(self, data: bytes) -> Generator[Tuple[dict, bytes], None, None]:
        """
        Feeds incoming bytes into parser and yields (header_dict, payload) for each complete frame.
        """
        self._buffer.extend(data)

        while True:
            # Look for 4-byte magic word
            if len(self._buffer) < BASE_HEADER_SIZE:
                return

            magic_idx = self._buffer.find(b"\x55\xaa\x55\xaa")
            if magic_idx == -1:
                # Keep last 3 bytes in case magic is split across chunks
                del self._buffer[:-3]
                return

            if magic_idx > 0:
                del self._buffer[:magic_idx]

            if len(self._buffer) < BASE_HEADER_SIZE:
                return

            # Unpack base header
            magic, flags, payload_len, timestamp_ms, packet_id, extra_hdr_len = (
                struct.unpack_from("<IH I Q I H", self._buffer, 0)
            )

            total_hdr_len = BASE_HEADER_SIZE + extra_hdr_len
            total_packet_len = total_hdr_len + 1 + payload_len

            if len(self._buffer) < total_packet_len:
                return  # Need more data

            # Verify XOR checksum
            expected_chk = self._buffer[total_hdr_len]
            actual_chk = compute_checksum(self._buffer[:total_hdr_len])

            if actual_chk != expected_chk:
                # Corrupted header - drop 1 byte and re-sync
                del self._buffer[:1]
                continue

            extra_hdr = bytes(self._buffer[BASE_HEADER_SIZE:total_hdr_len])
            payload = bytes(self._buffer[total_hdr_len + 1 : total_packet_len])

            hdr_info = {
                "magic": magic,
                "flags": flags,
                "payload_len": payload_len,
                "timestamp_ms": timestamp_ms,
                "packet_id": packet_id,
                "extra_hdr_len": extra_hdr_len,
                "extra_hdr": extra_hdr,
            }

            # Advance buffer
            del self._buffer[:total_packet_len]
            yield hdr_info, payload
