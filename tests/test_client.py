"""
Unit tests for Revopoint protocol framing and parser.
"""

import struct
import pytest
from client.framing import PacketParser, pack_packet, compute_checksum, MAGIC


def test_checksum_computation():
    data = bytes([0xAA, 0x55, 0xAA, 0x55, 0x01, 0x02, 0x03, 0x04])
    expected = 0
    for b in data:
        expected ^= b
    assert compute_checksum(data) == expected


def test_pack_and_parse_packet():
    parser = PacketParser()
    payload = b"hello revopoint pop 4 depth data"
    packed = pack_packet(packet_id=101, payload=payload, flags=2, timestamp_ms=12345678)

    # Base header 24 + 0 extra + 1 checksum + len(payload)
    assert len(packed) == 24 + 1 + len(payload)
    assert packed[:4] == struct.pack("<I", MAGIC)

    # Feed into parser
    packets = list(parser.feed(packed))
    assert len(packets) == 1
    hdr, received_payload = packets[0]

    assert hdr["magic"] == MAGIC
    assert hdr["packet_id"] == 101
    assert hdr["flags"] == 2
    assert hdr["timestamp_ms"] == 12345678
    assert hdr["payload_len"] == len(payload)
    assert received_payload == payload


def test_parser_corrupted_checksum():
    parser = PacketParser()
    packed = bytearray(pack_packet(packet_id=202, payload=b"valid data"))
    
    # Corrupt checksum byte (at index 24)
    packed[24] ^= 0xFF

    packets = list(parser.feed(bytes(packed)))
    assert len(packets) == 0


def test_parser_fragmented_stream():
    parser = PacketParser()
    packed = pack_packet(packet_id=303, payload=b"a" * 100)

    # Feed in small chunks of 10 bytes
    packets = []
    for i in range(0, len(packed), 10):
        chunk = packed[i : i + 10]
        packets.extend(parser.feed(chunk))

    assert len(packets) == 1
    hdr, payload = packets[0]
    assert hdr["packet_id"] == 303
    assert payload == b"a" * 100


def test_scanner_emitter_commands(monkeypatch):
    from client.scanner import RevopointPOP4

    client = RevopointPOP4("127.0.0.1")
    called_urls = []

    class DummyResponse:
        def __init__(self, text):
            self.text = text

    def dummy_get(query):
        called_urls.append(query)
        return DummyResponse('{"return":0}')

    monkeypatch.setattr(client, "_get", dummy_get)

    assert client.set_projector_power(True) is True
    assert "system_cmd=" in called_urls[-1]
    assert "0x912%201" in called_urls[-1] or "0x912 1" in called_urls[-1]

    assert client.set_speckle_luminance(25) is True
    assert "0xb1b%2025" in called_urls[-1] or "0xb1b 25" in called_urls[-1]

    assert client.set_ir_led_luminance(15) is True
    assert "0xb07%2015" in called_urls[-1] or "0xb07 15" in called_urls[-1]

    assert client.set_rgb_led_luminance(50) is True
    assert "0xb12%2050" in called_urls[-1] or "0xb12 50" in called_urls[-1]

    # Test laser line & mode commands
    assert client.set_laser_diode_enable(True) is True
    assert "0xb04%200" in called_urls[-1] or "0xb04 0" in called_urls[-1]

    assert client.set_laser_mode("cross") is True
    assert any("0x3010" in u for u in called_urls[-2:])
    assert any("0x3001" in u for u in called_urls[-2:])

    assert client.set_laser_mode("speckle") is True
    assert any("0x3001%2012" in u or "0x3001 12" in u for u in called_urls[-2:])

    assert client.set_laser_luminance(80, mode="cross") is True
    assert "0x48d%2080%201" in called_urls[-1] or "0x48d 80 1" in called_urls[-1]

    assert client.set_laser_exposure(4500, mode="cross") is True
    assert "0xb08%204500%201" in called_urls[-1] or "0xb08 4500 1" in called_urls[-1]


