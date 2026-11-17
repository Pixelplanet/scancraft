# Revopoint Protocol Framing

> **Historical document.** Written during the first protocol analysis; several statements here were later contradicted by measurement. Current state: [the documentation map](../README.md) and [device-internals.md](device-internals.md).

> Confirmed from binary analysis of `3DCamera.dll` (v6.4.1, Jenkins build `3DCameraSDK-windows-x64`).
> Implementation files: `src\common\deviceDataPacketParser.cpp`, `src\common\deviceUpEventDispatch.cpp`.

---

## 1. Magic Bytes and Synchronization

Every data packet starts with a 4-byte synchronization magic word:

```
Hex bytes: 55 AA 55 AA
Value:     0xAA55AA55 (32-bit little-endian)
```

The stream parser (`cs::DeviceDataPacketParser::consumeStream` at `0x181249450`) scans the incoming buffer byte-by-byte for `0xAA55AA55`. If an invalid packet or checksum mismatch occurs, it drops 1 byte and re-synchronizes to the next `0xAA55AA55`.

---

## 2. Packet Header Layout

The base packet header is **24 bytes** (`0x18` bytes) long, optionally followed by extended header bytes:

| Offset (Hex) | Offset (Dec) | Size (Bytes) | Type | Name | Description |
| :--- | :--- | :--- | :--- | :--- | :--- |
| `0x00..0x03` | 0..3 | 4 | `uint32_t` | `magic` | Constant `0xAA55AA55` (`55 AA 55 AA`) |
| `0x04..0x05` | 4..5 | 2 | `uint16_t` | `flags` / `sub_type` | Stream subtype / channel indicator |
| `0x06..0x09` | 6..9 | 4 | `uint32_t` | `payload_len` | Size of following payload data |
| `0x0A..0x11` | 10..17 | 8 | `uint64_t` | `timestamp_ms` | Device timestamp in milliseconds |
| `0x12..0x15` | 18..21 | 4 | `uint32_t` | `packet_id` | Packet opcode or event identifier |
| `0x16..0x17` | 22..23 | 2 | `uint16_t` | `extra_hdr_len` | Additional header length (max `0x1000`) |
| `0x18..` | 24.. | `extra_hdr_len` | `bytes` | `extra_hdr` | Extended header metadata (if `extra_hdr_len > 0`) |
| `0x18 + N` | 24 + N | 1 | `uint8_t` | `checksum` | **XOR checksum** over preceding header bytes |

---

## 3. Checksum Verification

- **Algorithm**: Single-byte bitwise XOR accumulation of all preceding header bytes:
  $$\text{checksum} = \bigoplus_{i=0}^{23 + \text{extra\_hdr\_len}} \text{header\_bytes}[i]$$
- **Location**: Placed immediately following the header at offset `0x18 + extra_hdr_len`.
- **Implementation**: Located at `0x18124a020` in `3DCamera.dll`, vectorized with SSE2/AVX (`pxor`, `psrldq`).
- **Error Handling**: On checksum failure, the driver emits:
  `[SDK WARN] [deviceDataParser] xor mismatch at off=%zu (drop 1 byte) got=0x%02x exp=0x%02x hdr_len=%u`

---

## 4. Known Packet Types / Events

Dispatched by `cs::logDeviceDataPacketEvent` (`deviceUpEventDispatch.cpp`):

| Packet / Event | Opcode / Identifier | Payload Fields |
| :--- | :--- | :--- |
| `DEVICE_UP_EVENT_START` | Dynamic / stream start | `seq` (`uint32_t`), `timestamp_ms` (`uint64_t`) |
| `DEVICE_UP_EVENT_KEY` | Hardware button press | `key_num` (`int32`), `key_level` (`int32`, `0` = down, `1` = up), `key_time` (`int32`) |
| `DEVICE_UP_EVENT_POWER_SUPPLY_STATE` | Power / Battery status | `isSupplyMet` (`int32`), `dcPower` (`int32`), `vot` (voltage `int32`), `cur` (current `int32`) |
| `DEVICE_UP_EVENT_FIREWARE_LOG` | Device log stream | Null-terminated string (`text`) from device OS |
| `DeviceDataEvent` | General data / sensor packet | Raw payload processed per `packet_id` |

---

## 5. QuickLZ Level 1 Depth Compression & Framing

Disassembled from `cs::DecoderDepthQuicklz::decoderAFrame` (`0x1812f5344`) and `0x181397850`:

Depth frames delivered over network streams (e.g. `/cgi-bin/zx_media.cgi?camera_id=21`) are compressed using **standard QuickLZ Level 1 Stream Mode** (hash table size = 4096 entries, 32 KB decompressor state).

### 5.1 QuickLZ Header Layout

The QuickLZ compressed chunk begins with a 3-byte or 9-byte header:

- **Byte 0 (Flags)**:
  - `Bit 0`: `1` if compressed, `0` if raw uncompressed data.
  - `Bit 1 / 2`: Header format selector.
    - If `(flags & 2) == 0`: **3-byte header** (used when compressed size $\le 65535$).
      - `source[1..2]`: 16-bit compressed size.
    - If `(flags & 2) == 2`: **9-byte header** (used for full depth frames, e.g. $640 \times 512$):
      - `source[1..4]`: 32-bit unsigned integer = compressed stream size.
      - `source[5..8]`: 32-bit unsigned integer = uncompressed output size ($640 \times 512 \times 2 = 655,360\text{ bytes}$).

### 5.2 Depth Scaling & Unpacking

- **Resolution**: $640 \times 512$ pixels.
- **Pixel Type**: 16-bit unsigned integer (`uint16_t` little-endian).
- **Scale Factor**: `0.01 mm` ($10\ \mu\text{m}$) per LSB (from `3DCamera.dll` constant `0x3c23d70a`).
- **Depth Formula**:
  $$Z(\text{mm}) = d_{raw} \times 0.01$$

---

## 6. Container Frame Header (`frame_header_t`)

Extracted from `parseMarkerFrameHeader` (`0x181a82216`):

```cpp
struct frame_header_t {
    uint32_t total_length;      // Total payload length
    uint32_t frame_header_size; // Size of this header struct
    uint16_t send_width;        // Frame pixel width (e.g. 640 or 1280)
    uint16_t send_height;       // Frame pixel height (e.g. 512 or 1024)
    uint32_t jpeg_data_size;    // JPEG payload size if compressed RGB/IR preview
    uint32_t marker_count_1;    // Detected target markers count (group 1)
    uint32_t marker_count_2;    // Detected target markers count (group 2)
    uint32_t marker_count_3;    // Detected target markers count (group 3)
    uint64_t timestamp_ms;      // Hardware capture timestamp in milliseconds
};
```

