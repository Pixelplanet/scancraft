# Revopoint POP 4 Protocol Reverse Engineering Plan

> **Historical document.** Written during the first protocol analysis; several statements here were later contradicted by measurement. Current state: [the documentation map](docs/README.md) and [docs/HANDOFF.md](docs/HANDOFF.md).

## Goal
Fully document the Revopoint POP 4 communication protocol and build a lightweight Python reference client capable of discovering the device, connecting, performing the handshake, and capturing scan data without Revo Scan 6.

## Tasks
- [x] Task 1: Disassemble packet framing parser in `3DCamera.dll` → Verify: Extracted magic bytes (`0xAA55AA55`), 24-byte header layout, and XOR checksum function (`0x18124a020`).
- [x] Task 2: Map USB & Network transports and document initial specifications → Verify: Created `docs/protocol/overview.md` and `docs/protocol/framing.md`.
- [x] Task 3: Extract UDP probe port, JSON beacon schema, and HTTP CGI command opcodes (`/cgi-bin/zx_cmd.cgi`) → Verify: Complete `docs/protocol/commands.md` cataloging discovery, parameter queries, and stream control endpoints.
- [x] Task 4: Connect to live hardware at `192.168.178.34`, map open ports, and verify CGI endpoints → Verify: Discovered open ports 80, 8871, 9999, 22. Confirmed live queries for serial `D00000000B0S00E1B`, temperatures, and downloaded full stereo calibration YAML.
- [x] Task 5: Implement protocol framing decoder and packet builder in Python (`client/framing.py`) → Verify: Unit tests pass (`tests/test_client.py`: 4/4 passed).
- [x] Task 6: Implement device client and real-time IMU telemetry listener (`client/scanner.py`, `client/imu.py`) → Verify: Live demo script (`scripts/live_demo.py`) streamed 138 Hz IMU samples from hardware.
- [ ] Task 7: Extract complete USB Vendor ID / Product ID (VID/PID) and HID report descriptor / control packet layouts → Verify: Document primary HID control commands and secondary HID stream formats in `docs/protocol/overview.md`.
- [ ] Task 8: Stream depth and RGB data using reference client with QuickLZ / H.264 decoders → Verify: Python client triggers stream over TCP port 8871 / HTTP and receives valid depth frames.
- [ ] Task 9: Verification & Documentation Finalization → Verify: End-to-end handshake + point cloud generation succeeds without proprietary binaries running.

## Done When
- [x] Live hardware communication, device parameters, and IMU telemetry established.
- [ ] All protocol specifications (`overview.md`, `framing.md`, `commands.md`) are complete and verified against live traffic.
- [ ] Standalone Python reference client can stream depth sensor frames and generate a point cloud.
