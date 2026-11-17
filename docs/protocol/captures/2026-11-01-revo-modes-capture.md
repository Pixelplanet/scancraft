# Revo Scan 6 mode start-up, exposure and discovery (2026-11-01)

Source: Wireshark/dumpcap capture of the PC's Ethernet adapter, filtered to the POP 4 plus
broadcast/multicast (`revo-modes.pcapng`, 5.1 GB, **not committed**: it contains the scanner's
Wi-Fi configuration files, which Revo downloads), plus Revo's own SDK log of the same session
(`%LOCALAPPDATA%\Revopoint\RevoScan6\logs\sdk.log`, 19:17–19:37). The SDK log lists every
`zx_cmd.cgi` request with a timestamp; the capture adds discovery and the media streams.

Software: Revo Scan 6 (updated just before the session; SDK V3.2.427). Firmware
v5.3.94.20260808 (unchanged since 2026-10-12), algorithm v101.9.4.0622.

User actions, in order: connect; Full-field HD + Feature preview, exposure slider down/up/middle;
scan ~5 s; VCSEL Rapid preview + scan (twice, Revo was restarted in between); Hybrid HD
preview + scan; Marker + colour scan; close.

Tags: [C] confirmed (log/capture/measurement), [I] inferred.

## Discovery [C]

The host sends the literal 10 bytes `{intent:0}` (not valid JSON) by UDP to port **8080**, to the
LAN broadcast address (`192.168.178.255`) and to the scanner directly. The scanner answers from
an ephemeral port to the sender's port with JSON:

```json
{"intent":0, "equip_id":"<serial>", "ip":"192.168.178.34", "firmware":"v5.3.94.20260808",
 "algo":"v101.9.4.0622", "boot_mode":"normal", "network_speed":"-1Mb"}
```

Revo repeats the request about every 2 s. Implemented in `crates/scancraft-driver-revopoint/src/discovery.rs`
(found the POP 4 in 1.5 s). The old HTTP sweep of the default route's /24 missed it because this
PC's default route is a different adapter.

## Connect sequence [C]

`download=/data/rtnFile`, data socket TCP 8871, `get_depth_reso`,
calibration downloads (`camparam` and `camparam_blue`: `camparam.yaml`, `LC_RT.bin`,
`Distort.bin`, `Pl.bin`, `camparamLR/Q.bin`, `mapparamL/R.bin`, `Prgb.bin`, `metroExtra.bin`),
`system_cmd=echo '123' > /tmp/use_hard_encoder`, `algo_get_cmd_buf=1168` (FPGA version),
`0xb03 0`, `download=/data/brightnessGainMap.json`, `algo_get_cmd_buf=2328`,
`cam_type=imu&get_IMUStatus`, `get_IMUVersion`, `get_thermal`, `mkdir /data/deviceactive`,
`algo_get_cmd_buf=1162` ×2, `0x48b 10`.

## Mode start sequences [C]

All start with `close_stream_all`, `set_display_reso=1&&set_display_width=640&&set_display_height=512&&set_display_type=<t>`,
`set_depth_output_fmt=<f>`, `set_trigger_mode=0`, `0x707 200 600`, RGB setup
(`cam_type=usb&set_resolution=1&width=1280&height=1024`, `set_aa_mode=1`, `set_aeMaxExp=<n>`),
and end with `0xb04 1`, `0xb03 0`, RGB auto white balance. Media: the mode's stream plus the RGB
stream `camera_id=50&type_id=20` 2–6 s later. Stop: `0x483`, `0xb04 0`, `0x481 1`, `0x483`.

| Mode | fmt/type | media | registers after the common part |
| --- | --- | --- | --- |
| Full-field HD (General) | 3 / 4 | `camera_id=21` | `0x488 0`, `0xb11 11000`, `0x911 3000`, `0x903 0x10`, `0xb07 0`, `0x910 7000`, `0x911 3000`, `0xb07 0`, `0x707 250 500`, `0x705 10`, `0xa04 0`, `0xb03 0`, `0xb04 1` |
| VCSEL Rapid (General) | 1 / 2 | `camera_id=33&type_id=20` | `0x3001 12`, `0x48d 80`, `0x488 1`, `0xb11 11000`, `0x911 666`, `0x903 0x20`, `0xb08 1366`, `0x910 50000`, `0x911 666`, `0x48d 100`, `0xb08 1366`, `0xb04 1`, `0xb03 0`, `0xb04 1`; the second session also sent `0x701 1` after `0xb03 0` |
| Hybrid HD (General) | 5 / 10 | `camera_id=33&type_id=21` | `0x48d 80`, `0x488 0`, `0x911 3000`, `0x903 0x10`, `0xb1b 65`, `0xb11 11000`, `0x910 7000`, `0x911 3000`, `0xb1b 65`, `0xb1c 18000`, `0xb04 1`, `0xb03 0`, `0xb04 1` |
| Marker + colour (Full-field) | 3 / 6 | `camera_id=21` | `set_aeMaxExp=4`, `0x488 0`, `0xb11 6000`, `0x911 3000`, `0x903 0x10`, `0xb07 34`, `0xb13 1`, `0xb01 1`, `0x910 7000`, `0x911 3000`, `0xb07 34`, `0xb05 0`, `0xb03 1`, `0xb02 0`, `0xb12 255`, `0x707 250 500`, `0x705 10`, `0xa04 0`, `0xb04 1` |

`0xb11` differs by mode (11000 Full-field/VCSEL/Hybrid, 6000 marker); meaning unknown [I: timing].
Revo's own log shows the first VCSEL session produced no points ("Point cloud has too few points
after conversion (0)"); the second produced small previews (~10k points).

## Exposure slider (Full-field HD) [C]

Each slider step sends `0x488 1` (manual), `0x911 <µs>`, `0x903 <gain>`, `0xb07 0`; "auto" sends
`0x488 0`, `0x911 3000`, `0x903 0x10`, `0xb07 0`. Steps seen: 4600/0x10, 5100/0x18, 5400/0x20,
5500/0x28, 5600/0x30, 5700/0x38, 5800/0x40, 5900/0x48, 6000/0x50 (from `brightnessGainMap.json`
[I]). `0xb07` is the IR fill-light brightness (`modes.md`), resent as 0 to keep it off.

Replaying exactly these writes on a running Full-field HD stream (`examples/exposure_probe.rs`,
static scene) changed neither the IR plane (mean 95.0–95.4) nor depth coverage (11.0–11.1 %),
the same result as the earlier probe without `0xb07`. Open: whether Revo's preview changes
visibly when the slider moves.

## Frame rate [C]

Revo's stream table rates 640×512 depth (fmt 3) at **5 fps**; ScanCraft measures 6.5 fps
(40 frames) on the improved Wi-Fi link, the same as before. The device, not the link, limits
Full-field HD.

## VCSEL follow-up [C]

With Revo's exact sequence and media (`camera_id=33&type_id=20`, JPEG pair), the pair is the same
as on `camera_id=21`: top half dark without dots (mean ~17), bottom half dotted (mean ~64). Sending
`0x701 1` after the setup turns the VCSEL off for both. Revo's matcher is stereo
(`scan.dll`: `speckleStereo_*` SGM settings; `RPAFusionRGBD.dll`: `createSpeckleStereo`), so a
lit pair should exist. See `docs/protocol/vcsel.md`.
