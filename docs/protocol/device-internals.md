# POP 4 internals: shell access, the DSP log, what every register means

Found 2026-11-10 on firmware v5.3.99.20261010 (updated from v5.3.94 by Revo Scan on 2026-11-07).
Everything here was observed on our own scanner over Wi-Fi. Tags: [C] confirmed, [I] inferred.

This file is the ground truth for the driver. Before changing a mode's setup, check what the DSP
says about it (section 2) instead of inferring from images.

## 1. The control CGI is a shell

`/cgi-bin/zx_cmd.cgi?system_cmd=<command>` runs `<command>` in a shell on the scanner (BusyBox,
root) and answers `{"return":0}` whether or not the command did what was meant [C].
`?download=<path>` returns any file. Together they give read access to everything:

```
system_cmd=cat /proc/loadavg > /tmp/out.txt      then      download=/tmp/out.txt
```

**Encoding rules of this firmware's CGI** [C, each tested]:

| Character | How to send | Note |
| --- | --- | --- |
| space | `%20` | |
| `>` | `%3E` | |
| `\` | `%5C` | |
| `/` `'` `;` | raw | `%2F` and `%27` are **not** decoded |
| `\|` | cannot be sent | arrives as the text `%7C`; produce it on the device with `printf '\174'` |
| `&` `#` | cannot be sent | they end the query |
| long commands | split | commands beyond roughly 150 characters are dropped silently; Revo writes `/tmp/.longCmd.sh` |

The driver used to encode `/` as `%2F`. On v5.3.99 every register write
(`echo s .. > /dev/rk_preisp`) therefore went into a file named `%2Fdev%2Frk_preisp` next to the
CGI, and the scanner answered `{"return":0}`. Every "this setting has no effect" result between
the firmware update and 2026-11-10 came from that. Fixed in `protocol.rs::system_cmd_query`.

**Soft reboot** [C]: `system_cmd=sync`, then `system_cmd=reboot`. The control interface answers
again after about 35 s. This clears a hung media server (no power cycle needed).
`Control::reboot()`; shell helper `tools/scanner/scanner_reboot.sh`.

Processes: `/tmp/zxmain` (streams, CGI back end), `lighttpd`, `rkaiq_3A_server`, `dropbear` (SSH
on port 22), `wpa_supplicant`. Listening: 80, 22, 8871. Port 9999 (IMU) is **not** open on v5.3.99.
A load average around 10 is normal for it.

## 2. The DSP log

The image DSP (Rockchip RK1608, driver `rk1608 spi1.0`) logs every message it receives and what
it does with it to the kernel log. `dmesg` is flooded by per-frame lines, so filter on the device:

```
bash tools/scanner/dsplog_start.sh     # once per boot: background loop, filtered lines -> /tmp/dsp.log
bash tools/scanner/dsplog_get.sh       # print and empty the log
```

Typical lines: `prepare send[0x911 to dsp]`, `id_msg_cam_set_exposure_time_task 3000 ====`,
`exposure time i2c cfg failed!`, `projector(0) init (type:12 ZX_SPECKLE)`,
`algorithm(0) init (type:1 PSP)`, `cam(0) unknown msg(0xa04)`.

## 3. A format change reloads the DSP

`set_depth_output_fmt=<n>` with a **different** format powers the RK1608 off, downloads its
firmware again (`preisp.rkl`) and re-initialises cameras, projector and algorithm for that format
[C]. Everything written to `/dev/rk_preisp` before is gone. With the **same** format nothing is
reloaded and all state persists, including the projector type, `0xb04`, and a stopped trigger.

At that reload the DSP uses the display type set at that moment. Setting the format before the
display type built a Full-field layout into the Hybrid container: one frame, silence, hung media
server (reproduced with `SCANCRAFT_FMT_FIRST=1`). Hence the one start order for all modes:

1. `close_stream_all`
2. `set_display_reso=1&&set_display_width=..&&set_display_height=..&&set_display_type=..`
3. `set_depth_output_fmt=..`
4. open `zx_media.cgi?..`
5. `set_trigger_mode=0`
6. wait for the first frame (Revo: up to 20 s)
7. the mode's DSP messages

What the DSP sets up per format [C]:

| fmt | Mode | Algorithm | Projector type |
| --- | --- | --- | --- |
| 1 | IR pair (VCSEL, old lasers) | 0 `PAIR` | kept / set by `0x3001` |
| 3 | Full-field HD, markers | 1 `PSP` | 6 `ZX_ELECTROSTATIC` |
| 5 | Hybrid HD | (speckle + stripes) | set by firmware |
| 7 | Cross Lines + speckle (new in v5.3.99) | 12 `PAIR_N_REMAP` | 14 `ZX_MULTI_CROSS_LINE_ADD_SPECKLE` |

## 4. DSP messages (`echo s <id> <values> > /dev/rk_preisp`)

Names are the DSP's own [C].

| id | DSP name | Meaning / values seen |
| --- | --- | --- |
| `0x481 1` | `proj_create_trigger` | start capturing (frames are produced even without a client) |
| `0x483` | `proj_stop_trigger` | stop capturing |
| `0x488` | `proj_return_scan_control` | 0 / 1; Revo sends 1 before manual exposure, 0 for the preset |
| `0x48b` | `proj_fov` | 10 at connect |
| `0x48c` | — | **unknown message** on this firmware |
| `0x48d` | `proj_set_ld_luma` | laser-diode level; Revo sends `255 1` for lines |
| `0x701 1` | `algo_calibration` | restarts the algorithm chain |
| `0x705` | `algo_set_background_threshold` | 5 / 10 |
| `0x707 a b` | `algo_set_z_range` | depth window, mm |
| `0x903` | `cam_set_gain_reg` | `0x10` per gain step |
| `0x904` | `cam_set_fake_mode` | |
| `0x910` | `cam_set_frame_time` | µs |
| `0x911` | `cam_set_exposure_time` | µs; fails with `i2c cfg failed` when the cameras are not streaming |
| `0x912` | `cam_set_auto_expo_mode` | 0..3 |
| `0xa04` | — | **unknown message** on this firmware (Revo still sends it) |
| `0xa05` | `set_fringe_pattern` | 0..6 |
| `0xb00` | `ir_led_control_type` | |
| `0xb01` | `ir_led_enable` | IR fill LED (markers) |
| `0xb02` | `rgb_led_control_type` | |
| `0xb03` | `rgb_led_enable` | white LED |
| `0xb04` | `ld_enable` | laser diodes: Full-field projector gate, VCSEL, blue lines |
| `0xb05` | `projector_white_pattern_on_off` | |
| `0xb07` | `set_ir_led_luma` | IR fill LED level |
| `0xb08` | `set_ld_exposure_time` | laser exposure, µs |
| `0xb11` | `set_rgb_led_exposure_time` | 6000 / 11000 |
| `0xb12` | `set_rgb_led_luma` | white LED level |
| `0xb13` | `ir_led_wavelength_switch` | |
| `0xb1b` | `speckle_ld_luma` | dot-projector level (65 Hybrid, 170 Cross + speckle) |
| `0xb1c` | `min_trigger_interval` | µs |
| `0x3001 n` | `projector_init` | 6 `ZX_ELECTROSTATIC`, 7 `ZX_MULTI_CROSS_LINE`, 9 `ZX_SINGLE_LINE`, 12 `ZX_SPECKLE`, 14 `ZX_MULTI_CROSS_LINE_ADD_SPECKLE`, 15 `ZX_PARALLEL_LINE_ADD_SPECKLE` |
| `0x3010 ..` | `projector_prop` | |

`cam_type=mipi&algo_get_cmd_buf=<decimal id>` reads a value (1162 = `0x48a`, 1163 = `0x48b`,
1168 = FPGA version, 2328/2329 = `0x918`/`0x919`) [I from Revo's log].

## 5. What changed with firmware v5.3.99

- CGI decodes fewer escapes (section 1).
- IMU: `get_IMUStatus` answers 0 (was 1), no IMU values in the frame header, port 9999 closed.
  Revo Scan sees the same status 0. The driver reports `has_imu = false` and tracks without the
  motion prior.
- Cross Lines in Revo Scan now runs format 7 / display type 4 at 1280x1024 on
  `camera_id=33&type_id=22` (SDK format 272): one JPEG of four 1280x1024 planes per frame, the
  device calls it `depth+sp+ls`. Not decoded yet (laser.md).
- Exposure and gain work in Full-field HD (measured: IR mean 27 at 0.7 ms, 100 at 6 ms, 162 at
  6 ms with gain 5).

## 6. Tools

| Tool | Purpose |
| --- | --- |
| `examples/mode_check.rs` | one light/tracking/object combination through driver + pipeline + tracker |
| `examples/stream_probe.rs` | any format/stream in Revo's order, timed register writes, per-plane statistics |
| `examples/exposure_probe.rs` | register steps on a running Full-field stream, IR/depth/colour statistics |
| `tools/scanner/rd.sh "<cmd>"` | run a shell command on the scanner and print its output |
| `tools/scanner/dsplog_start.sh`, `dsplog_get.sh` | DSP log (section 2) |
| `tools/scanner/scanner_reboot.sh` | soft reboot |
