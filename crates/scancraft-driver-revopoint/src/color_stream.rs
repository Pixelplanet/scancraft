//! The POP 4 RGB camera stream (`zx_media.cgi?camera_id=50&type_id=20`), run next to a depth
//! stream. Wire format and evidence: `docs/protocol/color.md`.
//!
//! Packets use the media-stream framing `[u32 LE 0x11223344][u32 LE len][len bytes]`, but the
//! payload is not QuickLZ: it is one baseline JPEG (1280x1024, 4:2:0) followed by a 4-byte
//! little-endian capture timestamp in milliseconds on the same device clock as the depth
//! stream's frame counter. The RGB camera is triggered with the IR cameras: every RGB frame lies
//! 1-2 ms after a depth frame, so frames are paired by timestamp (as Revo's SDK does with
//! `setDepthRgbMatchParam iDifThreshold = 15`).

use crate::control::Control;
use crate::media::PACKET_MAGIC;
use futures::StreamExt;
use scancraft_core::{ColorCamera, ColorFrame, RgbImage};
use scancraft_driver::DriverError;
use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Condvar, Mutex};
use std::time::{Duration, Instant};
use tokio::task::JoinHandle;

pub const MEDIA_QUERY: &str = "camera_id=50&type_id=20";
pub const WIDTH: u32 = 1280;
pub const HEIGHT: u32 = 1024;
/// Largest RGB packet accepted (observed 250-310 kB).
const MAX_PACKET_LEN: usize = 4 * 1024 * 1024;
/// Revo pairs depth and RGB frames whose timestamps differ by at most this (ms).
pub const MATCH_TOLERANCE_MS: u64 = 15;

/// Splits the RGB stream body into packets. Feed it bytes as they arrive.
#[derive(Default)]
pub struct RgbPacketParser {
    buf: Vec<u8>,
}

impl RgbPacketParser {
    pub fn push(&mut self, bytes: &[u8]) {
        self.buf.extend_from_slice(bytes);
    }

    /// The next complete packet payload (JPEG + timestamp trailer), or `None` until more bytes
    /// arrive. A magic whose payload does not start with a JPEG SOI marker is skipped.
    pub fn next_packet(&mut self) -> Option<Vec<u8>> {
        loop {
            let Some(pos) = self.buf.windows(4).position(|w| w == PACKET_MAGIC) else {
                let keep = self.buf.len().min(3);
                self.buf.drain(..self.buf.len() - keep);
                return None;
            };
            self.buf.drain(..pos);
            if self.buf.len() < 10 {
                return None;
            }
            let len = u32::from_le_bytes(self.buf[4..8].try_into().unwrap()) as usize;
            if !(8..=MAX_PACKET_LEN).contains(&len) || self.buf[8..10] != [0xff, 0xd8] {
                self.buf.drain(..4);
                continue;
            }
            if self.buf.len() < 8 + len {
                return None;
            }
            let payload = self.buf[8..8 + len].to_vec();
            self.buf.drain(..8 + len);
            return Some(payload);
        }
    }
}

/// Splits a packet payload into the JPEG and its device timestamp (ms).
pub fn split_packet(payload: &[u8]) -> Option<(&[u8], u32)> {
    let n = payload.len();
    if n < 8 || payload[..2] != [0xff, 0xd8] || payload[n - 6..n - 4] != [0xff, 0xd9] {
        return None;
    }
    Some((&payload[..n - 4], u32::from_le_bytes(payload[n - 4..].try_into().unwrap())))
}

/// Decodes a packet payload into a colour frame carrying its JPEG and calibration.
pub fn decode_packet(payload: &[u8], camera: Option<&Arc<ColorCamera>>) -> Result<ColorFrame, String> {
    let (jpeg, timestamp) = split_packet(payload).ok_or("not a JPEG + timestamp packet")?;
    let rgb = image::load_from_memory_with_format(jpeg, image::ImageFormat::Jpeg).map_err(|e| e.to_string())?.to_rgb8();
    let (width, height) = rgb.dimensions();
    let camera = camera.map(|c| if (c.width, c.height) == (width, height) { c.clone() } else { Arc::new(c.scaled(width, height)) });
    Ok(ColorFrame {
        image: RgbImage { width, height, channels: 3, data: rgb.into_raw() },
        jpeg: Some(Arc::new(jpeg.to_vec())),
        timestamp_ms: timestamp as u64,
        camera,
    })
}

/// Hands colour frames from the RGB stream to the depth stream's decode thread, which attaches
/// the one taken at the same instant.
#[derive(Default)]
pub struct ColorSync {
    state: Mutex<SyncState>,
    arrived: Condvar,
}

#[derive(Default)]
struct SyncState {
    frames: VecDeque<ColorFrame>,
    last_arrival: Option<Instant>,
}

impl ColorSync {
    /// Frames kept waiting for their depth partner.
    const CAPACITY: usize = 8;

    pub fn push(&self, frame: ColorFrame) {
        let mut s = self.state.lock().unwrap();
        s.frames.push_back(frame);
        while s.frames.len() > Self::CAPACITY {
            s.frames.pop_front();
        }
        s.last_arrival = Some(Instant::now());
        self.arrived.notify_all();
    }

    /// The colour frame taken within [`MATCH_TOLERANCE_MS`] of `timestamp_ms`. Waits up to `wait`
    /// for it while the RGB stream is delivering (the two streams arrive over separate
    /// connections); returns at once if the stream has delivered nothing for a second.
    pub fn take_match(&self, timestamp_ms: u64, wait: Duration) -> Option<ColorFrame> {
        let deadline = Instant::now() + wait;
        let mut s = self.state.lock().unwrap();
        loop {
            while s.frames.front().is_some_and(|f| f.timestamp_ms + MATCH_TOLERANCE_MS < timestamp_ms) {
                s.frames.pop_front();
            }
            if let Some(i) = s.frames.iter().position(|f| f.timestamp_ms.abs_diff(timestamp_ms) <= MATCH_TOLERANCE_MS) {
                return s.frames.remove(i);
            }
            if s.frames.front().is_some_and(|f| f.timestamp_ms > timestamp_ms + MATCH_TOLERANCE_MS) {
                return None;
            }
            let live = s.last_arrival.is_some_and(|t| t.elapsed() < Duration::from_secs(1));
            let now = Instant::now();
            if !live || now >= deadline {
                return None;
            }
            s = self.arrived.wait_timeout(s, deadline - now).unwrap().0;
        }
    }
}

/// RGB setup as logged by Revo Scan 6 when it starts the colour stream next to Full-field HD
/// (sdk.log 2026-10-12 09:24:33-36): resolution, auto exposure, auto-exposure limit 9 (ms; the
/// firmware answers `{"result":9}`), auto white balance.
pub async fn configure(ctl: &Control) -> Result<(), DriverError> {
    ctl.cgi_ok(&format!("cam_type=usb&set_resolution=1&width={WIDTH}&height={HEIGHT}")).await?;
    ctl.cgi_ok("cam_type=usb&set_aa_mode=1").await?;
    ctl.cgi("cam_type=usb&set_aeMaxExp=9").await?;
    ctl.cgi_ok("cam_type=usb&set_whitebalance_mode=1").await
}

/// Downloads and parses the RGB calibration (`/data/camparam/Prgb.bin`, `Distort.bin`,
/// `LC_RT.bin`; formats in [`crate::calibration::parse_color_calibration`]).
pub async fn download_calibration(ctl: &Control) -> Result<ColorCamera, DriverError> {
    download_calibration_files(ctl).await.map(|(camera, _)| camera)
}

/// [`download_calibration`], also returning the three files as received (name, bytes), so a
/// recording can be decoded offline.
pub async fn download_calibration_files(ctl: &Control) -> Result<(ColorCamera, Vec<(String, Vec<u8>)>), DriverError> {
    let mut files = Vec::new();
    for name in ["Prgb.bin", "Distort.bin", "LC_RT.bin"] {
        let url = format!("{}?download=/data/camparam/{name}", ctl.endpoints.base_url);
        let resp = ctl.http.get(&url).send().await.map_err(|e| DriverError::ConnectionFailed(format!("{name}: {e}")))?;
        let bytes = resp.error_for_status().map_err(|e| DriverError::Protocol(format!("{name}: {e}")))?.bytes().await;
        files.push(bytes.map_err(|e| DriverError::Protocol(format!("{name}: {e}")))?);
    }
    let camera = crate::calibration::parse_color_calibration(&files[0], &files[1], &files[2])
        .ok_or_else(|| DriverError::Protocol(format!("unexpected RGB calibration sizes {:?}", files.iter().map(|f| f.len()).collect::<Vec<_>>())))?;
    let named = ["Prgb.bin", "Distort.bin", "LC_RT.bin"].iter().zip(files).map(|(n, b)| (n.to_string(), b.to_vec())).collect();
    Ok((camera, named))
}

/// Streams RGB frames into `sync` until the task is aborted. JPEG decoding runs on its own
/// thread; with a recorder every packet is also written in the wire framing.
pub fn spawn(
    ip: String,
    camera: Option<Arc<ColorCamera>>,
    sync: Arc<ColorSync>,
    paused: Arc<AtomicBool>,
    recorder: Option<scancraft_driver::RawRecorder>,
) -> JoinHandle<()> {
    let (packet_tx, packet_rx) = std::sync::mpsc::sync_channel::<Vec<u8>>(2);
    let decode_sync = sync.clone();
    std::thread::spawn(move || {
        while let Ok(packet) = packet_rx.recv() {
            match decode_packet(&packet, camera.as_ref()) {
                Ok(frame) => decode_sync.push(frame),
                Err(e) => log::warn!("POP 4 RGB frame: {e}"),
            }
        }
    });
    let url = format!("http://{ip}/cgi-bin/zx_media.cgi?{MEDIA_QUERY}");
    tokio::spawn(async move {
        let client = reqwest::Client::builder().connect_timeout(Duration::from_secs(4)).tcp_nodelay(true).build().unwrap_or_default();
        loop {
            let resp = match client.get(&url).send().await.and_then(|r| r.error_for_status()) {
                Ok(r) => r,
                Err(e) => {
                    log::warn!("RGB stream connect: {e}");
                    tokio::time::sleep(Duration::from_millis(500)).await;
                    continue;
                }
            };
            let mut body = resp.bytes_stream();
            let mut parser = RgbPacketParser::default();
            while let Ok(Some(Ok(chunk))) = tokio::time::timeout(Duration::from_secs(4), body.next()).await {
                parser.push(&chunk);
                while let Some(packet) = parser.next_packet() {
                    if paused.load(Ordering::Relaxed) {
                        continue;
                    }
                    // Recorded through the writer thread: never waits for the disk.
                    if let Some(rec) = recorder.as_ref() {
                        rec.push(crate::device::wire_record(&packet));
                    }
                    match packet_tx.try_send(packet) {
                        Ok(()) | Err(std::sync::mpsc::TrySendError::Full(_)) => {}
                        Err(std::sync::mpsc::TrySendError::Disconnected(_)) => return,
                    }
                }
            }
            log::warn!("RGB stream stalled, reconnecting");
        }
    })
}

/// Reads a recorded RGB stream (wire framing, as written by [`spawn`] or captured with curl).
pub fn read_recording(bytes: &[u8]) -> Vec<Vec<u8>> {
    let mut parser = RgbPacketParser::default();
    parser.push(bytes);
    std::iter::from_fn(|| parser.next_packet()).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn jpeg(w: u32, h: u32) -> Vec<u8> {
        let img = image::RgbImage::from_fn(w, h, |x, y| image::Rgb([(x * 8) as u8, (y * 8) as u8, 100]));
        let mut out = std::io::Cursor::new(Vec::new());
        img.write_to(&mut out, image::ImageFormat::Jpeg).unwrap();
        out.into_inner()
    }

    fn packet(ts: u32) -> Vec<u8> {
        let mut payload = jpeg(32, 16);
        payload.extend_from_slice(&ts.to_le_bytes());
        let mut p = PACKET_MAGIC.to_vec();
        p.extend_from_slice(&(payload.len() as u32).to_le_bytes());
        p.extend_from_slice(&payload);
        p
    }

    #[test]
    fn parses_jpeg_packets_with_timestamp_trailer() {
        let mut stream = b"\r\n".to_vec();
        stream.extend_from_slice(&packet(7_872_154));
        stream.extend_from_slice(&[0x44, 0x33, 0x22, 0x11, 0x10, 0, 0, 0, 1, 2]); // not a JPEG
        stream.extend_from_slice(&packet(7_872_268));
        let mut parser = RgbPacketParser::default();
        let mut out = Vec::new();
        for chunk in stream.chunks(29) {
            parser.push(chunk);
            out.extend(std::iter::from_fn(|| parser.next_packet()));
        }
        assert_eq!(out.len(), 2);
        let frame = decode_packet(&out[1], None).unwrap();
        assert_eq!((frame.image.width, frame.image.height, frame.timestamp_ms), (32, 16, 7_872_268));
        assert_eq!(frame.jpeg.as_ref().unwrap()[..2], [0xff, 0xd8]);
        assert!(split_packet(&out[0][..out[0].len() - 1]).is_none(), "trailer must follow EOI");
    }

    fn frame(ts: u64) -> ColorFrame {
        ColorFrame { image: RgbImage { width: 1, height: 1, channels: 3, data: vec![0; 3] }, jpeg: None, timestamp_ms: ts, camera: None }
    }

    #[test]
    fn pairs_by_timestamp() {
        let sync = ColorSync::default();
        sync.push(frame(1000)); // stale
        sync.push(frame(5002));
        sync.push(frame(5116));
        assert_eq!(sync.take_match(5000, Duration::ZERO).map(|f| f.timestamp_ms), Some(5002));
        // No colour frame for this depth frame, and a later one already arrived: no waiting.
        let t = Instant::now();
        assert!(sync.take_match(5060, Duration::from_secs(1)).is_none());
        assert!(t.elapsed() < Duration::from_millis(100));
        assert_eq!(sync.take_match(5114, Duration::ZERO).map(|f| f.timestamp_ms), Some(5116));
    }

    #[test]
    fn waits_for_a_late_colour_frame() {
        let sync = Arc::new(ColorSync::default());
        sync.push(frame(100));
        let pusher = sync.clone();
        let handle = std::thread::spawn(move || {
            std::thread::sleep(Duration::from_millis(30));
            pusher.push(frame(201));
        });
        assert_eq!(sync.take_match(200, Duration::from_millis(500)).map(|f| f.timestamp_ms), Some(201));
        handle.join().unwrap();
    }
}
