//! Decoding of the `zx_media.cgi?camera_id=21` sensor stream.
//!
//! Wire format (see `docs/protocol/captures/2026-11-01-media-stream-layouts.md`): an endless
//! chunked HTTP body made of packets `[u32 LE 0x11223344][u32 LE len][len bytes QuickLZ]`.
//! Each packet decompresses to one frame whose planes depend on the configured output format.
//! The first bytes of the first plane are overwritten by an in-band header (`abcd` + IMU).
//!
//! Exception: the Hybrid HD stream (`S64Y8Y8`, `camera_id=33&type_id=21`) uses the same packet
//! framing but each payload is a baseline JPEG of all ten planes stacked vertically, followed by
//! the `abcd` header after the JPEG end marker (`docs/protocol/hybrid.md`).

use crate::quicklz::QuickLzDecompressor;
use glam::{Quat, Vec3};
use scancraft_core::{DepthMap, ImuSample, RgbImage};

pub const PACKET_MAGIC: [u8; 4] = [0x44, 0x33, 0x22, 0x11];
/// Largest packet we accept; 640x512x4 compresses to < 1 MB, larger means a false magic match.
const MAX_PACKET_LEN: usize = 4 * 1024 * 1024;
/// Bytes of in-band header at the start of every decompressed frame (81 observed, rounded up).
pub const FRAME_HEADER_LEN: usize = 128;
/// Raw depth unit of the POP 4 z16 plane.
pub const DEPTH_SCALE_MM: f32 = 0.01;

/// Splits the media stream body into compressed packets. Feed it bytes as they arrive.
#[derive(Default)]
pub struct PacketParser {
    buf: Vec<u8>,
}

impl PacketParser {
    pub fn push(&mut self, bytes: &[u8]) {
        self.buf.extend_from_slice(bytes);
    }

    /// Returns the next complete packet payload, or `None` until more bytes arrive.
    pub fn next_packet(&mut self) -> Option<Vec<u8>> {
        loop {
            let Some(pos) = self.buf.windows(4).position(|w| w == PACKET_MAGIC) else {
                // Keep a possible partial magic at the tail.
                let keep = self.buf.len().min(3);
                self.buf.drain(..self.buf.len() - keep);
                return None;
            };
            self.buf.drain(..pos);
            if self.buf.len() < 8 {
                return None;
            }
            let len = u32::from_le_bytes(self.buf[4..8].try_into().unwrap()) as usize;
            // A real packet's QuickLZ header repeats its own length, a JPEG packet starts with
            // SOI; anything else is a false match.
            let plausible = (9..=MAX_PACKET_LEN).contains(&len)
                && (self.buf.len() < 17
                    || qlz_compressed_len(&self.buf[8..17]) == Some(len)
                    || self.buf[8..11] == [0xff, 0xd8, 0xff]);
            if !plausible {
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

const JPEG_SOI: [u8; 3] = [0xff, 0xd8, 0xff];

fn qlz_compressed_len(header: &[u8]) -> Option<usize> {
    (header[0] & 2 != 0).then(|| u32::from_le_bytes(header[1..5].try_into().unwrap()) as usize)
}

/// Plane layout of a decompressed frame, selected by `set_depth_output_fmt` + `set_display_type`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameLayout {
    /// fmt 1, type 2: left IR y8 then right IR y8.
    IrPair,
    /// fmt 3, type 4: on-device depth z16, then left IR y8, then right IR y8.
    DepthIrPair,
    /// fmt 3, type 6 (`z16y8y8p`, Revo's Full-field HD marker stream): as `DepthIrPair`, followed
    /// by a 2 B/px plane that 3DCamera.dll names "leftOriIR"/"rightOriIR". It was never refreshed
    /// in our sessions (stale content), so it is not decoded; see `docs/protocol/markers.md`.
    DepthIrPairOriIr,
    /// fmt 1, type 2 on `camera_id=33&type_id=20` (Revo's "CPAIR", SDK format 266): one baseline
    /// JPEG per packet holding left IR (top half) over right IR (bottom half), followed by the
    /// in-band header after the JPEG EOI marker.
    JpegIrPair,
    /// fmt 5, type 10 (`S64Y8Y8`, Hybrid HD), JPEG-coded: left stripe images 0..3 (rectified),
    /// left speckle image (raw), right stripe images 0..3 (rectified), right speckle (raw), each
    /// y8 (`docs/protocol/hybrid.md`). Decoded into `ir_left`/`ir_right` = speckle pair and
    /// `extra_planes` = `[stripe L0..L3, stripe R0..R3]`.
    StripeSpeckle,
    /// fmt 1, type 2 selected at 640x1024 (VCSEL Rapid): two raw 640x1024 camera images, each
    /// holding two exposures, an ambient and a VCSEL-lit one, i.e. four 640x512 y8 slots: left
    /// camera's two, then the right camera's two. `width`/`height` are one slot. Decoded into
    /// `ir_left`/`ir_right` = the lit slots (raw, unrectified; chosen by brightness) and
    /// `extra_planes` = `[left ambient, right ambient]`. QuickLZ-coded on `camera_id=21`,
    /// JPEG-coded (640x2048) on `camera_id=33&type_id=20`. At 640x512 the same request returns
    /// only the left camera's two slots, which is why it never gave a stereo pair.
    IrSlotPairs,
    /// fmt 7, type 4 at 1280x1024 on `camera_id=33&type_id=22` (Cross Lines + dots, firmware
    /// v5.3.99; the device calls it `depth+sp+ls`): one JPEG of four raw 1280x1024 planes, left
    /// camera with the dot pattern, left camera with the laser lines, then the same for the right
    /// camera. `width`/`height` are one plane. Decoded into `ir_left`/`ir_right` = the line
    /// planes (full size) and `extra_planes` = `[left dots, right dots]` at half size.
    DotsAndLines,
}

impl FrameLayout {
    pub fn bytes_per_pixel(self) -> usize {
        match self {
            FrameLayout::IrPair | FrameLayout::JpegIrPair => 2,
            FrameLayout::DepthIrPair | FrameLayout::IrSlotPairs | FrameLayout::DotsAndLines => 4,
            FrameLayout::DepthIrPairOriIr => 6,
            FrameLayout::StripeSpeckle => 10,
        }
    }

    /// Rows to request with `set_display_height` for slots of `height` rows.
    pub fn display_rows(self, height: u32) -> u32 {
        match self {
            FrameLayout::IrSlotPairs => 2 * height,
            _ => height,
        }
    }
}

#[derive(Debug, Clone)]
pub struct DecodedFrame {
    /// Device timestamp counter from the in-band header.
    pub device_counter: u32,
    pub depth: Option<DepthMap>,
    pub ir_left: RgbImage,
    pub ir_right: RgbImage,
    pub imu: Option<ImuSample>,
    /// Further 8-bit planes of layouts with more than an IR pair (see [`FrameLayout`]).
    pub extra_planes: Vec<RgbImage>,
}

/// Decompresses packets and splits them into planes.
pub struct FrameDecoder {
    layout: FrameLayout,
    width: usize,
    height: usize,
    qlz: QuickLzDecompressor,
    scratch: Vec<u8>,
}

impl FrameDecoder {
    pub fn new(layout: FrameLayout, width: u32, height: u32) -> Self {
        let (width, height) = (width as usize, height as usize);
        Self {
            layout,
            width,
            height,
            qlz: QuickLzDecompressor::new(),
            scratch: vec![0; width * height * layout.bytes_per_pixel()],
        }
    }

    pub fn decode(&mut self, packet: &[u8]) -> Result<DecodedFrame, String> {
        if self.layout == FrameLayout::JpegIrPair {
            return decode_jpeg_pair(packet, self.width, self.height);
        }
        if self.layout == FrameLayout::StripeSpeckle {
            return decode_stripe_speckle(packet, self.width, self.height);
        }
        if self.layout == FrameLayout::DotsAndLines {
            return decode_dots_and_lines(packet, self.width, self.height);
        }
        // The slot pairs come QuickLZ-coded on `camera_id=21` and JPEG-coded on `camera_id=33`.
        if self.layout == FrameLayout::IrSlotPairs && packet.starts_with(&[0xff, 0xd8]) {
            return decode_jpeg_slot_pairs(packet, self.width, self.height);
        }
        let n = self.qlz.decompress(packet, &mut self.scratch).map_err(str::to_string)?;
        if n != self.scratch.len() {
            return Err(format!(
                "frame is {n} bytes, expected {} for {:?} at {}x{}",
                self.scratch.len(),
                self.layout,
                self.width,
                self.height
            ));
        }
        decode_planes(self.layout, self.width, self.height, &self.scratch)
    }
}

/// Luma plane of a JPEG-coded packet (the in-band header after the end marker is ignored):
/// `(width, height, pixels)`.
pub fn decode_jpeg_luma(packet: &[u8]) -> Result<(usize, usize, Vec<u8>), String> {
    use zune_core::bytestream::ZCursor;
    use zune_core::colorspace::ColorSpace;
    use zune_core::options::DecoderOptions;

    if packet.len() < 4 || packet[..2] != [0xff, 0xd8] {
        return Err("packet is not a JPEG".into());
    }
    let eoi = packet.windows(2).rposition(|w| w == [0xff, 0xd9]).ok_or("JPEG without EOI")?;
    let options = DecoderOptions::default().jpeg_set_out_colorspace(ColorSpace::Luma);
    let mut decoder = zune_jpeg::JpegDecoder::new_with_options(ZCursor::new(&packet[..eoi + 2]), options);
    let luma = decoder.decode().map_err(|e| format!("JPEG: {e:?}"))?;
    let info = decoder.info().ok_or("JPEG without header")?;
    Ok((info.width as usize, info.height as usize, luma))
}

/// Decodes a `JpegIrPair` packet: luma of a `width` x `2*height` JPEG, left image on top.
pub fn decode_jpeg_pair(packet: &[u8], width: usize, height: usize) -> Result<DecodedFrame, String> {
    use zune_core::bytestream::ZCursor;
    use zune_core::colorspace::ColorSpace;
    use zune_core::options::DecoderOptions;

    if packet.len() < 4 || packet[..2] != [0xff, 0xd8] {
        return Err("packet is not a JPEG".into());
    }
    let eoi = packet.windows(2).rposition(|w| w == [0xff, 0xd9]).ok_or("JPEG without EOI")?;
    let options = DecoderOptions::default().jpeg_set_out_colorspace(ColorSpace::Luma);
    let mut decoder = zune_jpeg::JpegDecoder::new_with_options(ZCursor::new(&packet[..eoi + 2]), options);
    let luma = decoder.decode().map_err(|e| format!("JPEG: {e:?}"))?;
    let info = decoder.info().ok_or("JPEG without header")?;
    if (info.width as usize, info.height as usize) != (width, 2 * height) || luma.len() != 2 * width * height {
        return Err(format!(
            "JPEG pair is {}x{} ({} bytes), expected {width}x{}",
            info.width,
            info.height,
            luma.len(),
            2 * height
        ));
    }
    let header = packet.get(eoi + 2..).filter(|t| t.len() >= 80).and_then(parse_header);
    let px = width * height;
    let gray = |bytes: &[u8]| RgbImage { width: width as u32, height: height as u32, channels: 1, data: bytes.to_vec() };
    Ok(DecodedFrame {
        device_counter: header.map(|h| h.0).unwrap_or(0),
        depth: None,
        ir_left: gray(&luma[..px]),
        ir_right: gray(&luma[px..]),
        extra_planes: Vec::new(),
        imu: header.and_then(|h| h.1),
    })
}

/// Four `width` x `height` slots (left camera's two exposures, then the right camera's) into a
/// frame: per camera the brighter exposure is the VCSEL-lit one. Which of the two that is depends
/// on the device state (it flipped once the VCSEL setup was applied), so it is not assumed.
fn slot_pairs(slots: &[u8], width: usize, height: usize, header: Option<(u32, Option<ImuSample>)>) -> DecodedFrame {
    let px = width * height;
    let gray = |i: usize| RgbImage { width: width as u32, height: height as u32, channels: 1, data: slots[i * px..(i + 1) * px].to_vec() };
    // Every 16th pixel is plenty to tell a lit exposure from an ambient one.
    let level = |i: usize| slots[i * px..(i + 1) * px].iter().step_by(16).map(|&v| v as u32).sum::<u32>();
    let (left_lit, left_ambient) = if level(1) >= level(0) { (1, 0) } else { (0, 1) };
    let (right_lit, right_ambient) = if level(3) >= level(2) { (3, 2) } else { (2, 3) };
    DecodedFrame {
        device_counter: header.map(|h| h.0).unwrap_or(0),
        depth: None,
        ir_left: gray(left_lit),
        ir_right: gray(right_lit),
        imu: header.and_then(|h| h.1),
        extra_planes: vec![gray(left_ambient), gray(right_ambient)],
    }
}

/// Decodes a JPEG-coded `IrSlotPairs` packet (`camera_id=33&type_id=20` at twice the slot height):
/// luma of a `width` x `4*height` JPEG.
pub fn decode_jpeg_slot_pairs(packet: &[u8], width: usize, height: usize) -> Result<DecodedFrame, String> {
    let (w, h, luma) = decode_jpeg_luma(packet)?;
    if (w, h) != (width, 4 * height) || luma.len() != 4 * width * height {
        return Err(format!("JPEG slot pairs are {w}x{h}, expected {width}x{}", 4 * height));
    }
    let eoi = packet.windows(2).rposition(|w| w == [0xff, 0xd9]).ok_or("JPEG without EOI")?;
    let header = packet.get(eoi + 2..).filter(|t| t.len() >= 80).and_then(parse_header);
    Ok(slot_pairs(&luma, width, height, header))
}

/// Decodes a `DotsAndLines` packet: luma of a `width` x `4*height` JPEG.
pub fn decode_dots_and_lines(packet: &[u8], width: usize, height: usize) -> Result<DecodedFrame, String> {
    let (w, h, luma) = decode_jpeg_luma(packet)?;
    if (w, h) != (width, 4 * height) || luma.len() != 4 * width * height {
        return Err(format!("dots + lines JPEG is {w}x{h}, expected {width}x{}", 4 * height));
    }
    let eoi = packet.windows(2).rposition(|w| w == [0xff, 0xd9]).ok_or("JPEG without EOI")?;
    let header = packet.get(eoi + 2..).filter(|t| t.len() >= 80).and_then(parse_header);
    let px = width * height;
    let full = |i: usize| RgbImage { width: width as u32, height: height as u32, channels: 1, data: luma[i * px..(i + 1) * px].to_vec() };
    // The dots feed a 640-wide stereo matcher: average 2x2 blocks.
    let half = |i: usize| {
        let plane = &luma[i * px..(i + 1) * px];
        let (hw, hh) = (width / 2, height / 2);
        let mut data = Vec::with_capacity(hw * hh);
        for y in 0..hh {
            let (a, b) = (&plane[2 * y * width..], &plane[(2 * y + 1) * width..]);
            for x in 0..hw {
                data.push(((a[2 * x] as u16 + a[2 * x + 1] as u16 + b[2 * x] as u16 + b[2 * x + 1] as u16 + 2) / 4) as u8);
            }
        }
        RgbImage { width: hw as u32, height: hh as u32, channels: 1, data }
    };
    Ok(DecodedFrame {
        device_counter: header.map(|h| h.0).unwrap_or(0),
        depth: None,
        ir_left: full(1),
        ir_right: full(3),
        imu: header.and_then(|h| h.1),
        extra_planes: vec![half(0), half(2)],
    })
}

/// Splits an already-decompressed frame into planes.
pub fn decode_planes(layout: FrameLayout, width: usize, height: usize, frame: &[u8]) -> Result<DecodedFrame, String> {
    let px = width * height;
    if frame.len() != px * layout.bytes_per_pixel() || frame.len() < FRAME_HEADER_LEN {
        return Err(format!("frame is {} bytes, expected {}", frame.len(), px * layout.bytes_per_pixel()));
    }
    let header = parse_header(frame);
    let gray = |bytes: &[u8]| RgbImage {
        width: width as u32,
        height: height as u32,
        channels: 1,
        data: bytes.to_vec(),
    };

    let (depth, ir_left, ir_right) = match layout {
        FrameLayout::IrPair | FrameLayout::JpegIrPair => {
            let mut left = gray(&frame[..px]);
            left.data[..FRAME_HEADER_LEN].fill(0);
            (None, left, gray(&frame[px..2 * px]))
        }
        FrameLayout::DepthIrPair | FrameLayout::DepthIrPairOriIr => {
            let mut z: Vec<u16> =
                frame[..2 * px].as_chunks::<2>().0.iter().map(|c| u16::from_le_bytes(*c)).collect();
            z[..FRAME_HEADER_LEN / 2].fill(0);
            let depth = DepthMap::new(width as u32, height as u32, DEPTH_SCALE_MM, z);
            (Some(depth), gray(&frame[2 * px..3 * px]), gray(&frame[3 * px..4 * px]))
        }
        FrameLayout::StripeSpeckle | FrameLayout::DotsAndLines => return Err("this stream is JPEG-coded; use FrameDecoder".into()),
        FrameLayout::IrSlotPairs => {
            let mut slots = frame.to_vec();
            slots[..FRAME_HEADER_LEN].fill(0);
            return Ok(slot_pairs(&slots, width, height, header));
        }
    };

    Ok(DecodedFrame {
        device_counter: header.map(|h| h.0).unwrap_or(0),
        depth,
        ir_left,
        ir_right,
        imu: header.and_then(|h| h.1),
        extra_planes: Vec::new(),
    })
}

/// Hybrid HD packet: a JPEG (Y only, as YCbCr with neutral chroma) of the ten planes stacked
/// vertically (`width` x `10 * height`), followed by the `abcd` header after the end marker.
pub fn decode_stripe_speckle(packet: &[u8], width: usize, height: usize) -> Result<DecodedFrame, String> {
    if packet.len() < 4 || packet[..3] != JPEG_SOI {
        return Err("Hybrid HD packet is not a JPEG".into());
    }
    // The trailer holds binary IMU floats that may contain FF D9 themselves, so look for the
    // end marker followed by the header magic first.
    let eoi = packet
        .windows(6)
        .rposition(|w| w == [0xff, 0xd9, b'a', b'b', b'c', b'd'])
        .or_else(|| packet.windows(2).rposition(|w| w == [0xff, 0xd9]))
        .ok_or("JPEG end marker missing")?;
    let luma = image::load_from_memory_with_format(&packet[..eoi + 2], image::ImageFormat::Jpeg)
        .map_err(|e| format!("JPEG: {e}"))?
        .into_luma8();
    if luma.width() as usize != width || luma.height() as usize != 10 * height {
        return Err(format!("JPEG is {}x{}, expected {width}x{}", luma.width(), luma.height(), 10 * height));
    }
    let data = luma.into_raw();
    let px = width * height;
    let plane = |i: usize| RgbImage { width: width as u32, height: height as u32, channels: 1, data: data[i * px..(i + 1) * px].to_vec() };
    let trailer = &packet[eoi + 2..];
    let header = (trailer.len() >= 76).then(|| parse_header(trailer)).flatten();
    Ok(DecodedFrame {
        device_counter: header.map(|h| h.0).unwrap_or(0),
        depth: None,
        ir_left: plane(4),
        ir_right: plane(9),
        imu: header.and_then(|h| h.1),
        extra_planes: [0, 1, 2, 3, 5, 6, 7, 8].into_iter().map(plane).collect(),
    })
}

/// In-band header: `abcd`, 16 bytes of flags, u32 counter @20, then f32 quaternion (w,x,y,z) @24,
/// gyro @40, accelerometer in g @52, Euler angles @64. The IMU interpretation is consistent with
/// live data (unit quaternion, |accel| ~ 1 g) but not yet confirmed against 3DCamera.dll.
fn parse_header(frame: &[u8]) -> Option<(u32, Option<ImuSample>)> {
    if &frame[..4] != b"abcd" {
        return None;
    }
    let f = |o: usize| f32::from_le_bytes(frame[o..o + 4].try_into().unwrap());
    let v3 = |o: usize| Vec3::new(f(o), f(o + 4), f(o + 8));
    let counter = u32::from_le_bytes(frame[20..24].try_into().unwrap());

    let q = Quat::from_xyzw(f(28), f(32), f(36), f(24));
    let accel = v3(52);
    let imu = (q.is_finite() && (q.length() - 1.0).abs() < 0.05 && accel.is_finite()).then(|| ImuSample {
        timestamp_ms: counter as u64,
        orientation: q.normalize(),
        angular_velocity: v3(40),
        linear_acceleration: accel,
        euler_degrees: v3(64),
    });
    Some((counter, imu))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn header() -> Vec<u8> {
        let mut h = vec![0u8; FRAME_HEADER_LEN];
        h[..4].copy_from_slice(b"abcd");
        h[20..24].copy_from_slice(&336094u32.to_le_bytes());
        for (i, v) in [0.7698f32, -0.6097, -0.1772, 0.0308, 0.0, 0.0, 0.0, 0.236, -0.9608, 0.1881]
            .iter()
            .enumerate()
        {
            h[24 + 4 * i..28 + 4 * i].copy_from_slice(&v.to_le_bytes());
        }
        h
    }

    #[test]
    fn splits_depth_and_ir_planes() {
        let (w, h) = (64usize, 8usize);
        let px = w * h;
        let mut frame = vec![0u8; px * 4];
        for i in 0..px {
            frame[2 * i..2 * i + 2].copy_from_slice(&25000u16.to_le_bytes());
            frame[2 * px + i] = 10;
            frame[3 * px + i] = 20;
        }
        frame[..FRAME_HEADER_LEN].copy_from_slice(&header());

        let f = decode_planes(FrameLayout::DepthIrPair, w, h, &frame).unwrap();
        let depth = f.depth.unwrap();
        assert_eq!(depth.valid_pixel_count(), px - FRAME_HEADER_LEN / 2, "header pixels are blanked");
        assert_eq!(depth.data[px - 1], 25000);
        assert!(f.ir_left.data.iter().all(|&b| b == 10));
        assert!(f.ir_right.data.iter().all(|&b| b == 20));
        assert_eq!(f.device_counter, 336094);
        let imu = f.imu.expect("header carries IMU");
        assert!((imu.linear_acceleration.length() - 1.0).abs() < 0.02);
    }

    #[test]
    fn ir_pair_has_no_depth() {
        let (w, h) = (64usize, 8usize);
        let mut frame = vec![7u8; w * h * 2];
        frame[..FRAME_HEADER_LEN].copy_from_slice(&header());
        let f = decode_planes(FrameLayout::IrPair, w, h, &frame).unwrap();
        assert!(f.depth.is_none());
        assert_eq!(f.ir_left.data[FRAME_HEADER_LEN], 7);
        assert_eq!(f.ir_left.data[0], 0);
        assert!(decode_planes(FrameLayout::DepthIrPair, w, h, &frame).is_err(), "size mismatch is an error");
    }

    fn packet(payload_len: usize, fill: u8) -> Vec<u8> {
        let mut qlz = vec![fill; payload_len];
        qlz[0] = 0x47;
        qlz[1..5].copy_from_slice(&(payload_len as u32).to_le_bytes());
        let mut p = PACKET_MAGIC.to_vec();
        p.extend_from_slice(&(payload_len as u32).to_le_bytes());
        p.extend_from_slice(&qlz);
        p
    }

    #[test]
    fn parser_handles_split_reads_and_false_magic() {
        let mut stream = b"\r\n".to_vec();
        stream.extend_from_slice(&packet(300, 1));
        // A magic inside garbage with an implausible length must be skipped.
        stream.extend_from_slice(&[0x44, 0x33, 0x22, 0x11, 0xff, 0xff, 0xff, 0x7f, 0, 0]);
        stream.extend_from_slice(&packet(500, 2));

        let mut parser = PacketParser::default();
        let mut out = Vec::new();
        for chunk in stream.chunks(37) {
            parser.push(chunk);
            while let Some(p) = parser.next_packet() {
                out.push(p);
            }
        }
        assert_eq!(out.len(), 2);
        assert_eq!((out[0].len(), out[0][10]), (300, 1));
        assert_eq!((out[1].len(), out[1][10]), (500, 2));
    }
}

#[cfg(test)]
mod hybrid_tests {
    use super::*;

    /// Builds a Hybrid HD packet payload: ten planes with distinct grey levels, JPEG-coded, plus
    /// the header trailer.
    fn hybrid_payload(w: u32, h: u32) -> Vec<u8> {
        let mut img = image::GrayImage::new(w, 10 * h);
        for (_, y, p) in img.enumerate_pixels_mut() {
            p.0 = [20 + 20 * (y / h) as u8];
        }
        let mut jpeg = Vec::new();
        image::codecs::jpeg::JpegEncoder::new_with_quality(&mut jpeg, 95).encode_image(&img).unwrap();
        let mut header = vec![0u8; 102];
        header[..4].copy_from_slice(b"abcd");
        header[20..24].copy_from_slice(&4242u32.to_le_bytes());
        jpeg.extend_from_slice(&header);
        jpeg
    }

    #[test]
    fn decodes_jpeg_planes_and_trailer() {
        let payload = hybrid_payload(64, 16);
        let f = FrameDecoder::new(FrameLayout::StripeSpeckle, 64, 16).decode(&payload).unwrap();
        assert_eq!(f.device_counter, 4242);
        let level = |img: &RgbImage| img.data.iter().map(|&v| v as u32).sum::<u32>() / img.data.len() as u32;
        assert_eq!(f.extra_planes.len(), 8);
        let stripes: Vec<u32> = f.extra_planes.iter().map(level).collect();
        for (got, want) in stripes.iter().zip([20, 40, 60, 80, 120, 140, 160, 180]) {
            assert!(got.abs_diff(want) <= 2, "{stripes:?}");
        }
        assert!(level(&f.ir_left).abs_diff(100) <= 2);
        assert!(level(&f.ir_right).abs_diff(200) <= 2);
        assert!(FrameDecoder::new(FrameLayout::StripeSpeckle, 64, 32).decode(&payload).is_err(), "size mismatch");
    }

    #[test]
    fn parser_accepts_jpeg_packets() {
        let payload = hybrid_payload(64, 16);
        let mut stream = PACKET_MAGIC.to_vec();
        stream.extend_from_slice(&(payload.len() as u32).to_le_bytes());
        stream.extend_from_slice(&payload);
        let mut parser = PacketParser::default();
        parser.push(&stream);
        assert_eq!(parser.next_packet().map(|p| p.len()), Some(payload.len()));
    }
}

#[cfg(test)]
mod fixture_tests {
    use super::*;

    /// A real compressed POP 4 IR-pair frame (fmt 1 / type 2) captured from the device.
    #[test]
    fn decodes_live_ir_pair_fixture() {
        let packet = std::fs::read(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/live_pop4_frame.bin")).unwrap();
        let mut decoder = FrameDecoder::new(FrameLayout::IrPair, 640, 512);
        let frame = decoder.decode(&packet).expect("decodes");
        assert!(frame.depth.is_none());
        assert_eq!(frame.ir_left.data.len(), 640 * 512);
        assert_eq!(frame.ir_right.data.len(), 640 * 512);
        assert!(frame.ir_left.data[FRAME_HEADER_LEN..].iter().any(|&b| b > 0));

        let mut wrong = FrameDecoder::new(FrameLayout::DepthIrPair, 640, 512);
        assert!(wrong.decode(&packet).is_err(), "layout mismatch is reported, not guessed");
    }
}
