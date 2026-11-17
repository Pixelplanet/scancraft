//! Opens any device stream the way Revo's SDK does and reports what arrives, for exploring
//! formats the driver does not decode yet:
//! `close_stream_all`, display geometry, output format, media stream, `set_trigger_mode=0`, wait
//! for the first packet (up to 20 s), then the given `rk_preisp` writes (one per ~80 ms).
//!
//! `cargo run --release -p scancraft-driver-revopoint --example stream_probe -- <ip> <fmt> <type> <width> <height> <media_query> <seconds> <out_dir> ["reg;reg;..."]`
//! e.g. Revo's Cross Lines on firmware v5.3.99:
//! `stream_probe <ip> 7 4 1280 1024 "camera_id=33&type_id=22" 8 E:\Claude\eval\x "0x707 200 600;0x3001 14;..."`
//!
//! A write prefixed `@<seconds>:` (e.g. `@6:0x48d 255 1`) is sent that long after the first packet
//! instead of in the initial batch. `PLANE_H=<rows>` prints, for JPEG payloads, each plane's mean
//! and the share of pixels brighter than 100 (lit laser lines / dots), with planes of that height.
//!
//! Per packet: size, and for JPEG payloads the image size, for QuickLZ payloads the decompressed
//! size. Saves packets 1, 10, 30 and every 50th (`packet<N>.jpg` or `.raw`). Takes the scanner lock, ends the
//! stream with `close_stream_all` while still holding the connection, and switches emitters off.

use futures::StreamExt;
use scancraft_driver_revopoint::control::Control;
use scancraft_driver_revopoint::quicklz::QuickLzDecompressor;
use scancraft_driver_revopoint::PacketParser;
use std::time::{Duration, Instant};

fn jpeg_size(p: &[u8]) -> Option<(u16, u16, u8)> {
    if !p.starts_with(&[0xff, 0xd8]) {
        return None;
    }
    let mut i = 2;
    while i + 9 < p.len() {
        if p[i] != 0xff {
            return None;
        }
        let (marker, len) = (p[i + 1], u16::from_be_bytes([p[i + 2], p[i + 3]]) as usize);
        if matches!(marker, 0xc0 | 0xc1 | 0xc2) {
            return Some((u16::from_be_bytes([p[i + 7], p[i + 8]]), u16::from_be_bytes([p[i + 5], p[i + 6]]), p[i + 9]));
        }
        i += 2 + len;
    }
    None
}

#[tokio::main]
async fn main() {
    let a: Vec<String> = std::env::args().collect();
    if a.len() < 9 {
        eprintln!("usage: stream_probe <ip> <fmt> <type> <width> <height> <media_query> <seconds> <out_dir> [\"reg;reg\"]");
        std::process::exit(1);
    }
    let (ip, fmt, ty, w, h) = (&a[1], a[2].parse::<u32>().unwrap(), a[3].parse::<u32>().unwrap(), a[4].parse::<u32>().unwrap(), a[5].parse::<u32>().unwrap());
    let (query, seconds, out) = (&a[6], a[7].parse::<f32>().unwrap(), std::path::PathBuf::from(&a[8]));
    let setup: Vec<String> = a.get(9).map(|s| s.split(';').map(|c| c.trim().to_string()).filter(|c| !c.is_empty()).collect()).unwrap_or_default();
    std::fs::create_dir_all(&out).unwrap();
    let _lock = scancraft_driver_revopoint::scanner_lock::ScannerLock::try_acquire("stream_probe").expect("scanner lock");
    let ctl = Control::new(ip).unwrap();

    ctl.cgi("close_stream_all").await.ok();
    if let Err(e) = ctl.set_stream_format(fmt, ty, w, h).await {
        println!("format: {e}");
    }
    println!("reso: {}", ctl.cgi("cam_type=mipi&get_display_reso").await.unwrap_or_default().trim());
    let url = format!("http://{ip}/cgi-bin/zx_media.cgi?{query}");
    let resp = reqwest::Client::new().get(&url).send().await.expect("media stream");
    println!("media: HTTP {}", resp.status());
    let mut body = resp.bytes_stream();
    let start = Instant::now();
    ctl.cgi_ok("cam_type=mipi&set_trigger_mode=0").await.ok();

    let mut parser = PacketParser::default();
    let mut qlz = QuickLzDecompressor::new();
    let mut scratch = vec![0u8; 64 << 20];
    let (mut n, mut bytes, mut setup_task) = (0usize, 0usize, None);
    let plane_h: Option<usize> = std::env::var("PLANE_H").ok().and_then(|v| v.parse().ok());
    let mut first_at = None;
    while start.elapsed().as_secs_f32() < seconds + first_at.unwrap_or(20.0) {
        let Ok(Some(Ok(chunk))) = tokio::time::timeout(Duration::from_secs(if n == 0 { 20 } else { 8 }), body.next()).await else {
            println!("t={:5.1}s no data (after {n} packets)", start.elapsed().as_secs_f32());
            break;
        };
        parser.push(&chunk);
        while let Some(p) = parser.next_packet() {
            n += 1;
            bytes += p.len();
            let t = start.elapsed().as_secs_f32();
            let kind = match jpeg_size(&p) {
                Some((jw, jh, c)) => format!("JPEG {jw}x{jh}, {c} components"),
                None => match qlz.decompress(&p, &mut scratch) {
                    Ok(len) => {
                        if plane_h.is_some() && n % 10 == 0 {
                            // One entry per w*h bytes: mean / share of non-zero bytes.
                            let stats: Vec<String> = scratch[..len]
                                .chunks((w * h) as usize)
                                .map(|pl| {
                                    let mean = pl.iter().map(|&v| v as f64).sum::<f64>() / pl.len() as f64;
                                    let nz = pl.iter().filter(|&&v| v != 0).count() as f64 / pl.len() as f64;
                                    format!("{mean:5.1}/{:.0}%", nz * 100.0)
                                })
                                .collect();
                            println!("t={t:5.1}s packet {n:3} byte planes mean/non-zero: {}", stats.join("  "));
                        }
                        format!("QuickLZ -> {len} bytes ({:.2} x {w}x{h})", len as f32 / (w * h) as f32)
                    }
                    Err(e) => format!("unknown ({e}), starts {:02x?}", &p[..p.len().min(8)]),
                },
            };
            if n <= 4 || n % 20 == 0 {
                println!("t={t:5.1}s packet {n:3}: {:7} bytes, {kind}", p.len());
            }
            if let (Some(rows), true) = (plane_h, n % 10 == 0) {
                if let Ok((jw, jh, luma)) = scancraft_driver_revopoint::media::decode_jpeg_luma(&p) {
                    let stats: Vec<String> = luma
                        .chunks(jw * rows)
                        .take(jh / rows)
                        .map(|pl| {
                            let mean = pl.iter().map(|&v| v as f64).sum::<f64>() / pl.len() as f64;
                            let lit = pl.iter().filter(|&&v| v > 100).count() as f64 / pl.len() as f64;
                            format!("{mean:5.1}/{:.2}%", lit * 100.0)
                        })
                        .collect();
                    println!("t={t:5.1}s packet {n:3} planes mean/lit: {}", stats.join("  "));
                }
            }
            if matches!(n, 1 | 10 | 30) || n % 50 == 0 {
                let ext = if p.starts_with(&[0xff, 0xd8]) { "jpg" } else { "raw" };
                std::fs::write(out.join(format!("packet{n}.{ext}")), &p).ok();
            }
            if n == 1 {
                first_at = Some(t);
                let (ip, setup) = (ip.clone(), setup.clone());
                setup_task = Some(tokio::spawn(async move {
                    let ctl = Control::new(&ip).unwrap();
                    let begun = Instant::now();
                    for reg in setup.iter().filter(|r| !r.starts_with('@')) {
                        if let Err(e) = ctl.preisp(reg).await {
                            println!("  {reg}: {e}");
                        }
                        tokio::time::sleep(Duration::from_millis(80)).await;
                    }
                    println!("  setup sent after {:.1} s", begun.elapsed().as_secs_f32());
                    let mut timed: Vec<(f32, &str)> = setup.iter().filter_map(|r| r.strip_prefix('@')?.split_once(':')).filter_map(|(t, r)| Some((t.parse().ok()?, r))).collect();
                    timed.sort_by(|a, b| a.0.total_cmp(&b.0));
                    for (at, reg) in timed {
                        tokio::time::sleep(Duration::from_secs_f32(at).saturating_sub(begun.elapsed())).await;
                        println!("  +{:.1}s write {reg}: {:?}", begun.elapsed().as_secs_f32(), ctl.preisp(reg).await.err());
                    }
                }));
            }
        }
    }
    if let Some(t) = setup_task {
        let _ = t.await;
    }
    let span = start.elapsed().as_secs_f32() - first_at.unwrap_or(0.0);
    println!("{n} packets, {:.1} MB, {:.1} fps after the first packet", bytes as f32 / 1e6, if n > 1 { (n - 1) as f32 / span } else { 0.0 });
    ctl.cgi("close_stream_all").await.ok();
    drop(body);
    ctl.turn_off_emitters().await;
}
