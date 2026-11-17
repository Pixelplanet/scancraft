//! Records short raw segments of the Full-field HD *marker* stream (fmt 3 / display type 6,
//! `z16y8y8p`) with different IR fill-LED / exposure settings, for offline marker work.
//!
//! `cargo run -p scancraft-driver-revopoint --example marker_record -- <ip> <out_dir> [seconds_per_segment] [display_type]`
//!
//! Takes the scanner lock and reproduces Revo Scan's order for Full-field HD + Marker
//! (`sdk.log_back` 2026-07-05 18:42:27..36, see `docs/protocol/markers.md`): stream format first,
//! media stream open, *then* fill-light / exposure registers. One media connection stays open for
//! the whole session; packets go to one file per segment in the wire framing
//! (`[0x11223344][len][QuickLZ]`). Emitters off and streams closed at the end. Keep recordings
//! outside the repository.

use futures::StreamExt;
use scancraft_driver_revopoint::control::Control;
use scancraft_driver_revopoint::media::{PacketParser, PACKET_MAGIC};
use scancraft_driver_revopoint::scanner_lock::ScannerLock;
use std::io::Write;
use std::sync::{Arc, Mutex};
use std::time::Duration;

/// Revo's register writes after the stream started (18:42:35.405 .. 18:42:36.507).
const REVO_MARKER_SETUP: &[&str] = &[
    "0xb13 1", "0xb01 1", "0x910 7000", "0x911 3000", "0x903 0x10", "0xb07 34", "0xb03 0", "0xb05 0", "0x707 250 500", "0x705 10",
    "0xa04 0", "0xb04 1",
];

/// (segment name, registers applied before the segment).
const SEGMENTS: &[(&str, &[&str])] = &[
    ("base", &[]),
    ("x_701", &["0x701 1"]),
    ("x_b05_1", &["0xb05 1"]),
    ("x_b05_0", &["0xb05 0"]),
    ("x_b01_0", &["0xb01 0"]),
];

type Sink = Arc<Mutex<Option<(std::io::BufWriter<std::fs::File>, usize)>>>;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();
    let args: Vec<String> = std::env::args().collect();
    let ip = args.get(1).map(String::as_str).unwrap_or("192.168.178.34").to_string();
    let out_dir = std::path::PathBuf::from(args.get(2).expect("usage: marker_record <ip> <out_dir> [seconds] [display_type]"));
    let seconds: f32 = args.get(3).and_then(|s| s.parse().ok()).unwrap_or(4.0);
    let display_type: u32 = args.get(4).and_then(|s| s.parse().ok()).unwrap_or(6);
    std::fs::create_dir_all(&out_dir)?;

    let _lock = ScannerLock::acquire("marker-agent marker_record", Duration::from_secs(600))?;
    let ctl = Control::new(&ip)?;
    let result = record(&ctl, &ip, &out_dir, seconds, display_type).await;
    ctl.turn_off_emitters().await;
    result
}

async fn record(ctl: &Control, ip: &str, out_dir: &std::path::Path, seconds: f32, display_type: u32) -> Result<(), Box<dyn std::error::Error>> {
    ctl.cgi("close_stream_all").await?;
    ctl.set_stream_format(3, display_type, 640, 512).await?;
    ctl.cgi_ok("cam_type=mipi&set_trigger_mode=0").await?;

    let sink: Sink = Arc::new(Mutex::new(None));
    let reader = {
        let sink = sink.clone();
        let url = format!("http://{ip}/cgi-bin/zx_media.cgi?camera_id=21");
        tokio::spawn(async move {
            let client = reqwest::Client::builder().connect_timeout(Duration::from_secs(4)).build().unwrap();
            let resp = match client.get(url).send().await.and_then(|r| r.error_for_status()) {
                Ok(r) => r,
                Err(e) => {
                    eprintln!("media stream: {e}");
                    return;
                }
            };
            let mut body = resp.bytes_stream();
            let mut parser = PacketParser::default();
            while let Ok(Some(Ok(chunk))) = tokio::time::timeout(Duration::from_secs(8), body.next()).await {
                parser.push(&chunk);
                while let Some(packet) = parser.next_packet() {
                    if let Some((file, n)) = sink.lock().unwrap().as_mut() {
                        let _ = file.write_all(&PACKET_MAGIC);
                        let _ = file.write_all(&(packet.len() as u32).to_le_bytes());
                        let _ = file.write_all(&packet);
                        *n += 1;
                    }
                }
            }
        })
    };
    tokio::time::sleep(Duration::from_millis(1500)).await;
    for r in REVO_MARKER_SETUP {
        ctl.preisp(r).await?;
    }
    // Depth engine warm-up after configuration.
    tokio::time::sleep(Duration::from_millis(2500)).await;

    for (name, regs) in SEGMENTS {
        for r in regs.iter() {
            ctl.preisp(r).await?;
        }
        tokio::time::sleep(Duration::from_millis(700)).await;
        let path = out_dir.join(format!("{name}_t{display_type}.bin"));
        *sink.lock().unwrap() = Some((std::io::BufWriter::new(std::fs::File::create(&path)?), 0));
        tokio::time::sleep(Duration::from_secs_f32(seconds)).await;
        if let Some((mut file, n)) = sink.lock().unwrap().take() {
            file.flush()?;
            println!("{name}: {n} packets -> {}", path.display());
        }
    }
    reader.abort();
    Ok(())
}
