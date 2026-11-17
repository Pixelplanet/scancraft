//! Replays Revo Scan 6's two VCSEL sessions from the 2026-11-01 capture as literally as possible
//! (docs/protocol/vcsel.md §5): session 1 configures the VCSEL with the stream running, the
//! session ends without switching emitters off, Revo's connect writes follow, and session 2
//! opens the stream before `set_trigger_mode=0`. Prints the IR means of both halves over time.
//! Run after a Full-field HD session (which arms the VCSEL):
//! `cargo run --release -p scancraft-driver-revopoint --example vcsel_revo_replay -- <ip> [seconds_per_session]`
//!
//! Takes the scanner lock; ends every stream with `close_stream_all` and leaves the emitters off.

use futures::StreamExt;
use scancraft_driver_revopoint::control::Control;
use scancraft_driver_revopoint::{FrameDecoder, FrameLayout, PacketParser};
use std::time::{Duration, Instant};

const SESSION_SETUP: &[&str] = &[
    "cam_type=mipi&set_trigger_mode=0",
    "@0x707 200 600",
    "@0x3001 12",
    "@0x48d 80",
    "@0x488 1",
    "@0xb11 11000",
    "@0x911 666",
    "@0x903 0x20",
    "@0xb08 1366",
    "@0x910 50000",
    "@0x911 666",
    "@0x48d 100",
    "@0xb08 1366",
    "@0xb04 1",
    "@0xb03 0",
    "@0xb04 1",
];

async fn send(ctl: &Control, cmd: &str) {
    let r = match cmd.strip_prefix('@') {
        Some(reg) => ctl.preisp(reg).await,
        None => ctl.cgi_ok(cmd).await,
    };
    if let Err(e) = r {
        println!("  {cmd}: {e}");
    }
}

/// Opens the VCSEL media stream, runs `during` (commands sent after the stream opened, with a
/// leading delay in ms), reports half means every 0.5 s for `secs`, then ends the stream with
/// `close_stream_all`.
async fn session(ctl: &Control, ip: &str, label: &str, during: &[(u64, &str)], secs: f32) {
    println!("== {label}");
    send(ctl, "close_stream_all").await;
    send(ctl, "cam_type=mipi&set_display_reso=1&&set_display_width=640&&set_display_height=512&&set_display_type=2").await;
    send(ctl, "cam_type=mipi&set_depth_output_fmt=1").await;
    let _ = ctl.wait_for_output_fmt(1).await;
    let url = format!("http://{ip}/cgi-bin/zx_media.cgi?camera_id=33&type_id=20");
    let client = reqwest::Client::new();
    let resp = client.get(&url).send().await.expect("media stream");
    let start = Instant::now();
    let reader = tokio::spawn(async move {
        let mut body = resp.bytes_stream();
        let mut parser = PacketParser::default();
        let mut decoder = FrameDecoder::new(FrameLayout::JpegIrPair, 640, 512);
        let (mut n, mut acc_l, mut acc_r, mut k, mut next) = (0usize, 0.0f64, 0.0f64, 0usize, 0.5f32);
        while let Ok(Some(Ok(chunk))) = tokio::time::timeout(Duration::from_secs(4), body.next()).await {
            parser.push(&chunk);
            while let Some(packet) = parser.next_packet() {
                let Ok(f) = decoder.decode(&packet) else { continue };
                let mean = |d: &[u8]| d.iter().map(|&v| v as f64).sum::<f64>() / d.len().max(1) as f64;
                acc_l += mean(&f.ir_left.data);
                acc_r += mean(&f.ir_right.data);
                k += 1;
                n += 1;
                let t = start.elapsed().as_secs_f32();
                if t >= next {
                    println!("  t={t:5.1}s frames {n:4}: L {:5.1} R {:5.1}", acc_l / k as f64, acc_r / k as f64);
                    (acc_l, acc_r, k) = (0.0, 0.0, 0);
                    next += 0.5;
                }
            }
        }
        n
    });
    for (delay_ms, cmd) in during {
        tokio::time::sleep(Duration::from_millis(*delay_ms)).await;
        println!("  t={:5.1}s send {cmd}", start.elapsed().as_secs_f32());
        send(ctl, cmd).await;
    }
    let left = Duration::from_secs_f32(secs).saturating_sub(start.elapsed());
    tokio::time::sleep(left).await;
    send(ctl, "close_stream_all").await;
    let n = reader.await.unwrap_or(0);
    println!("  {n} frames");
}

#[tokio::main]
async fn main() {
    let ip = std::env::args().nth(1).unwrap_or_else(|| "192.168.178.34".into());
    let secs: f32 = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(8.0);
    let _lock = scancraft_driver_revopoint::scanner_lock::ScannerLock::try_acquire("vcsel_revo_replay").expect("scanner lock");
    let ctl = Control::new(&ip).unwrap();

    // Session 1: setup sent with the stream already open (Revo: trigger mode ~0.6 s after the
    // stream opened, the rest ~3 s later).
    let mut during: Vec<(u64, &str)> = vec![(600, SESSION_SETUP[0])];
    during.push((2500, SESSION_SETUP[1]));
    during.extend(SESSION_SETUP[2..].iter().map(|c| (60, *c)));
    session(&ctl, &ip, "session 1 (Revo 19:27:53)", &during, secs).await;

    // Revo was closed and restarted: no emitter shutdown; its connect writes.
    println!("== reconnect writes");
    send(&ctl, "@0xb03 0").await;
    send(&ctl, "@0x48b 10").await;

    // Session 2: trigger mode 0.6 s after the stream opened; the setup again ~3 s later.
    session(&ctl, &ip, "session 2 (Revo 19:33:07)", &during, secs).await;

    ctl.turn_off_emitters().await;
}
