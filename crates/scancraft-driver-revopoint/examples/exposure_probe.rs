//! Does changing exposure / gain while streaming change the images? Streams Full-field HD and
//! steps through register writes, reporting the mean IR brightness and depth coverage of the
//! frames after each step, and the mean colour-camera brightness (which shows whether a write
//! lights the white texture LEDs).
//! `cargo run --release -p scancraft-driver-revopoint --example exposure_probe -- <ip>`

use scancraft_driver::{DeviceTransport, ScanConfig, ScannerDriver, TrackingMode};
use scancraft_driver_revopoint::control::Control;
use scancraft_driver_revopoint::RevopointDriver;
use std::time::Duration;

#[tokio::main]
async fn main() {
    let ip = std::env::args().nth(1).unwrap_or_else(|| "192.168.178.34".into());
    let transport = DeviceTransport::WiFi { ip: ip.clone(), control_port: 80, data_port: 8871, imu_port: 9999 };
    let mut dev = RevopointDriver.connect(&transport).await.expect("connect");
    let ctl = Control::new(&ip).unwrap();
    let mut rx = dev.start_scan(&ScanConfig {
        color_texture: true,
        // PROBE_TRACKING=marker probes the marker stream instead of plain Full-field HD.
        tracking: if std::env::var("PROBE_TRACKING").is_ok_and(|t| t == "marker") { TrackingMode::Marker } else { TrackingMode::Feature },
        ..ScanConfig::default()
    }).await.expect("scan");

    let steps: Vec<(String, Vec<String>)> = match std::env::var("PROBE_STEPS") {
        // e.g. PROBE_STEPS="0x707 150 500;0x488 1+0x911 6000+0xb07 0" (steps separated by `;`,
        // register writes within a step by `+`)
        Ok(list) => list.split(';').map(|c| (c.to_string(), c.split('+').map(|w| w.trim().to_string()).collect())).collect(),
        Err(_) => [
            ("preset (3000 us, gain 1)", vec![]),
            ("0x911 800", vec!["0x911 800"]),
            ("0x911 800 + 0x488 1", vec!["0x488 1", "0x911 800"]),
            ("0x911 6000 + 0x488 1", vec!["0x488 1", "0x911 6000"]),
            ("gain 0x40 + 0x488 1", vec!["0x488 1", "0x903 0x40"]),
            ("back: 0x911 3000, 0x903 0x10 + 0x488 1", vec!["0x488 1", "0x911 3000", "0x903 0x10"]),
        ]
        .into_iter()
        .map(|(l, c)| (l.to_string(), c.into_iter().map(String::from).collect()))
        .collect(),
    };
    let steps: Vec<(String, Vec<String>)> = std::iter::once(("start (preset)".to_string(), vec![])).chain(steps).collect();
    for (label, cmds) in &steps {
        // PROBE_GAP_MS spaces the writes of one step (back-to-back exposure writes stalled the
        // POP 4 over USB unless ~6 frames apart; All3DScan RE notes).
        let gap = Duration::from_millis(std::env::var("PROBE_GAP_MS").ok().and_then(|v| v.parse().ok()).unwrap_or(0));
        for c in cmds {
            if let Err(e) = ctl.preisp(c).await {
                println!("  {c}: {e}");
            }
            tokio::time::sleep(gap).await;
        }
        // Let the change settle, then measure a few frames.
        tokio::time::sleep(Duration::from_millis(800)).await;
        while rx.try_recv().is_ok() {}
        let (mut ir, mut cov, mut n) = (0.0f64, 0.0f64, 0);
        let (mut rgb, mut rgb_n, mut rgb_max) = (0.0f64, 0, 0.0f64);
        while n < 4 {
            let Ok(Some(f)) = tokio::time::timeout(Duration::from_secs(5), rx.recv()).await else { break };
            if let Some(c) = f.color.as_ref() {
                let m = c.image.data.iter().map(|&v| v as f64).sum::<f64>() / c.image.data.len().max(1) as f64;
                rgb += m;
                rgb_max = rgb_max.max(m);
                rgb_n += 1;
            }
            if let (Some(i), Some(d)) = (f.ir.as_ref(), f.depth.as_ref()) {
                ir += i.left.data.iter().map(|&v| v as f64).sum::<f64>() / i.left.data.len() as f64;
                cov += d.valid_pixel_count() as f64 / d.data.len() as f64;
                n += 1;
            }
        }
        let n = n.max(1) as f64;
        let rgb = if rgb_n > 0 { format!("colour mean {:6.1} (max {:6.1}, {rgb_n} images)", rgb / rgb_n as f64, rgb_max) } else { "no colour".into() };
        println!("{label:40}: IR mean {:6.1}, depth coverage {:5.1}%, {rgb}", ir / n, 100.0 * cov / n);
    }
    dev.stop_scan().await.ok();
    dev.disconnect().await.ok();
}
