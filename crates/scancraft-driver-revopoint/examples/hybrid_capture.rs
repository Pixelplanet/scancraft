//! Records a raw Hybrid HD stream (fmt 5 / display type 10, `camera_id=33&type_id=21`) with
//! Revo Scan's logged device setup, for offline decoding:
//! `cargo run -p scancraft-driver-revopoint --example hybrid_capture -- <ip> <seconds> <out.bin> [general|dark|face] [extra preisp args;...]`
//!
//! Takes the scanner lock, configures the device like Revo Scan 6 did on 2026-06-19 14:55
//! (see `docs/protocol/hybrid.md`), dumps the de-chunked media body to `out.bin`, then switches
//! all emitters off and closes the streams. `extra` is a `;`-separated list of `rk_preisp`
//! argument strings sent after the standard setup (for experiments, e.g. `0x3001 14`).
//! Object `fullfield` records a Full-field HD stream (fmt 3 / type 4) of the same scene instead,
//! as on-device depth reference. The device calibration is saved next to `out.bin` as
//! `camparam.yaml`.
//!
//! `REVO_ORDER=1` follows Revo's SDK (sdk.log 2026-11-01 19:34:05..13): open the stream, send
//! only `set_trigger_mode=0`, wait (up to 20 s) for the first complete packet, and only then send
//! the register setup. Revo's first Hybrid frame came 5.6 s after the stream opened.
//!
//! `WITH_RGB=1` also configures the RGB camera and reads (and discards) its stream
//! `camera_id=50&type_id=20` alongside, as Revo does: in Revo's capture the first Hybrid byte
//! arrived 0.4 s after its RGB stream opened (docs/protocol/hybrid.md).

use futures::StreamExt;
use scancraft_driver::ObjectType;
use scancraft_driver_revopoint::control::Control;
use scancraft_driver_revopoint::scanner_lock::ScannerLock;
use scancraft_driver_revopoint::streams::{hybrid, FullField, ModeStream};
use scancraft_driver_revopoint::{DeviceCalibration, StereoCalibration};
use std::io::Write;
use std::time::{Duration, Instant};

#[tokio::main]
async fn main() {
    env_logger::init();
    let args: Vec<String> = std::env::args().collect();
    let ip = args.get(1).cloned().unwrap_or_else(|| "192.168.178.34".into());
    let seconds: f32 = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(4.0);
    let out = args.get(3).cloned().unwrap_or_else(|| "hybrid.bin".into());
    let full_field = args.get(4).map(String::as_str) == Some("fullfield");
    let object = match args.get(4).map(String::as_str) {
        Some("dark") => ObjectType::Dark,
        Some("face") => ObjectType::Face,
        _ => ObjectType::General,
    };
    let extra: Vec<String> = args.get(5).map(|s| s.split(';').map(str::trim).filter(|s| !s.is_empty()).map(String::from).collect()).unwrap_or_default();

    let _lock = ScannerLock::acquire("hybrid-agent hybrid_capture", Duration::from_secs(600)).expect("scanner lock");
    let ctl = Control::new(&ip).expect("control");
    let _ = *T0;
    let result = match tokio::time::timeout(Duration::from_secs_f32(seconds + 60.0), capture(&ctl, object, full_field, &extra, seconds, &out)).await {
        Ok(r) => r,
        Err(_) => Err("capture timed out".into()),
    };
    println!("emitters off, streams closed");
    ctl.turn_off_emitters().await;
    drop(_lock); // `process::exit` would skip the destructor and leave the lock behind.
    if let Err(e) = result {
        eprintln!("capture failed: {e}");
        std::process::exit(1);
    }
}

async fn capture(ctl: &Control, object: ObjectType, full_field: bool, extra: &[String], seconds: f32, out: &str) -> Result<(), String> {
    let preset = hybrid::preset(object).ok_or("object not available in Hybrid HD")?;
    let yaml = ctl.cgi("download=/data/camparam/camparam.yaml").await.map_err(|e| e.to_string())?;
    let yaml_path = std::path::Path::new(out).with_file_name("camparam.yaml");
    std::fs::write(&yaml_path, &yaml).map_err(|e| e.to_string())?;
    let calibration = std::sync::Arc::new(DeviceCalibration::new(StereoCalibration::parse(&yaml).ok_or("bad camparam.yaml")?));
    let step = |s: &str| {
        println!("[{:>6.2}] {s}", T0.elapsed().as_secs_f32());
        let _ = std::io::stdout().flush();
    };
    step("close_stream_all");
    ctl.cgi("close_stream_all").await.map_err(|e| e.to_string())?;
    step("set_stream_format");
    let mode: Box<dyn ModeStream> = if full_field {
        Box::new(FullField::new(object, calibration).map_err(|e| e.to_string())?)
    } else {
        Box::new(hybrid::Hybrid::new(object, calibration).map_err(|e| e.to_string())?)
    };
    let spec = mode.spec();
    if full_field {
        // The driver's order for Full-field HD: configure, switch format, open the stream.
        mode.configure(ctl).await.map_err(|e| e.to_string())?;
    }
    ctl.set_stream_format(spec.output_fmt, spec.display_type, spec.width, spec.height).await.map_err(|e| e.to_string())?;

    // Revo opens the media stream first and configures while it is open (first frame ~5 s later).
    let url = format!("http://{}/cgi-bin/zx_media.cgi?{}", ctl.ip, spec.media_query);
    let client = reqwest::Client::builder().connect_timeout(Duration::from_secs(4)).build().map_err(|e| e.to_string())?;
    step("open media stream");
    let request = tokio::spawn(async move { client.get(&url).send().await });
    tokio::time::sleep(Duration::from_millis(300)).await;
    let revo_order = std::env::var_os("REVO_ORDER").is_some() && !full_field;
    let _ = preset;
    if revo_order {
        step("trigger mode only (setup after the first frame)");
        ctl.cgi_ok("cam_type=mipi&set_trigger_mode=0").await.map_err(|e| e.to_string())?;
    } else {
        step("configure");
        if !full_field {
            mode.configure(ctl).await.map_err(|e| e.to_string())?;
        }
    }
    for args in extra {
        step(&format!("extra: {args}"));
        ctl.preisp(args).await.map_err(|e| e.to_string())?;
    }
    let rgb = if std::env::var_os("WITH_RGB").is_some() {
        step("RGB camera configure + stream");
        scancraft_driver_revopoint::color_stream::configure(ctl).await.map_err(|e| e.to_string())?;
        let url = format!("http://{}/cgi-bin/zx_media.cgi?{}", ctl.ip, scancraft_driver_revopoint::color_stream::MEDIA_QUERY);
        Some(tokio::spawn(async move {
            let Ok(resp) = reqwest::Client::new().get(&url).send().await else { return 0usize };
            let mut body = resp.bytes_stream();
            let mut n = 0usize;
            while let Ok(Some(Ok(c))) = tokio::time::timeout(Duration::from_secs(5), body.next()).await {
                n += c.len();
            }
            n
        }))
    } else {
        None
    };
    step("waiting for response headers");
    let resp = tokio::time::timeout(Duration::from_secs(30), request)
        .await
        .map_err(|_| "no media response within 30 s".to_string())?
        .map_err(|e| e.to_string())?
        .and_then(|r| r.error_for_status())
        .map_err(|e| e.to_string())?;
    step(&format!("headers: {:?}", resp.headers()));
    let mut body = resp.bytes_stream();
    let mut file = std::io::BufWriter::new(std::fs::File::create(out).map_err(|e| e.to_string())?);
    let start = Instant::now();
    let mut total = 0usize;
    let mut parser = scancraft_driver_revopoint::PacketParser::default();
    let (mut packets, mut setup) = (0usize, None);
    while start.elapsed().as_secs_f32() < seconds {
        match tokio::time::timeout(Duration::from_secs(if revo_order && packets == 0 { 20 } else { 15 }), body.next()).await {
            Ok(Some(Ok(chunk))) => {
                total += chunk.len();
                file.write_all(&chunk).map_err(|e| e.to_string())?;
                parser.push(&chunk);
                while let Some(p) = parser.next_packet() {
                    packets += 1;
                    step(&format!("packet {packets}: {} bytes", p.len()));
                    if revo_order && setup.is_none() {
                        // Keep reading while the setup is sent, as Revo's SDK does.
                        step("first packet: sending the register setup");
                        let (ip, p) = (ctl.ip.clone(), preset);
                        setup = Some(tokio::spawn(async move {
                            let ctl = Control::new(&ip).map_err(|e| e.to_string())?;
                            hybrid::configure_device(&ctl, &p).await.map_err(|e| e.to_string())
                        }));
                    }
                }
            }
            Ok(Some(Err(e))) => return Err(e.to_string()),
            Ok(None) => break,
            Err(_) => {
                step("no data for 15 s");
                break;
            }
        }
    }
    if let Some(setup) = setup {
        step(&format!("setup result: {:?}", setup.await));
    }
    file.flush().map_err(|e| e.to_string())?;
    step(&format!("{packets} packets, {total} bytes in {:.1} s -> {out}", start.elapsed().as_secs_f32()));
    // End the stream on the device while the connection is still open: dropping a stalled
    // connection first is what wedged the media server before (AGENTS.md).
    step("close_stream_all (stream still held)");
    let _ = ctl.cgi("close_stream_all").await;
    drop(body);
    if let Some(rgb) = rgb {
        // The RGB response does not always end after `close_stream_all`; don't wait on it.
        match tokio::time::timeout(Duration::from_secs(6), rgb).await {
            Ok(n) => step(&format!("RGB stream: {} bytes", n.unwrap_or(0))),
            Err(_) => step("RGB stream did not end"),
        }
    }
    Ok(())
}

static T0: std::sync::LazyLock<Instant> = std::sync::LazyLock::new(Instant::now);
