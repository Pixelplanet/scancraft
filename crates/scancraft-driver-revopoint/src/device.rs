use crate::calibration::{DeviceCalibration, StereoCalibration};
use crate::control::Control;
use crate::imu::{parse_imu_packet, IMU_PACKET_SIZE};
use crate::media::{FrameDecoder, PacketParser};
use crate::streams::{self, ModeStream};
use async_trait::async_trait;
use futures::StreamExt;
use glam::{Mat3, Quat, Vec3};
use scancraft_core::{ImuSample, PinholeIntrinsics, PreviewFrame, SensorFrame};
use scancraft_driver::{
    Availability, DeviceInfo, DeviceParam, DeviceTransport, DriverError, LightSource, ObjectType, ScanConfig,
    ScannerCapabilities, ScannerDevice, TrackingMode,
};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use tokio::io::AsyncReadExt;
use tokio::net::TcpStream;
use tokio::sync::{broadcast, mpsc};
use tokio::task::JoinHandle;
use tokio::time::Duration;

pub struct RevopointDevice {
    info: DeviceInfo,
    capabilities: ScannerCapabilities,
    ctl: Control,
    calibration: Arc<DeviceCalibration>,
    imu_tx: broadcast::Sender<ImuSample>,
    imu_task: JoinHandle<()>,
    stream_task: Option<JoinHandle<()>>,
    /// RGB stream running next to the depth stream (colour scans, Color tracking).
    color_task: Option<JoinHandle<()>>,
    paused: Arc<AtomicBool>,
    /// Scan configuration of the last scan; preview reuses its object preset.
    last_config: ScanConfig,
    /// Held while streaming so no other process reconfigures the scanner under us.
    lock: Option<crate::scanner_lock::ScannerLock>,
    /// The calibration files as downloaded (name in a pass's calibration folder, bytes).
    calibration_files: Vec<(String, Vec<u8>)>,
    /// Algorithm version from the scanner's announcement.
    algo_version: String,
    /// Where the next scan is recorded (`set_recording`), and what the running one records to.
    recording: Option<scancraft_driver::RecordingTarget>,
    recording_files: Vec<std::path::PathBuf>,
    /// The stream setup of the last scan (format, display type, ...) and the control queries
    /// sent to start it.
    last_stream: serde_json::Value,
}

impl RevopointDevice {
    pub async fn new_wifi(ip: &str, sn: String) -> Result<Self, DriverError> {
        let ctl = Control::new(ip)?;

        // Without the device's own calibration every point would be in the wrong place,
        // so a failed download is a connection failure rather than a silent fallback.
        let yaml = ctl
            .cgi("download=/data/camparam/camparam.yaml")
            .await
            .map_err(|e| DriverError::ConnectionFailed(format!("calibration download: {e}")))?;
        let red = StereoCalibration::parse(&yaml).ok_or_else(|| DriverError::Protocol("camparam.yaml is missing pL/pR".into()))?;
        let mut calibration = DeviceCalibration::new(red);
        // The files are kept as received: a recorded scan is decoded offline with them.
        let mut calibration_files = vec![("camparam.yaml".to_string(), yaml.clone().into_bytes())];
        match crate::color_stream::download_calibration_files(&ctl).await {
            Ok((color, files)) => {
                calibration = calibration.with_color(color);
                calibration_files.extend(files);
            }
            Err(e) => log::warn!("POP 4 {sn}: no RGB calibration ({e}); colour scanning disabled"),
        }
        match crate::calibration::download_blue_files(&ctl).await {
            Ok((blue, files)) => {
                calibration = calibration.with_blue(blue);
                calibration_files.extend(files);
            }
            Err(e) => log::warn!("POP 4 {sn}: blue (laser) calibration unavailable, laser modes disabled: {e}"),
        }
        // Firmware and algorithm versions from the scanner's announcement (best effort).
        let announcement = match ip.parse::<std::net::Ipv4Addr>() {
            Ok(addr) => crate::discovery::query(addr, Duration::from_millis(1200)).await,
            Err(_) => None,
        };
        let (firmware, algo_version) = announcement.map(|a| (a.firmware, a.algo)).unwrap_or_default();
        if firmware.is_empty() {
            log::warn!("POP 4: the scanner did not say its firmware version");
        } else {
            log::info!("POP 4 firmware {firmware}, algorithm {algo_version}");
        }
        // The copies kept for capture records must not carry the serial number (projects are
        // shared as test data): `camparam.yaml` names it as `deviceID`, `metroExtra.bin` in its
        // header.
        let calibration_files = calibration_files.into_iter().map(|(name, bytes)| (name, crate::calibration::redact_serial(&bytes, &sn))).collect();
        let has_rgb = calibration.color.is_some();
        let calibration = Arc::new(calibration);
        let rig = calibration.red_rig(streams::full_field::WIDTH, streams::full_field::HEIGHT);
        log::info!("POP 4 {sn}: rectified intrinsics {:?}, baseline {:.1} mm", rig.intrinsics, rig.baseline_mm);

        let info = DeviceInfo {
            vendor: "Revopoint".to_string(),
            model: "POP 4".to_string(),
            serial_number: sn,
            firmware_version: if firmware.is_empty() { "—".to_string() } else { firmware },
            active_transport: DeviceTransport::WiFi { ip: ip.to_string(), control_port: 80, data_port: 8871, imu_port: 9999 },
            depth_resolution: (rig.width, rig.height),
            rgb_resolution: has_rgb.then_some((crate::color_stream::WIDTH, crate::color_stream::HEIGHT)),
            recommended_distance_range_mm: (150.0, 400.0),
            // IMU x, y, z -> scanner +X, +Z, -Y: found by matching the accelerometer's "up" to
            // the table normal over a handheld recording (`examples/imu_extrinsic_search.rs`).
            imu_to_sensor: Some(Quat::from_mat3(&Mat3::from_cols(Vec3::X, Vec3::Z, -Vec3::Y)).to_array()),
        };

        // Firmware v5.3.99 reports IMU status 0 (Revo Scan gets the same), sends no IMU data in
        // the frame header and does not listen on the IMU port; v5.3.94 reported 1 and did both.
        let has_imu = ctl.cgi_once("cam_type=imu&get_IMUStatus").await.ok().flatten().is_some_and(|r| r.replace(' ', "").contains("\"result\":1"));
        if !has_imu {
            log::warn!("POP 4 reports its IMU as off (get_IMUStatus); tracking runs without the motion prior");
        }
        let capabilities = ScannerCapabilities {
            has_rgb_camera: has_rgb,
            has_imu,
            has_hardware_turntable_control: false,
            supported_laser_modes: vec![LightSource::FullFieldHd.label().to_string()],
            supports_single_shot: true,
            supports_continuous_scan: true,
            supports_marker_tracking: true,
            supports_feature_tracking: true,
            supported_transports: vec!["WiFi".to_string()],
            modes: crate::modes::pop4_modes(),
            // Measured 2026-11-10 (firmware v5.3.99, Full-field HD, static scene): IR mean 27 at
            // 0.7 ms, 100 at 6 ms, 162 at 6 ms with gain 5; depth coverage 25 % / 34 % / 14 %.
            // (The 2026-11-01 "no effect" result predates the fix of the register writes.)
            manual_exposure: true,
        };

        let (imu_tx, _) = broadcast::channel(256);
        let imu_task = tokio::spawn(imu_listener(ip.to_string(), imu_tx.clone()));

        Ok(Self {
            info,
            capabilities,
            ctl,
            calibration,
            imu_tx,
            imu_task,
            stream_task: None,
            color_task: None,
            paused: Arc::new(AtomicBool::new(false)),
            last_config: ScanConfig::default(),
            lock: None,
            calibration_files,
            algo_version,
            recording: None,
            recording_files: Vec::new(),
            last_stream: serde_json::Value::Null,
        })
    }

    /// Rectified depth intrinsics of the Full-field HD stream.
    pub fn depth_intrinsics(&self) -> PinholeIntrinsics {
        self.calibration.red_rig(streams::full_field::WIDTH, streams::full_field::HEIGHT).intrinsics
    }

    pub fn calibration(&self) -> &Arc<DeviceCalibration> {
        &self.calibration
    }

    /// Sets the device up for `mode`, then streams it. `on_frame` runs on a dedicated decode
    /// thread and returns `false` to stop. With a `recorder` every received packet is also
    /// recorded in the wire framing ([`wire_record`]), through its writer thread, so a session
    /// can be replayed offline (`offline.rs`, `examples/replay_track.rs`).
    async fn start_stream<F>(&mut self, mode: Box<dyn ModeStream>, recorder: Option<scancraft_driver::RawRecorder>, mut on_frame: F) -> Result<(), DriverError>
    where
        F: FnMut(SensorFrame) -> bool + Send + 'static,
    {
        self.stop_stream().await;
        if self.lock.is_none() {
            let owner = format!("ScanCraft streaming {} {}", self.info.model, self.info.serial_number);
            self.lock = Some(crate::scanner_lock::ScannerLock::try_acquire(&owner).map_err(DriverError::Other)?);
        }
        // Revo's SDK order, the only one that works for every mode (docs/protocol/session.md):
        // close streams, display geometry, output format (the DSP powers off and reloads here,
        // forgetting every register), open the media stream, trigger mode, wait for the first
        // frame, and only then the mode's register setup.
        // Everything sent from here until the mode is configured goes into the capture record.
        self.ctl.start_transcript();
        self.ctl.cgi("close_stream_all").await?;
        let spec = mode.spec();
        self.ctl.set_stream_format(spec.output_fmt, spec.display_type, spec.width, spec.layout.display_rows(spec.height)).await?;
        // Verify the switch took effect (another client reconfiguring the device shows up here).
        let fmt = self.ctl.cgi_once("cam_type=mipi&get_depth_output_fmt").await.ok().flatten().unwrap_or_default();
        let reso = self.ctl.cgi_once("cam_type=mipi&get_display_reso").await.ok().flatten().unwrap_or_default();
        log::info!(
            "POP 4 stream setup: want fmt {} type {} {}x{}; device reports fmt {} reso {}",
            spec.output_fmt,
            spec.display_type,
            spec.width,
            spec.height,
            fmt.trim(),
            reso.trim()
        );

        // The decode thread and the configuration after the first frame share the mode.
        let mode = Arc::new(tokio::sync::Mutex::new(mode));
        let decode_mode = mode.clone();
        // Signalled when the first packet arrives (Revo's SDK configures only after that).
        let first_packet = Arc::new(tokio::sync::Notify::new());
        let first_packet_tx = first_packet.clone();
        let url = format!("http://{}/cgi-bin/zx_media.cgi?{}", self.ctl.ip, spec.media_query);
        let paused = self.paused.clone();
        paused.store(false, Ordering::SeqCst);

        // Socket reading stays on the runtime; decoding runs on its own thread. The hand-off holds
        // one packet and a newer one replaces it, so a slow consumer always gets the latest frame
        // (an older one would also have lost its colour partner by then).
        let (packet_tx, packet_rx) = latest_slot::<Vec<u8>>();
        std::thread::spawn(move || {
            let mut decoder = FrameDecoder::new(spec.layout, spec.width, spec.height);
            let mut index = 0u64;
            while let Some(packet) = packet_rx.take() {
                match decoder.decode(&packet) {
                    Ok(decoded) => {
                        let Some(frame) = decode_mode.blocking_lock().to_sensor_frame(decoded, index + 1) else { continue };
                        index += 1;
                        if !on_frame(frame) {
                            break;
                        }
                    }
                    Err(e) => log::warn!("POP 4 frame decode: {e}"),
                }
            }
        });

        self.stream_task = Some(tokio::spawn(async move {
            let client = reqwest::Client::builder().connect_timeout(Duration::from_secs(4)).tcp_nodelay(true).build().unwrap_or_default();
            loop {
                let resp = match client.get(&url).send().await.and_then(|r| r.error_for_status()) {
                    Ok(r) => r,
                    Err(e) => {
                        log::warn!("media stream connect: {e}");
                        tokio::time::sleep(Duration::from_millis(500)).await;
                        continue;
                    }
                };
                log::info!("media stream {url}: HTTP {}", resp.status());
                let mut body = resp.bytes_stream();
                let mut parser = PacketParser::default();
                let mut first_logged = false;
                let mut last_touch = std::time::Instant::now();
                while let Ok(Some(Ok(chunk))) = tokio::time::timeout(Duration::from_secs(4), body.next()).await {
                    if last_touch.elapsed() > Duration::from_secs(60) {
                        crate::scanner_lock::touch();
                        last_touch = std::time::Instant::now();
                    }
                    if !first_logged {
                        first_logged = true;
                        let head: String = chunk.iter().take(32).map(|b| format!("{b:02x}")).collect();
                        log::info!("media stream first chunk: {} bytes, starts {head}", chunk.len());
                    }
                    parser.push(&chunk);
                    while let Some(packet) = parser.next_packet() {
                        first_packet_tx.notify_one();
                        if paused.load(Ordering::Relaxed) {
                            continue;
                        }
                        // Through the writer thread: a slow disk drops records, never the stream.
                        if let Some(rec) = recorder.as_ref() {
                            rec.push(wire_record(&packet));
                        }
                        if !packet_tx.put(packet) {
                            return;
                        }
                    }
                }
                log::warn!("media stream stalled, reconnecting");
            }
        }));
        // Give the stream request time to reach the device first (Revo: ~0.6 s).
        tokio::time::sleep(Duration::from_millis(600)).await;
        self.ctl.cgi_ok("cam_type=mipi&set_trigger_mode=0").await?;
        // Revo's SDK waits up to 20 s for the first frame (the stream task reconnects meanwhile:
        // the first stream after a boot often delivers nothing).
        if tokio::time::timeout(Duration::from_secs(20), first_packet.notified()).await.is_err() {
            self.shut_down_stream().await;
            return Err(DriverError::Protocol(
                "the scanner delivered no frame within 20 s; if this repeats, restart the scanner (its media server hangs after a failed stream)".into(),
            ));
        }
        let configured = mode.lock().await.configure(&self.ctl).await;
        let commands = self.ctl.take_transcript();
        self.last_stream = serde_json::json!({
            "output_fmt": spec.output_fmt,
            "display_type": spec.display_type,
            "width": spec.width,
            "height": spec.height,
            "layout": format!("{:?}", spec.layout),
            "media_query": spec.media_query,
            "commands": commands,
        });
        if let Err(e) = configured {
            self.shut_down_stream().await;
            return Err(e);
        }
        Ok(())
    }

    /// Starts the RGB stream next to the running depth stream; frames go to `sync`. Must follow
    /// `start_stream` (which closes all streams first).
    async fn start_color_stream(&mut self, sync: Arc<crate::color_stream::ColorSync>, recorder: Option<scancraft_driver::RawRecorder>) -> Result<(), DriverError> {
        self.ctl.start_transcript();
        let configured = crate::color_stream::configure(&self.ctl).await;
        let commands = self.ctl.take_transcript();
        if let Some(map) = self.last_stream.as_object_mut() {
            map.insert("rgb".into(), serde_json::json!({ "media_query": crate::color_stream::MEDIA_QUERY, "width": crate::color_stream::WIDTH, "height": crate::color_stream::HEIGHT, "commands": commands }));
        }
        configured?;
        self.color_task = Some(crate::color_stream::spawn(self.ctl.ip.clone(), self.calibration.color.clone(), sync, self.paused.clone(), recorder));
        Ok(())
    }

    async fn stop_stream(&mut self) {
        if let Some(task) = self.color_task.take() {
            task.abort();
            let _ = task.await;
        }
        if let Some(task) = self.stream_task.take() {
            task.abort();
            let _ = task.await;
        }
    }

    /// Ends streaming: capture stopped and emitters off while the media connection is still
    /// held (dropping a live or stalled stream first is what hangs the device's media server),
    /// then the connections, then the scanner lock.
    async fn shut_down_stream(&mut self) {
        self.ctl.turn_off_emitters().await;
        self.stop_stream().await;
        self.lock = None;
    }
}

/// A one-item hand-off between threads where a newer item replaces an unread older one.
struct SlotState<T> {
    item: Option<T>,
    /// Sender or receiver is gone.
    closed: bool,
}

type Slot<T> = Arc<(std::sync::Mutex<SlotState<T>>, std::sync::Condvar)>;

struct SlotSender<T>(Slot<T>);
struct SlotReceiver<T>(Slot<T>);

fn latest_slot<T>() -> (SlotSender<T>, SlotReceiver<T>) {
    let slot: Slot<T> = Arc::new((std::sync::Mutex::new(SlotState { item: None, closed: false }), std::sync::Condvar::new()));
    (SlotSender(slot.clone()), SlotReceiver(slot))
}

impl<T> SlotSender<T> {
    /// Stores `item`, dropping an unread older one. `false` once the receiver is gone.
    fn put(&self, item: T) -> bool {
        let mut s = self.0 .0.lock().unwrap();
        if s.closed {
            return false;
        }
        s.item = Some(item);
        self.0 .1.notify_one();
        true
    }
}

impl<T> SlotReceiver<T> {
    /// Waits for the next item; `None` once the sender is gone.
    fn take(&self) -> Option<T> {
        let mut s = self.0 .0.lock().unwrap();
        loop {
            if let Some(item) = s.item.take() {
                return Some(item);
            }
            if s.closed {
                return None;
            }
            s = self.0 .1.wait(s).unwrap();
        }
    }
}

impl<T> Drop for SlotSender<T> {
    fn drop(&mut self) {
        self.0 .0.lock().unwrap().closed = true;
        self.0 .1.notify_all();
    }
}

impl<T> Drop for SlotReceiver<T> {
    fn drop(&mut self) {
        self.0 .0.lock().unwrap().closed = true;
    }
}

/// One packet as recorded: the media-stream framing `[u32 LE 0x11223344][u32 LE len][packet]`
/// (what `PacketParser` / `RgbPacketParser` read back).
pub(crate) fn wire_record(packet: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(packet.len() + 8);
    out.extend_from_slice(&crate::media::PACKET_MAGIC);
    out.extend_from_slice(&(packet.len() as u32).to_le_bytes());
    out.extend_from_slice(packet);
    out
}

/// A recorder for `path`, `None` (logged) when the file cannot be opened: scanning goes on.
fn open_recorder(path: &std::path::Path, append: bool, stats: &Arc<scancraft_driver::RecordStats>, queue: usize) -> Option<scancraft_driver::RawRecorder> {
    match scancraft_driver::RawRecorder::open(path, append, stats.clone(), queue) {
        Ok(r) => {
            log::info!("recording raw stream to {}", path.display());
            Some(r)
        }
        Err(e) => {
            log::warn!("cannot record to {}: {e}", path.display());
            None
        }
    }
}

/// `<SCANCRAFT_RECORD_DIR>/pop4_<unix time><suffix>.bin`; the RGB stream uses suffix `_rgb`.
/// Used when the app asked for no recording of its own (`set_recording`).
fn open_recording_named(suffix: &str) -> Option<scancraft_driver::RawRecorder> {
    let dir = std::env::var_os("SCANCRAFT_RECORD_DIR")?;
    let secs = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_secs()).unwrap_or(0);
    let path = std::path::Path::new(&dir).join(format!("pop4_{secs}{suffix}.bin"));
    open_recorder(&path, false, &scancraft_driver::RecordStats::new(), scancraft_driver::recording::DEFAULT_QUEUE_BYTES)
}

async fn imu_listener(ip: String, tx: broadcast::Sender<ImuSample>) {
    let Ok(mut stream) = TcpStream::connect((ip.as_str(), 9999)).await else {
        log::warn!("IMU port 9999 unreachable on {ip}");
        return;
    };
    let mut buf = [0u8; 1024];
    let mut pending = Vec::new();
    while let Ok(n) = stream.read(&mut buf).await {
        if n == 0 {
            break;
        }
        pending.extend_from_slice(&buf[..n]);
        while pending.len() >= IMU_PACKET_SIZE {
            if let Some(sample) = parse_imu_packet(&pending[..IMU_PACKET_SIZE]) {
                let _ = tx.send(sample);
            }
            pending.drain(..IMU_PACKET_SIZE);
        }
    }
}

/// Median of valid depth in the central 64x64 window, for the aiming distance readout.
fn centre_distance_mm(frame: &SensorFrame) -> Option<f32> {
    let depth = frame.depth.as_ref()?;
    let (w, h) = (depth.width as usize, depth.height as usize);
    let mut values: Vec<u16> = (h / 2 - 32..h / 2 + 32)
        .flat_map(|v| (w / 2 - 32..w / 2 + 32).map(move |u| v * w + u))
        .map(|i| depth.data[i])
        .filter(|&d| d != 0)
        .collect();
    if values.len() < 64 {
        return None;
    }
    let mid = values.len() / 2;
    Some(*values.select_nth_unstable(mid).1 as f32 * depth.scale_mm)
}

#[async_trait]
impl ScannerDevice for RevopointDevice {
    fn info(&self) -> &DeviceInfo {
        &self.info
    }

    fn capabilities(&self) -> &ScannerCapabilities {
        &self.capabilities
    }

    /// Aiming preview: Full-field HD with the last scan's object preset (depth gives the
    /// distance readout, the left IR image the camera view).
    async fn start_preview(&mut self) -> Result<mpsc::Receiver<PreviewFrame>, DriverError> {
        let (tx, rx) = mpsc::channel(4);
        let mut imu_rx = self.imu_tx.subscribe();
        let object = if crate::modes::full_field_preset(self.last_config.object).is_some() { self.last_config.object } else { ObjectType::General };
        let mode = Box::new(streams::FullField::new(object, self.calibration.clone())?);
        self.start_stream(mode, None, move |frame| {
            let preview = PreviewFrame {
                frame_index: frame.frame_index,
                timestamp_ms: frame.timestamp_ms,
                average_distance_mm: centre_distance_mm(&frame),
                imu: frame.imu.or_else(|| latest(&mut imu_rx)),
                ir_image: frame.ir.map(|ir| ir.left),
                rgb_image: frame.color.map(|c| c.image),
            };
            !matches!(tx.try_send(preview), Err(mpsc::error::TrySendError::Closed(_)))
        })
        .await?;
        Ok(rx)
    }

    async fn stop_preview(&mut self) -> Result<(), DriverError> {
        self.shut_down_stream().await;
        Ok(())
    }

    async fn start_scan(&mut self, config: &ScanConfig) -> Result<mpsc::Receiver<SensorFrame>, DriverError> {
        // Development: `SCANCRAFT_EXPERIMENTAL_MODES=1` starts modes marked NotImplemented.
        let experimental = std::env::var_os("SCANCRAFT_EXPERIMENTAL_MODES").is_some();
        if !experimental && !self.capabilities.modes.can_scan(config.light, config.tracking, config.object) {
            let modes = &self.capabilities.modes;
            let reason = [modes.light(config.light), modes.tracking(config.tracking), modes.object(config.object)]
                .into_iter()
                .find_map(|a| match a {
                    Availability::NotImplemented(why) => Some(why),
                    _ => None,
                })
                .unwrap_or_else(|| "this combination is not available on the POP 4".to_string());
            return Err(DriverError::InvalidParameter(format!(
                "{} + {} tracking + {}: {reason}",
                config.light.label(),
                config.tracking.label(),
                config.object.label()
            )));
        }
        let mode = streams::for_config(config, &self.calibration)?;
        self.last_config = config.clone();

        // Colour scans and Color tracking add the RGB stream; each depth frame gets the colour
        // frame taken at the same instant (docs/protocol/color.md).
        let wants_color = config.color_texture || config.tracking == TrackingMode::Color;
        if wants_color && self.calibration.color.is_none() {
            return Err(DriverError::InvalidParameter("colour needs the RGB calibration, which could not be read from the scanner".into()));
        }
        let color_sync = wants_color.then(|| Arc::new(crate::color_stream::ColorSync::default()));
        let frame_sync = color_sync.clone();

        // The app's recording target (raw files in the pass folder), else SCANCRAFT_RECORD_DIR.
        let target = self.recording.take();
        let (main_rec, rgb_rec) = match &target {
            Some(t) => (
                open_recorder(&t.main, t.append, &t.main_stats, t.queue_bytes),
                wants_color.then(|| open_recorder(&t.rgb, t.append, &t.rgb_stats, t.queue_bytes)).flatten(),
            ),
            None => (open_recording_named(""), wants_color.then(|| open_recording_named("_rgb")).flatten()),
        };
        self.recording_files = main_rec.iter().chain(rgb_rec.iter()).map(|r| r.path().to_path_buf()).collect();
        let (tx, rx) = mpsc::channel(4);
        let mut imu_rx = self.imu_tx.subscribe();
        self.start_stream(mode, main_rec, move |mut frame| {
            frame.imu = frame.imu.or_else(|| latest(&mut imu_rx));
            if let Some(sync) = &frame_sync {
                frame.color = sync.take_match(frame.timestamp_ms, Duration::from_millis(120));
            }
            !matches!(tx.try_send(frame), Err(mpsc::error::TrySendError::Closed(_)))
        })
        .await?;
        if let Some(sync) = color_sync {
            // Depth keeps streaming if the RGB setup fails; frames then simply carry no colour.
            if let Err(e) = self.start_color_stream(sync, rgb_rec).await {
                log::warn!("RGB stream not started: {e}");
            }
        }
        Ok(rx)
    }

    async fn pause_scan(&mut self) -> Result<(), DriverError> {
        self.paused.store(true, Ordering::SeqCst);
        Ok(())
    }

    async fn resume_scan(&mut self) -> Result<(), DriverError> {
        self.paused.store(false, Ordering::SeqCst);
        Ok(())
    }

    async fn stop_scan(&mut self) -> Result<(), DriverError> {
        self.shut_down_stream().await;
        Ok(())
    }

    fn imu_stream(&mut self) -> Option<broadcast::Receiver<ImuSample>> {
        Some(self.imu_tx.subscribe())
    }

    async fn set_parameter(&mut self, param: DeviceParam) -> Result<(), DriverError> {
        match param {
            // Revo's exposure slider: `0x488 1`, then the value (sdk.log 2026-11-01 19:19).
            DeviceParam::DepthExposure(us) => {
                self.ctl.preisp("0x488 1").await?;
                self.ctl.preisp(&format!("0x911 {}", us.clamp(100, 42_000))).await
            }
            DeviceParam::DepthGain(gain) => {
                self.ctl.preisp("0x488 1").await?;
                self.ctl.preisp(&format!("0x903 {:#x}", gain.clamp(1, 10) as u32 * 0x10)).await
            }
            other => Err(DriverError::InvalidParameter(format!("{other:?} is not supported by the POP 4 driver"))),
        }
    }

    async fn disconnect(&mut self) -> Result<(), DriverError> {
        self.shut_down_stream().await;
        self.imu_task.abort();
        Ok(())
    }

    fn set_recording(&mut self, target: Option<scancraft_driver::RecordingTarget>) -> bool {
        self.recording = target;
        true
    }

    fn recording_info(&self, config: &ScanConfig) -> Option<scancraft_driver::RecordingInfo> {
        let mut stream = self.last_stream.clone();
        if let Some(map) = stream.as_object_mut() {
            map.insert("format".into(), crate::offline::STREAM_FORMAT.into());
            map.insert("record_framing".into(), "u32 LE 0x11223344, u32 LE length, packet as received (rgb: JPEG + u32 LE timestamp)".into());
            map.insert("algorithm_version".into(), self.algo_version.clone().into());
            map.insert("light".into(), config.light.label().into());
        }
        Some(scancraft_driver::RecordingInfo { stream, calibration: self.calibration_files.clone(), files: self.recording_files.clone() })
    }
}

impl Drop for RevopointDevice {
    fn drop(&mut self) {
        self.imu_task.abort();
        if let Some(task) = self.stream_task.take() {
            task.abort();
        }
        if let Some(task) = self.color_task.take() {
            task.abort();
        }
    }
}

/// Newest sample in the IMU channel, discarding older ones.
fn latest(rx: &mut broadcast::Receiver<ImuSample>) -> Option<ImuSample> {
    let mut last = None;
    loop {
        match rx.try_recv() {
            Ok(s) => last = Some(s),
            Err(broadcast::error::TryRecvError::Lagged(_)) => continue,
            Err(_) => return last,
        }
    }
}
