//! HTTP control plane of the POP 4 (`/cgi-bin/zx_cmd.cgi`): commands, `rk_preisp` register
//! writes, and the firmware's quirks (busy replies, asynchronous format switches).

use crate::protocol::RevopointCgiEndpoints;
use scancraft_driver::DriverError;
use tokio::time::Duration;

/// Cheap to clone; every mode stream gets one to configure the device.
#[derive(Clone)]
pub struct Control {
    pub http: reqwest::Client,
    pub endpoints: RevopointCgiEndpoints,
    pub ip: String,
    /// While `Some`, every control query sent (shared by all clones): the setup of a stream as
    /// it was actually sent, for the capture record ([`Control::start_transcript`]).
    transcript: std::sync::Arc<std::sync::Mutex<Option<Vec<String>>>>,
}

impl Control {
    pub fn new(ip: &str) -> Result<Self, DriverError> {
        let http = reqwest::Client::builder()
            .timeout(Duration::from_secs(3))
            .build()
            .map_err(|e| DriverError::Other(e.to_string()))?;
        Ok(Self { http, endpoints: RevopointCgiEndpoints::new(ip), ip: ip.to_string(), transcript: Default::default() })
    }

    /// Starts keeping every control query sent from now on (a running transcript is restarted).
    pub fn start_transcript(&self) {
        if let Ok(mut t) = self.transcript.lock() {
            *t = Some(Vec::new());
        }
    }

    /// Ends the transcript and returns the queries sent since [`Self::start_transcript`].
    pub fn take_transcript(&self) -> Vec<String> {
        self.transcript.lock().ok().and_then(|mut t| t.take()).unwrap_or_default()
    }

    fn note(&self, query: &str) {
        if let Ok(mut t) = self.transcript.lock() {
            if let Some(list) = t.as_mut().filter(|l| l.len() < 2000) {
                list.push(query.to_string());
            }
        }
    }

    /// Issues a `zx_cmd.cgi` query, retrying while the firmware reports it is busy.
    pub async fn cgi(&self, query: &str) -> Result<String, DriverError> {
        for _ in 0..20 {
            if let Some(body) = self.cgi_once(query).await? {
                return Ok(body);
            }
            tokio::time::sleep(Duration::from_millis(400)).await;
        }
        Err(DriverError::DeviceBusy)
    }

    /// One `zx_cmd.cgi` request; `Ok(None)` when the firmware says it is busy. Busy replies arrive
    /// either as HTTP 503 or as HTTP 200 with a `503 Service Unavailable ----please wait` body.
    pub async fn cgi_once(&self, query: &str) -> Result<Option<String>, DriverError> {
        self.note(query);
        let url = format!("{}?{query}", self.endpoints.base_url);
        // A running 5-8 MB/s stream can starve a control request; they are idempotent, so retry.
        let mut attempt = 0;
        let resp = loop {
            match self.http.get(&url).send().await {
                Ok(resp) => break resp,
                Err(e) if attempt < 3 => {
                    attempt += 1;
                    log::warn!("{query}: {e} (retry {attempt})");
                    tokio::time::sleep(Duration::from_millis(200)).await;
                }
                Err(e) => return Err(DriverError::ConnectionFailed(format!("{query}: {e}"))),
            }
        };
        if resp.status() == reqwest::StatusCode::SERVICE_UNAVAILABLE {
            return Ok(None);
        }
        let resp = resp.error_for_status().map_err(|e| DriverError::Protocol(format!("{query}: {e}")))?;
        let body = resp.text().await.map_err(|e| DriverError::Protocol(format!("{query}: {e}")))?;
        Ok((!is_busy_reply(&body)).then_some(body))
    }

    /// Downloads a device file as bytes (`zx_cmd.cgi?download=<path>`). A missing file answers
    /// `{download:-3}` (seen in Revo's log), which is reported as an error.
    pub async fn download(&self, path: &str) -> Result<Vec<u8>, DriverError> {
        let url = format!("{}?download={path}", self.endpoints.base_url);
        let resp = self.http.get(&url).send().await.map_err(|e| DriverError::ConnectionFailed(format!("{path}: {e}")))?;
        let resp = resp.error_for_status().map_err(|e| DriverError::Protocol(format!("{path}: {e}")))?;
        let body = resp.bytes().await.map_err(|e| DriverError::Protocol(format!("{path}: {e}")))?.to_vec();
        if body.starts_with(b"{download:") || is_busy_reply(&String::from_utf8_lossy(&body[..body.len().min(64)])) {
            return Err(DriverError::Protocol(format!("{path}: {}", String::from_utf8_lossy(&body[..body.len().min(64)]))));
        }
        Ok(body)
    }

    /// `cgi` that must answer `result: 0`.
    pub async fn cgi_ok(&self, query: &str) -> Result<(), DriverError> {
        expect_ok(self.cgi(query).await?)
    }

    /// Sends a message to the DSP: `echo s <args> > /dev/rk_preisp` (`<args>` = message id and
    /// values, e.g. `0x911 3000`). The reply only says the shell command ran, not that the DSP
    /// accepted the message; its own verdict is in the kernel log (device-internals.md).
    /// Writes are spaced: back-to-back exposure writes have stalled the stream (prior-art.md).
    pub async fn preisp(&self, args: &str) -> Result<(), DriverError> {
        let query = RevopointCgiEndpoints::system_cmd_query(&format!("echo s {args} > /dev/rk_preisp"));
        let body = self.cgi(&query).await?;
        tokio::time::sleep(Duration::from_millis(40)).await;
        if body.replace(' ', "").contains("\"return\":0") {
            Ok(())
        } else {
            Err(DriverError::Protocol(format!("rk_preisp '{args}' -> {body}")))
        }
    }

    /// Selects the MIPI stream format and waits until the firmware has actually switched.
    pub async fn set_stream_format(&self, output_fmt: u32, display_type: u32, width: u32, height: u32) -> Result<(), DriverError> {
        // Revo's order (capture 2026-11-01): display geometry first, then the output format. The
        // format switch rebuilds the device pipeline with the display type current at that moment;
        // the other order left a Full-field frame layout inside the Hybrid container (hybrid.md).
        // Development: SCANCRAFT_FMT_FIRST=1 restores the old (wrong) order for A/B tests.
        let fmt_first = std::env::var_os("SCANCRAFT_FMT_FIRST").is_some();
        if fmt_first {
            self.cgi_ok(&format!("cam_type=mipi&set_depth_output_fmt={output_fmt}")).await?;
        }
        self.cgi_ok(&format!(
            "cam_type=mipi&set_display_reso=1&&set_display_width={width}&&set_display_height={height}&&set_display_type={display_type}"
        ))
        .await?;
        if !fmt_first {
            self.cgi_ok(&format!("cam_type=mipi&set_depth_output_fmt={output_fmt}")).await?;
        }
        self.wait_for_output_fmt(output_fmt).await
    }

    /// The firmware applies `set_depth_output_fmt` on a background thread: `get_depth_output_fmt`
    /// reports the new value at once, then the device answers "busy" for ~2 s while it switches.
    /// A stream opened before that finishes carries the old plane layout, so wait it out.
    pub async fn wait_for_output_fmt(&self, fmt: u32) -> Result<(), DriverError> {
        let deadline = tokio::time::Instant::now() + Duration::from_secs(8);
        let mut settled = 0;
        tokio::time::sleep(Duration::from_millis(600)).await;
        while settled < 3 {
            if tokio::time::Instant::now() > deadline {
                return Err(DriverError::Timeout);
            }
            // What matters is that the device stopped answering "busy". The value itself is not
            // reliable for every format: after `set_depth_output_fmt=7` it reports 2.
            let body = self.cgi_once("cam_type=mipi&get_depth_output_fmt").await?;
            settled = match body.as_deref() {
                Some(b) => {
                    if settled == 2 && !b.replace([' ', '"'], "").contains(&format!("result:{fmt}")) {
                        log::debug!("output format {fmt} selected; device reports {}", b.trim());
                    }
                    settled + 1
                }
                None => 0,
            };
            tokio::time::sleep(Duration::from_millis(300)).await;
        }
        Ok(())
    }

    /// Stops capturing and switches every emitter off, then closes the streams.
    ///
    /// Message meanings from the DSP's own log (docs/protocol/device-internals.md): `0x483` stops
    /// the projector trigger (no frame is exposed any more), `0xb04 0` disables the laser diodes
    /// (blue lines, VCSEL), `0xb01 0` the IR fill LED, `0xb03 0` the white LED. Revo Scan ends a
    /// scan with `0x483`, `0xb04 0`.
    pub async fn turn_off_emitters(&self) {
        for args in ["0x483", "0xb04 0", "0xb01 0", "0xb03 0"] {
            if let Err(e) = self.preisp(args).await {
                log::warn!("emitter shutoff {args}: {e}");
            }
        }
        if let Err(e) = self.cgi("close_stream_all").await {
            log::warn!("close_stream_all: {e}");
        }
    }

    /// Restarts the scanner (`sync`, `reboot`; what Revo's SDK sends, e.g. around firmware
    /// updates) and waits until its control interface answers again (about 35 s). Clears a hung
    /// media server without touching the power switch.
    pub async fn reboot(&self) -> Result<(), DriverError> {
        let _ = self.cgi_once(&RevopointCgiEndpoints::system_cmd_query("sync")).await;
        let _ = self.cgi_once(&RevopointCgiEndpoints::system_cmd_query("reboot")).await;
        tokio::time::sleep(Duration::from_secs(12)).await;
        for _ in 0..40 {
            if let Ok(Some(body)) = self.cgi_once("cam_type=mipi&get_depth_output_fmt").await {
                if body.contains("result") {
                    return Ok(());
                }
            }
            tokio::time::sleep(Duration::from_secs(2)).await;
        }
        Err(DriverError::ConnectionFailed("the scanner did not come back after the restart".into()))
    }
}

pub(crate) fn is_busy_reply(body: &str) -> bool {
    body.trim_start().starts_with("503 Service Unavailable")
}

pub(crate) fn expect_ok(body: String) -> Result<(), DriverError> {
    if body.replace([' ', '"'], "").contains("result:0") {
        Ok(())
    } else {
        Err(DriverError::Protocol(format!("unexpected reply: {body}")))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn recognises_busy_replies() {
        assert!(is_busy_reply("503 Service Unavailable ----please wait for a moment by tank"));
        assert!(!is_busy_reply("{\"result\":3}"));
        assert!(!is_busy_reply("D00000000B0S00E1B"));
    }
}
