use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RevopointCgiEndpoints {
    pub base_url: String,
}

impl RevopointCgiEndpoints {
    pub fn new(ip: &str) -> Self {
        Self {
            base_url: format!("http://{}/cgi-bin/zx_cmd.cgi", ip),
        }
    }

    pub fn get_sn_url(&self) -> String {
        format!("{}?get_sn", self.base_url)
    }

    pub fn get_device_info_url(&self) -> String {
        format!("{}?get_device_info", self.base_url)
    }

    pub fn get_depth_reso_url(&self) -> String {
        format!("{}?cam_type=mipi&get_depth_reso", self.base_url)
    }

    pub fn close_stream_all_url(&self) -> String {
        format!("{}?close_stream_all", self.base_url)
    }

    pub fn download_calibration_url(&self, file_path: &str) -> String {
        format!("{}?download={}", self.base_url, file_path)
    }

    pub fn system_cmd_url(&self, cmd: &str) -> String {
        format!("{}?{}", self.base_url, Self::system_cmd_query(cmd))
    }

    /// `system_cmd=<shell command>` as the device's CGI accepts it.
    ///
    /// The CGI of firmware v5.3.99 decodes only a few escapes (`%20`, `%3E`, `%5C` verified;
    /// `%2F`, `%27`, `%7C` are passed on literally), so everything else must go raw, as Revo Scan
    /// sends it. Encoding `/` as `%2F` made every `> /dev/rk_preisp` write land in a file named
    /// `%2Fdev%2Frk_preisp` next to the CGI while the device answered `{"return":0}`
    /// (found 2026-11-10 with the DSP log, docs/protocol/device-internals.md). `&` and `#` cannot
    /// be sent at all (they end the query), and `|` is not passed through.
    pub fn system_cmd_query(cmd: &str) -> String {
        let mut encoded = String::with_capacity(cmd.len() + 16);
        for c in cmd.chars() {
            match c {
                ' ' => encoded.push_str("%20"),
                '>' => encoded.push_str("%3E"),
                '\\' => encoded.push_str("%5C"),
                _ => encoded.push(c),
            }
        }
        format!("system_cmd={encoded}")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn register_writes_keep_the_device_path_raw() {
        assert_eq!(
            RevopointCgiEndpoints::system_cmd_query("echo s 0x911 3000 > /dev/rk_preisp"),
            "system_cmd=echo%20s%200x911%203000%20%3E%20/dev/rk_preisp"
        );
        assert_eq!(RevopointCgiEndpoints::system_cmd_query("echo '123' > /tmp/x"), "system_cmd=echo%20'123'%20%3E%20/tmp/x");
    }
}
