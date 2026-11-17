//! Network discovery the way Revo Scan does it (capture 2026-11-01, `revo-modes.pcapng`): the
//! host sends the literal text `{intent:0}` (not valid JSON) by UDP to port 8080 — to the LAN
//! broadcast address and to known scanner addresses — and each scanner answers from an
//! ephemeral port to the sender with JSON:
//!
//! ```text
//! {"intent":0, "equip_id":"D00000000B0S00E1B", "ip":"192.168.178.34", "firmware":"v5.3.94.20260808",
//!  "algo":"v101.9.4.0622", "boot_mode":"normal", "network_speed":"-1Mb"}
//! ```

use serde::Deserialize;
use std::collections::BTreeMap;
use std::net::{IpAddr, Ipv4Addr, SocketAddr, ToSocketAddrs};
use std::time::Duration;
use tokio::net::UdpSocket;

pub const PORT: u16 = 8080;
pub const REQUEST: &[u8] = b"{intent:0}";

/// Scanner addresses that are not on a LAN broadcast domain we can derive: the scanner's own
/// Wi-Fi access point and USB-Ethernet mode.
const FIXED: [Ipv4Addr; 2] = [Ipv4Addr::new(192, 168, 179, 1), Ipv4Addr::new(192, 168, 3, 99)];

#[derive(Debug, Clone, Deserialize, PartialEq)]
pub struct Announcement {
    pub equip_id: String,
    pub ip: String,
    #[serde(default)]
    pub firmware: String,
    #[serde(default)]
    pub algo: String,
    #[serde(default)]
    pub boot_mode: String,
}

impl Announcement {
    pub fn parse(payload: &[u8]) -> Option<Self> {
        let a: Self = serde_json::from_slice(payload).ok()?;
        (!a.equip_id.is_empty() && a.ip.parse::<Ipv4Addr>().is_ok()).then_some(a)
    }
}

/// IPv4 addresses of this machine's adapters (from resolving the host name; loopback and
/// link-local excluded).
fn local_ipv4() -> Vec<Ipv4Addr> {
    let host = std::env::var("COMPUTERNAME")
        .or_else(|_| std::env::var("HOSTNAME"))
        .or_else(|_| std::fs::read_to_string("/etc/hostname").map(|s| s.trim().to_string()))
        .unwrap_or_else(|_| "localhost".into());
    let mut ips: Vec<Ipv4Addr> = (host.as_str(), 0)
        .to_socket_addrs()
        .map(|addrs| {
            addrs
                .filter_map(|a| match a.ip() {
                    IpAddr::V4(v4) if !v4.is_loopback() && !v4.is_link_local() => Some(v4),
                    _ => None,
                })
                .collect()
        })
        .unwrap_or_default();
    ips.sort();
    ips.dedup();
    ips
}

/// Broadcasts the discovery request from every adapter and collects answers for `wait`.
pub async fn discover(wait: Duration) -> Vec<Announcement> {
    let mut binds: Vec<Ipv4Addr> = local_ipv4();
    binds.push(Ipv4Addr::UNSPECIFIED);
    let mut sockets = Vec::new();
    for local in binds {
        let Ok(socket) = UdpSocket::bind(SocketAddr::from((local, 0))).await else { continue };
        if socket.set_broadcast(true).is_err() {
            continue;
        }
        let mut targets: Vec<Ipv4Addr> = FIXED.to_vec();
        targets.push(Ipv4Addr::BROADCAST);
        if !local.is_unspecified() {
            // Assume a /24 (home and office LANs); the limited broadcast covers the rest.
            let [a, b, c, _] = local.octets();
            targets.push(Ipv4Addr::new(a, b, c, 255));
        }
        for t in targets {
            let _ = socket.send_to(REQUEST, (t, PORT)).await;
        }
        sockets.push(socket);
    }

    let mut found: BTreeMap<String, Announcement> = BTreeMap::new();
    let deadline = tokio::time::Instant::now() + wait;
    let listen = sockets.iter().map(|socket| async move {
        let mut out = Vec::new();
        let mut buf = [0u8; 2048];
        while let Ok(Ok((n, _))) = tokio::time::timeout_at(deadline, socket.recv_from(&mut buf)).await {
            out.extend(Announcement::parse(&buf[..n]));
        }
        out
    });
    for answers in futures::future::join_all(listen).await {
        for a in answers {
            found.insert(a.equip_id.clone(), a);
        }
    }
    found.into_values().collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_pop4_announcement() {
        let reply = br#"{"intent":0, "equip_id":"D00000000B0S00E1B", "ip":"192.168.178.34", "firmware":"v5.3.94.20260808", "algo":"v101.9.4.0622", "boot_mode":"normal", "network_speed":"-1Mb"}"#;
        let a = Announcement::parse(reply).unwrap();
        assert_eq!(a.equip_id, "D00000000B0S00E1B");
        assert_eq!(a.ip, "192.168.178.34");
        assert_eq!(a.firmware, "v5.3.94.20260808");
        // Our own request echoed back by another host's broadcast is not an answer.
        assert!(Announcement::parse(REQUEST).is_none());
    }
}
