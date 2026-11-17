#!/usr/bin/env python3
"""
ScanCraft Hardware Probing Utility
----------------------------------
Use this tool to discover and inspect unknown or third-party 3D scanners across
local network interfaces (Ethernet / Wi-Fi) and USB ports.

Outputs a standardized `scanner_probe_report.json` detailing:
- Network open ports, HTTP banner info, API responses
- USB Vendor ID (VID), Product ID (PID), and interface classes
- IMU/telemetry candidate sockets
"""

import json
import os
import platform
import socket
import sys
import urllib.request
import urllib.error

COMMON_SCANNER_PORTS = [80, 8080, 8871, 9999, 5000, 554, 8000, 8088, 1234, 3000]

COMMON_HTTP_ENDPOINTS = [
    "/cgi-bin/zx_cmd.cgi?get_sn",
    "/cgi-bin/zx_cmd.cgi?get_device_info",
    "/info",
    "/api/v1/device",
    "/status",
    "/version",
    "/camera/parameters",
]


def probe_network_target(ip: str) -> dict:
    print(f"[*] Probing network target: {ip}...")
    report = {"ip": ip, "open_ports": [], "http_endpoints": {}}

    for port in COMMON_SCANNER_PORTS:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(0.4)
        try:
            res = s.connect_ex((ip, port))
            if res == 0:
                report["open_ports"].append(port)
                print(f"  [+] Port {port} is OPEN")
        except Exception:
            pass
        finally:
            s.close()

    # If port 80 or 8080 is open, test known HTTP endpoints
    for web_port in [80, 8080]:
        if web_port in report["open_ports"]:
            base_url = f"http://{ip}:{web_port}"
            for ep in COMMON_HTTP_ENDPOINTS:
                url = base_url + ep
                try:
                    req = urllib.request.Request(url, headers={"User-Agent": "ScanCraft-Probe/1.0"})
                    with urllib.request.urlopen(req, timeout=1.0) as resp:
                        content = resp.read().decode("utf-8", errors="replace").strip()
                        if content:
                            report["http_endpoints"][ep] = content[:300]
                            print(f"  [+] Discovered HTTP endpoint {ep} -> {content[:80]}")
                except Exception:
                    pass

    return report


def probe_usb_devices() -> list:
    print("[*] Probing local USB devices...")
    devices = []
    if platform.system() == "Windows":
        try:
            import subprocess
            cmd = "Get-PnpDevice -PresentOnly | Where-Object { $_.InstanceId -match 'USB\\\\VID_' } | Select-Object FriendlyName, InstanceId, Status | ConvertTo-Json"
            out = subprocess.check_output(["powershell", "-NoProfile", "-Command", cmd], text=True)
            if out.strip():
                raw = json.loads(out)
                if isinstance(raw, dict):
                    raw = [raw]
                for dev in raw:
                    devices.append({
                        "name": dev.get("FriendlyName"),
                        "instance_id": dev.get("InstanceId"),
                        "status": dev.get("Status"),
                    })
        except Exception as e:
            print(f"  [-] USB probe error: {e}")
    return devices


def main():
    target_ip = sys.argv[1] if len(sys.argv) > 1 else "192.168.178.34"
    print(f"=== ScanCraft Hardware Probe v1.0 ===")
    print(f"Target Default IP: {target_ip}")
    print()

    net_report = probe_network_target(target_ip)
    usb_report = probe_usb_devices()

    final_report = {
        "network_probe": net_report,
        "usb_probe": usb_report,
        "system": {
            "os": platform.system(),
            "release": platform.release(),
            "machine": platform.machine(),
        },
    }

    report_path = "scanner_probe_report.json"
    with open(report_path, "w", encoding="utf-8") as f:
        json.dump(final_report, f, indent=2)

    print()
    print(f"[✓] Probe complete! Report saved to '{report_path}'.")
    print("Submit this report or use its parameters to create a new `scancraft-driver-*` module.")


if __name__ == "__main__":
    main()
