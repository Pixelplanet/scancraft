"""
High-level Revopoint POP 4 Scanner Client.
Interacts via HTTP CGI commands (/cgi-bin/zx_cmd.cgi), telemetry, and stream control.
"""

import json
import re
from typing import Callable, Dict, Optional
import requests
import yaml

from .imu import IMUClient, IMUSample


class RevopointPOP4:
    """
    Client for interacting with Revopoint POP 4 3D Scanner over Wi-Fi / Ethernet.
    """

    def __init__(self, host: str = "192.168.178.34", timeout: float = 4.0):
        self.host = host
        self.timeout = timeout
        self.cgi_url = f"http://{self.host}/cgi-bin/zx_cmd.cgi"
        self._imu_client: Optional[IMUClient] = None

    def _get(self, query: str) -> requests.Response:
        url = f"{self.cgi_url}?{query}" if query else self.cgi_url
        return requests.get(url, timeout=self.timeout)

    def get_serial_number(self) -> str:
        """
        Retrieves device serial number.
        """
        r = self._get("get_sn")
        r.raise_for_status()
        return r.text.strip()

    def get_temperatures(self) -> Dict[str, float]:
        """
        Retrieves main sensor temperature and NTC thermistor temperatures in deg C.
        """
        r_th = self._get("get_thermal")
        r_ntc = self._get("get_ntcTemperature")

        thermal = None
        ntc = []

        try:
            th_data = json.loads(r_th.text.strip())
            thermal = th_data.get("thermal", 0) / 1000.0
        except Exception:
            pass

        try:
            ntc_data = json.loads(r_ntc.text.strip())
            ntc = [t / 1000.0 for t in ntc_data.get("ntc", [])]
        except Exception:
            pass

        return {
            "sensor_degC": thermal,
            "ntc_degC": ntc,
        }

    def get_exposure_range(self) -> Dict[str, int]:
        """
        Retrieves stereo IR exposure bounds in microseconds.
        """
        r = self._get("cam_type=mipi&get_exposureRange")
        r.raise_for_status()
        return json.loads(r.text.strip())

    def get_supported_resolutions(self) -> Dict[str, str]:
        """
        Parses supported sensor stream resolutions from MIPI sensor bus.
        """
        r = self._get("cam_type=mipi&get_depth_reso")
        r.raise_for_status()

        # Output is concatenated JSON objects: {"k":"v"}{"k":"v"}
        matches = re.findall(r'\{"([^"]+)":"([^"]+)"\}', r.text)
        return {k: v for k, v in matches}

    def download_calibration_yaml(self) -> str:
        """
        Downloads raw OpenCV calibration YAML from /data/camparam/camparam.yaml.
        """
        r = self._get("download=/data/camparam/camparam.yaml")
        r.raise_for_status()
        return r.text

    def get_calibration_matrices(self) -> Dict[str, dict]:
        """
        Downloads and parses OpenCV camera intrinsics, stereo baseline, and Q matrix.
        """
        raw_yaml = self.download_calibration_yaml()
        # Clean OpenCV custom tags and non-standard directive
        clean = re.sub(r'%YAML:1\.0', '%YAML 1.1', raw_yaml)
        clean = re.sub(r'!!opencv-matrix', '', clean)
        data = yaml.safe_load(clean)
        return data

    def start_imu_stream(self, callback: Callable[[IMUSample], None]) -> IMUClient:
        """
        Connects to Port 9999 and starts background IMU telemetry stream.
        """
        if self._imu_client:
            self._imu_client.stop()
        self._imu_client = IMUClient(self.host, 9999)
        self._imu_client.start(callback)
        return self._imu_client

    def stop_imu_stream(self):
        """
        Stops active IMU telemetry client.
        """
        if self._imu_client:
            self._imu_client.stop()
            self._imu_client = None

    def close_all_streams(self) -> bool:
        """
        Instructs scanner to close all video, depth, and preview streams.
        """
        r = self._get("close_stream_all")
        return "{result:0}" in r.text or '{"result":0}' in r.text

    def set_isp_register(self, cmd: str) -> bool:
        """
        Sends low-level hardware command to Rockchip Pre-ISP (/dev/rk_preisp).
        """
        r = self._get(f"system_cmd={requests.utils.quote(cmd)}")
        return '{"return":0}' in r.text or '{"return": 0}' in r.text

    def set_projector_power(self, enable: bool) -> bool:
        """
        Turns the structured light / laser projector power ON (True) or OFF (False).
        """
        val = 1 if enable else 0
        return self.set_isp_register(f"echo s 0x912 {val} > /dev/rk_preisp")

    def set_speckle_luminance(self, luminance: int) -> bool:
        """
        Sets the speckle projector light intensity (range 1..255).
        """
        lum = max(1, min(255, luminance))
        return self.set_isp_register(f"echo s 0xb1b {lum} > /dev/rk_preisp")

    def set_ir_led_luminance(self, luminance: int) -> bool:
        """
        Sets the infrared fill LED brightness for marker & dark object tracking (range 0..255).
        """
        lum = max(0, min(255, luminance))
        return self.set_isp_register(f"echo s 0xb07 {lum} > /dev/rk_preisp")

    def set_rgb_led_luminance(self, luminance: int) -> bool:
        """
        Sets the white flash / fill light LED brightness for color texture mapping (range 0..255).
        """
        lum = max(0, min(255, luminance))
        return self.set_isp_register(f"echo s 0xb12 {lum} > /dev/rk_preisp")

    def set_laser_diode_enable(self, enable: bool) -> bool:
        """
        Enables or disables laser diode emission (active-low register 0xb04: 0 = ON, 1 = OFF).
        """
        val = 0 if enable else 1
        return self.set_isp_register(f"echo s 0xb04 {val} > /dev/rk_preisp")

    def set_laser_mode(self, mode: str = "cross") -> bool:
        """
        Configures scanner optical mode.
        Supported modes:
          - 'cross': Cross Line Blue Laser (0x3010 0x17 1, 0x3001 7)
          - 'parallel': Parallel Blue Lines (0x3010 0x17 1, 0x3001 8)
          - 'single': Single Blue Line (0x3010 0x17 1, 0x3001 9)
          - 'speckle': Infrared Structured Light Speckle (0x3010 0x17 0, 0x3001 12)
        """
        m = mode.lower()
        if m in ("cross", "cross_line"):
            ok1 = self.set_isp_register("echo s 0x3010 0x17 1 > /dev/rk_preisp")
            ok2 = self.set_isp_register("echo s 0x3001 7 > /dev/rk_preisp")
            return ok1 and ok2
        elif m in ("parallel", "parallel_lines"):
            ok1 = self.set_isp_register("echo s 0x3010 0x17 1 > /dev/rk_preisp")
            ok2 = self.set_isp_register("echo s 0x3001 8 > /dev/rk_preisp")
            return ok1 and ok2
        elif m in ("single", "single_line"):
            ok1 = self.set_isp_register("echo s 0x3010 0x17 1 > /dev/rk_preisp")
            ok2 = self.set_isp_register("echo s 0x3001 9 > /dev/rk_preisp")
            return ok1 and ok2
        elif m in ("speckle", "ir"):
            ok1 = self.set_isp_register("echo s 0x3010 0x17 0 > /dev/rk_preisp")
            ok2 = self.set_isp_register("echo s 0x3001 12 > /dev/rk_preisp")
            return ok1 and ok2
        else:
            raise ValueError(f"Unknown laser mode: {mode}. Expected 'cross', 'parallel', 'single', or 'speckle'.")

    def set_laser_luminance(self, luminance: int, mode: str = "cross") -> bool:
        """
        Sets laser diode optical output power (range 1..255).
        Line mode index: 1 = Cross, 2 = Parallel, 3 = Single.
        """
        lum = max(1, min(255, luminance))
        mode_idx = 1
        m = mode.lower()
        if "parallel" in m:
            mode_idx = 2
        elif "single" in m:
            mode_idx = 3
        return self.set_isp_register(f"echo s 0x48d {lum} {mode_idx} > /dev/rk_preisp")

    def set_laser_exposure(self, exposure_us: int, mode: str = "cross") -> bool:
        """
        Sets camera exposure specifically for laser line capture in microseconds.
        Line mode index: 1 = Cross, 2 = Parallel, 3 = Single.
        """
        exp_us = max(500, min(50000, exposure_us))
        mode_idx = 1
        m = mode.lower()
        if "parallel" in m:
            mode_idx = 2
        elif "single" in m:
            mode_idx = 3
        return self.set_isp_register(f"echo s 0xb08 {exp_us} {mode_idx} > /dev/rk_preisp")

    def download_blue_calibration(self) -> Dict[str, bytes]:
        """
        Downloads all blue laser calibration files from /data/camparam_blue/.
        Returns dictionary mapping filename to raw bytes.
        """
        files = ["camparam.yaml", "metroExtra.bin", "mapparamL.bin", "mapparamR.bin", "camparamLR/Q.bin"]
        result = {}
        for f in files:
            r = self._get(f"download=/data/camparam_blue/{f}")
            if r.status_code == 200 and not r.text.startswith("{download:"):
                result[f] = r.content
        return result


