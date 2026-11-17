"""
Real-time IMU Telemetry client for Revopoint 3D Scanners (Port 9999).
"""

import socket
import struct
import threading
from dataclasses import dataclass
from typing import Callable, Optional, Tuple


@dataclass
class IMUSample:
    timestamp_ms: int
    quaternion: Tuple[float, float, float, float]  # (w, x, y, z)
    gyro: Tuple[float, float, float]  # (gx, gy, gz)
    accel: Tuple[float, float, float]  # (ax, ay, az) in Gs
    attitude: Tuple[float, float, float]  # (mx, my, mz) / Euler


class IMUClient:
    """
    Client for receiving real-time 9-axis IMU telemetry on port 9999.
    """

    def __init__(self, host: str, port: int = 9999):
        self.host = host
        self.port = port
        self._sock: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._callback: Optional[Callable[[IMUSample], None]] = None

    def start(self, callback: Callable[[IMUSample], None]):
        """
        Connects to scanner IMU stream and runs listener thread.
        """
        self._callback = callback
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.connect((self.host, self.port))
        self._running = True
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()

    def stop(self):
        """
        Stops telemetry receiver.
        """
        self._running = False
        if self._sock:
            try:
                self._sock.close()
            except Exception:
                pass
            self._sock = None

    def _worker(self):
        buf = bytearray()
        while self._running and self._sock:
            try:
                chunk = self._sock.recv(1024)
                if not chunk:
                    break
                buf.extend(chunk)

                while len(buf) >= 60:
                    raw = buf[:60]
                    del buf[:60]

                    ts = struct.unpack_from("<Q", raw, 0)[0]
                    floats = struct.unpack_from("<13f", raw, 8)

                    sample = IMUSample(
                        timestamp_ms=ts,
                        quaternion=(floats[0], floats[1], floats[2], floats[3]),
                        gyro=(floats[4], floats[5], floats[6]),
                        accel=(floats[7], floats[8], floats[9]),
                        attitude=(floats[10], floats[11], floats[12]),
                    )

                    if self._callback:
                        self._callback(sample)
            except Exception:
                break
        self._running = False
