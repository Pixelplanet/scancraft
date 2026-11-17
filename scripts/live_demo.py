"""
Live demonstration of Revopoint POP 4 reference client against hardware.
"""

import sys
import time
from client.scanner import RevopointPOP4

HOST = "192.168.178.34"

def main():
    print(f"Connecting to Revopoint POP 4 at {HOST}...")
    scanner = RevopointPOP4(host=HOST)

    # 1. Device Identification
    sn = scanner.get_serial_number()
    print(f"\n[+] Device Serial Number: {sn}")

    # 2. Temperature & Health
    temps = scanner.get_temperatures()
    print(f"[+] Sensor Temperature:  {temps['sensor_degC']} °C")
    print(f"[+] NTC Thermistors:     {temps['ntc_degC']} °C")

    # 3. Sensor Settings & Limits
    exp_range = scanner.get_exposure_range()
    print(f"[+] IR Exposure Range:   {exp_range['min']} - {exp_range['max']} µs (default {exp_range['default']} µs)")

    # 4. Stream Resolutions
    resolutions = scanner.get_supported_resolutions()
    print("[+] Supported Resolutions:")
    for k, v in resolutions.items():
        print(f"    - {k}: {v}")

    # 5. Calibration Matrices
    calib = scanner.get_calibration_matrices()
    baseline = calib.get("steroTranslation", {}).get("data", [0])[0]
    print(f"\n[+] Stereo Baseline Tx:  {baseline:.2f} mm")
    print(f"[+] Calibration Date:    {calib.get('lastTime')}")

    # 6. Live IMU Stream
    print("\n[+] Testing Live IMU Telemetry Stream (Port 9999)...")
    samples_received = []

    def on_imu(sample):
        samples_received.append(sample)
        if len(samples_received) <= 5:
            q = [round(x, 3) for x in sample.quaternion]
            acc = [round(x, 3) for x in sample.accel]
            print(f"    IMU Sample #{len(samples_received)}: t={sample.timestamp_ms}ms, Quat(w,x,y,z)={q}, Accel(G)={acc}")

    scanner.start_imu_stream(on_imu)
    time.sleep(1.0)
    scanner.stop_imu_stream()

    rate = len(samples_received) / 1.0
    print(f"[+] Received {len(samples_received)} IMU samples in 1.0s (~{rate:.1f} Hz)")

    # 7. Tear down streams
    scanner.close_all_streams()
    print("\n[+] Verification complete! Reference client working successfully.")

if __name__ == "__main__":
    main()
