use crate::device::RevopointDevice;
use async_trait::async_trait;
use scancraft_driver::{
    DeviceTransport, DiscoveredDevice, DriverError, ScannerDevice, ScannerDriver,
};
use std::time::Duration;

pub struct RevopointDriver;

#[async_trait]
impl ScannerDriver for RevopointDriver {
    fn id(&self) -> &'static str {
        "revopoint"
    }

    fn name(&self) -> &'static str {
        "Revopoint 3D Scanners (POP, Inspire, Range, Miraco)"
    }

    async fn discover(&self) -> Result<Vec<DiscoveredDevice>, DriverError> {
        let found = crate::discovery::discover(Duration::from_millis(1500)).await;
        Ok(found
            .into_iter()
            .map(|a| {
                let transport = DeviceTransport::WiFi { ip: a.ip.clone(), control_port: 80, data_port: 8871, imu_port: 9999 };
                DiscoveredDevice {
                    driver_id: "revopoint".to_string(),
                    // Serial numbers carry the model code ("0E1" = POP 4, see docs/protocol/modes.md).
                    model_name: if a.equip_id.contains("0E1") { "Revopoint POP 4" } else { "Revopoint scanner" }.to_string(),
                    serial_number: a.equip_id,
                    available_transports: vec![transport.clone()],
                    selected_transport: transport,
                }
            })
            .collect())
    }

    async fn connect(&self, transport: &DeviceTransport) -> Result<Box<dyn ScannerDevice>, DriverError> {
        match transport {
            DeviceTransport::WiFi { ip, .. } => {
                let sn = crate::control::Control::new(ip)?.cgi("get_sn").await?;

                let dev = RevopointDevice::new_wifi(ip, sn.trim().to_string()).await?;
                Ok(Box::new(dev))
            }
            DeviceTransport::Usb { vid, pid, .. } => {
                Err(DriverError::Other(format!(
                    "USB transport ({:04X}:{:04X}) is not implemented for Revopoint yet; connect over Wi-Fi",
                    vid, pid
                )))
            }
            DeviceTransport::Virtual { .. } => {
                Err(DriverError::InvalidParameter("Virtual transport not supported by Revopoint driver".to_string()))
            }
        }
    }
}
