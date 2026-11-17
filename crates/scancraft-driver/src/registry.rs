use crate::traits::{ScannerDevice, ScannerDriver};
use crate::types::{DeviceTransport, DiscoveredDevice, DriverError};
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::RwLock;

/// Global registry managing available scanner drivers
#[derive(Clone, Default)]
pub struct DriverRegistry {
    drivers: Arc<RwLock<HashMap<String, Arc<dyn ScannerDriver>>>>,
}

impl DriverRegistry {
    pub fn new() -> Self {
        Self::default()
    }

    /// Register a new scanner driver implementation
    pub async fn register(&self, driver: Arc<dyn ScannerDriver>) {
        let mut map = self.drivers.write().await;
        map.insert(driver.id().to_string(), driver);
    }

    /// List all registered driver IDs
    pub async fn registered_drivers(&self) -> Vec<String> {
        let map = self.drivers.read().await;
        map.keys().cloned().collect()
    }

    /// Perform a discovery scan across all registered drivers
    pub async fn discover_all(&self) -> Vec<DiscoveredDevice> {
        let drivers = {
            let map = self.drivers.read().await;
            map.values().cloned().collect::<Vec<_>>()
        };

        let mut all_devices = Vec::new();
        for driver in drivers {
            match driver.discover().await {
                Ok(devices) => all_devices.extend(devices),
                Err(e) => {
                    log::warn!("Driver '{}' discovery error: {}", driver.id(), e);
                }
            }
        }
        all_devices
    }

    /// Connect to a device using its driver ID and chosen transport
    pub async fn connect(
        &self,
        driver_id: &str,
        transport: &DeviceTransport,
    ) -> Result<Box<dyn ScannerDevice>, DriverError> {
        let driver = {
            let map = self.drivers.read().await;
            map.get(driver_id).cloned()
        };

        match driver {
            Some(d) => d.connect(transport).await,
            None => Err(DriverError::DeviceNotFound(format!(
                "No registered driver with ID '{}'",
                driver_id
            ))),
        }
    }
}
