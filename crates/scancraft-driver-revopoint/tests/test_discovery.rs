use scancraft_driver::ScannerDriver;
use scancraft_driver_revopoint::RevopointDriver;

#[tokio::test]
async fn test_revopoint_discovery() {
    let driver = RevopointDriver;
    let devices = driver.discover().await.expect("Discovery execution should succeed");
    if devices.is_empty() {
        println!("Note: Physical POP 4 scanner is currently offline or unreachable on Wi-Fi.");
    } else {
        let dev = &devices[0];
        assert_eq!(dev.model_name, "Revopoint POP 4");
        assert!(!dev.serial_number.is_empty());
        println!("Discovered Revopoint scanner: {} (SN: {})", dev.model_name, dev.serial_number);
    }
}
