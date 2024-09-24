use std::sync::mpsc;

mod constants;
mod device_info;
mod driver_threads;
mod error;
mod flags;
mod numeric;
mod packet;
mod scan;
mod serial;
mod time;
mod ydlidar_models;

use crate::constants::{
    HEADER_SIZE, LIDAR_ANS_LENGTH_DEVHEALTH, LIDAR_ANS_LENGTH_DEVINFO, LIDAR_ANS_TYPE_DEVHEALTH,
    LIDAR_ANS_TYPE_DEVINFO, LIDAR_CMD_GET_DEVICE_HEALTH, LIDAR_CMD_GET_DEVICE_INFO
};
use crate::driver_threads::{parse_packets, read_device_signal, DriverThreads};
use crate::flags::{InterferenceFlag};
use crate::packet::validate_response_header;
use crate::scan::Scan;
use crate::serial::{read, send_command, start_scan, stop_scan_and_flush};
use crate::time::sleep_ms;
use crate::ydlidar_models::{model_baud_rate};
use crossbeam_channel::{bounded};
use serialport::SerialPort;

pub use crate::ydlidar_models::YdlidarModels;
pub use crate::error::YDLidarError;

pub fn check_device_health(port: &mut Box<dyn SerialPort>) -> Result<(), YDLidarError> {
    send_command(port, LIDAR_CMD_GET_DEVICE_HEALTH)?;
    let header = read(port, HEADER_SIZE)?;
    validate_response_header(
        &header,
        Some(LIDAR_ANS_LENGTH_DEVHEALTH),
        LIDAR_ANS_TYPE_DEVHEALTH,
    )?;
    let health = read(port, LIDAR_ANS_LENGTH_DEVHEALTH.into())?;

    match health[0] {
        0 => Ok(()),
        _ => Err(YDLidarError::DeviceHealthError(health[0].into())),
    }
}

pub fn get_device_info(
    port: &mut Box<dyn SerialPort>,
) -> Result<device_info::DeviceInfo, YDLidarError> {
    send_command(port, LIDAR_CMD_GET_DEVICE_INFO)?;
    let header = read(port, HEADER_SIZE)?;
    validate_response_header(
        &header,
        Some(LIDAR_ANS_LENGTH_DEVINFO),
        LIDAR_ANS_TYPE_DEVINFO,
    )?;
    let info = read(port, LIDAR_ANS_LENGTH_DEVINFO.into())?;
    Ok(device_info::DeviceInfo {
        model_number: info[0],
        firmware_major_version: info[2],
        firmware_minor_version: info[1],
        hardware_version: info[3],
        serial_number: info[4..20].try_into().unwrap(),
    })
}

/// Function to launch YDLiDAR.
/// # Arguments
///
/// * `port_name` - Serial port name such as `/dev/ttyUSB0`.
/// * `model` - Model
pub fn run_driver(
    port_name: &str,
    model: YdlidarModels,
) -> Result<(DriverThreads, mpsc::Receiver<Scan>), YDLidarError> {
    let baud_rate = model_baud_rate(model);
    let maybe_port = serialport::new(port_name, baud_rate)
        .timeout(std::time::Duration::from_millis(10))
        .open();

    let mut port = match maybe_port {
        Ok(port) => port,
        Err(e) => {
            eprintln!("Failed to open \"{}\". Error: {}", port_name, e);
            std::process::exit(1);
        }
    };

    if !cfg!(test) {
        // In testing, disable flushing to receive dummy signals
        stop_scan_and_flush(&mut port)?;
        sleep_ms(10);
        stop_scan_and_flush(&mut port)?;
    }

    check_device_health(&mut port)?;
    get_device_info(&mut port)?;

    let (reader_terminator_tx, reader_terminator_rx) = bounded(10);
    let (parser_terminator_tx, parser_terminator_rx) = bounded(10);
    let (scan_data_tx, scan_data_rx) = mpsc::sync_channel::<Vec<u8>>(200);

    start_scan(&mut port)?;

    let reader_thread = Some(std::thread::spawn(move || {
        read_device_signal(&mut port, scan_data_tx, reader_terminator_rx);
    }));

    let (scan_tx, scan_rx) = mpsc::sync_channel::<Scan>(10);
    let receiver_thread = Some(std::thread::spawn(move || {
        parse_packets(scan_data_rx, parser_terminator_rx, scan_tx);
    }));

    let driver_threads = DriverThreads {
        reader_thread,
        receiver_thread,
        reader_terminator_tx,
        parser_terminator_tx,
    };

    Ok((driver_threads, scan_rx))
}

#[cfg(test)]
mod tests {
    use std::io::{Read, Write};
    use super::*;
    use serialport::TTYPort;
    use crate::flags::to_flag;
    use crate::serial::flush;

    fn radian_to_degree(e: f64) -> f64 {
        e * 180. / std::f64::consts::PI
    }

    #[test]
    fn test_to_flag() {
        assert_eq!(to_flag(2), InterferenceFlag::SpecularReflection);
        assert_eq!(to_flag(3), InterferenceFlag::AmbientLight);
        assert_eq!(to_flag(1), InterferenceFlag::Nothing);
    }

    #[test]
    fn test_validate_response_header() {
        assert!(
            matches!(
                validate_response_header(
                    &vec![0xA5, 0x5A, 0x14, 0x00, 0x00, 0x00, 0x04],
                    Some(0x14),
                    0x04
                ),
                Ok(())
            )
        );

        assert!(
            matches!(
                validate_response_header(
                    &vec![0xA5, 0x5A, 0x14, 0x00, 0x00, 0x00, 0x04, 0x09],
                    Some(0x14),
                    0x04
                ),
                Err(YDLidarError::InvalidHeaderLength(8))
            )
        );

        assert!(
            matches!(
                validate_response_header(
                    &vec![0xA6, 0x5A, 0x14, 0x00, 0x00, 0x00, 0x04],
                    Some(0x14),
                    0x04
                ),
                Err(YDLidarError::InvalidMagicNumber(_))
            )
        );

        assert!(
            matches!(
                validate_response_header(
                    &vec![0xA5, 0x2A, 0x14, 0x00, 0x00, 0x00, 0x04],
                    Some(0x14),
                    0x04
                ),
                Err(YDLidarError::InvalidMagicNumber(_))
            )
        );

        assert!(
            matches!(
                validate_response_header(
                    &vec![0xA5, 0x5A, 0x14, 0x00, 0x00, 0x00, 0x04],
                    Some(0x12),
                    0x04
                ),
                Err(YDLidarError::InvalidResponseLength(18, 20))
            )
        );

        assert!(
            matches!(
                validate_response_header(
                    &vec![0xA5, 0x5A, 0x14, 0x00, 0x00, 0x00, 0x08],
                    Some(0x14),
                    0x04
                ),
                Err(YDLidarError::InvalidTypeCode(4, 8))
            )
        );
    }

    #[test]
    fn test_send_command() {
        let (master, mut slave) = TTYPort::pair().expect("Unable to create ptty pair");
        let mut master_ptr = Box::new(master) as Box<dyn SerialPort>;
        send_command(&mut master_ptr, 0x68).unwrap();

        sleep_ms(10);
        let mut buf = [0u8; 2];
        slave.read(&mut buf).unwrap();
        assert_eq!(buf, [0xA5, 0x68]);
    }

    #[test]
    fn test_check_device_health() {
        let (mut master, slave) = TTYPort::pair().expect("Unable to create ptty pair");
        let mut slave_ptr = Box::new(slave) as Box<dyn SerialPort>;

        master
            .write(&[0xA5, 0x5A, 0x03, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00])
            .unwrap();
        sleep_ms(10);
        assert!(matches!(check_device_health(&mut slave_ptr), Ok(())));

        master
            .write(&[0xA5, 0x5A, 0x03, 0x00, 0x00, 0x00, 0x06, 0x02, 0x00, 0x00])
            .unwrap();
        sleep_ms(10);
        assert!(
            matches!(
                check_device_health(&mut slave_ptr),
                Err(YDLidarError::DeviceHealthError(0x02))
            )
        );
    }

    #[test]
    fn test_flush() {
        let (mut master, slave) = TTYPort::pair().expect("Unable to create ptty pair");
        master
            .write(&[0xA5, 0x5A, 0x03, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00])
            .unwrap();

        let mut slave_ptr = Box::new(slave) as Box<dyn SerialPort>;

        sleep_ms(10);

        assert_eq!(slave_ptr.bytes_to_read().unwrap(), 10);
        flush(&mut slave_ptr).unwrap();
        assert_eq!(slave_ptr.bytes_to_read().unwrap(), 0);

        // when zero bytes to read
        flush(&mut slave_ptr).unwrap();
        assert_eq!(slave_ptr.bytes_to_read().unwrap(), 0);
    }

    #[test]
    fn test_get_device_info() {
        let (mut master, slave) = TTYPort::pair().expect("Unable to create ptty pair");
        master
            .write(&[
                0xA5, 0x5A, 0x14, 0x00, 0x00, 0x00, 0x04, 0x96, 0x00, 0x01, 0x02, 0x02, 0x00, 0x02,
                0x02, 0x01, 0x01, 0x00, 0x03, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
            ])
            .unwrap();

        sleep_ms(10);

        let mut slave_ptr = Box::new(slave) as Box<dyn SerialPort>;
        let info = get_device_info(&mut slave_ptr).unwrap();
        assert_eq!(info.model_number, 150);
        assert_eq!(info.firmware_major_version, 1);
        assert_eq!(info.firmware_minor_version, 0);
        assert_eq!(info.hardware_version, 2);
        assert_eq!(
            info.serial_number,
            [2, 0, 2, 2, 1, 1, 0, 3, 0, 1, 1, 1, 1, 1, 1, 1]
        );
    }


    #[test]
    fn test_run_driver_normal_data() {
        let (mut master, slave) = TTYPort::pair().expect("Unable to create ptty pair");

        let device_health_packet = [0xA5, 0x5A, 0x03, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00];
        master.write(&device_health_packet).unwrap();

        let device_info_packet = [
            0xA5, 0x5A, 0x14, 0x00, 0x00, 0x00, 0x04, 0x96, 0x00, 0x01, 0x02, 0x02, 0x00, 0x02,
            0x02, 0x01, 0x01, 0x00, 0x03, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
        ];
        master.write(&device_info_packet).unwrap();

        let start_scan_response_header = [0xA5, 0x5A, 0x05, 0x00, 0x00, 0x40, 0x81];
        master.write(&start_scan_response_header).unwrap();

        sleep_ms(10);

        let name = slave.name().unwrap();
        let (thread, scan_rx) = run_driver(&name, YdlidarModels::TMiniPro).unwrap();

        let packet = [
            // beginning of a lap
            0xAA, 0x55, 0xC7, 0x01, 0x01, 0x15, 0x01, 0x15, 0x1B, 0x56, // packet header
            0x14, 0x62, 0x02, // SpecularReflection
            // lap data
            0xAA, 0x55, 0xB0, 0x10, 0x81, 0x16, 0x01, 0x2D, 0x57, 0x7D, // packet header
            0xDD, 0x76, 0x03, // Specular Reflection
            0xD4, 0x76, 0x03, // Specular Reflection
            0xC3, 0x72, 0x03, // Specular Reflection
            0xB3, 0x7B, 0x03, // Ambient Light
            0x8E, 0x8A, 0x03, // Specular Reflection
            0x97, 0x6E, 0x04, // Specular Reflection
            0x9C, 0x22, 0x05, // Specular Reflection
            0xA7, 0x6A, 0x05, // Specular Reflection
            0xAB, 0x7A, 0x05, // Specular Reflection
            0x93, 0x82, 0x05, // Specular Reflection
            0x6D, 0xC2, 0x05, // Specular Reflection
            0x55, 0xA6, 0x05, // Specular Reflection
            0x57, 0x16, 0x05, // Specular Reflection
            0x67, 0x62, 0x02, // Specular Reflection
            0x80, 0x16, 0x02, // Specular Reflection
            0x9B, 0xE6, 0x01, // Specular Reflection
            // new lap
            0xAA, 0x55, 0xC7, 0x01, 0x81, 0x2E, 0x81, 0x2E, 0x1B, 0x56, // packet header
            0x14, 0x62, 0x02, // This signal will be regarded as a new lap
        ];
        master.write(&packet).unwrap();

        let scan = scan_rx.recv().unwrap();
        assert_eq!(scan.angles_radian.len(), 0);

        let scan = scan_rx.recv().unwrap();
        assert_eq!(scan.angles_radian.len(), 17);

        let expected = vec![
            42., 45., 48., 51., 54., 57., 60., 63., 66., 69., 72., 75., 78., 81., 84., 87., 90.,
        ];

        assert_eq!(scan.angles_radian.len(), expected.len());
        for i in 0..expected.len() {
            let degree = radian_to_degree(scan.angles_radian[i]);
            assert!(f64::abs(degree - expected[i]) < 1e-8);
        }

        let expected = vec![
            0x14, 0xDD, 0xD4, 0xC3, 0xB3, 0x8E, 0x97, 0x9C, 0xA7, 0xAB, 0x93, 0x6D, 0x55, 0x57,
            0x67, 0x80, 0x9B,
        ];
        assert_eq!(scan.intensities, expected);

        let expected = vec![
            ((0x62 as u16) >> 2) + ((0x02 as u16) << 6),
            ((0x76 as u16) >> 2) + ((0x03 as u16) << 6),
            ((0x76 as u16) >> 2) + ((0x03 as u16) << 6),
            ((0x72 as u16) >> 2) + ((0x03 as u16) << 6),
            ((0x7B as u16) >> 2) + ((0x03 as u16) << 6),
            ((0x8A as u16) >> 2) + ((0x03 as u16) << 6),
            ((0x6E as u16) >> 2) + ((0x04 as u16) << 6),
            ((0x22 as u16) >> 2) + ((0x05 as u16) << 6),
            ((0x6A as u16) >> 2) + ((0x05 as u16) << 6),
            ((0x7A as u16) >> 2) + ((0x05 as u16) << 6),
            ((0x82 as u16) >> 2) + ((0x05 as u16) << 6),
            ((0xC2 as u16) >> 2) + ((0x05 as u16) << 6),
            ((0xA6 as u16) >> 2) + ((0x05 as u16) << 6),
            ((0x16 as u16) >> 2) + ((0x05 as u16) << 6),
            ((0x62 as u16) >> 2) + ((0x02 as u16) << 6),
            ((0x16 as u16) >> 2) + ((0x02 as u16) << 6),
            ((0xE6 as u16) >> 2) + ((0x01 as u16) << 6),
        ];
        assert_eq!(scan.distances, expected);

        let expected = vec![
            InterferenceFlag::SpecularReflection,
            InterferenceFlag::SpecularReflection,
            InterferenceFlag::SpecularReflection,
            InterferenceFlag::SpecularReflection,
            InterferenceFlag::AmbientLight,
            InterferenceFlag::SpecularReflection,
            InterferenceFlag::SpecularReflection,
            InterferenceFlag::SpecularReflection,
            InterferenceFlag::SpecularReflection,
            InterferenceFlag::SpecularReflection,
            InterferenceFlag::SpecularReflection,
            InterferenceFlag::SpecularReflection,
            InterferenceFlag::SpecularReflection,
            InterferenceFlag::SpecularReflection,
            InterferenceFlag::SpecularReflection,
            InterferenceFlag::SpecularReflection,
            InterferenceFlag::SpecularReflection,
        ];
        assert_eq!(scan.flags, expected);

        assert!(scan.checksum_correct);

        drop(thread);
    }

    #[test]
    fn test_run_driver_mod_at_360() {
        let (mut master, slave) = TTYPort::pair().expect("Unable to create ptty pair");

        let device_health_packet = [0xA5, 0x5A, 0x03, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00];
        master.write(&device_health_packet).unwrap();

        let device_info_packet = [
            0xA5, 0x5A, 0x14, 0x00, 0x00, 0x00, 0x04, 0x96, 0x00, 0x01, 0x02, 0x02, 0x00, 0x02,
            0x02, 0x01, 0x01, 0x00, 0x03, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
        ];
        master.write(&device_info_packet).unwrap();

        let start_scan_response_header = [0xA5, 0x5A, 0x05, 0x00, 0x00, 0x40, 0x81];
        master.write(&start_scan_response_header).unwrap();

        sleep_ms(10);

        let name = slave.name().unwrap();
        let (thread, scan_rx) = run_driver(&name, YdlidarModels::TMiniPro).unwrap();

        let packet = [
            // lap data
            0xAA, 0x55, 0xB0, 0x10, 0x01, 0x96, 0x01, 0x0F, 0xD6, 0xDF, 0xDD, 0x76, 0x03, 0xD4,
            0x76, 0x03, 0xC3, 0x72, 0x03, 0xB3, 0x7A, 0x03, 0x8E, 0x8A, 0x03, 0x97, 0x6E, 0x04,
            0x9C, 0x22, 0x05, 0xA7, 0x6A, 0x05, 0xAB, 0x7A, 0x05, 0x93, 0x82, 0x05, 0x6D, 0xC2,
            0x05, 0x55, 0xA6, 0x05, 0x57, 0x16, 0x05, 0x67, 0x62, 0x02, 0x80, 0x16, 0x02, 0x9B,
            0xE6, 0x01, // new lap
            0xAA, 0x55, 0xC7, 0x01, 0x81, 0x2E, 0x81, 0x2E, 0x1B, 0x56, 0x14, 0x62, 0x02,
        ];
        master.write(&packet).unwrap();

        sleep_ms(10);

        let scan = scan_rx.recv().unwrap();
        assert_eq!(scan.angles_radian.len(), 16);

        let expected = vec![
            300., 306., 312., 318., 324., 330., 336., 342., 348., 354., 0., 6., 12., 18., 24., 30.,
        ];

        assert_eq!(scan.angles_radian.len(), expected.len());
        for i in 0..expected.len() {
            let degree = radian_to_degree(scan.angles_radian[i]);
            assert!(f64::abs(degree - expected[i]) < 1e-8);
        }

        assert!(scan.checksum_correct);

        drop(thread);
    }

    #[test]
    fn test_run_driver_checksum() {
        let (mut master, slave) = TTYPort::pair().expect("Unable to create ptty pair");

        let device_health_packet = [0xA5, 0x5A, 0x03, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00];
        master.write(&device_health_packet).unwrap();

        let device_info_packet = [
            0xA5, 0x5A, 0x14, 0x00, 0x00, 0x00, 0x04, 0x96, 0x00, 0x01, 0x02, 0x02, 0x00, 0x02,
            0x02, 0x01, 0x01, 0x00, 0x03, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
        ];
        master.write(&device_info_packet).unwrap();

        let start_scan_response_header = [0xA5, 0x5A, 0x05, 0x00, 0x00, 0x40, 0x81];
        master.write(&start_scan_response_header).unwrap();

        sleep_ms(10);

        let name = slave.name().unwrap();
        let (thread, scan_rx) = run_driver(&name, YdlidarModels::TMiniPro).unwrap();

        let packet = [
            // lap data
            0xAA, 0x55, 0xB0, 0x10, 0x01, 0x96, 0x01, 0x0F, 0xD6, 0xDA, 0xDD, 0x76, 0x03, 0xD4,
            0x76, 0x03, 0xC3, 0x72, 0x03, 0xB3, 0x7A, 0x03, 0x8E, 0x8A, 0x03, 0x97, 0x6E, 0x04,
            0x9C, 0x22, 0x05, 0xA7, 0x6A, 0x05, 0xAB, 0x7A, 0x05, 0x93, 0x82, 0x05, 0x6D, 0xC2,
            0x05, 0x55, 0xA6, 0x05, 0x57, 0x16, 0x05, 0x67, 0x62, 0x02, 0x80, 0x16, 0x02, 0x9B,
            0xE6, 0x01, // new lap
            0xAA, 0x55, 0xC7, 0x01, 0x81, 0x2E, 0x81, 0x2E, 0x1B, 0x56, 0x14, 0x62, 0x02,
        ];
        master.write(&packet).unwrap();

        let scan = scan_rx.recv().unwrap();
        assert!(!scan.checksum_correct);

        drop(thread);
    }
}
