use crate::constants::{
    HEADER_SIZE, LIDAR_ANS_TYPE_MEASUREMENT, LIDAR_CMD_FORCE_STOP, LIDAR_CMD_SCAN, LIDAR_CMD_STOP,
    LIDAR_CMD_SYNC_BYTE, N_READ_TRIALS,
};
use crate::error::YDLidarError;
use crate::packet::validate_response_header;
use crate::time::sleep_ms;
use serialport::SerialPort;
use std::io::Read;

pub(crate) fn start_scan(port: &mut Box<dyn SerialPort>) -> Result<(), YDLidarError> {
    send_command(port, LIDAR_CMD_SCAN)?;
    let header = read(port, HEADER_SIZE)?;
    validate_response_header(&header, None, LIDAR_ANS_TYPE_MEASUREMENT)?;
    Ok(())
}

fn stop_scan(port: &mut Box<dyn SerialPort>) -> Result<(), YDLidarError> {
    send_command(port, LIDAR_CMD_FORCE_STOP)?;
    send_command(port, LIDAR_CMD_STOP)?;
    Ok(())
}

pub(crate) fn stop_scan_and_flush(port: &mut Box<dyn SerialPort>) -> Result<(), YDLidarError> {
    stop_scan(port)?;
    flush(port)?;
    Ok(())
}

fn send_data(port: &mut Box<dyn SerialPort>, data: &[u8]) -> std::io::Result<usize> {
    port.write(data)
}

pub(crate) fn send_command(port: &mut Box<dyn SerialPort>, command: u8) -> std::io::Result<usize> {
    let data: [u8; 2] = [LIDAR_CMD_SYNC_BYTE, command];
    send_data(port, &data)
}

pub(crate) fn get_n_read(port: &mut Box<dyn SerialPort>) -> Result<usize, YDLidarError> {
    let n_u32: u32 = port.bytes_to_read()?;
    Ok(n_u32.try_into().unwrap_or(0))
}

pub(crate) fn flush(port: &mut Box<dyn SerialPort>) -> Result<(), YDLidarError> {
    let n_read: usize = get_n_read(port).unwrap_or(0);
    if n_read == 0 {
        return Ok(());
    }
    let mut packet: Vec<u8> = vec![0; n_read];
    port.read(packet.as_mut_slice())?;
    Ok(())
}

pub(crate) fn read(
    port: &mut Box<dyn SerialPort>,
    data_size: usize,
) -> Result<Vec<u8>, YDLidarError> {
    assert!(data_size > 0);
    for _ in 0..N_READ_TRIALS {
        let n_read: usize = get_n_read(port)?;

        if n_read < data_size {
            sleep_ms(10);
            continue;
        }

        let mut packet: Vec<u8> = vec![0; data_size];
        if let Err(e) = port.read(packet.as_mut_slice()) {
            return Err(YDLidarError::IoError(e));
        }
        return Ok(packet);
    }
    Err(YDLidarError::TimeoutError())
}

#[cfg(test)]
mod tests {
    use super::*;
    use serialport::TTYPort;
    use std::io::{Read, Write};
    #[test]
    fn test_start_scan() {
        let (mut master, slave) = TTYPort::pair().expect("Unable to create ptty pair");
        master
            .write(&[0xA5, 0x5A, 0x05, 0x00, 0x00, 0x40, 0x81])
            .unwrap();

        let mut slave_ptr = Box::new(slave) as Box<dyn SerialPort>;
        start_scan(&mut slave_ptr).unwrap();

        sleep_ms(10);

        let mut buf = [0u8; 2];
        master.read(&mut buf).unwrap();
        assert_eq!(buf, [0xA5, 0x60]);
    }

    #[test]
    fn test_stop_scan() {
        let (master, mut slave) = TTYPort::pair().expect("Unable to create ptty pair");
        let mut master_ptr = Box::new(master) as Box<dyn SerialPort>;
        stop_scan(&mut master_ptr).unwrap();

        sleep_ms(10);

        let mut buf = [0u8; 4];
        slave.read(&mut buf).unwrap();
        assert_eq!(buf, [0xA5, 0x00, 0xA5, 0x65]);
    }
}
