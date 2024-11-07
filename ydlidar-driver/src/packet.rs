#[allow(dead_code)] // Temporary fix until feature flags to select ydlidar
use crate::constants::{HEADER_SIZE, LIDAR_CMD_SYNC_BYTE, PACKET_HEADER_SIZE};
use crate::error::YDLidarError;
use crate::numeric::{to_string, to_u16};
use std::collections::VecDeque;

fn get_packet_size(buffer: &VecDeque<u8>, start_index: usize) -> Result<usize, ()> {
    let index = start_index + 3;
    if index >= buffer.len() {
        return Err(());
    }
    let n_scan_samples = match buffer.get(index) {
        Some(n) => n,
        None => return Err(()),
    };
    Ok(10 + (*n_scan_samples as usize) * 3)
}

pub(crate) fn validate_response_header(
    header: &[u8],
    maybe_response_length: Option<u8>,
    type_code: u8,
) -> Result<(), YDLidarError> {
    if header.len() != HEADER_SIZE {
        return Err(YDLidarError::InvalidHeaderLength(header.len()));
    }
    if header[0..2] != [LIDAR_CMD_SYNC_BYTE, 0x5A] {
        return Err(YDLidarError::InvalidMagicNumber(to_string(&header[0..2])));
    }
    match maybe_response_length {
        None => (),
        Some(len) => {
            if header[2] != len {
                return Err(YDLidarError::InvalidResponseLength(
                    len.into(),
                    header[2].into(),
                ));
            }
        }
    }
    if header[6] != type_code {
        return Err(YDLidarError::InvalidTypeCode(
            type_code.into(),
            header[6].into(),
        ));
    }
    Ok(())
}

#[allow(dead_code)] // Temporary fix until feature flags to select ydlidar
fn validate_packet_response(header: &[u8]) -> Result<(), YDLidarError> {
    if header.len() != PACKET_HEADER_SIZE {
        return Err(YDLidarError::InvalidHeaderLength(header.len()));
    }
    if !is_packet_header(header[0], header[1]) {
        return Err(YDLidarError::InvalidMagicNumber(to_string(&header[0..2])));
    }
    Ok(())
}

fn calc_checksum(packet: &[u8]) -> u16 {
    let n_scan = packet[3] as usize;

    let mut checksum: u16 = to_u16(packet[1], packet[0]);
    checksum ^= to_u16(packet[5], packet[4]);
    for i in 0..n_scan {
        let s0 = packet[10 + 3 * i];
        let s1 = packet[10 + 3 * i + 1];
        let s2 = packet[10 + 3 * i + 2];
        checksum ^= to_u16(0x00, s0);
        checksum ^= to_u16(s2, s1);
    }
    checksum ^= to_u16(packet[3], packet[2]);
    checksum ^= to_u16(packet[7], packet[6]);
    checksum
}

fn is_packet_header(element0: u8, element1: u8) -> bool {
    element0 == 0xAA && element1 == 0x55
}

pub(crate) fn is_beginning_of_cycle(packet: &[u8]) -> bool {
    packet[2] & 0x01 == 1
}

fn find_start_index(buffer: &VecDeque<u8>) -> Result<usize, ()> {
    if buffer.is_empty() {
        return Err(());
    }
    for i in 0..(buffer.len() - 1) {
        let e0 = match buffer.get(i) {
            Some(e) => e,
            None => continue,
        };
        let e1 = match buffer.get(i + 1) {
            Some(e) => e,
            None => continue,
        };
        if is_packet_header(*e0, *e1) {
            return Ok(i);
        }
    }
    Err(())
}

pub(crate) fn sendable_packet_range(buffer: &VecDeque<u8>) -> Result<(usize, usize), ()> {
    let start_index = find_start_index(buffer)?;
    let end_index = get_packet_size(buffer, start_index)?;
    Ok((start_index, end_index))
}
pub(crate) fn err_if_checksum_mismatched(packet: &[u8]) -> Result<(), YDLidarError> {
    let calculated = calc_checksum(packet);
    let expected = to_u16(packet[9], packet[8]);
    match calculated != expected {
        true => Err(YDLidarError::ChecksumMismatch(expected, calculated)),
        false => Ok(()),
    }
}

pub(crate) fn scan_index(idx: usize) -> usize {
    10 + idx * 3
}

pub(crate) fn n_scan_samples(packet: &[u8]) -> usize {
    packet[3] as usize
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_validate_response_header() {
        assert!(matches!(
            validate_response_header(
                &vec![0xA5, 0x5A, 0x14, 0x00, 0x00, 0x00, 0x04],
                Some(0x14),
                0x04
            ),
            Ok(())
        ));

        assert!(matches!(
            validate_response_header(
                &vec![0xA5, 0x5A, 0x14, 0x00, 0x00, 0x00, 0x04, 0x09],
                Some(0x14),
                0x04
            ),
            Err(YDLidarError::InvalidHeaderLength(8))
        ));

        assert!(matches!(
            validate_response_header(
                &vec![0xA6, 0x5A, 0x14, 0x00, 0x00, 0x00, 0x04],
                Some(0x14),
                0x04
            ),
            Err(YDLidarError::InvalidMagicNumber(_))
        ));

        assert!(matches!(
            validate_response_header(
                &vec![0xA5, 0x2A, 0x14, 0x00, 0x00, 0x00, 0x04],
                Some(0x14),
                0x04
            ),
            Err(YDLidarError::InvalidMagicNumber(_))
        ));

        assert!(matches!(
            validate_response_header(
                &vec![0xA5, 0x5A, 0x14, 0x00, 0x00, 0x00, 0x04],
                Some(0x12),
                0x04
            ),
            Err(YDLidarError::InvalidResponseLength(18, 20))
        ));

        assert!(matches!(
            validate_response_header(
                &vec![0xA5, 0x5A, 0x14, 0x00, 0x00, 0x00, 0x08],
                Some(0x14),
                0x04
            ),
            Err(YDLidarError::InvalidTypeCode(4, 8))
        ));
    }

    #[test]
    fn test_calc_checksum() {
        let packet = vec![
            0xAA, 0x55, 0xB0, 0x27, 0xE3, 0x28, 0xF3, 0x39, 0x0E, 0x61, 0x79, 0xB6, 0x05, 0x6F,
            0x4E, 0x06, 0x61, 0x06, 0x06, 0x7A, 0x9A, 0x02, 0x9E, 0x5E, 0x02, 0xA6, 0x0A, 0x02,
            0xA7, 0xE6, 0x01, 0xAE, 0xD6, 0x01, 0xBA, 0xD6, 0x01, 0xB8, 0xD2, 0x01, 0xB2, 0xD6,
            0x01, 0xBD, 0xD2, 0x01, 0xDF, 0xDA, 0x01, 0xE1, 0xDA, 0x01, 0xDF, 0xDA, 0x01, 0xDC,
            0xDE, 0x01, 0xDE, 0xDE, 0x01, 0xD8, 0xE2, 0x01, 0xD4, 0xDE, 0x01, 0xBA, 0xDE, 0x01,
            0x84, 0xDF, 0x01, 0x2F, 0xAB, 0x01, 0x17, 0xEE, 0x01, 0x0F, 0x22, 0x02, 0x0C, 0x7E,
            0x02, 0x0A, 0x02, 0x00, 0x0C, 0x9E, 0x02, 0x16, 0xA6, 0x02, 0x21, 0xA2, 0x02, 0x3A,
            0x32, 0x03, 0x55, 0x4E, 0x0A, 0x87, 0x46, 0x0A, 0x85, 0x5A, 0x0A, 0x8A, 0x6E, 0x0A,
            0x84, 0x9A, 0x0A, 0x7E, 0xCE, 0x0A, 0x4E, 0x7E, 0x04, 0x51, 0x6E, 0x03, 0x66, 0xA6,
            0x02,
        ];
        let checksum = calc_checksum(&packet);
        let expected = to_u16(packet[9], packet[8]);
        assert_eq!(checksum, expected);

        let packet = vec![
            0xAA, 0x55, 0x24, 0x28, 0xF5, 0x4C, 0x85, 0x5E, 0x9D, 0x70, 0xCE, 0xE2, 0x07, 0xBC,
            0xFA, 0x07, 0xCC, 0xB6, 0x07, 0xC8, 0xB6, 0x07, 0xC4, 0xBA, 0x07, 0xCB, 0xCA, 0x07,
            0xC8, 0xAE, 0x09, 0xC5, 0x9E, 0x09, 0xC7, 0x9E, 0x09, 0xC2, 0x9E, 0x09, 0xC1, 0x92,
            0x09, 0xC0, 0x8A, 0x09, 0xC1, 0x86, 0x09, 0xBE, 0x86, 0x09, 0xC5, 0x86, 0x09, 0xC3,
            0x8A, 0x09, 0xBC, 0x8A, 0x09, 0xC6, 0x8A, 0x09, 0xC6, 0x8A, 0x09, 0xC2, 0x8E, 0x09,
            0xC5, 0x8E, 0x09, 0xC3, 0x92, 0x09, 0xC4, 0xAA, 0x09, 0xC9, 0xB2, 0x09, 0xC9, 0xBA,
            0x09, 0xC5, 0xC2, 0x09, 0xC9, 0xCE, 0x09, 0xBF, 0xCE, 0x09, 0xBE, 0xCE, 0x09, 0xBA,
            0xCE, 0x09, 0xBE, 0xD6, 0x09, 0xBB, 0xD6, 0x09, 0xBF, 0xE2, 0x09, 0xBB, 0xF2, 0x09,
            0xC1, 0x0A, 0x0A, 0xBF, 0x1A, 0x0A, 0xB9, 0x1E, 0x0A, 0xAA, 0x22, 0x0A, 0x9E, 0x2A,
            0x0A, 0xCB, 0x7A, 0x15,
        ];
        let checksum = calc_checksum(&packet);
        let expected = to_u16(packet[9], packet[8]);
        assert_eq!(checksum, expected);
    }
}
