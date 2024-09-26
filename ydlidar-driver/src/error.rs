use std::error::Error;
use std::fmt::{Debug, Display};
use std::{fmt, io};

#[derive(Debug)]
pub enum YDLidarError {
    InvalidHeaderLength(usize),
    InvalidMagicNumber(String),
    InvalidResponseLength(usize, usize),
    InvalidTypeCode(usize, usize),
    DeviceHealthError(usize),
    UnsupportedModel(u8),
    ChecksumMismatch(u16, u16),
    TimeoutError(),
    SerialError(serialport::Error),
    IoError(io::Error),
}
impl fmt::Display for YDLidarError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            YDLidarError::InvalidHeaderLength(len) => write!(f, "Response header must be always seven bytes. Actually {} bytes.", len),
            YDLidarError::InvalidMagicNumber(magic) => write!(f, "Header sign must start with 0xA5 0x5A. Observed = {}.", magic),
            YDLidarError::InvalidResponseLength(expected, actual) => write!(f, "Expected response length of {} bytes but found {} bytes.",
                                                                            expected, actual),
            YDLidarError::InvalidTypeCode(expected, actual) => write!(f, "Expected type code {} but obtained {}.", expected, actual),
            // Last two bit are reserved bits, which should be ignored.
            YDLidarError::DeviceHealthError(error) => write!(f, "Device health error. Error code = {:#010b}. See the development manual for details.", error),
            YDLidarError::UnsupportedModel(model) => write!(f, "The model #{} is not supported", model),
            YDLidarError::ChecksumMismatch(expected, calculated) => write!(f, "Checksum mismatched. Calculated = {:04X}, expected = {:04X}.", calculated, expected),
            YDLidarError::TimeoutError() => write!(f, "Operation timed out"),
            YDLidarError::IoError(err) => Display::fmt(&err, f),
            YDLidarError::SerialError(err) => Display::fmt(&err,f),
        }
    }
}

impl Error for YDLidarError {}

impl From<io::Error> for YDLidarError {
    fn from(err: io::Error) -> Self {
        YDLidarError::IoError(err)
    }
}
impl From<serialport::Error> for YDLidarError {
    fn from(err: serialport::Error) -> Self {
        YDLidarError::SerialError(err)
    }
}
