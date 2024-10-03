use crate::flags::InterferenceFlag;

/// Struct to hold one lap of lidar scan data.
#[derive(Clone, Debug, PartialEq)]
pub struct Scan {
    /// Scan angle in radian.
    pub angles_radian: Vec<f64>,
    /// Distance to an object (in mm, rounded down).
    pub distances: Vec<u16>,
    /// Interference status of the returned signal.
    pub flags: Vec<InterferenceFlag>,
    /// Return strength of the laser pulse.
    pub intensities: Vec<u8>,
    /// Checksum validation result of the scan signal.
    pub checksum_correct: bool,
}
