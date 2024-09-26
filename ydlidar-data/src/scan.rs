use crate::flags::InterferenceFlag;

/// Struct to hold one lap of lidar scan data.
pub struct Scan {
    /// Scan angle in radian.
    pub angles_radian: Vec<f64>,
    /// Distance to an object.
    pub distances: Vec<u16>,
    /// Interference status of the returned signal.
    pub flags: Vec<InterferenceFlag>,
    /// Return strength of the laser pulse.
    pub intensities: Vec<u8>,
    /// Checksum valiadtion result of the scan signal.
    pub checksum_correct: bool,
}
