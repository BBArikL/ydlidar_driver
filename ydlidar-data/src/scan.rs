#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Struct to hold one lap of lidar scan data.
#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Scan {
    /// Scan angle in radian.
    pub angles_radian: Vec<f64>,
    /// Distance to an object (in mm, rounded down).
    pub distances: Vec<u16>,
    /// Checksum validation result of the scan signal.
    pub checksum_correct: bool,
}
