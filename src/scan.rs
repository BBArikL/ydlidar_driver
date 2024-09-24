use crate::flags::to_flag;
use crate::numeric::{calc_distance, degree_to_radian, to_angle};
use crate::InterferenceFlag;

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

impl Scan {
    pub fn new() -> Scan {
        Scan {
            angles_radian: Vec::new(),
            distances: Vec::new(),
            flags: Vec::new(),
            intensities: Vec::new(),
            checksum_correct: true,
        }
    }

    pub(crate) fn push_angles(&mut self, packet: &[u8]) {
        let n = Scan::n_scan_samples(packet);
        if n == 1 {
            assert_eq!(packet[4], packet[6]);
            assert_eq!(packet[5], packet[7]);
            let angle_degree = to_angle(packet[4], packet[5]);
            let angle_radian = degree_to_radian(angle_degree);
            self.angles_radian.push(angle_radian);
            return;
        }

        let start_angle = to_angle(packet[4], packet[5]);
        let end_angle = to_angle(packet[6], packet[7]);

        let angle_rate: f64 = if start_angle < end_angle {
            (end_angle - start_angle) / ((n - 1) as f64)
        } else {
            (end_angle - start_angle + 360.) / ((n - 1) as f64)
        };

        for i in 0..n {
            let angle_degree = (start_angle + (i as f64) * angle_rate) % 360.;
            let angle_radian = degree_to_radian(angle_degree);
            self.angles_radian.push(angle_radian);
        }
    }

    pub(crate) fn push_flags(&mut self, packet: &[u8]) {
        for i in Scan::scan_indices(Scan::n_scan_samples(packet)) {
            self.flags.push(to_flag(packet[i + 1] & 0x03));
        }
    }

    pub(crate) fn push_intensities(&mut self, packet: &[u8]) {
        for i in Scan::scan_indices(Scan::n_scan_samples(packet)) {
            self.intensities.push(packet[i])
        }
    }

    pub(crate) fn push_distances(&mut self, packet: &[u8]) {
        for i in Scan::scan_indices(Scan::n_scan_samples(packet)) {
            let d = calc_distance(packet[i + 1], packet[i + 2]);
            self.distances.push(d);
        }
    }

    fn scan_indices(n_scan_samples: usize) -> impl Iterator<Item = usize> {
        (0..n_scan_samples).map(|i| 10 + i * 3)
    }

    pub(crate) fn n_scan_samples(packet: &[u8]) -> usize {
        packet[3] as usize
    }
}
