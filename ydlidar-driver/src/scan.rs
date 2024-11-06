use crate::numeric::{calc_distance, degree_to_radian, to_angle};
use ydlidar_data::scan::Scan;
use ydlidar_data::InterferenceFlag;

pub(crate) trait YdLidarScan {
    fn new() -> Self;
    fn push_angles(&mut self, packet: &[u8]);
    fn push_flags(&mut self, packet: &[u8]);
    fn push_intensities(&mut self, packet: &[u8]);
    fn push_distances(&mut self, packet: &[u8]);
}

impl YdLidarScan for Scan {
    fn new() -> Scan {
        Scan {
            angles_radian: Vec::new(),
            distances: Vec::new(),
            flags: Vec::new(),
            intensities: Vec::new(),
            checksum_correct: true,
        }
    }

    fn push_angles(&mut self, packet: &[u8]) {
        let n = n_scan_samples(packet);
        if n == 1 {
            // Start Angle == End Angle
            assert_eq!(packet[4], packet[6]);
            assert_eq!(packet[5], packet[7]);
            let angle_degree = to_angle(packet[4], packet[5]);
            let angle_radian = degree_to_radian(angle_degree);
            self.angles_radian.push(angle_radian);
            return;
        }

        let start_angle = to_angle(packet[4], packet[5]);
        // let start_angle = correct_angle(start_angle, self.distances[self.angles_radian.len()]);
        self.angles_radian.push(degree_to_radian(start_angle));

        let end_angle = to_angle(packet[6], packet[7]);
        // let end_angle = correct_angle(end_angle, self.distances[self.angles_radian.len() + n - 2]);

        let angle_shift = if start_angle < end_angle { 0f64 } else { 360. };
        let angle_diff = end_angle - start_angle + angle_shift;
        let angle_rate: f64 = angle_diff / ((n - 1) as f64);

        for i in 2..n {
            let angle_degree = (start_angle + (i as f64) * angle_rate) % 360.;
            let distance = self.distances[self.angles_radian.len()];
            // let angle_degree = correct_angle(angle_degree, distance);
            let angle_radian = degree_to_radian(angle_degree);
            self.angles_radian.push(angle_radian);
        }
        self.angles_radian.push(degree_to_radian(end_angle));
    }

    fn push_flags(&mut self, packet: &[u8]) {
        for _ in scan_indices(n_scan_samples(packet)) {
            // X2 lidar does not provide Interference data so we put no Interference
            // to_flag(packet[i + 1] & 0x03)
            self.flags.push(InterferenceFlag::Nothing);
        }
    }

    fn push_intensities(&mut self, packet: &[u8]) {
        for _ in scan_indices(n_scan_samples(packet)) {
            // X2 lidar does not provide intensity data, so we put 255
            // packet[i]
            self.intensities.push(255)
        }
    }

    fn push_distances(&mut self, packet: &[u8]) {
        for i in scan_indices(n_scan_samples(packet)) {
            self.distances
                .push(calc_distance(packet[i + 1], packet[i + 2]));
        }
    }
}

fn scan_indices(n_scan_samples: usize) -> impl Iterator<Item=usize> {
    (0..n_scan_samples).map(|i| 10 + i * 3)
}

fn n_scan_samples(packet: &[u8]) -> usize {
    packet[3] as usize
}
