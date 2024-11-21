use ydlidar_data::scan::Scan;

pub(crate) trait YdLidarScan {
    fn new() -> Self;
}

impl YdLidarScan for Scan {
    fn new() -> Scan {
        Scan {
            angles_radian: Vec::new(),
            distances: Vec::new(),
            checksum_correct: true,
        }
    }
}
