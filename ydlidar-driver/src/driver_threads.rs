use crate::constants::LIDAR_MAX_DISTANCE_VALUE;
use crate::numeric::{calc_distance, correct_angle, degree_to_radian, to_angle};
use crate::packet::{
    err_if_checksum_mismatched, is_beginning_of_cycle, n_scan_samples, scan_index,
    sendable_packet_range,
};
use crate::scan::YdLidarScan;
use crate::serial::{get_n_read, read, stop_scan_and_flush};
use crate::time::sleep_ms;
use crossbeam_channel::{Receiver, Sender};
use serialport::SerialPort;
use std::collections::VecDeque;
use std::sync::mpsc;
use std::thread::JoinHandle;
use ydlidar_data::{InterferenceFlag, Scan};

/// Struct that contains driver threads.
pub struct DriverThreads {
    pub(crate) reader_terminator_tx: Sender<bool>,
    pub(crate) parser_terminator_tx: Sender<bool>,
    pub(crate) reader_thread: Option<JoinHandle<()>>,
    pub(crate) receiver_thread: Option<JoinHandle<()>>,
}

pub(crate) fn read_device_signal(
    port: &mut Box<dyn SerialPort>,
    scan_data_tx: mpsc::SyncSender<Vec<u8>>,
    reader_terminator_rx: Receiver<bool>,
) {
    loop {
        if do_terminate(&reader_terminator_rx) {
            if let Err(e) = stop_scan_and_flush(port) {
                eprintln!("{e}");
            }
            return;
        }

        let n_read: usize = get_n_read(port).unwrap_or(0);
        if n_read == 0 {
            continue;
        }

        if let Ok(signal) = read(port, n_read) {
            if let Err(e) = scan_data_tx.send(signal) {
                eprintln!("{e}");
            }
        }
    }
}

pub(crate) fn parse_packets(
    scan_data_rx: mpsc::Receiver<Vec<u8>>,
    parser_terminator_rx: Receiver<bool>,
    scan_tx: mpsc::SyncSender<Scan>,
) {
    let mut buffer = VecDeque::<u8>::new();
    let mut scan = Scan::new();
    while !do_terminate(&parser_terminator_rx) {
        match scan_data_rx.try_recv() {
            Ok(data) => buffer.extend(data),
            Err(_) => sleep_ms(10),
        }

        if buffer.is_empty() {
            continue;
        }

        let (start_index, n_packet_bytes) = match sendable_packet_range(&buffer) {
            Ok(t) => t,
            Err(_) => continue,
        };
        buffer.drain(..start_index); // remove leading bytes
        if buffer.len() < n_packet_bytes {
            // insufficient buffer size to extract a packet
            continue;
        }
        let packet = buffer.drain(0..n_packet_bytes).collect::<Vec<_>>();
        if is_beginning_of_cycle(&packet) {
            scan_tx.send(scan).unwrap();
            scan = Scan::new();
        }

        if err_if_checksum_mismatched(&packet).is_err() {
            scan.checksum_correct = false;
        }

        let n = n_scan_samples(&packet);
        if n == 1 {
            // Start Angle == End Angle
            // assert_eq!(packet[4], packet[6]);
            // assert_eq!(packet[5], packet[7]);
            let dist_idx = scan_index(0);
            let d = calc_distance(packet[dist_idx + 1], packet[dist_idx + 2]);
            if d > LIDAR_MAX_DISTANCE_VALUE || d == 0 {
                continue;
            }
            scan.distances.push(d);
            let angle_degree = to_angle(packet[4], packet[5]);
            let angle_degree = correct_angle(angle_degree, d);
            let angle_radian = degree_to_radian(angle_degree);
            scan.angles_radian.push(angle_radian);
            // X2 does not provide Flag
            scan.flags.push(InterferenceFlag::Nothing);
            // X2 lidar does not provide intensity data, so we put 255
            scan.intensities.push(255);
        } else {
            let start_angle = to_angle(packet[4], packet[5]);
            let end_angle = to_angle(packet[6], packet[7]);
            let angle_shift = if start_angle < end_angle { 0f64 } else { 360. };
            let angle_diff = end_angle - start_angle + angle_shift;
            let angle_rate: f64 = angle_diff / ((n - 1) as f64);
            (0..n)
                .map(|idx| (idx, scan_index(idx)))
                .for_each(|(packet_idx, dist_idx)| {
                    let d = calc_distance(packet[dist_idx + 1], packet[dist_idx + 2]);
                    if d > LIDAR_MAX_DISTANCE_VALUE || d == 0 {
                        return;
                    }
                    scan.distances.push(d);
                    if packet_idx == 0 {
                        let start_angle = correct_angle(start_angle, d);
                        scan.angles_radian.push(degree_to_radian(start_angle));
                    } else if packet_idx == n - 1 {
                        let end_angle = correct_angle(end_angle, d);
                        scan.angles_radian.push(degree_to_radian(end_angle));
                    } else {
                        let angle_degree = (start_angle + (packet_idx as f64) * angle_rate) % 360.;
                        let angle_degree = correct_angle(angle_degree, d);
                        let angle_radian = degree_to_radian(angle_degree);
                        scan.angles_radian.push(angle_radian);
                    }
                    // X2 does not provide Flag
                    scan.flags.push(InterferenceFlag::Nothing);
                    // X2 lidar does not provide intensity data, so we put 255
                    scan.intensities.push(255);
                });
        }
    }
}

pub(crate) fn do_terminate(terminator_rx: &Receiver<bool>) -> bool {
    terminator_rx.try_recv().unwrap_or(false)
}

/// Function to join driver threads.
/// This function is automatically called when `driver_threads` is dropped.
pub fn join(driver_threads: &mut DriverThreads) {
    driver_threads.reader_terminator_tx.send(true).unwrap();
    driver_threads.parser_terminator_tx.send(true).unwrap();

    if driver_threads.reader_thread.is_some() {
        let thread = driver_threads.reader_thread.take().unwrap();
        thread.join().unwrap();
    }
    if driver_threads.receiver_thread.is_some() {
        let thread = driver_threads.receiver_thread.take().unwrap();
        thread.join().unwrap();
    }
}

impl Drop for DriverThreads {
    fn drop(&mut self) {
        join(self);
    }
}
