use crate::packet::{err_if_checksum_mismatched, is_beginning_of_cycle, sendable_packet_range};
use crate::scan::YdLidarScan;
use crate::serial::{get_n_read, read, stop_scan_and_flush};
use crate::time::sleep_ms;
use crossbeam_channel::{Receiver, Sender};
use serialport::SerialPort;
use std::collections::VecDeque;
use std::sync::mpsc;
use std::thread::JoinHandle;
use ydlidar_data::Scan;

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

        scan.push_distances(&packet);
        scan.push_angles(&packet);
        scan.push_intensities(&packet);
        scan.push_flags(&packet);
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
