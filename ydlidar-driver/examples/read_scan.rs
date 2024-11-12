use clap::{Arg, Command};
use std::io::Write;
use std::net::TcpListener;
use ydlidar_data::YdlidarModel;
use ydlidar_driver::run_driver;

fn get_port_name() -> String {
    let matches = Command::new("LiDAR data receiver.")
        .about("Reads data from LiDAR.")
        .disable_version_flag(true)
        .arg(
            Arg::new("port")
                .help("The device path to a serial port")
                .use_value_delimiter(false)
                .required(true),
        )
        .get_matches();

    let port_name: &String = matches.get_one("port").unwrap();
    port_name.to_string()
}

fn main() {
    let port_name = get_port_name();

    let listener = TcpListener::bind("0.0.0.0:1500").unwrap();
    let (mut socket, _) = listener.accept().unwrap();

    let (driver_threads, scan_rx) = run_driver(&port_name, YdlidarModel::X2).unwrap();

    loop {
        let scan = scan_rx.recv();
        if scan.is_err() {
            break;
        }
        let scan = scan.unwrap();
        let scans: Vec<(f64, f64)> = scan
            .angles_radian
            .iter()
            .take(1000)
            .zip(scan.distances.iter())
            .map(|(w, d)| {
                let x = (*d as f64) * f64::cos(*w - std::f64::consts::PI / 2.0);
                let y = (*d as f64) * f64::sin(*w - std::f64::consts::PI / 2.0);
                (x, y)
            })
            .collect();
        let data = rmp_serde::to_vec(&scans).unwrap();
        let res = socket.write_all(&data);
        if res.is_err() {
            break;
        }
    }

    drop(driver_threads);
}
