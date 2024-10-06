use clap::{Arg, Command};
use std::net::TcpListener;
use ydlidar_data::YdlidarModels;
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

    let port_name : &String = matches.get_one("port").unwrap();
    port_name.to_string()
}

fn main() {
    let port_name = get_port_name();

    let listener = TcpListener::bind("0.0.0.0:0").unwrap();
    let (mut socket, _) = listener.accept().unwrap();

    let (driver_threads, scan_rx) = run_driver(&port_name, YdlidarModels::X2).unwrap();
    let mut run = true;

    while run {
        if let Ok(scan) = scan_rx.try_recv() {
            println!("Received {:03} samples.", scan.distances.len());
            serde_json::to_writer(&mut socket, &scan).unwrap();
        } else {
            run = false;
        }
    }

    drop(driver_threads);
}
