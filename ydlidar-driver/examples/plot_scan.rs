use clap::{Arg, Command};
use piston_window::{EventLoop, PistonWindow, WindowSettings};
use plotters::drawing::IntoDrawingArea;
use plotters::prelude::{ChartBuilder, Circle, BLACK, BLUE, CYAN, GREEN, RED, WHITE, YELLOW};
use plotters::style::Color;
use plotters_piston::{draw_piston_window, PistonBackend};
use std::net::TcpStream;
use std::sync::mpsc::Receiver;
use ydlidar_data::{Scan, YdlidarModel};

use ydlidar_driver::{run_driver, DriverThreads};

fn get_args() -> (Option<String>, Option<String>) {
    let matches = Command::new("LiDAR data receiver.")
        .about("Reads data from LiDAR and plot scan.")
        .disable_version_flag(true)
        .arg(
            Arg::new("port")
                .long("port")
                .help("The device path to a serial port")
                .use_value_delimiter(false)
                .exclusive(true),
        )
        .arg(
            Arg::new("ip")
                .long("ip")
                .help("The remote machine where the lidar is being broadcasted")
                .use_value_delimiter(false)
                .exclusive(true),
        )
        .get_matches();

    let port_name: Option<String> = matches.get_one("port").cloned();
    let ip: Option<String> = matches.get_one("ip").cloned();
    (port_name, ip)
}

const WINDOW_RANGE: f64 = 4000.;
const FPS: u64 = 60;
fn main() {
    let (port_name, ip) = get_args();

    let mut listener: Option<TcpStream> = None;
    let mut driver_threads: Option<DriverThreads> = None;
    let mut scan_rx: Option<Receiver<Scan>> = None;

    match (port_name, ip) {
        (_, Some(ip)) => {
            listener = Some(TcpStream::connect(format!("{}:1500", ip)).unwrap());
        }
        (Some(port), _) => {
            let driver = run_driver(&port, YdlidarModel::X2, 200, 10, 0, 100).unwrap();
            driver_threads = Some(driver.0);
            scan_rx = Some(driver.1);
        }
        (_, _) => panic!("Either --port or --ip has to be passed!"),
    }

    let mut window: PistonWindow = WindowSettings::new("LiDAR scan", [800, 800])
        .build()
        .unwrap();

    window.set_max_fps(FPS);
    let draw = |b: PistonBackend| {
        let scan: Vec<(f64, f64)>;
        match (&listener, &scan_rx) {
            (Some(listener), _) => {
                scan = rmp_serde::from_read(listener).unwrap();
            }
            (_, Some(scan_rx)) => {
                let raw_scan = scan_rx.recv().unwrap();
                scan = raw_scan
                    .angles_radian
                    .iter()
                    .zip(raw_scan.distances.iter())
                    .map(|(w, d)| {
                        let x = (*d as f64) * f64::cos(*w - std::f64::consts::PI / 2.0);
                        let y = (*d as f64) * f64::sin(*w - std::f64::consts::PI / 2.0);
                        (x, y)
                    })
                    .collect();
            }
            (None, None) => {
                panic!("Either a TcpStream or a Receiver<Scan> should be set!");
            }
        }

        println!("Received {} points.", scan.len());

        let root = b.into_drawing_area();
        root.fill(&WHITE)?;

        let mut cc = ChartBuilder::on(&root)
            .build_cartesian_2d(-WINDOW_RANGE..WINDOW_RANGE, -WINDOW_RANGE..WINDOW_RANGE)?;

        let mut circles: Vec<_> = scan
            .iter()
            .map(|(x, y)| Circle::new((*x - 2000., *y), 2, GREEN.filled()))
            .collect();
        circles.push(Circle::new((-2000., 0.), 3, RED.filled()));
        circles.push(Circle::new((-1950., 0.), 3, BLUE.filled()));
        circles.push(Circle::new((-2000., 50.), 3, YELLOW.filled()));
        circles.push(Circle::new((-2050., 0.), 3, BLACK.filled()));
        circles.push(Circle::new((-2000., -50.), 3, CYAN.filled()));
        cc.draw_series(circles)?;

        Ok(())
    };

    while let Some(_) = draw_piston_window(&mut window, draw) {}

    if driver_threads.is_some() {
        drop(driver_threads);
    }
}
