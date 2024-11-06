use clap::{Arg, Command};
use piston_window::{EventLoop, PistonWindow, WindowSettings};
use plotters::drawing::IntoDrawingArea;
use plotters::prelude::{ChartBuilder, Circle, GREEN, WHITE};
use plotters::style::Color;
use plotters_piston::{draw_piston_window, PistonBackend};
use std::net::TcpStream;

#[allow(dead_code)] // Temporary fix until feature flags to select ydlidar
fn get_port_name() -> String {
    let matches = Command::new("LiDAR data receiver.")
        .about("Reads data from LiDAR and plot scan.")
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

const WINDOW_RANGE: f64 = 4000.;
const FPS: u64 = 60;
fn main() {
    let listener = TcpStream::connect("192.168.0.117:1500").unwrap();

    // let port_name = get_port_name();
    //let (driver_threads, scan_rx) = run_driver(&port_name, YdlidarModels::X2).unwrap();

    let mut window: PistonWindow = WindowSettings::new("LiDAR scan", [800, 800])
        .build()
        .unwrap();

    window.set_max_fps(FPS);
    let draw = |b: PistonBackend| {
        let scan: Vec<(f64, f64)> = rmp_serde::from_read(&listener).unwrap();

        println!("Received {} points.", scan.len());

        let root = b.into_drawing_area();
        root.fill(&WHITE)?;

        let mut cc = ChartBuilder::on(&root)
            .build_cartesian_2d(-WINDOW_RANGE..WINDOW_RANGE, -WINDOW_RANGE..WINDOW_RANGE)?;

        let circles: Vec<_> = scan
            .iter()
            .map(|(x, y)| {
                Circle::new((*x, *y), 2, GREEN.filled())
            }).collect();
        cc.draw_series(circles)?;

        Ok(())
    };

    while let Some(_) = draw_piston_window(&mut window, draw) {}
    // drop(driver_threads);
}
