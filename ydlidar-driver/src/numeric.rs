pub(crate) fn to_u16(a: u8, b: u8) -> u16 {
    ((a as u16) << 8) + (b as u16)
}

pub(crate) fn degree_to_radian(degree: f64) -> f64 {
    degree * std::f64::consts::PI / 180.
}

pub(crate) fn to_angle(bit1: u8, bit2: u8) -> f64 {
    let a = ((bit1 as u16) + ((bit2 as u16) << 8)) >> 1;
    (a as f64) / 64.
}

pub(crate) fn calc_distance(b1: u8, b2: u8) -> u16 {
    ((b2 as u16) << 6) + ((b1 as u16) >> 2)
}

pub(crate) fn to_string(data: &[u8]) -> String {
    data.iter()
        .map(|e| format!("{:02X}", e))
        .collect::<Vec<_>>()
        .join(" ")
}

pub(crate) fn correct_angle(angle: f64, distance: u16) -> f64 {
    // Correcting the angle for the X2 lidar
    if distance == 0 {
        angle
    } else {
        angle - (21.8 * ((155.3 - distance as f64) / (155.3 * distance as f64))).atan()
    }
}
