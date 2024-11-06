pub(crate) const HEADER_SIZE: usize = 7;
pub(crate) const PACKET_HEADER_SIZE: usize = 10;
pub(crate) const LIDAR_CMD_GET_DEVICE_HEALTH: u8 = 0x92;
pub(crate) const LIDAR_CMD_GET_DEVICE_INFO: u8 = 0x90;
pub(crate) const LIDAR_CMD_SYNC_BYTE: u8 = 0xA5;
#[allow(dead_code)] // Temporary fix until feature flags to select ydlidar
pub(crate) const LIDAR_CMD_FORCE_STOP: u8 = 0x00;
#[allow(dead_code)] // Temporary fix until feature flags to select ydlidar
pub(crate) const LIDAR_CMD_STOP: u8 = 0x65;
#[allow(dead_code)] // Temporary fix until feature flags to select ydlidar
pub(crate) const LIDAR_CMD_SCAN: u8 = 0x60;
pub(crate) const LIDAR_ANS_TYPE_DEVINFO: u8 = 0x4;
pub(crate) const LIDAR_ANS_LENGTH_DEVINFO: u8 = 20;
pub(crate) const LIDAR_ANS_TYPE_DEVHEALTH: u8 = 0x6;
pub(crate) const LIDAR_ANS_LENGTH_DEVHEALTH: u8 = 3;
#[allow(dead_code)] // Temporary fix until feature flags to select ydlidar
pub(crate) const LIDAR_ANS_TYPE_MEASUREMENT: u8 = 0x81;
pub(crate) const N_READ_TRIALS: usize = 3;
// Specific for each lidar
pub(crate) const LIDAR_MAX_DISTANCE_VALUE: u16 = 8000;
