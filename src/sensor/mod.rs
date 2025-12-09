//! Sensor data module for RoboMaster S1
//!
//! This module handles parsing and storage of sensor data received from the robot's
//! motion controller via CAN bus.
//!
//! Based on the RoboMaster CAN protocol (message type 0x0903).

use std::sync::{Arc, RwLock};

/// ESC (Electronic Speed Controller) data for all 4 wheels
/// Wheel order: front-left, front-right, rear-left, rear-right
#[derive(Debug, Clone, Default)]
pub struct EscData {
    /// Wheel speeds in RPM, range: -8192 to 8191
    pub speeds: [i16; 4],
    /// Wheel angles, range: 0-32767 maps to 0-360 degrees
    pub angles: [i16; 4],
    /// Timestamps for each wheel (microseconds)
    pub timestamps: [u32; 4],
    /// State flags for each wheel
    pub states: [u8; 4],
    /// Indicates if data has been received
    pub has_data: bool,
}

impl EscData {
    /// Convert wheel speed from RPM to rad/s
    pub fn speed_rad_s(&self, wheel_index: usize) -> f32 {
        if wheel_index >= 4 {
            return 0.0;
        }
        // RPM to rad/s: (RPM * 2 * PI) / 60
        (self.speeds[wheel_index] as f32) * std::f32::consts::PI * 2.0 / 60.0
    }

    /// Get all wheel speeds in rad/s
    pub fn speeds_rad_s(&self) -> [f32; 4] {
        [
            self.speed_rad_s(0),
            self.speed_rad_s(1),
            self.speed_rad_s(2),
            self.speed_rad_s(3),
        ]
    }

    /// Convert wheel angle from raw value to radians
    pub fn angle_rad(&self, wheel_index: usize) -> f32 {
        if wheel_index >= 4 {
            return 0.0;
        }
        // 0-32767 maps to 0-2*PI
        (self.angles[wheel_index] as f32) * std::f32::consts::PI * 2.0 / 32768.0
    }
}

/// IMU (Inertial Measurement Unit) data
#[derive(Debug, Clone, Default)]
pub struct ImuData {
    /// Acceleration in m/s² (x, y, z)
    pub accel: [f32; 3],
    /// Angular velocity in rad/s (x, y, z)
    pub gyro: [f32; 3],
    /// Indicates if data has been received
    pub has_data: bool,
}

/// Attitude data (orientation)
#[derive(Debug, Clone, Default)]
pub struct AttitudeData {
    /// Yaw angle in radians
    pub yaw: f32,
    /// Pitch angle in radians
    pub pitch: f32,
    /// Roll angle in radians
    pub roll: f32,
    /// Indicates if data has been received
    pub has_data: bool,
}

/// Position data from motion controller's internal odometry
#[derive(Debug, Clone, Default)]
pub struct PositionData {
    /// X position in meters (since power-on)
    pub x: f32,
    /// Y position in meters
    pub y: f32,
    /// Z position in meters (usually 0 for ground robot)
    pub z: f32,
    /// Indicates if data has been received
    pub has_data: bool,
}

/// Velocity data from motion controller
#[derive(Debug, Clone, Default)]
pub struct VelocityData {
    /// Velocity in global frame (m/s)
    pub global: [f32; 3],
    /// Velocity in body frame (m/s)
    pub body: [f32; 3],
    /// Indicates if data has been received
    pub has_data: bool,
}

/// Battery status data
#[derive(Debug, Clone, Default)]
pub struct BatteryData {
    /// ADC raw value
    pub adc: u16,
    /// Temperature in Celsius
    pub temperature: u16,
    /// Current in mA (can be negative for charging)
    pub current: i32,
    /// Battery percentage (0-100)
    pub percent: u8,
    /// Indicates if data has been received
    pub has_data: bool,
}

/// Complete sensor state containing all sensor data
#[derive(Debug, Clone, Default)]
pub struct SensorState {
    pub esc: EscData,
    pub imu: ImuData,
    pub attitude: AttitudeData,
    pub position: PositionData,
    pub velocity: VelocityData,
    pub battery: BatteryData,
    /// Timestamp of last update (microseconds since epoch)
    pub last_update_us: u64,
}

/// Thread-safe sensor data store
pub type SharedSensorState = Arc<RwLock<SensorState>>;

/// Create a new shared sensor state
pub fn new_shared_sensor_state() -> SharedSensorState {
    Arc::new(RwLock::new(SensorState::default()))
}

/// Sensor data parser for RoboMaster CAN protocol
pub struct SensorParser;

impl SensorParser {
    /// Message type for motion controller state data
    pub const MSG_TYPE_MOTION_STATE: u16 = 0x0903;

    /// Payload header for RoboMaster state data
    pub const STATE_HEADER: [u8; 4] = [0x20, 0x48, 0x08, 0x00];

    /// Subtype IDs in the state payload
    pub const SUBTYPE_ATTITUDE_VELOCITY: u8 = 0x01;
    pub const SUBTYPE_ESC: u8 = 0x02;
    pub const SUBTYPE_IMU: u8 = 0x03;
    pub const SUBTYPE_POSITION: u8 = 0x04;

    /// ESC data payload size
    pub const ESC_DATA_SIZE: usize = 36;
    /// IMU data payload size
    pub const IMU_DATA_SIZE: usize = 24;
    /// Position data payload size
    pub const POSITION_DATA_SIZE: usize = 12;
    /// Velocity data payload size
    pub const VELOCITY_DATA_SIZE: usize = 24;
    /// Attitude data payload size
    pub const ATTITUDE_DATA_SIZE: usize = 12;
    /// Battery data payload size
    pub const BATTERY_DATA_SIZE: usize = 10;
    /// Attitude+Velocity combined payload size (subtype 0x01)
    pub const ATTITUDE_VELOCITY_SIZE: usize = 46;

    /// Parse ESC data from payload bytes
    /// Expected format: 4x int16 speeds + 4x int16 angles + 4x uint32 timestamps + 4x uint8 states
    pub fn parse_esc_data(payload: &[u8]) -> Option<EscData> {
        if payload.len() < Self::ESC_DATA_SIZE {
            return None;
        }

        let mut data = EscData::default();

        // Parse speeds (int16, little-endian)
        for i in 0..4 {
            let offset = i * 2;
            data.speeds[i] = i16::from_le_bytes([payload[offset], payload[offset + 1]]);
        }

        // Parse angles (int16, little-endian)
        for i in 0..4 {
            let offset = 8 + i * 2;
            data.angles[i] = i16::from_le_bytes([payload[offset], payload[offset + 1]]);
        }

        // Parse timestamps (uint32, little-endian)
        for i in 0..4 {
            let offset = 16 + i * 4;
            data.timestamps[i] = u32::from_le_bytes([
                payload[offset],
                payload[offset + 1],
                payload[offset + 2],
                payload[offset + 3],
            ]);
        }

        // Parse states (uint8)
        for i in 0..4 {
            data.states[i] = payload[32 + i];
        }

        data.has_data = true;
        Some(data)
    }

    /// Parse IMU data from payload bytes
    /// Expected format: 3x float accel + 3x float gyro
    pub fn parse_imu_data(payload: &[u8]) -> Option<ImuData> {
        if payload.len() < Self::IMU_DATA_SIZE {
            return None;
        }

        let mut data = ImuData::default();

        // Parse acceleration (float, little-endian)
        for i in 0..3 {
            let offset = i * 4;
            data.accel[i] = f32::from_le_bytes([
                payload[offset],
                payload[offset + 1],
                payload[offset + 2],
                payload[offset + 3],
            ]);
        }

        // Parse gyroscope (float, little-endian)
        for i in 0..3 {
            let offset = 12 + i * 4;
            data.gyro[i] = f32::from_le_bytes([
                payload[offset],
                payload[offset + 1],
                payload[offset + 2],
                payload[offset + 3],
            ]);
        }

        data.has_data = true;
        Some(data)
    }

    /// Parse position data from payload bytes
    /// Expected format: 3x float (x, y, z)
    pub fn parse_position_data(payload: &[u8]) -> Option<PositionData> {
        if payload.len() < Self::POSITION_DATA_SIZE {
            return None;
        }

        let mut data = PositionData::default();

        data.x = f32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]]);
        data.y = f32::from_le_bytes([payload[4], payload[5], payload[6], payload[7]]);
        data.z = f32::from_le_bytes([payload[8], payload[9], payload[10], payload[11]]);

        data.has_data = true;
        Some(data)
    }

    /// Parse velocity data from payload bytes
    /// Expected format: 3x float global + 3x float body
    pub fn parse_velocity_data(payload: &[u8]) -> Option<VelocityData> {
        if payload.len() < Self::VELOCITY_DATA_SIZE {
            return None;
        }

        let mut data = VelocityData::default();

        // Parse global velocity
        for i in 0..3 {
            let offset = i * 4;
            data.global[i] = f32::from_le_bytes([
                payload[offset],
                payload[offset + 1],
                payload[offset + 2],
                payload[offset + 3],
            ]);
        }

        // Parse body velocity
        for i in 0..3 {
            let offset = 12 + i * 4;
            data.body[i] = f32::from_le_bytes([
                payload[offset],
                payload[offset + 1],
                payload[offset + 2],
                payload[offset + 3],
            ]);
        }

        data.has_data = true;
        Some(data)
    }

    /// Parse attitude data from payload bytes
    /// Expected format: 3x float (yaw, pitch, roll)
    pub fn parse_attitude_data(payload: &[u8]) -> Option<AttitudeData> {
        if payload.len() < Self::ATTITUDE_DATA_SIZE {
            return None;
        }

        let mut data = AttitudeData::default();

        data.yaw = f32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]]);
        data.pitch = f32::from_le_bytes([payload[4], payload[5], payload[6], payload[7]]);
        data.roll = f32::from_le_bytes([payload[8], payload[9], payload[10], payload[11]]);

        data.has_data = true;
        Some(data)
    }

    /// Parse combined attitude and velocity data from subtype 0x01 payload
    /// Based on RoboMaster S1 protocol analysis and observed data patterns.
    ///
    /// Verified layout (46 bytes total):
    /// [0-3]: counter/timestamp (u32)
    /// [4-21]: unknown/reserved
    /// [22-25]: yaw (float) in radians
    /// [26-29]: pitch (float) in radians
    /// [30-33]: roll (float) in radians
    /// [34-37]: body vx (float) in m/s - forward velocity
    /// [38-41]: body vy (float) in m/s - lateral velocity
    /// [42-45]: body vz / omega (float) - vertical/angular
    pub fn parse_attitude_velocity_combined(payload: &[u8]) -> Option<(AttitudeData, VelocityData)> {
        if payload.len() < 46 {
            return None;
        }

        let mut attitude = AttitudeData::default();
        let mut velocity = VelocityData::default();

        // Attitude (offsets 22-33)
        attitude.yaw = f32::from_le_bytes([payload[22], payload[23], payload[24], payload[25]]);
        attitude.pitch = f32::from_le_bytes([payload[26], payload[27], payload[28], payload[29]]);
        attitude.roll = f32::from_le_bytes([payload[30], payload[31], payload[32], payload[33]]);

        // Body velocity (offsets 34-45)
        velocity.body[0] = f32::from_le_bytes([payload[34], payload[35], payload[36], payload[37]]);
        velocity.body[1] = f32::from_le_bytes([payload[38], payload[39], payload[40], payload[41]]);
        velocity.body[2] = f32::from_le_bytes([payload[42], payload[43], payload[44], payload[45]]);

        attitude.has_data = true;
        velocity.has_data = true;

        Some((attitude, velocity))
    }

    /// Parse battery data from payload bytes
    pub fn parse_battery_data(payload: &[u8]) -> Option<BatteryData> {
        if payload.len() < Self::BATTERY_DATA_SIZE {
            return None;
        }

        let mut data = BatteryData::default();

        data.adc = u16::from_le_bytes([payload[0], payload[1]]);
        data.temperature = u16::from_le_bytes([payload[2], payload[3]]);
        data.current = i32::from_le_bytes([payload[4], payload[5], payload[6], payload[7]]);
        data.percent = payload[8];

        data.has_data = true;
        Some(data)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_esc_speed_conversion() {
        let mut esc = EscData::default();
        esc.speeds[0] = 60; // 60 RPM

        // 60 RPM = 2*PI rad/s
        let speed = esc.speed_rad_s(0);
        assert!((speed - std::f32::consts::PI * 2.0).abs() < 0.01);
    }

    #[test]
    fn test_parse_esc_data() {
        let mut payload = vec![0u8; 36];

        // Set speed[0] = 100 (little-endian)
        payload[0] = 100;
        payload[1] = 0;

        // Set speed[1] = -100 (little-endian, two's complement)
        let neg_100 = (-100i16).to_le_bytes();
        payload[2] = neg_100[0];
        payload[3] = neg_100[1];

        let esc = SensorParser::parse_esc_data(&payload).unwrap();
        assert_eq!(esc.speeds[0], 100);
        assert_eq!(esc.speeds[1], -100);
        assert!(esc.has_data);
    }

    #[test]
    fn test_parse_imu_data() {
        let mut payload = vec![0u8; 24];

        // Set accel[0] = 9.81 (little-endian float)
        let gravity = 9.81f32.to_le_bytes();
        payload[0..4].copy_from_slice(&gravity);

        let imu = SensorParser::parse_imu_data(&payload).unwrap();
        assert!((imu.accel[0] - 9.81).abs() < 0.01);
        assert!(imu.has_data);
    }

    #[test]
    fn test_parse_position_data() {
        let mut payload = vec![0u8; 12];

        // Set x = 1.5
        let x = 1.5f32.to_le_bytes();
        payload[0..4].copy_from_slice(&x);

        let pos = SensorParser::parse_position_data(&payload).unwrap();
        assert!((pos.x - 1.5).abs() < 0.01);
        assert!(pos.has_data);
    }

    #[test]
    fn test_insufficient_data() {
        let payload = vec![0u8; 10]; // Too small for ESC data
        assert!(SensorParser::parse_esc_data(&payload).is_none());
    }
}
