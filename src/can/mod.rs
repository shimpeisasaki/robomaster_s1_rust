use anyhow::Result;
use crate::error::{RoboMasterError, CanError};
use crate::sensor::{SensorParser, SharedSensorState};
use crate::crc::{calculate_crc8, calculate_crc16, CRC16_INIT};
use socketcan::{CanSocket, CanFrame, Socket, EmbeddedFrame, StandardId};
use std::time::{Duration, Instant};
use std::os::unix::io::AsRawFd;

/// CAN arbitration ID for sending commands to RoboMaster
pub const ROBOMASTER_CAN_ID: u16 = 0x201;

/// CAN arbitration ID for receiving sensor data from RoboMaster
pub const ROBOMASTER_SENSOR_ID: u16 = 0x202;

/// Default timeout for CAN operations
pub const DEFAULT_CAN_TIMEOUT: Duration = Duration::from_millis(200);

/// Maximum CAN frame data length
pub const CAN_MAX_DATA_LEN: usize = 8;

/// CAN interface abstraction for RoboMaster communication
pub struct CanInterface {
    socket: CanSocket,
    interface_name: String,
}

impl CanInterface {
    /// Create a new CAN interface
    pub fn new(interface_name: &str) -> Result<Self, RoboMasterError> {
        println!("----------------------can open----------------------");

        let socket = CanSocket::open(interface_name)
            .map_err(|e| RoboMasterError::CanInterface(CanError::OpenFailed {
                interface: interface_name.to_string(),
                source: e,
            }))?;

        // Set socket to non-blocking mode
        let fd = socket.as_raw_fd();
        unsafe {
            let flags = libc::fcntl(fd, libc::F_GETFL);
            libc::fcntl(fd, libc::F_SETFL, flags | libc::O_NONBLOCK);
        }

        println!("generated can bus (non-blocking mode)");

        Ok(Self {
            socket,
            interface_name: interface_name.to_string(),
        })
    }

    /// Send a single CAN message
    pub fn send_message(&self, data: &[u8]) -> Result<(), RoboMasterError> {
        if data.len() > CAN_MAX_DATA_LEN {
            return Err(RoboMasterError::CanInterface(CanError::InvalidDataLength {
                length: data.len(),
                max_length: CAN_MAX_DATA_LEN,
            }));
        }

        let standard_id = StandardId::new(ROBOMASTER_CAN_ID)
            .ok_or_else(|| RoboMasterError::CanInterface(CanError::InvalidMessage {
                reason: "Invalid CAN ID".to_string(),
            }))?;
            
        let frame = CanFrame::new(standard_id, data)
            .ok_or_else(|| RoboMasterError::CanInterface(CanError::FrameCreation(
                std::io::Error::new(std::io::ErrorKind::InvalidData, "Failed to create CAN frame")
            )))?;

        self.socket.write_frame(&frame)
            .map_err(|e| RoboMasterError::CanInterface(CanError::SendFailed(e)))?;

        Ok(())
    }

    /// Send multiple CAN messages
    pub fn send_messages(&self, messages: &[Vec<u8>]) -> Result<(), RoboMasterError> {
        for msg in messages {
            self.send_message(msg)?;
        }
        Ok(())
    }

    /// Receive a CAN message (non-blocking)
    pub fn receive_message_nonblocking(&self) -> Result<Option<CanFrame>, RoboMasterError> {
        match self.socket.read_frame() {
            Ok(frame) => Ok(Some(frame)),
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => Ok(None),
            Err(e) => Err(RoboMasterError::CanInterface(CanError::ReceiveFailed(e))),
        }
    }

    /// Receive multiple CAN messages within a time budget
    pub fn receive_messages_batch(&self, timeout_duration: Duration, max_frames: usize) -> Vec<CanFrame> {
        let start = Instant::now();
        let mut frames = Vec::with_capacity(max_frames);

        while frames.len() < max_frames && start.elapsed() < timeout_duration {
            match self.receive_message_nonblocking() {
                Ok(Some(frame)) => frames.push(frame),
                Ok(None) => {
                    // No more data available, sleep briefly before retrying
                    std::thread::sleep(Duration::from_micros(100));
                }
                Err(_) => break,
            }
        }

        frames
    }

    /// Receive and process messages to extract command counters
    pub async fn receive_and_process(&self, cmd_counters: &mut CommandCounters) -> Result<(), RoboMasterError> {
        let frames = self.receive_messages_batch(DEFAULT_CAN_TIMEOUT, 64);

        for frame in frames {
            let frame_id = match frame.id() {
                socketcan::Id::Standard(std_id) => std_id.as_raw(),
                socketcan::Id::Extended(_) => continue,
            };

            if frame_id == ROBOMASTER_CAN_ID {
                let data = frame.data();
                if data.len() >= 8 && data[0..6] == [0x55, 0x1b, 0x04, 0x75, 0x09, 0xc3] {
                    let counter = (data[6] as u16) | ((data[7] as u16) << 8);
                    cmd_counters.joy = counter + 1;
                }
            }
        }
        Ok(())
    }

    /// Receive and process messages with sensor data extraction
    pub async fn receive_and_process_sensors(
        &self,
        cmd_counters: &mut CommandCounters,
        sensor_state: &SharedSensorState,
        message_buffer: &mut MessageBuffer,
    ) -> Result<(), RoboMasterError> {
        // Read multiple frames at once (up to 64 frames within timeout)
        let frames = self.receive_messages_batch(DEFAULT_CAN_TIMEOUT, 64);

        for frame in frames {
            let frame_id = match frame.id() {
                socketcan::Id::Standard(std_id) => std_id.as_raw(),
                socketcan::Id::Extended(_) => continue,
            };

            // Accept both 0x201 (commands) and 0x202 (sensor data)
            if frame_id == ROBOMASTER_CAN_ID || frame_id == ROBOMASTER_SENSOR_ID {
                let data = frame.data();

                // Accumulate multi-frame messages
                message_buffer.append(data);

                // Try to parse complete messages (there may be multiple)
                while let Some(complete_msg) = message_buffer.try_extract_message() {
                    self.process_complete_message(&complete_msg, cmd_counters, sensor_state)?;
                }
            }
        }
        Ok(())
    }

    /// Process a complete message and extract sensor data
    fn process_complete_message(
        &self,
        message: &[u8],
        cmd_counters: &mut CommandCounters,
        sensor_state: &SharedSensorState,
    ) -> Result<(), RoboMasterError> {
        // Minimum message: header(4) + type(2) + seq(2) + crc16(2) = 10 bytes
        if message.len() < 10 {
            return Ok(());
        }

        // Verify header
        if message[0] != 0x55 {
            return Ok(());
        }

        let length = message[1] as usize;
        if message.len() < length {
            return Ok(());
        }

        // Verify CRC8 (byte 3 is CRC8 of bytes 0-2)
        let crc8 = calculate_crc8(&message[0..3]);
        if message[3] != crc8 {
            return Ok(());
        }

        // Verify CRC16 (last 2 bytes are CRC16 of message[0..len-2])
        let crc16 = calculate_crc16(&message[0..length - 2], CRC16_INIT);
        let msg_crc16 = u16::from_le_bytes([message[length - 2], message[length - 1]]);
        if crc16 != msg_crc16 {
            return Ok(());
        }

        // Extract message type (bytes 4-5, little-endian)
        let msg_type = u16::from_le_bytes([message[4], message[5]]);

        // Extract sequence number (bytes 6-7)
        let seq = u16::from_le_bytes([message[6], message[7]]);
        cmd_counters.joy = seq + 1;

        // Process based on message type
        if msg_type == SensorParser::MSG_TYPE_MOTION_STATE {
            // Payload starts at byte 8
            let payload = &message[8..length - 2];
            self.parse_motion_state_payload(payload, sensor_state)?;
        }

        Ok(())
    }

    /// Parse motion controller state payload
    fn parse_motion_state_payload(
        &self,
        payload: &[u8],
        sensor_state: &SharedSensorState,
    ) -> Result<(), RoboMasterError> {
        // Check for state header [0x20, 0x48, 0x08, 0x00]
        if payload.len() < 5 {
            return Ok(());
        }

        if payload[0..4] != SensorParser::STATE_HEADER {
            return Ok(());
        }

        // Get subtype ID (byte 4 after header)
        let subtype = payload[4];
        let data = &payload[5..];

        match subtype {
            SensorParser::SUBTYPE_ATTITUDE_VELOCITY => {
                // Subtype 0x01: Combined attitude and velocity data
                if let Some((attitude, velocity)) = SensorParser::parse_attitude_velocity_combined(data) {
                    if let Ok(mut state) = sensor_state.write() {
                        state.attitude = attitude;
                        state.velocity = velocity;
                        state.last_update_us = std::time::SystemTime::now()
                            .duration_since(std::time::UNIX_EPOCH)
                            .unwrap_or_default()
                            .as_micros() as u64;
                    }
                }
            }
            SensorParser::SUBTYPE_ESC => {
                // Subtype 0x02: ESC (wheel motor) data
                if let Some(esc) = SensorParser::parse_esc_data(data) {
                    if let Ok(mut state) = sensor_state.write() {
                        state.esc = esc;
                    }
                }
            }
            SensorParser::SUBTYPE_IMU => {
                // Subtype 0x03: IMU data
                if let Some(imu) = SensorParser::parse_imu_data(data) {
                    if let Ok(mut state) = sensor_state.write() {
                        state.imu = imu;
                    }
                }
            }
            SensorParser::SUBTYPE_POSITION => {
                // Subtype 0x04: Position data
                if let Some(position) = SensorParser::parse_position_data(data) {
                    if let Ok(mut state) = sensor_state.write() {
                        state.position = position;
                    }
                }
            }
            _ => {
                // Unknown subtype, ignore
            }
        }

        Ok(())
    }

    /// Close the CAN interface
    pub fn shutdown(&self) {
        println!("----------------------shutdown----------------------");
        // The socket will be automatically closed when dropped
    }

    /// Get the interface name
    pub fn interface_name(&self) -> &str {
        &self.interface_name
    }
}

impl Drop for CanInterface {
    fn drop(&mut self) {
        self.shutdown();
    }
}

/// Command counters for different command types
#[derive(Debug, Clone)]
pub struct CommandCounters {
    pub joy: u16,
    pub led: u16,
    pub gimbal: u16,
}

impl Default for CommandCounters {
    fn default() -> Self {
        Self {
            joy: 0,
            led: 0,
            gimbal: 0,
        }
    }
}

/// Buffer for accumulating multi-frame CAN messages
#[derive(Default)]
pub struct MessageBuffer {
    buffer: Vec<u8>,
}

impl MessageBuffer {
    /// Create a new message buffer
    pub fn new() -> Self {
        Self { buffer: Vec::with_capacity(256) }
    }

    /// Append data from a CAN frame
    pub fn append(&mut self, data: &[u8]) {
        self.buffer.extend_from_slice(data);

        // Prevent buffer from growing too large
        if self.buffer.len() > 1024 {
            self.buffer.clear();
        }
    }

    /// Try to extract a complete message from the buffer
    pub fn try_extract_message(&mut self) -> Option<Vec<u8>> {
        // Find message start (0x55)
        let start_pos = self.buffer.iter().position(|&b| b == 0x55)?;

        // Remove any garbage before the start
        if start_pos > 0 {
            self.buffer.drain(0..start_pos);
        }

        // Need at least 2 bytes for header + length
        if self.buffer.len() < 2 {
            return None;
        }

        let length = self.buffer[1] as usize;

        // Check if we have the complete message
        if self.buffer.len() < length {
            return None;
        }

        // Extract the complete message
        let message: Vec<u8> = self.buffer.drain(0..length).collect();
        Some(message)
    }

    /// Clear the buffer
    pub fn clear(&mut self) {
        self.buffer.clear();
    }
}

/// Message splitter for converting commands to CAN frames
pub struct MessageSplitter;

impl MessageSplitter {
    /// Split a command into 8-byte CAN frames
    pub fn split_command(command: &[u8]) -> Vec<Vec<u8>> {
        let mut can_command_list = Vec::new();
        let chunks = (command.len() + CAN_MAX_DATA_LEN - 1) / CAN_MAX_DATA_LEN;
        
        for i in 0..chunks {
            let start = i * CAN_MAX_DATA_LEN;
            let end = std::cmp::min(start + CAN_MAX_DATA_LEN, command.len());
            can_command_list.push(command[start..end].to_vec());
        }
        
        can_command_list
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_message_splitter_exact_size() {
        let command = vec![1, 2, 3, 4, 5, 6, 7, 8];
        let result = MessageSplitter::split_command(&command);
        assert_eq!(result.len(), 1);
        assert_eq!(result[0], command);
    }

    #[test]
    fn test_message_splitter_multiple_frames() {
        let command = vec![1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12];
        let result = MessageSplitter::split_command(&command);
        assert_eq!(result.len(), 2);
        assert_eq!(result[0], vec![1, 2, 3, 4, 5, 6, 7, 8]);
        assert_eq!(result[1], vec![9, 10, 11, 12]);
    }

    #[test]
    fn test_message_splitter_uneven_split() {
        let command = vec![1, 2, 3, 4, 5, 6, 7, 8, 9];
        let result = MessageSplitter::split_command(&command);
        assert_eq!(result.len(), 2);
        assert_eq!(result[0], vec![1, 2, 3, 4, 5, 6, 7, 8]);
        assert_eq!(result[1], vec![9]);
    }

    #[test]
    fn test_command_counters_default() {
        let counters = CommandCounters::default();
        assert_eq!(counters.joy, 0);
        assert_eq!(counters.led, 0);
        assert_eq!(counters.gimbal, 0);
    }
}
