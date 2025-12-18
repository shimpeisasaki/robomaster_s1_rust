/// Command builder for creating RoboMaster protocol messages
/// This module contains the core logic for building commands from templates

use crate::command::{get_command_table, commands, get_command_length, is_crc8_position, is_counter_position};
use crate::crc::{crc8::append_crc8_checksum, crc16::append_crc16_checksum};
use crate::can::CommandCounters;
use crate::error::{RoboMasterError, ProtocolError};
use anyhow::Result;

/// Movement command parameters
#[derive(Debug, Clone, Copy, Default)]
pub struct MovementParams {
    pub vx: f32,  // Linear velocity X (forward/backward)
    pub vy: f32,  // Linear velocity Y (left/right)  
    pub vz: f32,  // Angular velocity Z (rotation)
}

/// Gimbal command parameters
#[derive(Debug, Clone, Copy)]
pub struct GimbalParams {
    pub ry: f32,  // Rotation around Y axis (pitch)
    pub rz: f32,  // Rotation around Z axis (yaw)
}

/// LED color parameters
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct LedColor {
    pub red: u8,
    pub green: u8,
    pub blue: u8,
}

/// Command builder for creating protocol messages
pub struct CommandBuilder {
    command_table: Vec<Vec<u8>>,
}

impl CommandBuilder {
    /// Create a new command builder
    pub fn new() -> Self {
        Self {
            command_table: get_command_table(),
        }
    }

    /// Build boot sequence commands
    pub fn build_boot_sequence(&self) -> Result<Vec<u8>, RoboMasterError> {
        let mut boot_commands = Vec::new();
        
        // Build boot commands (26-34)
        for command_no in 26..=34 {
            let cmd = self.build_command_from_template(command_no, &CommandCounters::default())?;
            boot_commands.extend(cmd);
        }
        
        // Add LED on command
        let led_on_cmd = self.build_led_on_command(&CommandCounters::default())?;
        boot_commands.extend(led_on_cmd);
        
        Ok(boot_commands)
    }

    /// Build LED on command
    pub fn build_led_on_command(&self, counters: &CommandCounters) -> Result<Vec<u8>, RoboMasterError> {
        self.build_command_with_counter(commands::LED_ON, counters.led)
    }

    /// Build LED color command
    pub fn build_led_command(&self, color: LedColor, counters: &CommandCounters) -> Result<Vec<u8>, RoboMasterError> {
        let command_no = commands::LED_COLOR;
        let template = self.get_command_template(command_no)?;
        let command_length = get_command_length(template)
            .ok_or_else(|| RoboMasterError::Protocol(ProtocolError::InvalidCommandLength {
                command_id: command_no,
            }))?;

        let mut header_command = Vec::new();
        
        // Build command excluding CRC16 (last 2 bytes)
        for i in 0..(command_length - 2) {
            if is_crc8_position(template, i) {
                append_crc8_checksum(&mut header_command);
            } else if is_counter_position(template, i) {
                if i == 6 {
                    header_command.push((counters.led & 0xFF) as u8);
                } else if i == 7 {
                    header_command.push(((counters.led >> 8) & 0xFF) as u8);
                }
            } else if i == 14 {
                // RED color
                header_command.push(color.red);
            } else if i == 15 {
                // GREEN color
                header_command.push(color.green);
            } else if i == 16 {
                // BLUE color
                header_command.push(color.blue);
            } else {
                header_command.push(template[i]);
            }
        }
        
        append_crc16_checksum(&mut header_command, crate::crc::crc16::CRC16_INIT);
        Ok(header_command)
    }

    /// Build twist (movement) command
    pub fn build_twist_command(&self, params: MovementParams, counters: &CommandCounters) -> Result<Vec<u8>, RoboMasterError> {
        let command_no = commands::TWIST;
        let template = self.get_command_template(command_no)?;
        let command_length = get_command_length(template)
            .ok_or_else(|| RoboMasterError::Protocol(ProtocolError::InvalidCommandLength {
                command_id: command_no,
            }))?;

        let mut header_command = Vec::new();

        // Convert movement parameters to protocol values
        // Deadzone compensation: Add offset corresponding to 0.2 m/s to overcome static friction
        let deadzone = 0.2;
        let vx_offset = if params.vx > 0.0 { deadzone } else if params.vx < 0.0 { -deadzone } else { 0.0 };
        let vy_offset = if params.vy > 0.0 { deadzone } else if params.vy < 0.0 { -deadzone } else { 0.0 };
        let vz_offset = if params.vz > 0.0 { deadzone } else if params.vz < 0.0 { -deadzone } else { 0.0 };

        // Gain adjustment: 1024.0 * (1.5 / 1.09) approx 1409.0
        let gain_x = 400.0;
        let gain_y = 256.0;
        let gain_z = 256.0;
        let linear_x = ((gain_x * (params.vx + vx_offset) + 1024.0) as i32).clamp(0, 2047) as u16;
        let linear_y = ((gain_y * (params.vy + vy_offset) + 1024.0) as i32).clamp(0, 2047) as u16;
        let angular_z = ((gain_z * (params.vz + vz_offset) + 1024.0) as i32).clamp(0, 2047) as u16;

        eprintln!("Twist Params - vx: {}, vy: {}, vz: {}", params.vx, params.vy, params.vz);
        eprintln!("CAN linear_x: {}", linear_x);

        // Build command excluding CRC16 (last 2 bytes)
        for i in 0..(command_length - 2) {
            if is_crc8_position(template, i) {
                append_crc8_checksum(&mut header_command);
            } else if is_counter_position(template, i) {
                if i == 6 {
                    header_command.push((counters.joy & 0xFF) as u8);
                } else if i == 7 {
                    header_command.push(((counters.joy >> 8) & 0xFF) as u8);
                }
            } else if i == 13 {
                let tmp = (template[i] & 0xC0) | (((linear_x >> 5) & 0x3F) as u8);
                header_command.push(tmp);
            } else if i == 12 {
                let tmp = ((linear_x << 3) & 0xFF) | ((linear_y >> 8) & 0x07);
                header_command.push(tmp as u8);
            } else if i == 11 {
                header_command.push((linear_y & 0xFF) as u8);
            } else if i == 17 {
                header_command.push(((angular_z >> 4) & 0xFF) as u8);
            } else if i == 16 {
                let tmp = ((angular_z << 4) & 0xFF) | 0x08;
                header_command.push(tmp as u8);
            } else if i == 18 {
                header_command.push(0x00);
            } else if i == 19 {
                let tmp = 0x02 | ((angular_z << 2) & 0xFF);
                header_command.push(tmp as u8);
            } else if i == 20 {
                header_command.push(((angular_z >> 6) & 0xFF) as u8);
            } else if i == 21 {
                header_command.push(0x04);
            } else if i == 22 {
                header_command.push(0x0C); // Enable Flag 4:x-y 8:yaw 0x0c
            } else if i == 23 {
                header_command.push(0x00);
            } else if i == 24 {
                header_command.push(0x04);
            } else {
                header_command.push(template[i]);
            }
        }

        append_crc16_checksum(&mut header_command, crate::crc::crc16::CRC16_INIT);
        Ok(header_command)
    }

    /// Build gimbal command
    pub fn build_gimbal_command(&self, params: GimbalParams, counters: &CommandCounters) -> Result<Vec<u8>, RoboMasterError> {
        let command_no = commands::GIMBAL;
        let template = self.get_command_template(command_no)?;
        let command_length = get_command_length(template)
            .ok_or_else(|| RoboMasterError::Protocol(ProtocolError::InvalidCommandLength {
                command_id: command_no,
            }))?;

        let mut header_command = Vec::new();

        // Convert gimbal parameters to protocol values
        let angular_y = (-1024.0 * params.ry) as i16;
        let angular_z = (-1024.0 * params.rz) as i16;

        // Build command excluding CRC16 (last 2 bytes)
        for i in 0..(command_length - 2) {
            if is_crc8_position(template, i) {
                append_crc8_checksum(&mut header_command);
            } else if is_counter_position(template, i) {
                if i == 6 {
                    header_command.push((counters.gimbal & 0xFF) as u8);
                } else if i == 7 {
                    header_command.push(((counters.gimbal >> 8) & 0xFF) as u8);
                }
            } else if i == 14 {
                header_command.push(((angular_y >> 8) & 0xFF) as u8);
            } else if i == 13 {
                header_command.push((angular_y & 0xFF) as u8);
            } else if i == 16 {
                header_command.push(((angular_z >> 8) & 0xFF) as u8);
            } else if i == 15 {
                header_command.push((angular_z & 0xFF) as u8);
            } else {
                header_command.push(template[i]);
            }
        }

        append_crc16_checksum(&mut header_command, crate::crc::crc16::CRC16_INIT);
        Ok(header_command)
    }

    /// Build touch command
    pub fn build_touch_command(&self, counters: &CommandCounters) -> Result<Vec<Vec<u8>>, RoboMasterError> {
        let touch_msg_list = vec![
            vec![
                0x55, 0x0f, 0x04, 0xa2, 0x09, 0x04,
                (counters.joy & 0xFF) as u8,
                ((counters.joy >> 8) & 0xFF) as u8,
            ],
            vec![0x40, 0x04, 0x4c, 0x00, 0x00],
        ];

        // Calculate CRC16 for combined message
        let mut combined_msg = touch_msg_list[0].clone();
        combined_msg.extend(&touch_msg_list[1]);
        
        let crc16 = crate::crc::crc16::get_crc16_checksum(&combined_msg, crate::crc::crc16::CRC16_INIT);
        
        let mut result = touch_msg_list;
        result[1].push((crc16 & 0xFF) as u8);
        result[1].push(((crc16 >> 8) & 0xFF) as u8);

        Ok(result)
    }

    /// Generic command builder from template
    fn build_command_from_template(&self, command_no: usize, _counters: &CommandCounters) -> Result<Vec<u8>, RoboMasterError> {
        let template = self.get_command_template(command_no)?;
        let command_length = get_command_length(template)
            .ok_or_else(|| RoboMasterError::Protocol(ProtocolError::InvalidCommandLength {
                command_id: command_no,
            }))?;

        let mut header_command = Vec::new();

        // Build command excluding CRC16 (last 2 bytes)
        for i in 0..(command_length - 2) {
            if is_crc8_position(template, i) {
                append_crc8_checksum(&mut header_command);
            } else {
                header_command.push(template[i]);
            }
        }

        append_crc16_checksum(&mut header_command, crate::crc::crc16::CRC16_INIT);
        Ok(header_command)
    }

    /// Generic command builder with counter
    fn build_command_with_counter(&self, command_no: usize, counter: u16) -> Result<Vec<u8>, RoboMasterError> {
        let template = self.get_command_template(command_no)?;
        let command_length = get_command_length(template)
            .ok_or_else(|| RoboMasterError::Protocol(ProtocolError::InvalidCommandLength {
                command_id: command_no,
            }))?;

        let mut header_command = Vec::new();

        // Build command excluding CRC16 (last 2 bytes)
        for i in 0..(command_length - 2) {
            if is_crc8_position(template, i) {
                append_crc8_checksum(&mut header_command);
            } else if is_counter_position(template, i) {
                if i == 6 {
                    header_command.push((counter & 0xFF) as u8);
                } else if i == 7 {
                    header_command.push(((counter >> 8) & 0xFF) as u8);
                }
            } else {
                header_command.push(template[i]);
            }
        }

        append_crc16_checksum(&mut header_command, crate::crc::crc16::CRC16_INIT);
        Ok(header_command)
    }

    /// Get command template by index
    fn get_command_template(&self, command_no: usize) -> Result<&Vec<u8>, RoboMasterError> {
        self.command_table.get(command_no)
            .ok_or_else(|| RoboMasterError::Protocol(ProtocolError::CommandNotFound {
                command_id: command_no,
            }))
    }
}

impl Default for CommandBuilder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_command_builder_creation() {
        let builder = CommandBuilder::new();
        assert_eq!(builder.command_table.len(), 38);
    }

    #[test]
    fn test_led_color_command() {
        let builder = CommandBuilder::new();
        let color = LedColor { red: 255, green: 128, blue: 64 };
        let counters = CommandCounters::default();
        
        let result = builder.build_led_command(color, &counters);
        assert!(result.is_ok());
        
        let cmd = result.unwrap();
        assert!(!cmd.is_empty());
        assert_eq!(cmd[0], 0x55); // Header
        
        // Check that RGB values are in the command
        assert!(cmd.contains(&255)); // Red
        assert!(cmd.contains(&128)); // Green
        assert!(cmd.contains(&64));  // Blue
    }

    #[test]
    fn test_movement_params() {
        let params = MovementParams {
            vx: 1.0,
            vy: 0.5,
            vz: -0.5,
        };
        
        let builder = CommandBuilder::new();
        let counters = CommandCounters::default();
        
        let result = builder.build_twist_command(params, &counters);
        assert!(result.is_ok());
        
        let cmd = result.unwrap();
        assert!(!cmd.is_empty());
        assert_eq!(cmd[0], 0x55); // Header
    }

    #[test]
    fn test_gimbal_params() {
        let params = GimbalParams {
            ry: 0.1,
            rz: -0.2,
        };
        
        let builder = CommandBuilder::new();
        let counters = CommandCounters::default();
        
        let result = builder.build_gimbal_command(params, &counters);
        assert!(result.is_ok());
        
        let cmd = result.unwrap();
        assert!(!cmd.is_empty());
        assert_eq!(cmd[0], 0x55); // Header
    }

    #[test]
    fn test_boot_sequence() {
        let builder = CommandBuilder::new();
        let result = builder.build_boot_sequence();
        assert!(result.is_ok());
        
        let cmd = result.unwrap();
        assert!(!cmd.is_empty());
    }

    #[test]
    fn test_touch_command() {
        let builder = CommandBuilder::new();
        let counters = CommandCounters::default();
        
        let result = builder.build_touch_command(&counters);
        assert!(result.is_ok());
        
        let msgs = result.unwrap();
        assert_eq!(msgs.len(), 2);
        assert_eq!(msgs[0][0], 0x55);
        assert_eq!(msgs[1][0], 0x40);
    }

    #[test]
    fn test_invalid_command_index() {
        let builder = CommandBuilder::new();
        let result = builder.get_command_template(999);
        assert!(result.is_err());
    }
}
