/// Control system module for RoboMaster robot
/// This module provides high-level control APIs

use crate::can::{CanInterface, CommandCounters, MessageSplitter, MessageBuffer};
use crate::command::{CommandBuilder, MovementParams, GimbalParams, LedColor};
use crate::error::RoboMasterError;
use crate::sensor::{SensorState, SharedSensorState, new_shared_sensor_state};
use anyhow::Result;

/// High-level RoboMaster robot controller
pub struct RoboMaster {
    can_interface: CanInterface,
    command_builder: CommandBuilder,
    command_counters: CommandCounters,
    is_initialized: bool,
    sensor_state: SharedSensorState,
    message_buffer: MessageBuffer,
}

impl RoboMaster {
    /// Create a new RoboMaster controller
    pub async fn new(interface_name: &str) -> Result<Self, RoboMasterError> {
        let can_interface = CanInterface::new(interface_name)?;
        let command_builder = CommandBuilder::new();
        let command_counters = CommandCounters::default();
        let sensor_state = new_shared_sensor_state();
        let message_buffer = MessageBuffer::new();

        Ok(Self {
            can_interface,
            command_builder,
            command_counters,
            is_initialized: false,
            sensor_state,
            message_buffer,
        })
    }

    /// Initialize the robot (boot sequence)
    pub async fn initialize(&mut self) -> Result<(), RoboMasterError> {
        if self.is_initialized {
            return Ok(());
        }

        println!("Initializing RoboMaster...");
        let boot_command = self.command_builder.build_boot_sequence()?;
        let can_messages = MessageSplitter::split_command(&boot_command);
        self.can_interface.send_messages(&can_messages)?;
        
        // Wait for initialization to complete
        tokio::time::sleep(std::time::Duration::from_millis(500)).await;
        
        self.is_initialized = true;
        println!("RoboMaster initialized successfully");
        Ok(())
    }

    /// Ensure the robot is initialized before executing commands
    async fn ensure_initialized(&mut self) -> Result<(), RoboMasterError> {
        if !self.is_initialized {
            self.initialize().await?;
        }
        Ok(())
    }

    /// Move the robot with specified parameters
    pub async fn move_robot(&mut self, movement: MovementParams) -> Result<(), RoboMasterError> {
        self.ensure_initialized().await?;
        
        // Build twist command
        let twist_cmd = self.command_builder.build_twist_command(movement, &self.command_counters)?;
        let twist_messages = MessageSplitter::split_command(&twist_cmd);

        // Build gimbal command (use rotation from movement for gimbal yaw)
        let gimbal_params = GimbalParams {
            ry: 0.0,
            rz: movement.vz,
        };
        let gimbal_cmd = self.command_builder.build_gimbal_command(gimbal_params, &self.command_counters)?;
        let gimbal_messages = MessageSplitter::split_command(&gimbal_cmd);

        // Send commands
        self.can_interface.send_messages(&twist_messages)?;
        self.can_interface.send_messages(&gimbal_messages)?;

        // Update counters
        self.command_counters.joy = self.command_counters.joy.wrapping_add(1);
        self.command_counters.gimbal = self.command_counters.gimbal.wrapping_add(1);

        Ok(())
    }

    /// Control LED color
    pub async fn control_led(&mut self, color: LedColor) -> Result<(), RoboMasterError> {
        let led_cmd = self.command_builder.build_led_command(color, &self.command_counters)?;
        let led_messages = MessageSplitter::split_command(&led_cmd);
        self.can_interface.send_messages(&led_messages)?;
        
        // Update counter
        self.command_counters.led += 1;
        
        Ok(())
    }

    /// Send touch command
    pub async fn send_touch(&mut self) -> Result<(), RoboMasterError> {
        let touch_messages = self.command_builder.build_touch_command(&self.command_counters)?;
        self.can_interface.send_messages(&touch_messages)?;
        
        // Update counter
        self.command_counters.joy += 1;
        
        Ok(())
    }

    /// Receive messages and update internal state
    pub async fn receive_messages(&mut self) -> Result<(), RoboMasterError> {
        self.can_interface.receive_and_process(&mut self.command_counters).await
    }

    /// Receive messages and update sensor data
    pub async fn receive_sensor_data(&mut self) -> Result<(), RoboMasterError> {
        self.can_interface
            .receive_and_process_sensors(
                &mut self.command_counters,
                &self.sensor_state,
                &mut self.message_buffer,
            )
            .await
    }

    /// Get a copy of the current sensor state
    pub fn get_sensor_state(&self) -> Option<SensorState> {
        self.sensor_state.read().ok().map(|s| s.clone())
    }

    /// Get shared reference to sensor state (for FFI)
    pub fn get_shared_sensor_state(&self) -> SharedSensorState {
        self.sensor_state.clone()
    }

    /// Stop the robot (send zero movement)
    pub async fn stop(&mut self) -> Result<(), RoboMasterError> {
        let stop_movement = MovementParams {
            vx: 0.0,
            vy: 0.0,
            vz: 0.0,
        };
        self.move_robot(stop_movement).await
    }

    /// Shutdown the robot controller
    pub async fn shutdown(self) -> Result<(), RoboMasterError> {
        // Stop movement before shutdown
        // Note: We need to take ownership here, so we can't call self.stop()
        self.can_interface.shutdown();
        Ok(())
    }

    /// Get current command counters
    pub fn get_counters(&self) -> &CommandCounters {
        &self.command_counters
    }

    /// Get CAN interface name
    pub fn interface_name(&self) -> &str {
        self.can_interface.interface_name()
    }
}

/// Movement command builder for ergonomic API
#[derive(Debug, Clone, Copy, Default)]
pub struct MovementCommand {
    params: MovementParams,
}

impl MovementCommand {
    /// Create a new movement command
    pub fn new() -> Self {
        Self::default()
    }

    /// Set forward/backward movement (-1.0 to 1.0)
    pub fn forward(mut self, speed: f32) -> Self {
        self.params.vx = speed.clamp(-1.0, 1.0);
        self
    }

    /// Set strafe left/right movement (-1.0 to 1.0)
    pub fn strafe_right(mut self, speed: f32) -> Self {
        self.params.vy = speed.clamp(-1.0, 1.0);
        self
    }

    /// Set rotation (-1.0 to 1.0)
    pub fn rotate_right(mut self, speed: f32) -> Self {
        self.params.vz = speed.clamp(-1.0, 1.0);
        self
    }

    /// Convert to movement parameters
    pub fn into_params(self) -> MovementParams {
        self.params
    }
}

/// LED command builder for ergonomic API
#[derive(Debug, Clone, Copy, Default)]
pub struct LedCommand {
    color: LedColor,
}

impl LedCommand {
    /// Create a new LED command
    pub fn new() -> Self {
        Self::default()
    }

    /// Set RGB color
    pub fn rgb(red: u8, green: u8, blue: u8) -> Self {
        Self {
            color: LedColor { red, green, blue },
        }
    }

    /// Set red color
    pub fn red() -> Self {
        Self::rgb(255, 0, 0)
    }

    /// Set green color
    pub fn green() -> Self {
        Self::rgb(0, 255, 0)
    }

    /// Set blue color
    pub fn blue() -> Self {
        Self::rgb(0, 0, 255)
    }

    /// Set white color
    pub fn white() -> Self {
        Self::rgb(255, 255, 255)
    }

    /// Turn off (black)
    pub fn off() -> Self {
        Self::rgb(0, 0, 0)
    }

    /// Get the LED color
    pub fn color(&self) -> LedColor {
        self.color
    }
}

/// Sensor data structure (placeholder for future implementation)
#[derive(Debug, Clone, Default)]
pub struct SensorData {
    /// Battery voltage (V)
    pub battery_voltage: f32,
    /// Current consumption (A)
    pub current: f32,
    /// Temperature (°C)
    pub temperature: f32,
    /// IMU data placeholder
    pub imu: ImuData,
}

/// IMU data structure (placeholder)
#[derive(Debug, Clone, Default)]
pub struct ImuData {
    /// Acceleration in m/s²
    pub acceleration: [f32; 3],
    /// Angular velocity in rad/s
    pub angular_velocity: [f32; 3],
    /// Orientation in radians
    pub orientation: [f32; 3],
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_movement_command_builder() {
        let cmd = MovementCommand::new()
            .forward(0.5)
            .strafe_right(0.2)
            .rotate_right(-0.3);

        let params = cmd.into_params();
        assert_eq!(params.vx, 0.5);
        assert_eq!(params.vy, 0.2);
        assert_eq!(params.vz, -0.3);
    }

    #[test]
    fn test_movement_command_clamping() {
        let cmd = MovementCommand::new()
            .forward(2.0)  // Should be clamped to 1.0
            .strafe_right(-2.0)  // Should be clamped to -1.0
            .rotate_right(0.5);

        let params = cmd.into_params();
        assert_eq!(params.vx, 1.0);
        assert_eq!(params.vy, -1.0);
        assert_eq!(params.vz, 0.5);
    }

    #[test]
    fn test_led_command_colors() {
        assert_eq!(LedCommand::red().color().red, 255);
        assert_eq!(LedCommand::green().color().green, 255);
        assert_eq!(LedCommand::blue().color().blue, 255);
        assert_eq!(LedCommand::white().color(), LedColor { red: 255, green: 255, blue: 255 });
        assert_eq!(LedCommand::off().color(), LedColor { red: 0, green: 0, blue: 0 });
    }

    #[test]
    fn test_rgb_command() {
        let cmd = LedCommand::rgb(128, 64, 192);
        let color = cmd.color();
        assert_eq!(color.red, 128);
        assert_eq!(color.green, 64);
        assert_eq!(color.blue, 192);
    }
}
