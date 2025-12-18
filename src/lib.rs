//! # RoboMaster Rust Control Library
//!
//! A high-performance, safe Rust library for controlling RoboMaster S1 robots via CAN bus.
//!
//! ## Features
//!
//! - **Fast & Safe**: Written in Rust for memory safety and high performance
//! - **Async Support**: Built on Tokio for non-blocking operations
//! - **Protocol Complete**: Full implementation of RoboMaster CAN protocol
//! - **Joystick Control**: Real-time joystick input handling
//! - **Sensor Monitoring**: Battery, current, temperature monitoring
//! - **LED Control**: RGB LED control with animations
//! - **Configurable**: TOML-based configuration system
//!
//! ## Quick Start
//!
//! ```rust,no_run
//! use robomaster_rust::{MovementParams, LedColor};
//! 
//! #[tokio::main]
//! async fn main() -> Result<(), Box<dyn std::error::Error>> {
//!     // Example usage will be implemented in control module
//!     Ok(())
//! }
//! ```

#![allow(missing_docs)]
#![warn(rust_2018_idioms)]
#![allow(dead_code)] // Remove this as implementation progresses

// Core modules
pub mod can;
pub mod command;
pub mod control;
pub mod crc;
pub mod error;
pub mod sensor;

// Optional modules
#[cfg(feature = "cli")]
pub mod joystick;

// Re-exports for convenience
pub use crate::command::{MovementParams, GimbalParams, LedColor};
pub use crate::can::{CanInterface, CommandCounters};
pub use crate::control::{RoboMaster, MovementCommand, LedCommand, SensorData};
pub use crate::error::RoboMasterError;
pub use crate::sensor::{EscData, ImuData, AttitudeData, PositionData, VelocityData, SensorState};
#[cfg(feature = "cli")]
pub use crate::joystick::{JoystickController, JoystickManager, ControllerInput};

#[cfg(feature = "cli")]
pub use crate::joystick::JoystickController as JoystickControllerCli;

/// Library version
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// Default CAN interface name
pub const DEFAULT_CAN_INTERFACE: &str = "can0";

/// Maximum safe speed value (normalized, -1.0 to 1.0)
pub const MAX_SPEED: f32 = 1.0;

/// Control loop frequency in Hz
pub const CONTROL_FREQUENCY: u32 = 100;

/// CAN message timeout in milliseconds
pub const CAN_TIMEOUT_MS: u64 = 200;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_constants() {
        assert_eq!(DEFAULT_CAN_INTERFACE, "can0");
        assert_eq!(MAX_SPEED, 1.0);
        assert_eq!(CONTROL_FREQUENCY, 100);
        assert_eq!(CAN_TIMEOUT_MS, 200);
    }
}
