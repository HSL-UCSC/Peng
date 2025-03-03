//! Configuration module
//!
//! This module contains the configuration for the simulation, quadrotor, PID controller, IMU, maze, camera, mesh, and planner schedule.
//! The configuration is loaded from a YAML file using the serde library.
//! The configuration is then used to initialize the simulation, quadrotor, PID controller, IMU, maze, camera, mesh, and planner schedule.

use nalgebra::Matrix3;

use crate::SimulationError;

use serde::de::{Deserializer, Error};
use serde::Deserialize;
use serde_yaml::Value;

#[derive(Clone, serde::Deserialize)]
/// Configuration for the simulation
pub struct Config {
    /// Simulation configuration
    pub simulation: SimulationConfig,
    /// Quadrotor configuration
    #[serde(default, deserialize_with = "single_or_multiple_quadrotors")]
    pub quadrotor: Vec<QuadrotorConfigurations>,
    /// PID Controller configuration
    pub pid_controller: PIDControllerConfig,
    /// IMU configuration
    pub imu: ImuConfig,
    /// Maze configuration
    pub maze: MazeConfig,
    /// Camera configuration
    pub camera: CameraConfig,
    /// Mesh configuration
    pub mesh: MeshConfig,
    /// Planner schedule configuration
    pub planner_schedule: Vec<PlannerStep>,
    /// Rerun blueprint path
    pub rerun_blueprint: String,
    /// Use rerun.io for recording
    pub use_rerun: bool,
    /// Render depth
    pub render_depth: bool,
    /// MultiThreading depth rendering
    pub use_multithreading_depth_rendering: bool,
    /// Angle limits
    pub angle_limits: Option<Vec<f32>>,
}

#[derive(Clone, serde::Deserialize)]
/// Configuration for a planner step
pub struct PlannerStep {
    /// Step number that the planner should be executed (Unit: ms)
    pub step: usize,
    /// Type of planner to use
    pub planner_type: String,
    /// Parameters for the planner
    pub params: serde_yaml::Value,
}

#[derive(Clone, serde::Deserialize)]
/// Configuration for the simulation
pub struct SimulationConfig {
    /// Gravity in m/s^2
    pub gravity: f32,
    /// Control frequency in Hz
    pub control_frequency: usize,
    /// Simulation frequency in Hz
    pub simulation_frequency: usize,
    /// Log frequency in Hz
    pub log_frequency: usize,
    /// Duration of the simulation in seconds
    pub duration: f32,
    /// Use RK4 for updating quadrotor dynamics_with_controls
    pub use_rk4_for_dynamics_control: bool,
    /// Use RK4 for updating quadrotor dynamics without controls
    pub use_rk4_for_dynamics_update: bool,
    /// Run the simulation in real time mode
    pub real_time: bool,
}

impl Default for SimulationConfig {
    fn default() -> Self {
        SimulationConfig {
            gravity: 9.81,
            control_frequency: 200,
            simulation_frequency: 1000,
            log_frequency: 70,
            duration: 70.0,
            use_rk4_for_dynamics_control: false,
            use_rk4_for_dynamics_update: false,
            real_time: false,
        }
    }
}

#[derive(Clone, Debug, serde::Deserialize)]
#[serde(tag = "type")]
/// Vehicle Specifig configuration
pub enum QuadrotorConfigurations {
    Peng(QuadrotorConfig),
    Liftoff(LiftoffQuadrotorConfig),
    Betaflight(Betaflight),
}

impl QuadrotorConfigurations {
    pub fn get_id(&self) -> String {
        match self {
            QuadrotorConfigurations::Peng(quadrotor_config) => quadrotor_config.id.clone(),
            QuadrotorConfigurations::Liftoff(config) => config.quadrotor_config.id.clone(),
            QuadrotorConfigurations::Betaflight(config) => config.quadrotor_config.id.clone(),
        }
    }
}

#[derive(Clone, Debug, serde::Deserialize)]
#[serde(default)]
/// Configuration for the quadrotor
pub struct QuadrotorConfig {
    /// Mass of the quadrotor in kg
    pub id: String,
    /// Mass of the quadrotor in kg
    pub mass: f32,
    /// Drag coefficient in Ns^2/m^2
    pub drag_coefficient: f32,
    /// Inertia matrix in kg*m^2
    pub inertia_matrix: [f32; 9],
    /// Maximum thrust in kilograms
    pub max_thrust_kg: f32,
    /// Arm length in meters
    pub arm_length_m: f32,
    /// Yaw torque constant
    pub yaw_torque_constant: f32,
    /// Initial Position
    pub initial_position: [f32; 3],
}

impl Default for QuadrotorConfig {
    fn default() -> Self {
        QuadrotorConfig {
            id: "quadrotor".to_string(),
            mass: 1.3,
            drag_coefficient: 0.000,
            inertia_matrix: [3.04e-3, 0.0, 0.0, 0.0, 4.55e-3, 0.0, 0.0, 0.0, 2.82e-3],
            max_thrust_kg: 1.3 * 2.5,
            arm_length_m: 0.150,
            yaw_torque_constant: 0.05,
            initial_position: [0.0, 0.0, 0.0],
        }
    }
}

impl QuadrotorConfig {
    pub fn inertia_matrix(&self) -> Matrix3<f32> {
        Matrix3::from_row_slice(&self.inertia_matrix)
    }

    pub fn inverse_inertia_matrix(&self) -> Result<Matrix3<f32>, SimulationError> {
        self.inertia_matrix()
            .try_inverse()
            .ok_or(SimulationError::NalgebraError(
                "Failed to invert inertia matrix".to_string(),
            ))
    }
}

#[derive(Clone, Debug, serde::Deserialize)]
#[serde(default)]
/// Configuration for the quadrotor
pub struct LiftoffQuadrotorConfig {
    /// Base quadrotor configurations
    pub quadrotor_config: QuadrotorConfig,
    /// The IP address where Liftoff is publishing state data
    pub ip_address: String,
    pub connection_timeout: tokio::time::Duration,
    pub max_retry_delay: tokio::time::Duration,
    pub serial_port: Option<String>,
    pub baud_rate: u32,
}

impl Default for LiftoffQuadrotorConfig {
    fn default() -> Self {
        LiftoffQuadrotorConfig {
            quadrotor_config: QuadrotorConfig {
                id: "LiftoffQuad".to_string(),
                ..Default::default()
            },
            ip_address: String::from("0.0.0.0:9001"),
            connection_timeout: tokio::time::Duration::from_secs(5 * 60),
            max_retry_delay: tokio::time::Duration::from_secs(30),
            serial_port: None,
            baud_rate: 460800,
        }
    }
}

#[derive(Clone, Debug, serde::Deserialize)]
#[serde(default)]
/// Configuration for the quadrotor
pub struct Betaflight {
    pub quadrotor_config: QuadrotorConfig,
    /// The IP address where Liftoff is publishing state data
    pub vicon_address: String,
    pub connection_timeout: tokio::time::Duration,
    pub max_retry_delay: tokio::time::Duration,
    pub serial_port: Option<String>,
    pub baud_rate: u32,
}

impl Default for Betaflight {
    fn default() -> Self {
        Betaflight {
            quadrotor_config: QuadrotorConfig::default(),
            vicon_address: String::from("0.0.0.0:51001"),
            connection_timeout: tokio::time::Duration::from_secs(5 * 60),
            max_retry_delay: tokio::time::Duration::from_secs(30),
            serial_port: None,
            baud_rate: 460800,
        }
    }
}
impl Betaflight {
    /// Calculate all maximum torques and return them as a tuple
    pub fn max_torques(&self) -> (f32, f32, f32) {
        let motor_thrust = self.quadrotor_config.max_thrust_kg / 4.0;
        // The maximum roll and pitch torques
        let max_rp_torque = 2.0 * self.quadrotor_config.arm_length_m * motor_thrust;
        let yaw_torque = 2.0 * self.quadrotor_config.yaw_torque_constant * motor_thrust;
        (max_rp_torque, max_rp_torque, yaw_torque)
    }
}

#[derive(Clone, serde::Deserialize)]
/// Configuration for the PID controller
pub struct PIDControllerConfig {
    /// Position gains
    pub pos_gains: PIDGains,
    /// Attitude gains
    pub att_gains: PIDGains,
    /// Maximum integral error for position control
    pub pos_max_int: [f32; 3],
    /// Maximum integral error for attitude control
    pub att_max_int: [f32; 3],
}

#[derive(Clone, Copy, serde::Deserialize)]
/// Configuration for PID gains
pub struct PIDGains {
    /// Proportional gains
    pub kp: [f32; 3],
    /// Integral gains
    pub ki: [f32; 3],
    /// Derivative gains
    pub kd: [f32; 3],
}

#[derive(Clone, serde::Deserialize, Default)]
/// Configuration for the IMU
pub struct ImuConfig {
    /// Accelerometer noise standard deviation
    pub accel_noise_std: f32,
    /// Gyroscope noise standard deviation
    pub gyro_noise_std: f32,
    /// Accelerometer bias drift standard deviation
    pub accel_bias_std: f32,
    /// Gyroscope bias drift standard deviation
    pub gyro_bias_std: f32,
}

#[derive(Clone, serde::Deserialize)]
/// Configuration for the maze
pub struct MazeConfig {
    /// Upper bounds of the maze in meters (x, y, z)
    pub upper_bounds: [f32; 3],
    /// Lower bounds of the maze in meters (x, y, z)
    pub lower_bounds: [f32; 3],
    /// Number of obstacles in the maze
    pub num_obstacles: usize,
    /// Obstacle velocity maximum bounds in m/s in (x, y, z) directions
    pub obstacles_velocity_bounds: [f32; 3],
    /// Obstacle radius bounds in meters (min, max)
    pub obstacles_radius_bounds: [f32; 2],
}

#[derive(Clone, serde::Deserialize)]
/// Configuration for the camera
pub struct CameraConfig {
    /// Camera resolution in pixels (width, height)
    pub resolution: (usize, usize),
    /// Camera field of view in height in degrees
    pub fov_vertical: f32,
    /// Camera near clipping plane in meters
    pub near: f32,
    /// Camera far clipping plane in meters
    pub far: f32,
    /// Camera transform matrix for depth
    pub rotation_transform: [f32; 9],
}

#[derive(Clone, serde::Deserialize)]
/// Configuration for the mesh
pub struct MeshConfig {
    /// Division of the 2D mesh, the mesh will be division x division squares
    pub division: usize,
    /// Spacing between the squares in meters
    pub spacing: f32,
}
/// Implementation of the Config struct
impl Config {
    /// Load configuration from a YAML file.
    /// # Arguments
    /// * `filename` - The name of the file to load.
    /// # Returns
    /// * The configuration object.
    /// # Errors
    /// * If the file cannot be read or the YAML cannot be parsed.
    pub fn from_yaml(filename: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let contents = std::fs::read_to_string(filename)?;
        Ok(serde_yaml::from_str(&contents)?)
    }
}

/// Helper function to allow deserializing either a single `QuadrotorConfigurations`
/// or a list of them into a `Vec<QuadrotorConfigurations>`.
fn single_or_multiple_quadrotors<'de, D>(
    deserializer: D,
) -> Result<Vec<QuadrotorConfigurations>, D::Error>
where
    D: Deserializer<'de>,
{
    let parsed: Value = Deserialize::deserialize(deserializer)?;

    match parsed {
        // If it's a YAML list (sequence), parse as Vec
        Value::Sequence(seq) => {
            serde_yaml::from_value(Value::Sequence(seq)).map_err(D::Error::custom)
        }
        // If it's a single object (mapping), wrap it in a list
        Value::Mapping(_) => {
            serde_yaml::from_value(Value::Sequence(vec![parsed])).map_err(D::Error::custom)
        }
        _ => Err(D::Error::custom(
            "Expected a mapping (object) or a sequence (array)",
        )),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_base_config() {
        let config = Config::from_yaml("tests/testdata/test_config_base.yaml").unwrap();
        assert!(config.quadrotor.len() == 1);
        let quad_config = config.quadrotor.first().unwrap();
        let quadrotor_config: QuadrotorConfig = match quad_config.clone() {
            QuadrotorConfigurations::Peng(quadrotor_config) => quadrotor_config,
            _ => panic!("Failed to load Peng configuration"),
        };

        assert_eq!(config.simulation.control_frequency, 200);
        assert_eq!(config.simulation.simulation_frequency, 1000);
        assert_eq!(config.simulation.log_frequency, 20);
        assert_eq!(config.simulation.duration, 70.0);
        assert_eq!(quadrotor_config.mass, 1.3);
        assert_eq!(quadrotor_config.drag_coefficient, 0.0);
        assert_eq!(config.pid_controller.pos_gains.kp, [7.1, 7.1, 11.9]);
        assert_eq!(config.pid_controller.att_gains.kd, [0.13, 0.13, 0.1]);
        assert_eq!(config.pid_controller.pos_max_int, [10.0, 10.0, 10.0]);
        assert_eq!(config.imu.accel_noise_std, 0.02);
        assert_eq!(config.maze.upper_bounds, [4.0, 2.0, 2.0]);
    }

    #[test]
    fn test_liftoff_config() {
        let config = Config::from_yaml("tests/testdata/test_liftoff_base.yaml").unwrap();
        assert!(config.quadrotor.len() == 1);
        let quad_config = config.quadrotor.first().unwrap();
        let liftoff_config = match quad_config {
            QuadrotorConfigurations::Liftoff(liftoff_config) => liftoff_config,
            _ => panic!("Failed to load Liftoff configuration"),
        };
        assert_eq!(liftoff_config.ip_address, "0.0.0.0:9001");
    }

    #[test]
    fn test_betaflight_config() {
        let config = Config::from_yaml("tests/testdata/test_betaflight_base.yaml")
            .expect("failed to unwrap");
        assert!(config.quadrotor.len() == 1);
        let quad_config = config.quadrotor.first().unwrap();
        let liftoff_config = match quad_config {
            QuadrotorConfigurations::Betaflight(liftoff_config) => liftoff_config,
            _ => panic!("Failed to load Liftoff configuration"),
        };
        assert_eq!(liftoff_config.vicon_address, "0.0.0.0:51001");
    }
}
