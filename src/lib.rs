//! # Peng - A Minimal Quadrotor Autonomy Framework
//!
//! A high-performance quadrotor autonomy framework written in Rust that provides
//! real-time dynamics simulation, trajectory planning, and control with modern
//! visualization capabilities.
//!
//! # Features
//!
//! ## Real-time Simulation
//! - High-fidelity quadrotor dynamics with configurable parameters
//! - IMU and depth sensor simulation
//! - Optional RK4 integration for accurate dynamics
//!
//! ## Advanced Control
//! - PID control for position and attitude with tunable gains
//! - Integral windup prevention
//! - Support for different control frequencies
//!
//! ## Rich Trajectory Planning
//! - Minimum jerk line trajectory planner
//! - Lissajous curve planner
//! - Circular trajectory planner
//! - Obstacle avoidance planner
//! - Waypoint navigation planner
//! - Landing planner
//!
//! ## Visualization & Debug
//! - Real-time 3D visualization via rerun.io
//! - Depth map rendering
//! - State telemetry logging
//! - Configurable logging frequencies
//!
//! ## Performance
//! - Memory-safe and Efficient Rust implementation
//! - Multi-threaded depth rendering
//!
//! ## Example
//! ```
//! use peng_quad::config;
//! use nalgebra::Vector3;
//! use peng_quad::{Quadrotor, SimulationError};
//! let (time_step, mass, gravity, drag_coefficient) = (0.01, 1.3, 9.81, 0.01);
//! let inertia_matrix = [0.0347563, 0.0, 0.0, 0.0, 0.0458929, 0.0, 0.0, 0.0, 0.0977];
//! let quadrotor = Quadrotor::new(time_step, config::SimulationConfig::default(), config::QuadrotorConfig::default(), config::ImuConfig::default()).unwrap();
//! ```
pub mod betaflight_quad;
pub mod config;
pub mod environment;
pub mod liftoff_quad;
pub mod logger;
pub mod planners;
pub mod quadrotor;
pub mod sensors;
pub mod sync;

use nalgebra::{Matrix3, Quaternion, Rotation3, UnitQuaternion, Vector3};
use std::f32::consts::PI;

#[derive(thiserror::Error, Debug)]
/// Represents errors that can occur during simulation
/// # Example
/// ```
/// use peng_quad::SimulationError;
/// let error = SimulationError::NalgebraError("Matrix inversion failed".to_string());
/// ```
pub enum SimulationError {
    /// Error related to Rerun visualization
    #[error("Rerun error: {0}")]
    RerunError(#[from] rerun::RecordingStreamError),
    /// Error related to Rerun spawn process
    #[error("Rerun spawn error: {0}")]
    RerunSpawnError(#[from] rerun::SpawnError),
    /// Error related to linear algebra operations
    #[error("Nalgebra error: {0}")]
    NalgebraError(String),
    /// Error related to normal distribution calculations
    #[error("Normal error: {0}")]
    NormalError(#[from] rand_distr::NormalError),
    /// Other general errors
    #[error("Other error: {0}")]
    OtherError(String),
}

/// Represents a quadrotor with its physical properties and state
/// # Example
/// ```
/// use peng_quad::config;
/// use nalgebra::Vector3;
/// use peng_quad::Quadrotor;
/// let (time_step, mass, gravity, drag_coefficient) = (0.01, 1.3, 9.81, 0.01);
/// let inertia_matrix = [0.0347563, 0.0, 0.0, 0.0, 0.0458929, 0.0, 0.0, 0.0, 0.0977];
/// let quadrotor = Quadrotor::new(time_step, config::SimulationConfig::default(), config::QuadrotorConfig::default(), config::ImuConfig::default()).unwrap();
/// ```
pub struct Quadrotor {
    /// The name or ID of the quadrotor
    pub name: String,
    // TODO: move all of these to an instance of quadrotor state
    /// Current position of the quadrotor in 3D space
    pub position: Vector3<f32>,
    /// Current velocity of the quadrotor
    pub velocity: Vector3<f32>,
    /// Current acceleration of the quadrotor
    pub acceleration: Vector3<f32>,
    /// Current orientation of the quadrotor
    pub orientation: UnitQuaternion<f32>,
    /// Current angular velocity of the quadrotor
    pub angular_velocity: Vector3<f32>,
    /// Initial Position
    pub initial_position: Vector3<f32>,
    /// Mass of the quadrotor in kg
    pub mass: f32,
    /// Gravitational acceleration in m/s^2
    pub gravity: f32,
    /// Simulation time step in seconds
    pub time_step: f32,
    /// Drag coefficient
    pub drag_coefficient: f32,
    /// Inertia matrix of the quadrotor
    pub inertia_matrix: Matrix3<f32>,
    /// Inverse of the inertia matrix
    pub inertia_matrix_inv: Matrix3<f32>,
    /// Previous Thrust
    pub previous_thrust: f32,
    /// Previous Torque
    pub previous_torque: Vector3<f32>,
    /// Config
    pub config: config::SimulationConfig,
    /// IMU
    pub imu: sensors::Imu,
}

impl quadrotor::QuadrotorInterface for Quadrotor {
    fn control(
        &mut self,
        step_number: usize,
        thrust: f32,
        torque: &Vector3<f32>,
    ) -> Result<Option<(f32, Vector3<f32>)>, SimulationError> {
        if step_number % (self.config.simulation_frequency / self.config.control_frequency) == 0 {
            if self.config.use_rk4_for_dynamics_control {
                self.update_dynamics_with_controls_rk4(thrust, torque);
            } else {
                self.update_dynamics_with_controls_euler(thrust, torque);
            }
            self.previous_thrust = thrust;
            self.previous_torque = *torque;
        } else {
            let previous_thrust = self.previous_thrust;
            let previous_torque = self.previous_torque;
            if self.config.use_rk4_for_dynamics_update {
                self.update_dynamics_with_controls_rk4(previous_thrust, &previous_torque);
            } else {
                self.update_dynamics_with_controls_euler(previous_thrust, &previous_torque);
            }
        }
        Ok(None)
    }

    fn observe(&mut self, t: f32) -> Result<quadrotor::QuadrotorState, SimulationError> {
        let (measured_acceleration, measured_angular_velocity) =
            self.imu.read(self.acceleration, self.angular_velocity)?;
        self.imu.update(t)?;
        Ok(quadrotor::QuadrotorState {
            time: 0.0,
            state: quadrotor::State {
                position: self.position,
                velocity: self.velocity,
                acceleration: self.acceleration,
                orientation: self.orientation,
                angular_velocity: self.angular_velocity,
            },
            measured_state: quadrotor::State {
                acceleration: measured_acceleration,
                angular_velocity: measured_angular_velocity,
                ..Default::default()
            },
        })
    }

    fn max_thrust(&self) -> f32 {
        2.5 * self.gravity
    }

    // TDO: need a method to retrieve quadrotor physical constants
    fn max_torque(&self) -> Vector3<f32> {
        let motor_thrust = self.max_thrust() / 4.0;
        let max_rp_torque = 2.0 * 0.65 * motor_thrust;
        let yaw_torque = 2.0 * 0.005 * motor_thrust;
        Vector3::new(max_rp_torque, max_rp_torque, yaw_torque)
    }

    /// Simulates IMU readings
    /// # Returns
    /// * A tuple containing the true acceleration and angular velocity of the quadrotor
    /// # Errors
    /// * Returns a SimulationError if the IMU readings cannot be calculated
    /// # Example
    /// ```
    /// use peng_quad::config;
    /// use nalgebra::Vector3;
    /// use peng_quad::Quadrotor;
    /// use peng_quad::quadrotor::QuadrotorInterface;
    ///
    /// let (time_step, mass, gravity, drag_coefficient) = (0.01, 1.3, 9.81, 0.01);
    /// let inertia_matrix = [0.0347563, 0.0, 0.0, 0.0, 0.0458929, 0.0, 0.0, 0.0, 0.0977];
    /// let quadrotor = Quadrotor::new(time_step, config::SimulationConfig::default(), config::QuadrotorConfig::default(), config::ImuConfig::default()).unwrap();
    /// let (true_acceleration, true_angular_velocity) = quadrotor.read_imu().unwrap();
    /// ```
    fn read_imu(&self) -> Result<(Vector3<f32>, Vector3<f32>), SimulationError> {
        Ok((self.acceleration, self.angular_velocity))
    }

    fn parameters(&self) -> config::QuadrotorConfig {
        config::QuadrotorConfig {
            id: self.name.clone(),
            mass: self.mass,
            drag_coefficient: self.drag_coefficient,
            inertia_matrix: self.inertia_matrix.as_slice().try_into().unwrap(),
            max_thrust_kg: self.max_thrust(),
            arm_length_m: 0.0,
            yaw_torque_constant: 0.0,
            initial_position: self.initial_position.into(),
        }
    }
}

/// Implementation of the Quadrotor struct
impl Quadrotor {
    /// Creates a new Quadrotor with default parameters
    /// # Arguments
    /// * `time_step` - The simulation time step in seconds
    /// * `mass` - The mass of the quadrotor in kg
    /// * `gravity` - The gravitational acceleration in m/s^2
    /// * `drag_coefficient` - The drag coefficient
    /// * `inertia_matrix` - The inertia matrix of the quadrotor
    /// # Returns
    /// * A new Quadrotor instance
    /// # Errors
    /// * Returns a SimulationError if the inertia matrix cannot be inverted
    /// # Example
    /// ```
    /// use peng_quad::config;;
    /// use nalgebra::Vector3;
    /// use peng_quad::Quadrotor;
    ///
    /// let (time_step, mass, gravity, drag_coefficient) = (0.01, 1.3, 9.81, 0.01);
    /// let inertia_matrix = [0.0347563, 0.0, 0.0, 0.0, 0.0458929, 0.0, 0.0, 0.0, 0.0977];
    /// let quadrotor = Quadrotor::new(time_step, config::SimulationConfig::default(), config::QuadrotorConfig::default(), config::ImuConfig::default()).unwrap();
    /// ```
    /// TODO: time step to simulation config, others to quadrotor config
    pub fn new(
        time_step: f32,
        config: config::SimulationConfig,
        quadrotor_config: config::QuadrotorConfig,
        imu_config: config::ImuConfig,
    ) -> Result<Self, SimulationError> {
        let inertia_matrix = Matrix3::from_row_slice(&quadrotor_config.inertia_matrix);
        let inertia_matrix_inv =
            inertia_matrix
                .try_inverse()
                .ok_or(SimulationError::NalgebraError(
                    "Failed to invert inertia matrix".to_string(),
                ))?;

        let imu = sensors::Imu::new(
            imu_config.accel_noise_std,
            imu_config.gyro_noise_std,
            imu_config.accel_bias_std,
            imu_config.gyro_bias_std,
        )?;
        Ok(Self {
            name: "PengQuad".to_string(),
            config: config.clone(),
            position: quadrotor_config.initial_position.into(),
            velocity: Vector3::zeros(),
            acceleration: Vector3::zeros(),
            orientation: UnitQuaternion::identity(),
            angular_velocity: Vector3::zeros(),
            initial_position: quadrotor_config.initial_position.into(),
            mass: quadrotor_config.mass,
            gravity: config.gravity,
            time_step,
            drag_coefficient: quadrotor_config.drag_coefficient,
            inertia_matrix,
            inertia_matrix_inv,
            previous_thrust: 0.0,
            previous_torque: Vector3::zeros(),
            imu,
        })
    }

    /// Updates the quadrotor's dynamics with control inputs
    /// # Arguments
    /// * `control_thrust` - The total thrust force applied to the quadrotor
    /// * `control_torque` - The 3D torque vector applied to the quadrotor
    /// # Example
    /// ```
    /// use peng_quad::config;;
    /// use nalgebra::Vector3;
    /// use peng_quad::Quadrotor;
    ///
    /// let (time_step, mass, gravity, drag_coefficient) = (0.01, 1.3, 9.81, 0.01);
    /// let inertia_matrix = [0.0347563, 0.0, 0.0, 0.0, 0.0458929, 0.0, 0.0, 0.0, 0.0977];
    /// let mut quadrotor = Quadrotor::new(time_step, config::SimulationConfig::default(), config::QuadrotorConfig::default(), config::ImuConfig::default()).unwrap();
    /// let control_thrust = mass * gravity;
    /// let control_torque = Vector3::new(0.0, 0.0, 0.0);
    /// quadrotor.update_dynamics_with_controls_euler(control_thrust, &control_torque);
    /// ```
    pub fn update_dynamics_with_controls_euler(
        &mut self,
        control_thrust: f32,
        control_torque: &Vector3<f32>,
    ) {
        let gravity_force = Vector3::new(0.0, 0.0, -self.mass * self.gravity);
        let drag_force = -self.drag_coefficient * self.velocity.norm() * self.velocity;
        let thrust_world = self.orientation * Vector3::new(0.0, 0.0, control_thrust);
        self.acceleration = (thrust_world + gravity_force + drag_force) / self.mass;
        self.velocity += self.acceleration * self.time_step;
        self.position += self.velocity * self.time_step;
        let inertia_angular_velocity = self.inertia_matrix * self.angular_velocity;
        let gyroscopic_torque = self.angular_velocity.cross(&inertia_angular_velocity);
        let angular_acceleration = self.inertia_matrix_inv * (control_torque - gyroscopic_torque);
        self.angular_velocity += angular_acceleration * self.time_step;
        self.orientation *=
            UnitQuaternion::from_scaled_axis(self.angular_velocity * self.time_step);
    }
    /// Updates the quadrotor's dynamics with control inputs using the Runge-Kutta 4th order method
    /// # Arguments
    /// * `control_thrust` - The total thrust force applied to the quadrotor
    /// * `control_torque` - The 3D torque vector applied to the quadrotor
    /// # Example
    /// ```
    /// use peng_quad::config;;
    /// use nalgebra::Vector3;
    /// use peng_quad::Quadrotor;
    /// let (time_step, mass, gravity, drag_coefficient) = (0.01, 1.3, 9.81, 0.01);
    /// let inertia_matrix = [0.0347563, 0.0, 0.0, 0.0, 0.0458929, 0.0, 0.0, 0.0, 0.0977];
    /// let mut quadrotor = Quadrotor::new(time_step, config::SimulationConfig::default(), config::QuadrotorConfig::default(), config::ImuConfig::default()).unwrap();
    /// let control_thrust = mass * gravity;
    /// let control_torque = Vector3::new(0.0, 0.0, 0.0);
    /// quadrotor.update_dynamics_with_controls_rk4(control_thrust, &control_torque);
    /// ```
    pub fn update_dynamics_with_controls_rk4(
        &mut self,
        control_thrust: f32,
        control_torque: &Vector3<f32>,
    ) {
        let h = self.time_step;
        let state = self.get_state();

        let k1 = self.state_derivative(&state, control_thrust, control_torque);
        let mut temp_state = [0.0; 13];
        for i in 0..13 {
            temp_state[i] = state[i] + 0.5 * h * k1[i];
        }
        let k2 = self.state_derivative(&temp_state, control_thrust, control_torque);

        for i in 0..13 {
            temp_state[i] = state[i] + 0.5 * h * k2[i];
        }
        let k3 = self.state_derivative(&temp_state, control_thrust, control_torque);

        for i in 0..13 {
            temp_state[i] = state[i] + h * k3[i];
        }
        let k4 = self.state_derivative(&temp_state, control_thrust, control_torque);

        let mut new_state = [0.0; 13];
        for i in 0..13 {
            new_state[i] = state[i] + (h / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
        }

        let mut q = Quaternion::new(new_state[9], new_state[6], new_state[7], new_state[8]);
        q = q.normalize();
        new_state[6..10].copy_from_slice(q.coords.as_slice());

        self.set_state(&new_state);
    }
    /// Returns the state derivative of the quadrotor
    /// # Arguments
    /// * `state` - The state of the quadrotor
    /// # Example
    /// ```
    /// use peng_quad::config;;
    /// use nalgebra::Vector3;
    /// use peng_quad::Quadrotor;
    /// use nalgebra::UnitQuaternion;
    /// let (time_step, mass, gravity, drag_coefficient) = (0.01, 1.3, 9.81, 0.01);
    /// let inertia_matrix = [0.0347563, 0.0, 0.0, 0.0, 0.0458929, 0.0, 0.0, 0.0, 0.0977];
    /// let quadrotor = Quadrotor::new(time_step, config::SimulationConfig::default(), config::QuadrotorConfig::default(), config::ImuConfig::default()).unwrap();
    /// let state = quadrotor.get_state();
    /// ```
    pub fn get_state(&self) -> [f32; 13] {
        let mut state = [0.0; 13];
        state[0..3].copy_from_slice(self.position.as_slice());
        state[3..6].copy_from_slice(self.velocity.as_slice());
        state[6..10].copy_from_slice(self.orientation.coords.as_slice());
        state[10..13].copy_from_slice(self.angular_velocity.as_slice());
        state
    }
    /// Sets the state of the quadrotor
    /// # Arguments
    /// * `state` - The state of the quadrotor
    /// # Example
    /// ```
    /// use peng_quad::config;;
    /// use nalgebra::Vector3;
    /// use peng_quad::Quadrotor;
    /// use nalgebra::UnitQuaternion;
    /// let (time_step, mass, gravity, drag_coefficient) = (0.01, 1.3, 9.81, 0.01);
    /// let inertia_matrix = [0.0347563, 0.0, 0.0, 0.0, 0.0458929, 0.0, 0.0, 0.0, 0.0977];
    /// let mut quadrotor = Quadrotor::new(time_step, config::SimulationConfig::default(), config::QuadrotorConfig::default(), config::ImuConfig::default()).unwrap();
    /// let state = [
    ///    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    /// ];
    /// quadrotor.set_state(&state);
    /// ```
    pub fn set_state(&mut self, state: &[f32; 13]) {
        self.position = Vector3::from_column_slice(&state[0..3]);
        self.velocity = Vector3::from_column_slice(&state[3..6]);
        self.orientation = UnitQuaternion::from_quaternion(Quaternion::new(
            state[9], state[6], state[7], state[8],
        ));
        self.angular_velocity = Vector3::from_column_slice(&state[10..13]);
    }
    /// Calculates the derivative of the state of the quadrotor
    /// # Arguments
    /// * `state` - The current state of the quadrotor
    /// * `control_thrust` - The thrust applied to the quadrotor
    /// * `control_torque` - The torque applied to the quadrotor
    /// # Returns
    /// The derivative of the state
    /// # Example
    /// ```
    /// use peng_quad::config;;
    /// use nalgebra::Vector3;
    /// use peng_quad::Quadrotor;
    /// use nalgebra::UnitQuaternion;
    /// let (time_step, mass, gravity, drag_coefficient) = (0.01, 1.3, 9.81, 0.01);
    /// let inertia_matrix = [0.0347563, 0.0, 0.0, 0.0, 0.0458929, 0.0, 0.0, 0.0, 0.0977];
    /// let mut quadrotor = Quadrotor::new(time_step, config::SimulationConfig::default(), config::QuadrotorConfig::default(), config::ImuConfig::default()).unwrap();
    /// let state = [
    ///   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    /// ];
    /// let control_thrust = 0.0;
    /// let control_torque = Vector3::new(0.0, 0.0, 0.0);
    /// let derivative = quadrotor.state_derivative(&state, control_thrust, &control_torque);
    /// ```
    pub fn state_derivative(
        &mut self,
        state: &[f32],
        control_thrust: f32,
        control_torque: &Vector3<f32>,
    ) -> [f32; 13] {
        let velocity = Vector3::from_column_slice(&state[3..6]);
        let orientation = UnitQuaternion::from_quaternion(Quaternion::new(
            state[9], state[6], state[7], state[8],
        ));
        // Quaternion deriviative
        let omega_quat = Quaternion::new(0.0, state[10], state[11], state[12]);
        let q_dot = orientation.into_inner() * omega_quat * 0.5;

        let angular_velocity = Vector3::from_column_slice(&state[10..13]);

        let gravity_force = Vector3::new(0.0, 0.0, -self.mass * self.gravity);
        let drag_force = -self.drag_coefficient * velocity.norm() * velocity;
        let thrust_world = orientation * Vector3::new(0.0, 0.0, control_thrust);
        self.acceleration = (thrust_world + gravity_force + drag_force) / self.mass;

        let inertia_angular_velocity = self.inertia_matrix * angular_velocity;
        let gyroscopic_torque = angular_velocity.cross(&inertia_angular_velocity);
        let angular_acceleration = self.inertia_matrix_inv * (control_torque - gyroscopic_torque);

        let mut derivative = [0.0; 13];
        derivative[0..3].copy_from_slice(velocity.as_slice());
        derivative[3..6].copy_from_slice(self.acceleration.as_slice());
        derivative[6..10].copy_from_slice(q_dot.coords.as_slice());
        derivative[10..13].copy_from_slice(angular_acceleration.as_slice());
        derivative
    }
}

/// PID controller for quadrotor position and attitude control
///
/// The kpid_pos and kpid_att gains are following the format of
/// porportional, derivative and integral gains
/// # Example
/// ```
/// use nalgebra::Vector3;
/// use peng_quad::PIDController;
/// let kpid_pos = [
///     [1.0, 1.0, 1.0],
///     [0.1, 0.1, 0.1],
///     [0.01, 0.01, 0.01],
/// ];
/// let kpid_att = [
///     [1.0, 1.0, 1.0],
///     [0.1, 0.1, 0.1],
///     [0.01, 0.01, 0.01],
/// ];
/// let max_integral_pos = [1.0, 1.0, 1.0];
/// let max_integral_att = [1.0, 1.0, 1.0];
/// let mass = 1.0;
/// let gravity = 9.81;
/// let pid_controller = PIDController::new(kpid_pos, kpid_att, max_integral_pos, max_integral_att, mass, gravity);
/// ```
pub struct PIDController {
    /// PID gain for position control including proportional, derivative, and integral gains
    pub kpid_pos: [Vector3<f32>; 3],
    /// PID gain for attitude control including proportional, derivative, and integral gains
    pub kpid_att: [Vector3<f32>; 3],
    /// Accumulated integral error for position
    pub integral_pos_error: Vector3<f32>,
    /// Accumulated integral error for attitude
    pub integral_att_error: Vector3<f32>,
    /// Maximum allowed integral error for position
    pub max_integral_pos: Vector3<f32>,
    /// Maximum allowed integral error for attitude
    pub max_integral_att: Vector3<f32>,
    /// Mass of the quadrotor
    pub mass: f32,
    /// Gravity constant
    pub gravity: f32,
}
/// Implementation of PIDController
impl PIDController {
    /// Creates a new PIDController with default gains
    /// gains are in the order of proportional, derivative, and integral
    /// # Arguments
    /// * `_kpid_pos` - PID gains for position control
    /// * `_kpid_att` - PID gains for attitude control
    /// * `_max_integral_pos` - Maximum allowed integral error for position
    /// * `_max_integral_att` - Maximum allowed integral error for attitude
    /// # Returns
    /// * A new PIDController instance
    /// # Example
    /// ```
    /// use nalgebra::Vector3;
    /// use peng_quad::PIDController;
    /// let kpid_pos = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    /// let kpid_att = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    /// let max_integral_pos = [1.0, 1.0, 1.0];
    /// let max_integral_att = [1.0, 1.0, 1.0];
    /// let mass = 1.0;
    /// let gravity = 9.81;
    /// let pid = PIDController::new(kpid_pos, kpid_att, max_integral_pos, max_integral_att, mass, gravity);
    /// ```
    pub fn new(
        _kpid_pos: [[f32; 3]; 3],
        _kpid_att: [[f32; 3]; 3],
        _max_integral_pos: [f32; 3],
        _max_integral_att: [f32; 3],
        _mass: f32,
        _gravity: f32,
    ) -> Self {
        Self {
            kpid_pos: _kpid_pos.map(Vector3::from),
            kpid_att: _kpid_att.map(Vector3::from),
            integral_pos_error: Vector3::zeros(),
            integral_att_error: Vector3::zeros(),
            max_integral_pos: Vector3::from(_max_integral_pos),
            max_integral_att: Vector3::from(_max_integral_att),
            mass: _mass,
            gravity: _gravity,
        }
    }
    /// Computes attitude control torques
    /// # Arguments
    /// * `desired_orientation` - The desired orientation quaternion
    /// * `current_orientation` - The current orientation quaternion
    /// * `current_angular_velocity` - The current angular velocity
    /// * `dt` - Time step
    /// # Returns
    /// * The computed attitude control torques
    /// # Example
    /// ```
    /// use nalgebra::{UnitQuaternion, Vector3};
    /// use peng_quad::PIDController;
    ///
    /// let kpid_pos = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    /// let kpid_att = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    /// let max_integral_pos = [1.0, 1.0, 1.0];
    /// let max_integral_att = [1.0, 1.0, 1.0];
    /// let mass = 1.0;
    /// let gravity = 9.81;
    /// let mut pid = PIDController::new(kpid_pos, kpid_att, max_integral_pos, max_integral_att, mass, gravity);
    /// let desired_orientation = UnitQuaternion::identity();
    /// let current_orientation = UnitQuaternion::identity();
    /// let current_angular_velocity = Vector3::zeros();
    /// let dt = 0.01;
    /// let control_torques = pid.compute_attitude_control(&desired_orientation, &current_orientation, &current_angular_velocity, dt);
    /// ```
    pub fn compute_attitude_control(
        &mut self,
        desired_orientation: &UnitQuaternion<f32>,
        current_orientation: &UnitQuaternion<f32>,
        current_angular_velocity: &Vector3<f32>,
        dt: f32,
    ) -> Vector3<f32> {
        // dbg!(desired_orientation);
        // dbg!(current_orientation);
        // dbg!(current_angular_velocity);
        // let frame = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), PI);
        let error_orientation = current_orientation.inverse() * desired_orientation;
        let (roll_error, pitch_error, yaw_error) = error_orientation.euler_angles();
        let error_angles = Vector3::new(roll_error, pitch_error, yaw_error);
        self.integral_att_error += error_angles * dt;
        self.integral_att_error = self
            .integral_att_error
            .zip_map(&self.max_integral_att, |int, max| int.clamp(-max, max));
        let error_angular_velocity = -current_angular_velocity; // TODO: Add desired angular velocity
        self.kpid_att[0].component_mul(&error_angles)
            + self.kpid_att[1].component_mul(&error_angular_velocity)
            + self.kpid_att[2].component_mul(&self.integral_att_error)
    }
    /// Computes position control thrust and desired orientation
    /// # Arguments
    /// * `desired_position` - The desired position
    /// * `desired_velocity` - The desired velocity
    /// * `desired_yaw` - The desired yaw angle
    /// * `current_position` - The current position
    /// * `current_velocity` - The current velocity
    /// * `dt` - Time step
    /// * `mass` - Mass of the quadrotor
    /// * `gravity` - Gravitational acceleration
    /// # Returns
    /// * A tuple containing the computed thrust and desired orientation quaternion
    /// # Example
    /// ```
    /// use nalgebra::{UnitQuaternion, Vector3};
    /// use peng_quad::PIDController;
    ///
    /// let kpid_pos = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    /// let kpid_att = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    /// let max_integral_pos = [1.0, 1.0, 1.0];
    /// let max_integral_att = [1.0, 1.0, 1.0];
    /// let mass = 1.0;
    /// let gravity = 9.81;
    /// let mut pid = PIDController::new(kpid_pos, kpid_att, max_integral_pos, max_integral_att, mass, gravity);
    /// let desired_position = Vector3::new(0.0, 0.0, 1.0);
    /// let desired_velocity = Vector3::zeros();
    /// let desired_yaw = 0.0;
    /// let current_position = Vector3::zeros();
    /// let current_velocity = Vector3::zeros();
    /// let dt = 0.01;
    /// let (thrust, desired_orientation) = pid.compute_position_control(&desired_position, &desired_velocity, desired_yaw, &current_position, &current_velocity, dt);
    /// ```
    pub fn compute_position_control(
        &mut self,
        desired_position: &Vector3<f32>,
        desired_velocity: &Vector3<f32>,
        desired_yaw: f32,
        current_position: &Vector3<f32>,
        current_velocity: &Vector3<f32>,
        dt: f32,
    ) -> (f32, UnitQuaternion<f32>) {
        let error_position = desired_position - current_position;
        let error_velocity = desired_velocity - current_velocity;
        self.integral_pos_error += error_position * dt;
        self.integral_pos_error = self
            .integral_pos_error
            .zip_map(&self.max_integral_pos, |int, max| int.clamp(-max, max));
        let acceleration = self.kpid_pos[0].component_mul(&error_position)
            + self.kpid_pos[1].component_mul(&error_velocity)
            + self.kpid_pos[2].component_mul(&self.integral_pos_error);
        let gravity_compensation = Vector3::new(0.0, 0.0, self.gravity);
        let total_acceleration = acceleration + gravity_compensation;
        let total_acc_norm = total_acceleration.norm();
        let thrust = self.mass * total_acc_norm;
        let desired_orientation = if total_acc_norm > 1e-3 {
            let z_body = total_acceleration / total_acc_norm;
            let yaw_rotation = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), desired_yaw);
            let x_body_horizontal = yaw_rotation * Vector3::new(1.0, 0.0, 0.0);
            let y_body = z_body.cross(&x_body_horizontal).normalize();
            let x_body = y_body.cross(&z_body);
            UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(
                Matrix3::from_columns(&[x_body, y_body, z_body]),
            ))
        } else {
            UnitQuaternion::from_euler_angles(0.0, 0.0, desired_yaw)
        };
        (thrust, desired_orientation)
    }
}

/// Helper function to parse Vector3 from YAML
/// # Arguments
/// * `value` - YAML value
/// * `key` - key to parse
/// # Returns
/// * `Vector3<f32>` - parsed vector
/// # Errors
/// * `SimulationError` - if the value is not a valid vector
/// # Example
/// ```
/// use nalgebra::Vector3;
/// use peng_quad::{parse_vector3, SimulationError};
/// let value = serde_yaml::from_str("test: [1.0, 2.0, 3.0]").unwrap();
/// assert_eq!(parse_vector3(&value, "test").unwrap(), Vector3::new(1.0, 2.0, 3.0));
/// ```
pub fn parse_vector3(
    value: &serde_yaml::Value,
    key: &str,
) -> Result<Vector3<f32>, SimulationError> {
    value[key]
        .as_sequence()
        .and_then(|seq| {
            if seq.len() == 3 {
                Some(Vector3::new(
                    seq[0].as_f64()? as f32,
                    seq[1].as_f64()? as f32,
                    seq[2].as_f64()? as f32,
                ))
            } else {
                None
            }
        })
        .ok_or_else(|| SimulationError::OtherError(format!("Invalid {} vector", key)))
}
/// Helper function to parse f32 from YAML
/// # Arguments
/// * `value` - YAML value
/// * `key` - key to parse
/// # Returns
/// * `f32` - parsed value
/// # Errors
/// * `SimulationError` - if the value is not a valid f32
/// # Example
/// ```
/// use peng_quad::{parse_f32, SimulationError};
/// let value = serde_yaml::from_str("key: 1.0").unwrap();
/// let result = parse_f32(&value, "key").unwrap();
/// assert_eq!(result, 1.0);
/// ```
pub fn parse_f32(value: &serde_yaml::Value, key: &str) -> Result<f32, SimulationError> {
    value[key]
        .as_f64()
        .map(|v| v as f32)
        .ok_or_else(|| SimulationError::OtherError(format!("Invalid {}", key)))
}

/// Casts a ray from the camera origin in the given direction
/// # Arguments
/// * `origin` - The origin of the ray
/// * `rotation_world_to_camera` - The rotation matrix from world to camera coordinates
/// * `direction` - The direction of the ray
/// * `maze` - The maze in the scene
/// * `near` - The minimum distance to consider
/// * `far` - The maximum distance to consider
/// # Returns
/// * The distance to the closest obstacle hit by the ray
/// # Errors
/// * If the ray does not hit any obstacles
/// # Example
/// ```
/// use peng_quad::ray_cast;
/// use peng_quad::environment::Maze;
/// use nalgebra::{Vector3, Matrix3};
/// let origin = Vector3::new(0.0, 0.0, 0.0);
/// let rotation_world_to_camera = Matrix3::identity();
/// let direction = Vector3::new(0.0, 0.0, 1.0);
/// let maze = Maze::new([-1.0, -1.0, -1.0], [1.0, 1.0, 1.0], 5, [0.1, 0.1, 0.1], [0.1, 0.5]);
/// let near = 0.1;
/// let far = 100.0;
/// let distance = ray_cast(&origin, &rotation_world_to_camera, &direction, &maze, near, far);
/// ```
pub fn ray_cast(
    origin: &Vector3<f32>,
    rotation_world_to_camera: &Matrix3<f32>,
    direction: &Vector3<f32>,
    maze: &environment::Maze,
    near: f32,
    far: f32,
) -> Result<f32, SimulationError> {
    let mut closest_hit = far;
    // Inline tube intersection
    for axis in 0..3 {
        let t_near = if direction[axis] > 0.0 {
            (maze.upper_bounds[axis] - origin[axis]) / direction[axis]
        } else {
            (maze.lower_bounds[axis] - origin[axis]) / direction[axis]
        };
        if t_near > near && t_near < closest_hit {
            let intersection_point = origin + direction * t_near;
            let mut valid = true;
            for i in 0..3 {
                if i != axis
                    && (intersection_point[i] < maze.lower_bounds[i]
                        || intersection_point[i] > maze.upper_bounds[i])
                {
                    valid = false;
                    break;
                }
            }
            if valid {
                closest_hit = t_near;
            }
        }
    }
    // Early exit if we've hit a wall closer than any possible obstacle
    if closest_hit <= near {
        return Ok(f32::INFINITY);
    }
    // Inline sphere intersection
    for obstacle in &maze.obstacles {
        let oc = origin - obstacle.position;
        let b = oc.dot(direction);
        let c = oc.dot(&oc) - obstacle.radius * obstacle.radius;
        let discriminant = b * b - c;
        if discriminant >= 0.0 {
            // let t = -b - discriminant.sqrt();
            let t = -b - fast_sqrt(discriminant);
            if t > near && t < closest_hit {
                closest_hit = t;
            }
        }
    }
    if closest_hit < far {
        Ok((rotation_world_to_camera * direction * closest_hit).x)
    } else {
        Ok(f32::INFINITY)
    }
}

/// log joystick positions
/// # Arguments
/// * `rec` - The rerun::RecordingStream instance
/// * `trajectory` - The Trajectory instance
/// # Errors
/// * If the data cannot be logged to the recording stream
/// # Example
/// ```no_run
/// use peng_quad::Trajectory;
/// use peng_quad::logger::log_trajectory;
/// use nalgebra::Vector3;
/// let rec = rerun::RecordingStreamBuilder::new("log.rerun").connect().unwrap();
/// let mut trajectory = Trajectory::new(nalgebra::Vector3::new(0.0, 0.0, 0.0));
/// trajectory.add_point(nalgebra::Vector3::new(1.0, 0.0, 0.0));
/// log_trajectory(&rec, &trajectory).unwrap();
/// ```
pub fn log_joy(
    rec: &rerun::RecordingStream,
    _thrust: f32,
    _torque: &Vector3<f32>,
) -> Result<(), SimulationError> {
    let num_points = 100;
    let radius = 1.0;
    let circle_points: Vec<(f32, f32)> = (0..num_points)
        .map(|i| {
            let theta = i as f32 * 2.5 * PI / num_points as f32;
            let x = radius * theta.cos();
            let y = radius * theta.sin();
            (x, y)
        })
        .collect();
    rec.log_static(
        "world/quad/joy/left",
        &rerun::LineStrips2D::new([circle_points.clone()])
            .with_colors([rerun::Color::from_rgb(0, 255, 255)]),
    )?;
    // rec.log(
    //     "joy/right",
    //     &rerun::LineStrips2D::new([circle_points])
    //         .with_colors([rerun::Color::from_rgb(0, 255, 255)]),
    // )?;
    rec.log(
        "world/quad/joy/left",
        &rerun::Points2D::new([(0.0, 0.5)]).with_colors([rerun::Color::from_rgb(0, 255, 255)]),
    )?;
    // rec.log(
    //     "joy/right",
    //     &rerun::Points2D::new([(0.0, -0.5)]).with_colors([rerun::Color::from_rgb(0, 255, 255)]),
    // )?;
    Ok(())
}

/// Fast square root function
/// # Arguments
/// * `x` - The input value
/// # Returns
/// * The square root of the input value
#[inline(always)]
pub fn fast_sqrt(x: f32) -> f32 {
    let i = x.to_bits();
    let i = 0x1fbd1df5 + (i >> 1);
    f32::from_bits(i)
}
