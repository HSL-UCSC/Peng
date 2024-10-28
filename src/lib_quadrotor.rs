use crate::SimulationError;

use nalgebra::{Matrix3, Quaternion, Rotation3, SMatrix, UnitQuaternion, Vector3};

/// Represents a quadrotor with its physical properties and state
/// # Example
/// ```
/// use nalgebra::Vector3;
/// use peng_quad::Quadrotor;
/// let (time_step, mass, gravity, drag_coefficient) = (0.01, 1.3, 9.81, 0.01);
/// let inertia_matrix = [0.0347563, 0.0, 0.0, 0.0, 0.0458929, 0.0, 0.0, 0.0, 0.0977];
/// let quadrotor = Quadrotor::new(time_step, mass, gravity, drag_coefficient, inertia_matrix);
/// ```
pub struct Quadrotor {
    /// Current position of the quadrotor in 3D space
    pub position: Vector3<f32>,
    /// Current velocity of the quadrotor
    pub velocity: Vector3<f32>,
    /// Current orientation of the quadrotor
    pub orientation: UnitQuaternion<f32>,
    /// Current angular velocity of the quadrotor
    pub angular_velocity: Vector3<f32>,
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
    /// use nalgebra::Vector3;
    /// use peng_quad::Quadrotor;
    ///
    /// let (time_step, mass, gravity, drag_coefficient) = (0.01, 1.3, 9.81, 0.01);
    /// let inertia_matrix = [0.0347563, 0.0, 0.0, 0.0, 0.0458929, 0.0, 0.0, 0.0, 0.0977];
    /// let quadrotor = Quadrotor::new(time_step, mass, gravity, drag_coefficient, inertia_matrix);
    /// ```
    pub fn new(
        time_step: f32,
        mass: f32,
        gravity: f32,
        drag_coefficient: f32,
        inertia_matrix: [f32; 9],
    ) -> Result<Self, SimulationError> {
        let inertia_matrix = Matrix3::from_row_slice(&inertia_matrix);
        let inertia_matrix_inv =
            inertia_matrix
                .try_inverse()
                .ok_or(SimulationError::NalgebraError(
                    "Failed to invert inertia matrix".to_string(),
                ))?;
        Ok(Self {
            position: Vector3::zeros(),
            velocity: Vector3::zeros(),
            orientation: UnitQuaternion::identity(),
            angular_velocity: Vector3::zeros(),
            mass,
            gravity,
            time_step,
            drag_coefficient,
            inertia_matrix,
            inertia_matrix_inv,
        })
    }
    /// Updates the quadrotor's dynamics with control inputs
    /// # Arguments
    /// * `control_thrust` - The total thrust force applied to the quadrotor
    /// * `control_torque` - The 3D torque vector applied to the quadrotor
    /// # Example
    /// ```
    /// use nalgebra::Vector3;
    /// use peng_quad::Quadrotor;
    ///
    /// let (time_step, mass, gravity, drag_coefficient) = (0.01, 1.3, 9.81, 0.01);
    /// let inertia_matrix = [0.0347563, 0.0, 0.0, 0.0, 0.0458929, 0.0, 0.0, 0.0, 0.0977];
    /// let mut quadrotor = Quadrotor::new(time_step, mass, gravity, drag_coefficient, inertia_matrix).unwrap();
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
        let acceleration = (thrust_world + gravity_force + drag_force) / self.mass;
        self.velocity += acceleration * self.time_step;
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
    /// use nalgebra::Vector3;
    /// use peng_quad::Quadrotor;
    /// let (time_step, mass, gravity, drag_coefficient) = (0.01, 1.3, 9.81, 0.01);
    /// let inertia_matrix = [0.0347563, 0.0, 0.0, 0.0, 0.0458929, 0.0, 0.0, 0.0, 0.0977];
    /// let mut quadrotor = Quadrotor::new(time_step, mass, gravity, drag_coefficient, inertia_matrix).unwrap();
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
    /// use nalgebra::Vector3;
    /// use peng_quad::Quadrotor;
    /// use nalgebra::UnitQuaternion;
    /// let (time_step, mass, gravity, drag_coefficient) = (0.01, 1.3, 9.81, 0.01);
    /// let inertia_matrix = [0.0347563, 0.0, 0.0, 0.0, 0.0458929, 0.0, 0.0, 0.0, 0.0977];
    /// let quadrotor = Quadrotor::new(time_step, mass, gravity, drag_coefficient, inertia_matrix).unwrap();
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
    /// use nalgebra::Vector3;
    /// use peng_quad::Quadrotor;
    /// use nalgebra::UnitQuaternion;
    /// let (time_step, mass, gravity, drag_coefficient) = (0.01, 1.3, 9.81, 0.01);
    /// let inertia_matrix = [0.0347563, 0.0, 0.0, 0.0, 0.0458929, 0.0, 0.0, 0.0, 0.0977];
    /// let mut quadrotor = Quadrotor::new(time_step, mass, gravity, drag_coefficient, inertia_matrix).unwrap();
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
    /// use nalgebra::Vector3;
    /// use peng_quad::Quadrotor;
    /// use nalgebra::UnitQuaternion;
    /// let (time_step, mass, gravity, drag_coefficient) = (0.01, 1.3, 9.81, 0.01);
    /// let inertia_matrix = [0.0347563, 0.0, 0.0, 0.0, 0.0458929, 0.0, 0.0, 0.0, 0.0977];
    /// let quadrotor = Quadrotor::new(time_step, mass, gravity, drag_coefficient, inertia_matrix).unwrap();
    /// let state = [
    ///   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    /// ];
    /// let control_thrust = 0.0;
    /// let control_torque = Vector3::new(0.0, 0.0, 0.0);
    /// let derivative = quadrotor.state_derivative(&state, control_thrust, &control_torque);
    /// ```
    pub fn state_derivative(
        &self,
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
        let acceleration = (thrust_world + gravity_force + drag_force) / self.mass;

        let inertia_angular_velocity = self.inertia_matrix * angular_velocity;
        let gyroscopic_torque = angular_velocity.cross(&inertia_angular_velocity);
        let angular_acceleration = self.inertia_matrix_inv * (control_torque - gyroscopic_torque);

        let mut derivative = [0.0; 13];
        derivative[0..3].copy_from_slice(velocity.as_slice());
        derivative[3..6].copy_from_slice(acceleration.as_slice());
        derivative[6..10].copy_from_slice(q_dot.coords.as_slice());
        derivative[10..13].copy_from_slice(angular_acceleration.as_slice());
        derivative
    }
    /// Simulates IMU readings
    /// # Returns
    /// * A tuple containing the true acceleration and angular velocity of the quadrotor
    /// # Errors
    /// * Returns a SimulationError if the IMU readings cannot be calculated
    /// # Example
    /// ```
    /// use nalgebra::Vector3;
    /// use peng_quad::Quadrotor;
    ///
    /// let (time_step, mass, gravity, drag_coefficient) = (0.01, 1.3, 9.81, 0.01);
    /// let inertia_matrix = [0.0347563, 0.0, 0.0, 0.0, 0.0458929, 0.0, 0.0, 0.0, 0.0977];
    /// let quadrotor = Quadrotor::new(time_step, mass, gravity, drag_coefficient, inertia_matrix).unwrap();
    /// let (true_acceleration, true_angular_velocity) = quadrotor.read_imu().unwrap();
    /// ```
    pub fn read_imu(&self) -> Result<(Vector3<f32>, Vector3<f32>), SimulationError> {
        let gravity_world = Vector3::new(0.0, 0.0, self.gravity);
        let true_acceleration =
            self.orientation.inverse() * (self.velocity / self.time_step - gravity_world);
        Ok((true_acceleration, self.angular_velocity))
    }
}
