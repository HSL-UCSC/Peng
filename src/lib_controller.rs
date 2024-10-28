use nalgebra::{Matrix3, Quaternion, Rotation3, SMatrix, UnitQuaternion, Vector3};
use rand_distr::{Distribution, Normal};

/// PID controller for quadrotor position and attitude control
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
        let thrust = self.mass * total_acceleration.norm();
        let desired_orientation = if total_acceleration.norm() > 1e-6 {
            let z_body = total_acceleration.normalize();
            let yaw_rotation = UnitQuaternion::from_euler_angles(0.0, 0.0, desired_yaw);
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
