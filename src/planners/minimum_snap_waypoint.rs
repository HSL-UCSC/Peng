use crate::SimulationError;
use async_trait::async_trait;
use nalgebra::{DMatrix, DVector, SMatrix, Vector3};

use super::TrajectoryPoint;
use crate::planners::Planner;

/// Waypoint planner that generates a minimum snap trajectory between waypoints
/// # Example
/// ```
/// use peng_quad::planners::MinimumSnapWaypointPlanner;
/// use nalgebra::Vector3;
/// let planner = MinimumSnapWaypointPlanner::new(
///     vec![Vector3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0)],
///     vec![0.0, 0.0],
///     vec![1.0],
///     0.0,
/// );
/// ```
pub struct MinimumSnapWaypointPlanner {
    /// List of waypoints
    pub waypoints: Vec<Vector3<f32>>,
    /// List of yaw angles
    pub yaws: Vec<f32>,
    /// List of segment times to reach each waypoint
    pub times: Vec<f32>,
    /// Coefficients for the x, y, and z components of the trajectory
    pub coefficients: Vec<Vec<Vector3<f32>>>,
    /// Coefficients for the yaw component of the trajectory
    pub yaw_coefficients: Vec<Vec<f32>>,
    /// Start time of the trajectory
    pub start_time: f32,
}
/// Implementation of the MinimumSnapWaypointPlanner
impl MinimumSnapWaypointPlanner {
    /// Generate a new minimum snap waypoint planner
    /// # Arguments
    /// * `waypoints` - List of waypoints
    /// * `yaws` - List of yaw angles
    /// * `segment_times` - List of segment times to reach each waypoint
    /// * `start_time` - Start time of the trajectory
    /// # Returns
    /// * A new minimum snap waypoint planner
    /// # Errors
    /// * Returns an error if the number of waypoints, yaws, and segment times do not match
    /// # Example
    /// ```
    /// use peng_quad::planners::MinimumSnapWaypointPlanner;
    /// use nalgebra::Vector3;
    /// let waypoints = vec![Vector3::zeros(), Vector3::new(1.0, 0.0, 0.0)];
    /// let yaws = vec![0.0, 0.0];
    /// let segment_times = vec![1.0];
    /// let start_time = 0.0;
    /// let planner = MinimumSnapWaypointPlanner::new(waypoints, yaws, segment_times, start_time);
    /// ```
    pub fn new(
        waypoints: Vec<Vector3<f32>>,
        yaws: Vec<f32>,
        segment_times: Vec<f32>,
        start_time: f32,
    ) -> Result<Self, SimulationError> {
        if waypoints.len() < 2 {
            return Err(SimulationError::OtherError(
                "At least two waypoints are required".to_string(),
            ));
        }
        if waypoints.len() != segment_times.len() + 1 || waypoints.len() != yaws.len() {
            return Err(SimulationError::OtherError("Number of segment times must be one less than number of waypoints, and yaws must match waypoints".to_string()));
        }
        let mut planner = Self {
            waypoints,
            yaws,
            times: segment_times,
            coefficients: Vec::new(),
            yaw_coefficients: Vec::new(),
            start_time,
        };
        planner.compute_minimum_snap_trajectories()?;
        planner.compute_minimum_acceleration_yaw_trajectories()?;
        Ok(planner)
    }
    /// Compute the coefficients for the minimum snap trajectory, calculated for each segment between waypoints
    /// # Errors
    /// * Returns an error if the nalgebra solver fails to solve the linear system
    /// # Example
    /// ```
    /// use peng_quad::planners::MinimumSnapWaypointPlanner;
    /// use nalgebra::Vector3;
    /// let waypoints = vec![Vector3::zeros(), Vector3::new(1.0, 0.0, 0.0)];
    /// let yaws = vec![0.0, 0.0];
    /// let segment_times = vec![1.0];
    /// let start_time = 0.0;
    /// let mut planner = MinimumSnapWaypointPlanner::new(waypoints, yaws, segment_times, start_time).unwrap();
    /// planner.compute_minimum_snap_trajectories();
    /// ```
    pub fn compute_minimum_snap_trajectories(&mut self) -> Result<(), SimulationError> {
        let n_segments = self.waypoints.len() - 1;
        if self.waypoints.len() != self.times.len() + 1 {
            return Err(SimulationError::OtherError(
                "Waypoints and time vector mismatch".into(),
            ));
        }

        // Estimate velocities using finite differences for Hermite slopes
        let mut velocities = vec![Vector3::zeros(); self.waypoints.len()];
        for i in 1..(self.waypoints.len() - 1) {
            let dt_prev = self.times[i - 1];
            let dt_next = self.times[i];
            let dp_prev = (self.waypoints[i] - self.waypoints[i - 1]) / dt_prev;
            let dp_next = (self.waypoints[i + 1] - self.waypoints[i]) / dt_next;
            velocities[i] = (dp_prev + dp_next) / 2.0;
        }
        velocities[0] = (self.waypoints[1] - self.waypoints[0]) / self.times[0];
        velocities[self.waypoints.len() - 1] = (self.waypoints[self.waypoints.len() - 1]
            - self.waypoints[self.waypoints.len() - 2])
            / self.times[self.times.len() - 1];

        self.coefficients.clear();

        // Construct cubic Hermite segments
        for i in 0..n_segments {
            let p0 = self.waypoints[i];
            let p1 = self.waypoints[i + 1];
            let v0 = velocities[i] * self.times[i];
            let v1 = velocities[i + 1] * self.times[i];

            // Coefficients for cubic Hermite spline (normalized time τ ∈ [0, 1])
            // p(τ) = a0 + a1*τ + a2*τ² + a3*τ³
            let a0 = p0;
            let a1 = v0;
            let a2 = 3.0 * (p1 - p0) - 2.0 * v0 - v1;
            let a3 = 2.0 * (p0 - p1) + v0 + v1;

            self.coefficients.push(vec![a0, a1, a2, a3]);
        }

        Ok(())
    }

    /// Evaluate cubic Hermite segment at normalized time τ ∈ [0, 1]
    pub fn evaluate_hermite_segment(
        coeffs: &[Vector3<f32>],
        tau: f32,
    ) -> (Vector3<f32>, Vector3<f32>, Vector3<f32>) {
        let t = tau;
        let dt = 1.0;
        let p = coeffs[0] + coeffs[1] * t + coeffs[2] * t.powi(2) + coeffs[3] * t.powi(3);
        let v = coeffs[1] + coeffs[2] * 2.0 * t + coeffs[3] * 3.0 * t.powi(2);
        let a = coeffs[2] * 2.0 + coeffs[3] * 6.0 * t;
        (p, v / dt, a / dt.powi(2))
    }

    fn normalized_derivative_term(order: usize, j: usize, tau: f32, T: f32) -> f32 {
        if j < order {
            0.0
        } else {
            let mut val = 1.0;
            for k in 0..order {
                val *= (j - k) as f32;
            }
            val * tau.powi((j - order) as i32) / T.powi(order as i32)
        }
    }

    fn snap_cost_matrix_unit_interval() -> SMatrix<f32, 6, 6> {
        let mut q = SMatrix::<f32, 6, 6>::zeros();
        for i in 4..6 {
            for j in 4..6 {
                let coeff_i = (1..=i).rev().take(4).product::<usize>() as f32;
                let coeff_j = (1..=j).rev().take(4).product::<usize>() as f32;
                let power = i + j - 7;
                q[(i, j)] = coeff_i * coeff_j / (power as f32);
            }
        }
        q
    }

    /// Compute the coefficients for yaw trajectories
    /// The yaw trajectory is a cubic polynomial and interpolated between waypoints
    /// # Errors
    /// * Returns an error if nalgebra fails to solve for the coefficients
    /// # Example
    /// ```
    /// use peng_quad::planners::MinimumSnapWaypointPlanner;
    /// use nalgebra::Vector3;
    /// let waypoints = vec![Vector3::zeros(), Vector3::new(1.0, 0.0, 0.0)];
    /// let yaws = vec![0.0, 0.0];
    /// let segment_times = vec![1.0];
    /// let start_time = 0.0;
    /// let mut planner = MinimumSnapWaypointPlanner::new(waypoints, yaws, segment_times, start_time).unwrap();
    /// planner.compute_minimum_snap_trajectories();
    /// planner.compute_minimum_acceleration_yaw_trajectories();
    /// ```
    pub fn compute_minimum_acceleration_yaw_trajectories(&mut self) -> Result<(), SimulationError> {
        let n = self.yaws.len() - 1; // Number of segments
        for i in 0..n {
            let (duration, start_yaw, end_yaw) = (self.times[i], self.yaws[i], self.yaws[i + 1]);
            let mut a = SMatrix::<f32, 4, 4>::zeros();
            let mut b = SMatrix::<f32, 4, 1>::zeros();
            (a[(0, 0)], a[(1, 1)]) = (1.0, 1.0);
            (b[0], b[2]) = (start_yaw, end_yaw);
            for j in 0..4 {
                a[(2, j)] = duration.powi(j as i32);
                if j > 0 {
                    a[(3, j)] = j as f32 * duration.powi(j as i32 - 1);
                }
            }
            let yaw_coeffs = a.lu().solve(&b).ok_or(SimulationError::NalgebraError(
                "Failed to solve for yaw coefficients in MinimumSnapWaypointPlanner".to_string(),
            ))?;
            self.yaw_coefficients.push(yaw_coeffs.as_slice().to_vec());
        }
        Ok(())
    }
    /// Evaluate the trajectory at a given time, returns the position, velocity, yaw, and yaw rate at the given time
    /// # Arguments
    /// * `t` - The time to evaluate the trajectory at
    /// * `coeffs` - The coefficients for the position trajectory
    /// * `yaw_coeffs` - The coefficients for the yaw trajectory
    /// # Returns
    /// * `position` - The position at the given time (meters)
    /// * `velocity` - The velocity at the given time (meters / second)
    /// * `yaw` - The yaw at the given time (radians)
    /// * `yaw_rate` - The yaw rate at the given time (radians / second)
    /// # Example
    /// ```
    /// use nalgebra::Vector3;
    /// use peng_quad::planners::MinimumSnapWaypointPlanner;
    /// let waypoints = vec![Vector3::zeros(), Vector3::new(1.0, 0.0, 0.0)];
    /// let yaws = vec![0.0, 0.0];
    /// let segment_times = vec![1.0];
    /// let start_time = 0.0;
    /// let mut planner = MinimumSnapWaypointPlanner::new(waypoints, yaws, segment_times, start_time).unwrap();
    /// planner.compute_minimum_snap_trajectories();
    /// planner.compute_minimum_acceleration_yaw_trajectories();
    /// let (position, velocity, yaw, yaw_rate) = planner.evaluate_polynomial(0.5, &planner.coefficients[0], &planner.yaw_coefficients[0]);
    /// ```
    pub fn evaluate_polynomial(
        &self,
        t: f32,
        coeffs: &[Vector3<f32>],
        yaw_coeffs: &[f32],
    ) -> (Vector3<f32>, Vector3<f32>, Vector3<f32>, f32, f32) {
        let mut position = Vector3::zeros();
        let mut velocity = Vector3::zeros();
        let mut acceleration = Vector3::zeros();
        let mut yaw = 0.0;
        let mut yaw_rate = 0.0;

        for (i, coeff) in coeffs.iter().enumerate() {
            let i_f32 = i as f32;
            let t_pow_i = t.powi(i as i32);
            position += coeff * t_pow_i;

            if i >= 1 {
                velocity += coeff * i_f32 * t.powi(i as i32 - 1);
            }
            if i >= 2 {
                acceleration += coeff * i_f32 * (i_f32 - 1.0) * t.powi(i as i32 - 2);
            }
        }

        for (i, &coeff) in yaw_coeffs.iter().enumerate() {
            let i_f32 = i as f32;
            yaw += coeff * t.powi(i as i32);
            if i >= 1 {
                yaw_rate += coeff * i_f32 * t.powi(i as i32 - 1);
            }
        }

        (position, velocity, acceleration, yaw, yaw_rate)
    }
}
/// Implement the `Planner` trait for `MinimumSnapWaypointPlanner`
#[async_trait]
impl Planner for MinimumSnapWaypointPlanner {
    async fn plan(
        &self,
        _current_position: Vector3<f32>,
        _current_velocity: Vector3<f32>,
        time: f32,
    ) -> TrajectoryPoint {
        // Returns desired position, desired velo
        let relative_time = time - self.start_time;
        // Find the current segment
        let mut segment_start_time = 0.0;
        let mut current_segment = 0;
        for (i, &segment_duration) in self.times.iter().enumerate() {
            if relative_time < segment_start_time + segment_duration {
                current_segment = i;
                break;
            }
            segment_start_time += segment_duration;
        }
        // Evaluate the polynomial for the current segment
        let segment_time = relative_time - segment_start_time;
        let (position, velocity, acceleration, yaw, _yaw_rate) = self.evaluate_polynomial(
            segment_time,
            &self.coefficients[current_segment],
            &self.yaw_coefficients[current_segment],
        );

        TrajectoryPoint {
            position,
            velocity,
            yaw,
            acceleration: Some(acceleration),
        }
    }

    fn is_finished(
        &self,
        current_position: Vector3<f32>,
        time: f32,
    ) -> Result<bool, SimulationError> {
        let last_waypoint = self.waypoints.last().ok_or(SimulationError::OtherError(
            "No waypoints available".to_string(),
        ))?;
        Ok(time >= self.start_time + self.times.iter().sum::<f32>()
            && (current_position - last_waypoint).norm() < 0.1)
    }
}
