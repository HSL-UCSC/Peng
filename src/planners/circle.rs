use crate::SimulationError;
use async_trait::async_trait;
use nalgebra::Vector3;

use crate::planners::Planner;

use super::TrajectoryPoint;

/// Planner for circular trajectories
/// # Example
/// ```
/// use nalgebra::Vector3;
/// use peng_quad::planners::CirclePlanner;
/// let circle_planner = CirclePlanner {
///     center: Vector3::new(1.0, 1.0, 1.0),
///     radius: 1.0,
///     angular_velocity: 1.0,
///     start_position: Vector3::new(0.0, 0.0, 0.0),
///     start_time: 0.0,
///     duration: 1.0,
///     start_yaw: 0.0,
///     end_yaw: 0.0,
///     ramp_time: 0.1,
/// };
/// ```
pub struct CirclePlanner {
    /// Center of the circular trajectory
    pub center: Vector3<f32>,
    /// Radius of the circular trajectory
    pub radius: f32,
    /// Angular velocity of the circular motion
    pub angular_velocity: f32,
    /// Starting position of the trajectory
    pub start_position: Vector3<f32>,
    /// Start time of the trajectory
    pub start_time: f32,
    /// Duration of the trajectory
    pub duration: f32,
    /// Starting yaw angle
    pub start_yaw: f32,
    /// Ending yaw angle
    pub end_yaw: f32,
    /// Ramp-up time for smooth transitions
    pub ramp_time: f32,
}
/// Implementation of the Planner trait for CirclePlanner
#[async_trait]
impl Planner for CirclePlanner {
    async fn plan(
        &self,
        _current_position: Vector3<f32>,
        _current_velocity: Vector3<f32>,
        time: f32,
    ) -> TrajectoryPoint {
        let t_actual = (time - self.start_time).clamp(0.0, self.duration);
        let t_norm = t_actual / self.duration;

        // Smooth ramp-in and ramp-out logic
        let smooth_start = if t_norm < self.ramp_time / self.duration {
            let t_ramp = t_norm / (self.ramp_time / self.duration);
            t_ramp * t_ramp * (3.0 - 2.0 * t_ramp)
        } else {
            1.0
        };

        let velocity_ramp = if t_norm < self.ramp_time / self.duration {
            smooth_start
        } else if t_norm > 1.0 - self.ramp_time / self.duration {
            let t_down = (1.0 - t_norm) / (self.ramp_time / self.duration);
            t_down * t_down * (3.0 - 2.0 * t_down)
        } else {
            1.0
        };

        // Core circular motion
        let angle = self.angular_velocity * t_actual;
        let cos_a = angle.cos();
        let sin_a = angle.sin();

        let circle_offset = Vector3::new(self.radius * cos_a, self.radius * sin_a, 0.0);

        let tangential_velocity = Vector3::new(
            -self.radius * self.angular_velocity * sin_a,
            self.radius * self.angular_velocity * cos_a,
            0.0,
        );

        let acceleration = Vector3::new(
            -self.radius * self.angular_velocity.powi(2) * cos_a,
            -self.radius * self.angular_velocity.powi(2) * sin_a,
            0.0,
        );

        let position = self.start_position
            + smooth_start * ((self.center + circle_offset) - self.start_position);

        let velocity = tangential_velocity * velocity_ramp;
        let acceleration = acceleration * velocity_ramp;

        let yaw = self.start_yaw + (self.end_yaw - self.start_yaw) * t_norm;

        TrajectoryPoint {
            position,
            velocity,
            acceleration: Some(acceleration),
            yaw,
        }
    }

    fn is_finished(
        &self,
        _current_position: Vector3<f32>,
        time: f32,
    ) -> Result<bool, SimulationError> {
        Ok(time >= self.start_time + self.duration)
    }
}
