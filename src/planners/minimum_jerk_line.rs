use crate::SimulationError;
use async_trait::async_trait;
use nalgebra::Vector3;

use crate::planners::Planner;

use super::TrajectoryPoint;

/// Planner for minimum jerk trajectories along a straight line
/// # Example
/// ```
/// use nalgebra::Vector3;
/// use peng_quad::planners::MinimumJerkLinePlanner;
/// let minimum_jerk_line_planner = MinimumJerkLinePlanner {
///     start_position: Vector3::new(0.0, 0.0, 0.0),
///     end_position: Vector3::new(1.0, 1.0, 1.0),
///     start_yaw: 0.0,
///     end_yaw: 0.0,
///     start_time: 0.0,
///     duration: 1.0,
/// };
/// ```
pub struct MinimumJerkLinePlanner {
    /// Starting position of the trajectory
    pub start_position: Vector3<f32>,
    /// Ending position of the trajectory
    pub end_position: Vector3<f32>,
    /// Starting yaw angle
    pub start_yaw: f32,
    /// Ending yaw angle
    pub end_yaw: f32,
    /// Start time of the trajectory
    pub start_time: f32,
    /// Duration of the trajectory
    pub duration: f32,
}
/// Implementation of the planner trait for minimum jerk line planner
#[async_trait]
impl Planner for MinimumJerkLinePlanner {
    async fn plan(
        &self,
        _current_position: Vector3<f32>,
        _current_velocity: Vector3<f32>,
        time: f32,
    ) -> TrajectoryPoint {
        let t = ((time - self.start_time) / self.duration).clamp(0.0, 1.0);
        let t2 = t * t;
        let t3 = t2 * t;
        let t4 = t3 * t;
        let s = 10.0 * t2 - 15.0 * t3 + 6.0 * t4;
        let s_dot = (30.0 * t.powi(2) - 60.0 * t.powi(3) + 30.0 * t.powi(4)) / self.duration;
        let position = self.start_position + (self.end_position - self.start_position) * s;
        let velocity = (self.end_position - self.start_position) * s_dot;
        let yaw = self.start_yaw + (self.end_yaw - self.start_yaw) * s;
        TrajectoryPoint {
            position,
            velocity,
            yaw,
            acceleration: None,
        }
    }

    fn is_finished(
        &self,
        _current_position: Vector3<f32>,
        _time: f32,
    ) -> Result<bool, SimulationError> {
        Ok((_current_position - self.end_position).norm() < 0.01
            && _time >= self.start_time + self.duration)
    }
}
