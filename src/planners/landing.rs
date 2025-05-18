use crate::SimulationError;
use async_trait::async_trait;
use nalgebra::Vector3;

use crate::planners::Planner;

/// Planner for landing maneuvers
/// # Example
/// ```
/// use nalgebra::Vector3;
/// use peng_quad::planners::LandingPlanner;
/// let landing_planner = LandingPlanner {
///    start_position: Vector3::new(0.0, 0.0, 1.0),
///     start_time: 0.0,
///     duration: 1.0,
///     start_yaw: 0.0,
/// };
/// ```
pub struct LandingPlanner {
    /// Starting position of the landing maneuver
    pub start_position: Vector3<f32>,
    /// Start time of the landing maneuver
    pub start_time: f32,
    /// Duration of the landing maneuver
    pub duration: f32,
    /// Starting yaw angle
    pub start_yaw: f32,
}
/// Implementation of the Planner trait for LandingPlanner
#[async_trait]
impl Planner for LandingPlanner {
    async fn plan(
        &self,
        _current_position: Vector3<f32>,
        _current_velocity: Vector3<f32>,
        time: f32,
    ) -> (Vector3<f32>, Vector3<f32>, f32) {
        let t = ((time - self.start_time) / self.duration).clamp(0.0, 1.0);
        let target_z = self.start_position.z * (1.0 - t);
        let target_position = Vector3::new(self.start_position.x, self.start_position.y, target_z);
        let target_velocity = Vector3::new(0.0, 0.0, -self.start_position.z / self.duration);
        (target_position, target_velocity, self.start_yaw)
    }

    fn is_finished(
        &self,
        current_position: Vector3<f32>,
        time: f32,
    ) -> Result<bool, SimulationError> {
        Ok(current_position.z < 0.05 || time >= self.start_time + self.duration)
    }
}
