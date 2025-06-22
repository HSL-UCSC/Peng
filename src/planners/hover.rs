use crate::SimulationError;
use async_trait::async_trait;
use nalgebra::Vector3;

use crate::planners::Planner;

use super::TrajectoryPoint;

/// Planner for hovering at a fixed position
/// # Example
/// ```
/// use nalgebra::Vector3;
/// use peng_quad::planners::HoverPlanner;
/// let hover_planner = HoverPlanner {
///     target_position: Vector3::new(0.0, 0.0, 0.0),
///     target_yaw: 0.0,
/// };
/// ```
pub struct HoverPlanner {
    /// Target position for hovering
    pub target_position: Vector3<f32>,
    /// Target yaw angle for hovering
    pub target_yaw: f32,
}
/// Implementation of the `Planner` trait for the `HoverPlanner`
#[async_trait]
impl Planner for HoverPlanner {
    async fn plan(
        &self,
        _current_position: Vector3<f32>,
        _current_velocity: Vector3<f32>,
        _time: f32,
    ) -> TrajectoryPoint {
        TrajectoryPoint {
            position: self.target_position,
            velocity: Vector3::zeros(),
            yaw: self.target_yaw,
            acceleration: None,
        }
    }

    fn is_finished(
        &self,
        _current_position: Vector3<f32>,
        _time: f32,
    ) -> Result<bool, SimulationError> {
        Ok(false) // Hover planner never "finished"
    }
}
