use crate::SimulationError;
use async_trait::async_trait;
use nalgebra::Vector3;
use std::f32::consts::PI;

use crate::planners::Planner;

use super::TrajectoryPoint;

/// Planner for Lissajous curve trajectories
/// # Example
/// ```
/// use nalgebra::Vector3;
/// use peng_quad::planners::LissajousPlanner;
/// let lissajous_planner = LissajousPlanner {
///     start_position: Vector3::new(0.0, 0.0, 0.0),
///     center: Vector3::new(1.0, 1.0, 1.0),
///     amplitude: Vector3::new(1.0, 1.0, 1.0),
///     frequency: Vector3::new(1.0, 1.0, 1.0),
///     phase: Vector3::new(0.0, 0.0, 0.0),
///     start_time: 0.0,
///     duration: 1.0,
///     start_yaw: 0.0,
///     end_yaw: 0.0,
///     ramp_time: 0.1,
/// };
/// ```
pub struct LissajousPlanner {
    /// Starting position of the trajectory
    pub start_position: Vector3<f32>,
    /// Center of the Lissajous curve
    pub center: Vector3<f32>,
    /// Amplitude of the Lissajous curve
    pub amplitude: Vector3<f32>,
    /// Frequency of the Lissajous curve
    pub frequency: Vector3<f32>,
    /// Phase of the Lissajous curve
    pub phase: Vector3<f32>,
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
/// Implementation of the planner trait for Lissajous curve trajectories
#[async_trait]
impl Planner for LissajousPlanner {
    async fn plan(
        &self,
        _current_position: Vector3<f32>,
        _current_velocity: Vector3<f32>,
        time: f32,
    ) -> TrajectoryPoint {
        let t = ((time - self.start_time) / self.duration).clamp(0.0, 1.0);
        let smooth_start = if t < self.ramp_time / self.duration {
            let t_ramp = t / (self.ramp_time / self.duration);
            t_ramp * t_ramp * (3.0 - 2.0 * t_ramp)
        } else {
            1.0
        };
        let velocity_ramp = if t < self.ramp_time / self.duration {
            smooth_start
        } else if t > 1.0 - self.ramp_time / self.duration {
            let t_down = (1.0 - t) / (self.ramp_time / self.duration);
            t_down * t_down * (3.0 - 2.0 * t_down)
        } else {
            1.0
        };
        let ang_pos = self.frequency * t * 2.0 * PI + self.phase;
        let lissajous = self.amplitude.component_mul(&ang_pos.map(f32::sin));
        let position =
            self.start_position + smooth_start * ((self.center + lissajous) - self.start_position);
        let mut velocity = Vector3::new(
            self.amplitude.x * self.frequency.x * 2.0 * PI * ang_pos.x.cos(),
            self.amplitude.y * self.frequency.y * 2.0 * PI * ang_pos.y.cos(),
            self.amplitude.z * self.frequency.z * 2.0 * PI * ang_pos.z.cos(),
        ) * velocity_ramp
            / self.duration;
        if t < self.ramp_time / self.duration {
            let transition_velocity = (self.center - self.start_position)
                * (2.0 * t / self.ramp_time - 2.0 * t * t / (self.ramp_time * self.ramp_time))
                / self.duration;
            velocity += transition_velocity;
        }
        let yaw = self.start_yaw + (self.end_yaw - self.start_yaw) * t;
        TrajectoryPoint {
            position,
            velocity,
            acceleration: None,
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
