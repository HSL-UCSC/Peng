use crate::environment::Obstacle;
use crate::SimulationError;
use async_trait::async_trait;
use nalgebra::Vector3;

use crate::planners::Planner;

/// Obstacle avoidance planner that uses a potential field approach to avoid obstacles
///
/// The planner calculates a repulsive force for each obstacle and an attractive force towards the goal
/// The resulting force is then used to calculate the desired position and velocity
/// # Example
/// ```
/// use nalgebra::Vector3;
/// use peng_quad::environment::{Obstacle};
/// use peng_quad::{ObstacleAvoidancePlanner};
/// let planner = ObstacleAvoidancePlanner {
///     target_position: Vector3::new(0.0, 0.0, 1.0),
///     start_time: 0.0,
///     duration: 10.0,
///     start_yaw: 0.0,
///     end_yaw: 0.0,
///     obstacles: vec![Obstacle {
///         position: Vector3::new(1.0, 0.0, 1.0),
///         velocity: Vector3::new(0.0, 0.0, 0.0),
///         radius: 0.5,
///     }],
///     k_att: 1.0,
///     k_rep: 1.0,
///     k_vortex: 1.0,
///     d0: 1.0,
///     d_target: 1.0,
///     max_speed: 1.0,
/// };
/// ```
pub struct ObstacleAvoidancePlanner {
    /// Target position of the planner
    pub target_position: Vector3<f32>,
    /// Start time of the planner
    pub start_time: f32,
    /// Duration of the planner
    pub duration: f32,
    /// Starting yaw angle
    pub start_yaw: f32,
    /// Ending yaw angle
    pub end_yaw: f32,
    /// List of obstacles
    pub obstacles: Vec<Obstacle>,
    /// Attractive force gain
    pub k_att: f32,
    /// Repulsive force gain
    pub k_rep: f32,
    /// Vortex force gain
    pub k_vortex: f32,
    /// Influence distance of obstacles
    pub d0: f32,
    /// Influence distance of target
    pub d_target: f32,
    /// Maximum speed of the quadrotor
    pub max_speed: f32,
}
/// Implementation of the Planner trait for ObstacleAvoidancePlanner
#[async_trait]
impl Planner for ObstacleAvoidancePlanner {
    async fn plan(
        &self,
        current_position: Vector3<f32>,
        current_velocity: Vector3<f32>,
        time: f32,
    ) -> (Vector3<f32>, Vector3<f32>, f32) {
        let t = ((time - self.start_time) / self.duration).clamp(0.0, 1.0);
        let distance_to_target = (self.target_position - current_position).norm();
        let f_att = self.k_att
            * self.smooth_attractive_force(distance_to_target)
            * (self.target_position - current_position).normalize();
        // Repulsive force from obstacles
        let mut f_rep = Vector3::zeros();
        let mut f_vortex = Vector3::zeros();
        for obstacle in &self.obstacles {
            let diff = current_position - obstacle.position;
            let distance = diff.norm();
            if distance < self.d0 {
                f_rep += self.k_rep
                    * (1.0 / distance - 1.0 / self.d0)
                    * (1.0 / distance.powi(2))
                    * diff.normalize();
                f_vortex +=
                    self.k_vortex * current_velocity.cross(&diff).normalize() / distance.powi(2);
            }
        }
        let f_total = f_att + f_rep + f_vortex;
        let desired_velocity = f_total.normalize() * self.max_speed.min(f_total.norm());
        let desired_position = current_position + desired_velocity * self.duration * (1.0 - t);
        let desired_yaw = self.start_yaw + (self.end_yaw - self.start_yaw) * t;
        (desired_position, desired_velocity, desired_yaw)
    }

    fn is_finished(
        &self,
        current_position: Vector3<f32>,
        time: f32,
    ) -> Result<bool, SimulationError> {
        Ok((current_position - self.target_position).norm() < 0.1
            && time >= self.start_time + self.duration)
    }
}
/// Implementation of the ObstacleAvoidancePlanner
impl ObstacleAvoidancePlanner {
    /// A smooth attractive force function that transitions from linear to exponential decay
    /// When the distance to the target is less than the target distance, the force is linear
    /// When the distance is greater, the force decays exponentially
    /// # Arguments
    /// * `distance` - The distance to the target
    /// # Returns
    /// * The attractive force
    /// # Example
    /// ```
    /// use peng_quad::ObstacleAvoidancePlanner;
    /// let planner = ObstacleAvoidancePlanner {
    ///    target_position: nalgebra::Vector3::new(0.0, 0.0, 1.0),
    ///     start_time: 0.0,
    ///     duration: 10.0,
    ///     start_yaw: 0.0,
    ///     end_yaw: 0.0,
    ///     obstacles: vec![],
    ///     k_att: 1.0,
    ///     k_rep: 1.0,
    ///     k_vortex: 1.0,
    ///     d0: 1.0,
    ///     d_target: 1.0,
    ///     max_speed: 1.0,
    /// };
    /// let distance = 1.0;
    /// let force = planner.smooth_attractive_force(distance);
    /// ```
    #[inline]
    pub fn smooth_attractive_force(&self, distance: f32) -> f32 {
        if distance <= self.d_target {
            distance
        } else {
            self.d_target + (distance - self.d_target).tanh()
        }
    }
}
