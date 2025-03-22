use crate::environment::Obstacle;
use nalgebra::{SMatrix, UnitQuaternion, Vector3};
use crate::quadrotor::{QuadrotorState};
use std::f32::consts::PI;
use crate::SimulationError;
use crate::{parse_vector3, parse_f32};

/// Enum representing different types of trajectory planners
/// # Example
/// ```
/// use peng_quad::PlannerType;
/// use peng_quad::HoverPlanner;
/// let hover_planner : PlannerType = PlannerType::Hover(HoverPlanner{
///     target_position: nalgebra::Vector3::new(0.0, 0.0, 1.0),
///     target_yaw: 0.0,
/// });
/// ```
pub enum PlannerType {
    /// Hover planner
    Hover(HoverPlanner),
    /// Minimum jerk line planner
    MinimumJerkLine(MinimumJerkLinePlanner),
    /// Minimum jerk circle planner
    Lissajous(LissajousPlanner),
    /// Minimum jerk circle planner
    Circle(CirclePlanner),
    /// Minimum jerk landing planner
    Landing(LandingPlanner),
    /// Obstacle avoidance planner
    ObstacleAvoidance(ObstacleAvoidancePlanner),
    /// Minimum snap waypoint planner
    MinimumSnapWaypoint(MinimumSnapWaypointPlanner),
    ///// Minimum snap waypoint planner
    //HeadingPlanner(MinimumSnapWaypointPlanner),
}
/// Implementation of the planner type
impl PlannerType {
    /// Plans the trajectory based on the current planner type
    /// # Arguments
    /// * `current_position` - The current position of the quadrotor
    /// * `current_velocity` - The current velocity of the quadrotor
    /// * `time` - The current simulation time
    /// # Returns
    /// * A tuple containing the desired position, velocity, and yaw angle
    /// # Example
    /// ```
    /// use nalgebra::Vector3;
    /// use peng_quad::PlannerType;
    /// use peng_quad::HoverPlanner;
    /// let hover_planner = HoverPlanner {
    ///     target_position: Vector3::new(0.0, 0.0, 1.0),
    ///     target_yaw: 0.0
    /// };
    /// let hover_planner_type = PlannerType::Hover(hover_planner);
    /// let (desired_position, desired_velocity, desired_yaw) = hover_planner_type.plan(Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 0.0), 0.0);
    /// ```
    pub fn plan(
        &self,
        current_position: Vector3<f32>,
        current_velocity: Vector3<f32>,
        time: f32,
    ) -> (Vector3<f32>, Vector3<f32>, f32) {
        match self {
            PlannerType::Hover(p) => p.plan(current_position, current_velocity, time),
            PlannerType::MinimumJerkLine(p) => p.plan(current_position, current_velocity, time),
            PlannerType::Lissajous(p) => p.plan(current_position, current_velocity, time),
            PlannerType::Circle(p) => p.plan(current_position, current_velocity, time),
            PlannerType::Landing(p) => p.plan(current_position, current_velocity, time),
            PlannerType::ObstacleAvoidance(p) => p.plan(current_position, current_velocity, time),
            PlannerType::MinimumSnapWaypoint(p) => p.plan(current_position, current_velocity, time),
        }
    }
    /// Checks if the current trajectory is finished
    /// # Arguments
    /// * `current_position` - The current position of the quadrotor
    /// * `time` - The current simulation time
    /// # Returns
    /// * `true` if the trajectory is finished, `false` otherwise
    /// # Example
    /// ```
    /// use nalgebra::Vector3;
    /// use peng_quad::PlannerType;
    /// use peng_quad::HoverPlanner;
    /// use peng_quad::Planner;
    /// let hover_planner = HoverPlanner{
    ///     target_position: Vector3::new(0.0, 0.0, 1.0),
    ///     target_yaw: 0.0,
    /// };
    /// let is_finished = hover_planner.is_finished(Vector3::new(0.0, 0.0, 0.0), 0.0);
    /// ```
    pub fn is_finished(
        &self,
        current_position: Vector3<f32>,
        time: f32,
    ) -> Result<bool, SimulationError> {
        match self {
            PlannerType::Hover(p) => p.is_finished(current_position, time),
            PlannerType::MinimumJerkLine(p) => p.is_finished(current_position, time),
            PlannerType::Lissajous(p) => p.is_finished(current_position, time),
            PlannerType::Circle(p) => p.is_finished(current_position, time),
            PlannerType::Landing(p) => p.is_finished(current_position, time),
            PlannerType::ObstacleAvoidance(p) => p.is_finished(current_position, time),
            PlannerType::MinimumSnapWaypoint(p) => p.is_finished(current_position, time),
        }
    }
}
/// Trait defining the interface for trajectory planners
/// # Example
/// ```
/// use nalgebra::Vector3;
/// use peng_quad::{Planner, SimulationError};
/// struct TestPlanner;
/// impl Planner for TestPlanner {
///    fn plan(
///         &self,
///         current_position: Vector3<f32>,
///         current_velocity: Vector3<f32>,
///         time: f32,
/// ) -> (Vector3<f32>, Vector3<f32>, f32) {
///         (Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 0.0), 0.0)
///     }
///     fn is_finished(
///         &self,
///         current_position: Vector3<f32>,
///         time: f32,
///     ) -> Result<bool, SimulationError> {
///         Ok(true)
///     }
/// }
/// ```
pub trait Planner {
    /// Plans the trajectory based on the current state and time
    /// # Arguments
    /// * `current_position` - The current position of the quadrotor
    /// * `current_velocity` - The current velocity of the quadrotor
    /// * `time` - The current simulation time
    /// # Returns
    /// * A tuple containing the desired position, velocity, and yaw angle
    /// # Example
    /// ```
    /// use nalgebra::Vector3;
    /// use peng_quad::{Planner, SimulationError};
    /// struct TestPlanner;
    /// impl Planner for TestPlanner {
    ///     fn plan(
    ///         &self,
    ///         current_position: Vector3<f32>,
    ///         current_velocity: Vector3<f32>,
    ///         time: f32,
    /// ) -> (Vector3<f32>, Vector3<f32>, f32) {
    ///         (Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 0.0), 0.0)
    ///     }
    ///     fn is_finished(
    ///         &self,
    ///         current_position: Vector3<f32>,
    ///         time: f32,
    ///     ) -> Result<bool, SimulationError> {
    ///         Ok(true)
    ///     }
    /// }
    /// ```
    fn plan(
        &self,
        current_position: Vector3<f32>,
        current_velocity: Vector3<f32>,
        time: f32,
    ) -> (Vector3<f32>, Vector3<f32>, f32);
    /// Checks if the current trajectory is finished
    /// # Arguments
    /// * `current_position` - The current position of the quadrotor
    /// * `time` - The current simulation time
    /// # Returns
    /// * `true` if the trajectory is finished, `false` otherwise
    /// # Example
    /// ```
    /// use nalgebra::Vector3;
    /// use peng_quad::{Planner, SimulationError};
    /// struct TestPlanner;
    /// impl Planner for TestPlanner {
    ///     fn plan(
    ///         &self,
    ///         current_position: Vector3<f32>,
    ///         current_velocity: Vector3<f32>,
    ///         time: f32,
    /// ) -> (Vector3<f32>, Vector3<f32>, f32) {
    ///         (Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 0.0), 0.0)
    ///     }
    ///     fn is_finished(
    ///         &self,
    ///         current_position: Vector3<f32>,
    ///         time: f32,
    ///     ) -> Result<bool, SimulationError> {
    ///         Ok(true)
    ///     }
    /// }
    /// ```
    fn is_finished(
        &self,
        current_position: Vector3<f32>,
        time: f32,
    ) -> Result<bool, SimulationError>;
}
/// Planner for hovering at a fixed position
/// # Example
/// ```
/// use nalgebra::Vector3;
/// use peng_quad::HoverPlanner;
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
impl Planner for HoverPlanner {
    fn plan(
        &self,
        _current_position: Vector3<f32>,
        _current_velocity: Vector3<f32>,
        _time: f32,
    ) -> (Vector3<f32>, Vector3<f32>, f32) {
        (self.target_position, Vector3::zeros(), self.target_yaw)
    }

    fn is_finished(
        &self,
        _current_position: Vector3<f32>,
        _time: f32,
    ) -> Result<bool, SimulationError> {
        Ok(false) // Hover planner never "finished"
    }
}
/// Planner for minimum jerk trajectories along a straight line
/// # Example
/// ```
/// use nalgebra::Vector3;
/// use peng_quad::MinimumJerkLinePlanner;
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
impl Planner for MinimumJerkLinePlanner {
    fn plan(
        &self,
        _current_position: Vector3<f32>,
        _current_velocity: Vector3<f32>,
        time: f32,
    ) -> (Vector3<f32>, Vector3<f32>, f32) {
        let t = ((time - self.start_time) / self.duration).clamp(0.0, 1.0);
        let t2 = t * t;
        let t3 = t2 * t;
        let t4 = t3 * t;
        let s = 10.0 * t2 - 15.0 * t3 + 6.0 * t4;
        let s_dot = (30.0 * t.powi(2) - 60.0 * t.powi(3) + 30.0 * t.powi(4)) / self.duration;
        let position = self.start_position + (self.end_position - self.start_position) * s;
        let velocity = (self.end_position - self.start_position) * s_dot;
        let yaw = self.start_yaw + (self.end_yaw - self.start_yaw) * s;
        (position, velocity, yaw)
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
/// Planner for Lissajous curve trajectories
/// # Example
/// ```
/// use nalgebra::Vector3;
/// use peng_quad::LissajousPlanner;
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
impl Planner for LissajousPlanner {
    fn plan(
        &self,
        _current_position: Vector3<f32>,
        _current_velocity: Vector3<f32>,
        time: f32,
    ) -> (Vector3<f32>, Vector3<f32>, f32) {
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
        (position, velocity, yaw)
    }

    fn is_finished(
        &self,
        _current_position: Vector3<f32>,
        time: f32,
    ) -> Result<bool, SimulationError> {
        Ok(time >= self.start_time + self.duration)
    }
}
/// Planner for circular trajectories
/// # Example
/// ```
/// use nalgebra::Vector3;
/// use peng_quad::CirclePlanner;
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
impl Planner for CirclePlanner {
    fn plan(
        &self,
        _current_position: Vector3<f32>,
        _current_velocity: Vector3<f32>,
        time: f32,
    ) -> (Vector3<f32>, Vector3<f32>, f32) {
        let t = (time - self.start_time) / self.duration;
        let t = t.clamp(0.0, 1.0);
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
        let angle = self.angular_velocity * t * self.duration;
        let circle_offset = Vector3::new(self.radius * angle.cos(), self.radius * angle.sin(), 0.0);
        let position = self.start_position
            + smooth_start * ((self.center + circle_offset) - self.start_position);
        let tangential_velocity = Vector3::new(
            -self.radius * self.angular_velocity * angle.sin(),
            self.radius * self.angular_velocity * angle.cos(),
            0.0,
        );
        let velocity = tangential_velocity * velocity_ramp;
        let yaw = self.start_yaw + (self.end_yaw - self.start_yaw) * t;
        (position, velocity, yaw)
    }

    fn is_finished(
        &self,
        _current_position: Vector3<f32>,
        time: f32,
    ) -> Result<bool, SimulationError> {
        Ok(time >= self.start_time + self.duration)
    }
}
/// Planner for landing maneuvers
/// # Example
/// ```
/// use nalgebra::Vector3;
/// use peng_quad::LandingPlanner;
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
impl Planner for LandingPlanner {
    fn plan(
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
/// Manages different trajectory planners and switches between them
/// # Example
/// ```
/// use nalgebra::Vector3;
/// use peng_quad::PlannerManager;
/// let initial_position = Vector3::new(0.0, 0.0, 1.0);
/// let initial_yaw = 0.0;
/// let planner_manager = PlannerManager::new(initial_position, initial_yaw);
/// ```
pub struct PlannerManager {
    /// The current planner
    pub current_planner: PlannerType,
}
/// Implementation of the PlannerManager
impl PlannerManager {
    /// Creates a new PlannerManager with an initial hover planner
    /// # Arguments
    /// * `initial_position` - The initial position for hovering
    /// * `initial_yaw` - The initial yaw angle for hovering
    /// # Returns
    /// * A new PlannerManager instance
    /// # Example
    /// ```
    /// use nalgebra::Vector3;
    /// use peng_quad::PlannerManager;
    /// let initial_position = Vector3::new(0.0, 0.0, 1.0);
    /// let initial_yaw = 0.0;
    /// let planner_manager = PlannerManager::new(initial_position, initial_yaw);
    /// ```
    pub fn new(initial_position: Vector3<f32>, initial_yaw: f32) -> Self {
        let hover_planner = HoverPlanner {
            target_position: initial_position,
            target_yaw: initial_yaw,
        };
        Self {
            current_planner: PlannerType::Hover(hover_planner),
        }
    }
    /// Sets a new planner
    /// # Arguments
    /// * `new_planner` - The new planner to be set
    /// # Example
    /// ```
    /// use nalgebra::Vector3;
    /// use peng_quad::{PlannerManager, CirclePlanner, PlannerType};
    /// let initial_position = Vector3::new(0.0, 0.0, 1.0);
    /// let initial_yaw = 0.0;
    /// let mut planner_manager = PlannerManager::new(initial_position, initial_yaw);
    /// let new_planner = CirclePlanner {
    ///     center: Vector3::new(0.0, 0.0, 1.0),
    ///     radius: 1.0,
    ///     angular_velocity: 1.0,
    ///     start_yaw: 0.0,
    ///     end_yaw: 0.0,
    ///     start_time: 0.0,
    ///     duration: 10.0,
    ///     ramp_time: 1.0,
    ///     start_position: Vector3::new(0.0, 0.0, 1.0),
    /// };
    /// planner_manager.set_planner(PlannerType::Circle(new_planner));
    /// ```
    pub fn set_planner(&mut self, new_planner: PlannerType) {
        self.current_planner = new_planner;
    }
    /// Updates the current planner and returns the desired position, velocity, and yaw
    /// # Arguments
    /// * `current_position` - The current position of the quadrotor
    /// * `current_orientation` - The current orientation of the quadrotor
    /// * `current_velocity` - The current velocity of the quadrotor
    /// * `time` - The current simulation time
    /// # Returns
    /// * A tuple containing the desired position, velocity, and yaw angle
    /// # Errors
    /// * Returns a SimulationError if the current planner is not finished
    /// # Example
    /// ```
    /// use nalgebra::{Vector3, UnitQuaternion};
    /// use peng_quad::{PlannerManager, SimulationError};
    /// let initial_position = Vector3::new(0.0, 0.0, 1.0);
    /// let initial_yaw = 0.0;
    /// let mut planner_manager = PlannerManager::new(initial_position, initial_yaw);
    /// let current_position = Vector3::new(0.0, 0.0, 1.0);
    /// let current_orientation = UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0);
    /// let current_velocity = Vector3::new(0.0, 0.0, 0.0);
    /// let obstacles = vec![];
    /// let time = 0.0;
    /// let result = planner_manager.update(current_position, current_orientation, current_velocity, time, &obstacles);
    /// match result {
    ///     Ok((target_position, target_velocity, target_yaw)) => {
    ///         println!("Target Position: {:?}", target_position);
    ///         println!("Target Velocity: {:?}", target_velocity);
    ///         println!("Target Yaw: {:?}", target_yaw);
    ///     }
    ///     Err(SimulationError) => {
    ///         log::error!("Error: Planner is not finished");
    ///     }
    /// }
    /// ```
    pub fn update(
        &mut self,
        current_position: Vector3<f32>,
        current_orientation: UnitQuaternion<f32>,
        current_velocity: Vector3<f32>,
        time: f32,
        obstacles: &[Obstacle],
    ) -> Result<(Vector3<f32>, Vector3<f32>, f32), SimulationError> {
        if self.current_planner.is_finished(current_position, time)? {
            log::info!("Time: {:.2} s,\tSwitch Hover", time);
            self.current_planner = PlannerType::Hover(HoverPlanner {
                target_position: current_position,
                target_yaw: current_orientation.euler_angles().2,
            });
        }
        // Update obstacles for ObstacleAvoidancePlanner if needed
        if let PlannerType::ObstacleAvoidance(ref mut planner) = self.current_planner {
            planner.obstacles = obstacles.to_owned();
        }
        Ok(self
            .current_planner
            .plan(current_position, current_velocity, time))
    }
}
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
impl Planner for ObstacleAvoidancePlanner {
    fn plan(
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
/// Waypoint planner that generates a minimum snap trajectory between waypoints
/// # Example
/// ```
/// use peng_quad::MinimumSnapWaypointPlanner;
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
    /// use peng_quad::MinimumSnapWaypointPlanner;
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
    /// use peng_quad::MinimumSnapWaypointPlanner;
    /// use nalgebra::Vector3;
    /// let waypoints = vec![Vector3::zeros(), Vector3::new(1.0, 0.0, 0.0)];
    /// let yaws = vec![0.0, 0.0];
    /// let segment_times = vec![1.0];
    /// let start_time = 0.0;
    /// let mut planner = MinimumSnapWaypointPlanner::new(waypoints, yaws, segment_times, start_time).unwrap();
    /// planner.compute_minimum_snap_trajectories();
    /// ```
    pub fn compute_minimum_snap_trajectories(&mut self) -> Result<(), SimulationError> {
        let n = self.waypoints.len() - 1;
        for i in 0..n {
            let duration = self.times[i];
            let (start, end) = (self.waypoints[i], self.waypoints[i + 1]);
            let mut a = SMatrix::<f32, 8, 8>::zeros();
            let mut b = SMatrix::<f32, 8, 3>::zeros();
            a.fixed_view_mut::<4, 4>(0, 0).fill_with_identity();
            b.fixed_view_mut::<1, 3>(0, 0).copy_from(&start.transpose());
            b.fixed_view_mut::<1, 3>(4, 0).copy_from(&end.transpose());
            // End point constraints
            for j in 0..8 {
                a[(4, j)] = duration.powi(j as i32);
                if j > 0 {
                    a[(5, j)] = j as f32 * duration.powi(j as i32 - 1);
                }
                if j > 1 {
                    a[(6, j)] = j as f32 * (j - 1) as f32 * duration.powi(j as i32 - 2);
                }
                if j > 2 {
                    a[(7, j)] =
                        j as f32 * (j - 1) as f32 * (j - 2) as f32 * duration.powi(j as i32 - 3);
                }
            }
            let coeffs = a.lu().solve(&b).ok_or(SimulationError::NalgebraError(
                "Failed to solve for coefficients in MinimumSnapWaypointPlanner".to_string(),
            ))?;
            self.coefficients.push(
                (0..8)
                    .map(|j| Vector3::new(coeffs[(j, 0)], coeffs[(j, 1)], coeffs[(j, 2)]))
                    .collect(),
            );
        }
        Ok(())
    }
    /// Compute the coefficients for yaw trajectories
    /// The yaw trajectory is a cubic polynomial and interpolated between waypoints
    /// # Errors
    /// * Returns an error if nalgebra fails to solve for the coefficients
    /// # Example
    /// ```
    /// use peng_quad::MinimumSnapWaypointPlanner;
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
    /// use peng_quad::MinimumSnapWaypointPlanner;
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
    ) -> (Vector3<f32>, Vector3<f32>, f32, f32) {
        let mut position = Vector3::zeros();
        let mut velocity = Vector3::zeros();
        let mut yaw = 0.0;
        let mut yaw_rate = 0.0;
        for (i, coeff) in coeffs.iter().enumerate() {
            let ti = t.powi(i as i32);
            position += coeff * ti;
            if i > 0 {
                velocity += coeff * (i as f32) * t.powi(i as i32 - 1);
            }
        }
        for (i, &coeff) in yaw_coeffs.iter().enumerate() {
            let ti = t.powi(i as i32);
            yaw += coeff * ti;
            if i > 0 {
                yaw_rate += coeff * (i as f32) * t.powi(i as i32 - 1);
            }
        }
        (position, velocity, yaw, yaw_rate)
    }
}
/// Implement the `Planner` trait for `MinimumSnapWaypointPlanner`
impl Planner for MinimumSnapWaypointPlanner {
    fn plan(
        &self,
        _current_position: Vector3<f32>,
        _current_velocity: Vector3<f32>,
        time: f32,
    ) -> (Vector3<f32>, Vector3<f32>, f32) {
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
        let (position, velocity, yaw, _yaw_rate) = self.evaluate_polynomial(
            segment_time,
            &self.coefficients[current_segment],
            &self.yaw_coefficients[current_segment],
        );
        (position, velocity, yaw)
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
#[derive(Debug)]
/// Represents a step in the planner schedule.
/// # Example
/// ```
/// use peng_quad::PlannerStepConfig;
/// let step = PlannerStepConfig {
///     step: 0,
///     planner_type: "MinimumJerkLocalPlanner".to_string(),
///     params: serde_yaml::Value::Null,
/// };
/// ```
pub struct PlannerStepConfig {
    /// The simulation step at which this planner should be activated (in ms unit).
    pub step: usize,
    /// The type of planner to use for this step.
    pub planner_type: String,
    /// Additional parameters for the planner, stored as a YAML value.
    pub params: serde_yaml::Value,
}
/// Updates the planner based on the current simulation step and configuration
/// # Arguments
/// * `planner_manager` - The PlannerManager instance to update
/// * `step` - The current simulation step in ms unit
/// * `time` - The current simulation time
/// * `simulation_frequency' - The simulation frequency in Hz
/// * `quad` - The Quadrotor instance
/// * `obstacles` - The current obstacles in the simulation
/// * `planner_config` - The planner configuration
/// # Errors
/// * If the planner could not be created
/// # Example
/// ```
/// use peng_quad::config;
/// use peng_quad::environment::Obstacle;
/// use peng_quad::{PlannerManager, Quadrotor, PlannerStepConfig, update_planner, quadrotor::QuadrotorInterface};
/// use nalgebra::Vector3;
/// let simulation_frequency = 1000;
/// let initial_position = Vector3::new(0.0, 0.0, 0.0);
/// let initial_yaw = 0.0;
/// let mut planner_manager = PlannerManager::new(initial_position, initial_yaw);
/// let step = 0;
/// let time = 0.0;
/// let (time_step, mass, gravity, drag_coefficient) = (0.01, 1.3, 9.81, 0.01);
/// let inertia_matrix = [0.0347563, 0.0, 0.0, 0.0, 0.0458929, 0.0, 0.0, 0.0, 0.0977];
/// let mut quadrotor = Quadrotor::new(time_step, config::SimulationConfig::default(), config::QuadrotorConfig::default(), config::ImuConfig::default()).unwrap();
/// let quad_state = quadrotor.observe(0.0).unwrap();
/// let obstacles = vec![Obstacle::new(Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 0.0), 1.0)];
/// let planner_config = vec![PlannerStepConfig {
///     step: 0,
///     planner_type: "MinimumJerkLine".to_string(),
///     params:
///        serde_yaml::from_str(r#"
///        end_position: [0.0, 0.0, 1.0]
///        end_yaw: 0.0
///        duration: 2.0
///        "#).unwrap(),
/// }];
/// update_planner(&mut planner_manager, step, time, simulation_frequency, &quad_state, &obstacles, &planner_config).unwrap();
/// ```
pub fn update_planner(
    planner_manager: &mut PlannerManager,
    step: usize,
    time: f32,
    _simulation_frequency: usize,
    quad_state: &QuadrotorState,
    obstacles: &[Obstacle],
    planner_config: &[PlannerStepConfig],
) -> Result<(), SimulationError> {
    if let Some(planner_step) = planner_config.iter().find(|s| s.step == step) {
        log::info!("Time: {:.2} s,\tSwitch {}", time, planner_step.planner_type);
        planner_manager.set_planner(create_planner(planner_step, quad_state, time, obstacles)?);
    }
    Ok(())
}
/// Creates a planner based on the configuration
/// # Arguments
/// * `step` - The configuration for the planner step in ms unit
/// * `quad` - The Quadrotor instance
/// * `time` - The current simulation time
/// * `obstacles` - The current obstacles in the simulation
/// # Returns
/// * `PlannerType` - The created planner
/// # Errors
/// * If the planner type is not recognized
/// # Example
/// ```
/// use peng_quad::config;
/// use peng_quad::environment::Obstacle;
/// use peng_quad::{PlannerType, Quadrotor, PlannerStepConfig, create_planner, quadrotor::QuadrotorInterface};
/// use nalgebra::Vector3;
/// let step = PlannerStepConfig {
///    step: 0,
///   planner_type: "MinimumJerkLine".to_string(),
///   params:
///       serde_yaml::from_str(r#"
///       end_position: [0.0, 0.0, 1.0]
///       end_yaw: 0.0
///       duration: 2.0
///       "#).unwrap(),
/// };
/// let time = 0.0;
/// let (time_step, mass, gravity, drag_coefficient) = (0.01, 1.3, 9.81, 0.01);
/// let inertia_matrix = [0.0347563, 0.0, 0.0, 0.0, 0.0458929, 0.0, 0.0, 0.0, 0.0977];
/// let mut quadrotor = Quadrotor::new(time_step, config::SimulationConfig::default(), config::QuadrotorConfig::default(), config::ImuConfig::default()).unwrap();
/// let quadrotor_state = quadrotor.observe(0.0).unwrap();
/// let obstacles = vec![Obstacle::new(Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 0.0), 1.0)];
/// let planner = create_planner(&step, &quadrotor_state, time, &obstacles).unwrap();
/// match planner {
///    PlannerType::MinimumJerkLine(_) => log::info!("Created MinimumJerkLine planner"),
///   _ => log::info!("Created another planner"),
/// }
/// ```
pub fn create_planner(
    step: &PlannerStepConfig,
    quad_state: &QuadrotorState,
    time: f32,
    obstacles: &[Obstacle],
) -> Result<PlannerType, SimulationError> {
    let params = &step.params;
    match step.planner_type.as_str() {
        "MinimumJerkLine" => Ok(PlannerType::MinimumJerkLine(MinimumJerkLinePlanner {
            start_position: quad_state.position,
            end_position: parse_vector3(params, "end_position")?,
            start_yaw: quad_state.orientation.euler_angles().2,
            end_yaw: parse_f32(params, "end_yaw")?,
            start_time: time,
            duration: parse_f32(params, "duration")?,
        })),
        "Hover" => Ok(PlannerType::Hover(HoverPlanner {
            target_position: parse_vector3(params, "end_position")?,
            target_yaw: parse_f32(params, "target_yaw")?,
        })),
        "Lissajous" => Ok(PlannerType::Lissajous(LissajousPlanner {
            start_position: quad_state.position,
            center: parse_vector3(params, "center")?,
            amplitude: parse_vector3(params, "amplitude")?,
            frequency: parse_vector3(params, "frequency")?,
            phase: parse_vector3(params, "phase")?,
            start_time: time,
            duration: parse_f32(params, "duration")?,
            start_yaw: quad_state.orientation.euler_angles().2,
            end_yaw: parse_f32(params, "end_yaw")?,
            ramp_time: parse_f32(params, "ramp_time")?,
        })),
        "Circle" => Ok(PlannerType::Circle(CirclePlanner {
            center: parse_vector3(params, "center")?,
            radius: parse_f32(params, "radius")?,
            angular_velocity: parse_f32(params, "angular_velocity")?,
            start_position: quad_state.position,
            start_time: time,
            duration: parse_f32(params, "duration")?,
            start_yaw: quad_state.orientation.euler_angles().2,
            end_yaw: quad_state.orientation.euler_angles().2,
            ramp_time: parse_f32(params, "ramp_time")?,
        })),
        "ObstacleAvoidance" => Ok(PlannerType::ObstacleAvoidance(ObstacleAvoidancePlanner {
            target_position: parse_vector3(params, "target_position")?,
            start_time: time,
            duration: parse_f32(params, "duration")?,
            start_yaw: quad_state.orientation.euler_angles().2,
            end_yaw: parse_f32(params, "end_yaw")?,
            obstacles: obstacles.to_owned(),
            k_att: parse_f32(params, "k_att")?,
            k_rep: parse_f32(params, "k_rep")?,
            k_vortex: parse_f32(params, "k_vortex")?,
            d0: parse_f32(params, "d0")?,
            d_target: parse_f32(params, "d_target")?,
            max_speed: parse_f32(params, "max_speed")?,
        })),
        "MinimumSnapWaypoint" => {
            let mut waypoints = vec![quad_state.position];
            waypoints.extend(
                params["waypoints"]
                    .as_sequence()
                    .ok_or_else(|| SimulationError::OtherError("Invalid waypoints".to_string()))?
                    .iter()
                    .map(|w| {
                        w.as_sequence()
                            .and_then(|coords| {
                                Some(Vector3::new(
                                    coords[0].as_f64()? as f32,
                                    coords[1].as_f64()? as f32,
                                    coords[2].as_f64()? as f32,
                                ))
                            })
                            .ok_or(SimulationError::OtherError("Invalid waypoint".to_string()))
                    })
                    .collect::<Result<Vec<Vector3<f32>>, SimulationError>>()?,
            );
            let mut yaws = vec![quad_state.orientation.euler_angles().2];
            yaws.extend(
                params["yaws"]
                    .as_sequence()
                    .ok_or(SimulationError::OtherError("Invalid yaws".to_string()))?
                    .iter()
                    .map(|y| {
                        y.as_f64()
                            .map(|v| v as f32)
                            .ok_or(SimulationError::OtherError("Invalid yaw".to_string()))
                    })
                    .collect::<Result<Vec<f32>, SimulationError>>()?,
            );
            let segment_times = params["segment_times"]
                .as_sequence()
                .ok_or_else(|| SimulationError::OtherError("Invalid segment_times".to_string()))?
                .iter()
                .map(|t| {
                    t.as_f64().map(|v| v as f32).ok_or_else(|| {
                        SimulationError::OtherError("Invalid segment time".to_string())
                    })
                })
                .collect::<Result<Vec<f32>, SimulationError>>()?;
            MinimumSnapWaypointPlanner::new(waypoints, yaws, segment_times, time)
                .map(PlannerType::MinimumSnapWaypoint)
        }
        "Landing" => Ok(PlannerType::Landing(LandingPlanner {
            start_position: quad_state.position,
            start_time: time,
            duration: parse_f32(params, "duration")?,
            start_yaw: quad_state.orientation.euler_angles().2,
        })),
        _ => Err(SimulationError::OtherError(format!(
            "Unknown planner type: {}",
            step.planner_type
        ))),
    }
}
