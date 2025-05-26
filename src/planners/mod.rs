use crate::environment::Obstacle;
use crate::quadrotor::QuadrotorState;
use crate::{parse_f32, parse_string, parse_vector3};
use crate::{parse_uint, SimulationError};
use async_trait::async_trait;
use nalgebra::Vector3;

mod circle;
mod hover;
#[cfg(feature = "hyrl")]
mod hyrl;
mod landing;
mod lissajous;
mod minimum_jerk_line;
mod minimum_snap_waypoint;
mod obstacle_avoidance;

// Re-export the types you need at the top-level
pub use circle::CirclePlanner;
pub use hover::HoverPlanner;
#[cfg(feature = "hyrl")]
pub use hyrl::HyRLPlanner;
pub use landing::LandingPlanner;
pub use lissajous::LissajousPlanner;
pub use minimum_jerk_line::MinimumJerkLinePlanner;
pub use minimum_snap_waypoint::MinimumSnapWaypointPlanner;
pub use obstacle_avoidance::ObstacleAvoidancePlanner;

/// A struct to hold trajectory data
/// # Example
/// ```
/// use peng_quad::planners::Trajectory;
/// let initial_point = nalgebra::Vector3::new(0.0, 0.0, 0.0);
/// let mut trajectory = Trajectory::new(initial_point);
/// ```
#[derive(Clone, Debug)]
pub struct Trajectory {
    /// A vector of 3D points
    pub points: Vec<Vector3<f32>>,
    /// The last point that was logged
    pub last_logged_point: Vector3<f32>,
    /// The minimum distance between points to log
    pub min_distance_threadhold: f32,
}
/// Implement the Trajectory struct
impl Trajectory {
    /// Create a new Trajectory instance
    /// # Arguments
    /// * `initial_point` - The initial point to add to the trajectory
    /// # Returns
    /// * A new Trajectory instance
    /// # Example
    /// ```
    /// use peng_quad::planners::Trajectory;
    /// let initial_point = nalgebra::Vector3::new(0.0, 0.0, 0.0);
    /// let mut trajectory = Trajectory::new(initial_point);
    /// ```
    pub fn new(initial_point: Vector3<f32>) -> Self {
        Self {
            points: vec![initial_point],
            last_logged_point: initial_point,
            min_distance_threadhold: 0.05,
        }
    }
    /// Add a point to the trajectory if it is further than the minimum distance threshold
    /// # Arguments
    /// * `point` - The point to add
    /// # Returns
    /// * `true` if the point was added, `false` otherwise
    /// # Example
    /// ```
    /// use peng_quad::planners::Trajectory;
    /// let mut trajectory = Trajectory::new(nalgebra::Vector3::new(0.0, 0.0, 0.0));
    /// let point = nalgebra::Vector3::new(1.0, 0.0, 0.0);
    /// assert_eq!(trajectory.add_point(point), true);
    /// assert_eq!(trajectory.add_point(point), false);
    /// ```
    pub fn add_point(&mut self, point: Vector3<f32>) -> bool {
        if (point - self.last_logged_point).norm() > self.min_distance_threadhold {
            self.points.push(point);
            self.last_logged_point = point;
            true
        } else {
            false
        }
    }
}

#[derive(Debug, Default)]
/// Represents a step in the planner schedule.
/// # Example
/// ```
/// use peng_quad::planners::PlannerStepConfig;
/// let step = PlannerStepConfig {
///     step: 0,
///     time: None,
///     planner_type: "MinimumJerkLocalPlanner".to_string(),
///     params: serde_yaml::Value::Null,
/// };
/// ```
pub struct PlannerStepConfig {
    /// The simulation step at which this planner should be activated (in ms unit).
    pub step: Option<usize>,
    /// The simulation time at which this planner should be activated (in ms unit).
    pub time: Option<f32>,
    /// The type of planner to use for this step.
    pub planner_type: String,
    /// Additional parameters for the planner, stored as a YAML value.
    pub params: serde_yaml::Value,
}

/// Trait defining the interface for trajectory planners
/// # Example
/// ```
/// use async_trait::async_trait;
/// use nalgebra::Vector3;
/// use peng_quad::planners::Planner;
/// use peng_quad::SimulationError;
/// struct TestPlanner;
/// #[async_trait]
/// impl Planner for TestPlanner {
///    async fn plan(
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
#[async_trait]
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
    /// use async_trait::async_trait;
    /// use futures::executor::block_on;
    /// use nalgebra::Vector3;
    /// use peng_quad::SimulationError;
    /// use peng_quad::planners::Planner;
    ///
    /// struct TestPlanner;
    ///
    /// #[async_trait]
    /// impl Planner for TestPlanner {
    ///     async fn plan(
    ///         &self,
    ///         _current_position: Vector3<f32>,
    ///         _current_velocity: Vector3<f32>,
    ///         _time: f32,
    ///     ) -> (Vector3<f32>, Vector3<f32>, f32) {
    ///         (Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 0.0), 0.0)
    ///     }
    ///
    ///     fn is_finished(
    ///         &self,
    ///         _current_position: Vector3<f32>,
    ///         _time: f32,
    ///     ) -> Result<bool, SimulationError> {
    ///         Ok(true)
    ///     }
    /// }
    ///
    /// let planner = TestPlanner;
    /// let (pos, vel, yaw) = block_on(planner.plan(
    ///     Vector3::new(1.0, 2.0, 3.0),
    ///     Vector3::new(0.1, 0.2, 0.3),
    ///     0.5,
    /// ));
    /// assert_eq!(pos, Vector3::new(0.0, 0.0, 0.0));
    /// ```
    async fn plan(
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
    /// use async_trait::async_trait;
    /// use peng_quad::SimulationError;
    /// use peng_quad::planners::Planner;
    /// struct TestPlanner;
    /// #[async_trait]
    /// impl Planner for TestPlanner {
    ///     async fn plan(
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

/// Manages different trajectory planners and switches between them
/// # Example
/// ```
/// use nalgebra::Vector3;
/// use peng_quad::planners::PlannerManager;
/// let initial_position = Vector3::new(0.0, 0.0, 1.0);
/// let initial_yaw = 0.0;
/// let planner_manager = PlannerManager::new(initial_position, initial_yaw);
/// ```
pub struct PlannerManager {
    /// The current planner
    pub current_planner: PlannerType,
    pub current_planner_idx: i32,
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
    /// use peng_quad::planners::PlannerManager;
    /// let initial_position = Vector3::new(0.0, 0.0, 1.0);
    /// let initial_yaw = 0.0;
    /// let planner_manager = PlannerManager::new(initial_position, initial_yaw);
    /// ```
    pub fn new(initial_position: Vector3<f32>, initial_yaw: f32) -> Self {
        let hover_planner = HoverPlanner {
            target_position: initial_position,
            target_yaw: initial_yaw,
        };

        #[cfg(not(feature = "hyrl"))]
        let channel = None;
        Self {
            current_planner: PlannerType::Hover(hover_planner),
            current_planner_idx: -1,
        }
    }

    /// Sets a new planner
    /// # Arguments
    /// * `new_planner` - The new planner to be set
    /// # Example
    /// ```
    /// use nalgebra::Vector3;
    /// use peng_quad::planners::{PlannerManager, CirclePlanner, PlannerType};
    ///
    /// let initial_position = Vector3::new(0.0, 0.0, 1.0);
    /// let initial_yaw = 0.0;
    /// let mut planner_manager = PlannerManager::new(initial_position, initial_yaw);
    ///
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
    ///
    /// planner_manager.set_planner(PlannerType::Circle(new_planner));
    /// assert!(matches!(planner_manager.current_planner, PlannerType::Circle(_)));
    /// ```
    pub fn set_planner(&mut self, new_planner: PlannerType) {
        self.current_planner = new_planner;
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
    /// use peng_quad::planners::{PlannerType, PlannerStepConfig};
    /// use peng_quad::{Quadrotor, quadrotor::QuadrotorInterface};
    /// use peng_quad::planners::PlannerManager;
    /// use nalgebra::Vector3;
    ///
    /// let step = PlannerStepConfig {
    ///     step: 0,
    ///     time: None,
    ///     planner_type: "MinimumJerkLine".to_string(),
    ///     params:
    ///     serde_yaml::from_str(r#"
    ///       end_position: [0.0, 0.0, 1.0]
    ///       end_yaw: 0.0
    ///       duration: 2.0
    ///       "#).unwrap(),
    /// };
    /// use futures::executor::block_on;
    /// let time = 0.0;
    /// let (time_step, mass, gravity, drag_coefficient) = (0.01, 1.3, 9.81, 0.01);
    /// let inertia_matrix = [0.0347563, 0.0, 0.0, 0.0, 0.0458929, 0.0, 0.0, 0.0, 0.0977];
    /// let mut quadrotor = Quadrotor::new(time_step, config::SimulationConfig::default(), config::QuadrotorConfig::default(), config::ImuConfig::default()).unwrap();
    /// let quadrotor_state = quadrotor.observe(0.0).unwrap();
    /// let obstacles = vec![Obstacle::new(Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 0.0), 1.0)];
    /// let initial_position = Vector3::new(0.0, 0.0, 1.0);
    /// let initial_yaw = 0.0;
    /// let planner_manager = PlannerManager::new(initial_position, initial_yaw);
    /// let planner = block_on( async { planner_manager.create_planner(&step, &quadrotor_state, time, &obstacles).await.unwrap() } );
    /// match planner {
    ///    PlannerType::MinimumJerkLine(_) => log::info!("Created MinimumJerkLine planner"),
    ///   _ => log::info!("Created another planner"),
    /// }
    /// ```
    pub async fn create_planner(
        &self,
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
                        .ok_or_else(|| {
                            SimulationError::OtherError("Invalid waypoints".to_string())
                        })?
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
                    .ok_or_else(|| {
                        SimulationError::OtherError("Invalid segment_times".to_string())
                    })?
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
            #[cfg(feature = "hyrl")]
            "HyRL" => {
                let url = parse_string(params, "url")?;
                let client = hyrl::HyRLClient::new(url)?;
                println!("Creating HyRL Planner---------");
                HyRLPlanner::new(
                    quad_state.position,
                    parse_f32(params, "start_yaw")?,
                    parse_vector3(params, "target_position")?,
                    time,
                    parse_f32(params, "duration")?,
                    parse_uint(params, "num_waypoints")?,
                    Box::new(client),
                )
                .await
                .map(PlannerType::HyRL)
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

    /// Advance or switch planners if needed, then produce the next
    /// desired setpoint from the active planner.
    ///
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
    /// use futures::executor::block_on;
    /// use nalgebra::{Vector3, UnitQuaternion};
    /// use peng_quad::SimulationError;
    /// use peng_quad::quadrotor::QuadrotorState;
    /// use peng_quad::planners::{PlannerManager, PlannerStepConfig};
    /// let initial_position = Vector3::new(0.0, 0.0, 1.0);
    /// let initial_yaw = 0.0;
    /// let mut planner_manager = PlannerManager::new(initial_position, initial_yaw);
    /// let current_position = Vector3::new(0.0, 0.0, 1.0);
    /// let current_orientation = UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0);
    /// let current_velocity = Vector3::new(0.0, 0.0, 0.0);
    /// let obstacles = vec![];
    /// let step = 0;
    /// let time = 0.0;
    /// let result = block_on(async { planner_manager.update(step, time, &QuadrotorState::default(), &obstacles, &vec![PlannerStepConfig::default()]).await });
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
    ///
    pub async fn update(
        &mut self,
        step: usize,
        time: f32,
        quad_state: &QuadrotorState,
        obstacles: &[Obstacle],
        planner_config: &[PlannerStepConfig],
    ) -> Result<(Vector3<f32>, Vector3<f32>, f32), SimulationError> {
        // println!("{:?}", planner_config);
        // println!("Time: {:.2} s,\tStep {}", time, step);
        if let Some((idx, ps)) = planner_config
            .iter()
            .enumerate()
            .rev()
            .find(|(_, ps)| ps.time.map(|t| time >= t).unwrap_or(false))
        {
            if self.current_planner_idx != idx as i32 {
                log::info!("Time: {:.2} s,\tSwitch {}", time, ps.planner_type);
                println!("Time: {:.2} s,\tSwitch {}", time, ps.planner_type);
                let new_planner = self.create_planner(ps, quad_state, time, obstacles).await?;
                self.current_planner = new_planner;
                self.current_planner_idx = idx as i32;
            }
        } else if let Some((idx, ps)) = planner_config
            .iter()
            .enumerate()
            .rev()
            .find(|(_, ps)| ps.step.map(|s| s == step).unwrap_or(false))
        {
            if self.current_planner_idx != idx as i32 {
                log::info!("Step {}:\tSwitch {}", step, ps.planner_type);
                println!("Step: {:.2} s,\tSwitch {}", time, ps.planner_type);
                let new_planner = self.create_planner(ps, quad_state, time, obstacles).await?;
                self.current_planner = new_planner;
                self.current_planner_idx = idx as i32;
            }
        }

        // 3) If the active planner needs obstacle updates (e.g. obstacle avoidance)
        if let PlannerType::ObstacleAvoidance(ref mut p) = self.current_planner {
            p.obstacles = obstacles.to_vec();
        }

        // 4) Check if the planner is finished, swap to Hover if so
        if self
            .current_planner
            .is_finished(quad_state.position, time)?
        {
            log::info!("Time: {:.2} s,\tSwitch Hover", time);
            self.current_planner = PlannerType::Hover(HoverPlanner {
                target_position: quad_state.position,
                target_yaw: quad_state.orientation.euler_angles().2,
            });
        }

        // 5) Finally call plan() on the active planner
        let (pos, vel, yaw) = self
            .current_planner
            .plan(quad_state.position, quad_state.velocity, time)
            .await;
        Ok((pos, vel, yaw))
    }
}

/// Enum representing different types of trajectory planners
/// # Example
/// ```
/// use peng_quad::planners::PlannerType;
/// use peng_quad::planners::HoverPlanner;
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
    /// Minimum snap waypoint planner
    #[cfg(feature = "hyrl")]
    HyRL(HyRLPlanner),
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
    /// use futures::executor::block_on;
    /// use nalgebra::Vector3;
    /// use peng_quad::planners::PlannerType;
    /// use peng_quad::planners::HoverPlanner;
    /// let hover_planner = HoverPlanner {
    ///     target_position: Vector3::new(0.0, 0.0, 1.0),
    ///     target_yaw: 0.0
    /// };
    /// let hover_planner_type = PlannerType::Hover(hover_planner);
    /// let (pos, _vel, _yaw) = block_on(async {
    ///     hover_planner_type
    ///         .plan(
    ///             Vector3::new(0.0, 0.0, 0.0),
    ///             Vector3::new(0.0, 0.0, 0.0),
    ///             0.0,
    ///         )
    ///         .await
    /// });
    ///
    /// ```
    pub async fn plan(
        &self,
        current_position: Vector3<f32>,
        current_velocity: Vector3<f32>,
        time: f32,
    ) -> (Vector3<f32>, Vector3<f32>, f32) {
        match self {
            PlannerType::Hover(p) => p.plan(current_position, current_velocity, time).await,
            PlannerType::MinimumJerkLine(p) => {
                p.plan(current_position, current_velocity, time).await
            }
            PlannerType::Lissajous(p) => p.plan(current_position, current_velocity, time).await,
            PlannerType::Circle(p) => p.plan(current_position, current_velocity, time).await,
            PlannerType::Landing(p) => p.plan(current_position, current_velocity, time).await,
            PlannerType::ObstacleAvoidance(p) => {
                p.plan(current_position, current_velocity, time).await
            }
            PlannerType::MinimumSnapWaypoint(p) => {
                p.plan(current_position, current_velocity, time).await
            }
            #[cfg(feature = "hyrl")]
            PlannerType::HyRL(p) => p.plan(current_position, current_velocity, time).await,
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
    /// use peng_quad::planners::PlannerType;
    /// use peng_quad::planners::HoverPlanner;
    /// use peng_quad::planners::Planner;
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
            #[cfg(feature = "hyrl")]
            PlannerType::HyRL(p) => p.is_finished(current_position, time),
        }
    }
}
