use crate::SimulationError;
use async_trait::async_trait;
use nalgebra::Vector3;
use std::collections::VecDeque;
use tonic::transport::Channel;
use tonic::transport::Endpoint;

#[cfg(feature = "hyrl")]
use crate::hyrl::{
    obstacle_avoidance_service_client::ObstacleAvoidanceServiceClient, DroneState, ModelType,
    TrajectoryRequest, TrajectoryResponse,
};
use crate::planners::{MinimumSnapWaypointPlanner, Planner};

/// The abstraction that both your real gRPC‐backed client and your mock client will implement.
#[async_trait]
pub trait HyRLObstacleAvoidanceService: Send + Sync {
    async fn request_trajectory(
        &mut self,
        start: Vector3<f32>,
        target: Vector3<f32>,
        duration_s: f32,
        num_waypoints: u32,
    ) -> Result<Vec<DroneState>, SimulationError>;
}

#[cfg(feature = "hyrl")]
pub struct HyRLClient {
    inner: ObstacleAvoidanceServiceClient<Channel>,
}

impl HyRLClient {
    pub fn new(url: String) -> Result<Self, SimulationError> {
        // build the real client from the channel:
        let channel = Endpoint::from_shared(url)
            .map_err(|_| SimulationError::OtherError(("Failed to get url from shared").into()))?
            .connect_lazy();
        let inner = ObstacleAvoidanceServiceClient::new(channel);
        Ok(Self { inner })
    }
}

#[cfg(feature = "hyrl")]
#[async_trait]
impl HyRLObstacleAvoidanceService for HyRLClient {
    async fn request_trajectory(
        &mut self,
        start_position: Vector3<f32>,
        target_position: Vector3<f32>,
        duration_s: f32,
        num_waypoints: u32,
    ) -> Result<Vec<DroneState>, SimulationError> {
        println!("Requesting trajectory from HyRL server...");
        let request = TrajectoryRequest {
            state: Some(DroneState {
                x: start_position.x,
                y: start_position.y,
                z: start_position.z,
            }),
            target_state: Some(DroneState {
                x: target_position.x,
                y: target_position.y,
                z: target_position.z,
            }),
            num_waypoints: num_waypoints.try_into().unwrap(),
            duration_s: duration_s as u32,
            sampling_time: 0.005,
            model_type: ModelType::Hybrid.into(),
        };

        // perform the unary RPC
        let response = self
            .inner
            .get_trajectory(request)
            .await
            .map_err(|e| SimulationError::OtherError(format!("gRPC error: {}", e)))?;

        println!("Received response from HyRL server");
        println!("{:#?}", response);

        // pull out the inner TrajectoryResponse
        let TrajectoryResponse { trajectory } = response.into_inner();
        Ok(trajectory)
    }
}

#[cfg(not(feature = "hyrl"))]
pub enum HyRLClient {
    Disabled,
}

/// Planner for HyRL obstacle avoidance for a single fixed obstacle
pub struct HyRLPlanner {
    /// Starting position of the trajectory
    pub start_position: Vector3<f32>,
    /// Starting yaw angle
    pub start_yaw: f32,
    /// Ending position of the trajectory
    pub target_position: Vector3<f32>,
    /// Ending yaw angle
    pub end_yaw: f32,
    /// Start time of the trajectory
    pub start_time: f32,
    /// Duration of the trajectory
    pub duration: f32,
    /// gRPC client wrapper
    pub client: Box<dyn HyRLObstacleAvoidanceService>,
    /// Minimum snap planner
    pub minimum_snap_planner: MinimumSnapWaypointPlanner,
}

impl HyRLPlanner {
    // Create a HyRL Planner instance.
    // The HyRL Planner maintains an inner minimum snap planner.
    // On creation, it requests a trajectory from the HyRL server and initializes the minimum snap
    // planner with the waypoints and yaw angles.
    pub async fn new(
        start_position: Vector3<f32>,
        start_yaw: f32,
        target_position: Vector3<f32>,
        start_time: f32,
        duration: f32,
        num_waypoints: u32,
        mut client: Box<dyn HyRLObstacleAvoidanceService>,
    ) -> Result<Self, SimulationError> {
        // The drone at 0 in Peng space is the agent at 1.5 in the models coordinate system.
        // So for a nominal start x ~ -1.5, we offset by 1.5 to zero out x
        let hyrl_start = start_position + Vector3::new(1.5, 0.0, 0.0);
        let drone_states = client
            .request_trajectory(hyrl_start, target_position, duration, num_waypoints)
            .await?;
        println!("Drone States: {:?}", drone_states);
        if drone_states.len() < 2 {
            return Err(SimulationError::OtherError(
                "HyRL returned <2 waypoints".into(),
            ));
        }
        // Process the results in model space back into Peng space, and set the altitude to the
        // start position's altitude.
        let drone_waypoints: Vec<Vector3<f32>> = drone_states
            .iter()
            .map(|state| Vector3::new(state.x - 1.5, state.y, start_position[2]))
            .collect();
        println!("Drone Waypoints: {:?}", drone_waypoints);

        let mut yaws: VecDeque<f32> = drone_waypoints
            .windows(2)
            .map(|window| {
                let a = window[0];
                let b = window[1];
                let delta_x = b.x - a.x;
                let delta_y = b.y - a.y;
                delta_y.atan2(delta_x)
            })
            // .map(|window| 0.0)
            .collect();
        let target_yaw = 0.0;
        let blend_window = (num_waypoints as f32 * 0.5) as usize; // number of points to smooth toward 0.0
        let len = yaws.len();

        if len >= blend_window as usize {
            for i in 0..blend_window {
                let alpha = (i as f32 + 1.0) / (blend_window as f32);
                yaws[len - blend_window + i] =
                    (1.0 - alpha) * yaws[len - blend_window + i] + alpha * target_yaw;
            }
        } else if !yaws.is_empty() {
            // Short path, just set final yaw directly
            yaws[len - 1] = target_yaw;
        }

        yaws.push_front(*yaws.front().unwrap());
        println!("yaws: {:?}", yaws);

        let segment_duration = duration / (drone_waypoints.len() - 1) as f32;
        let durations = vec![segment_duration; drone_waypoints.len() - 1];

        Ok(Self {
            start_position,
            start_yaw,
            target_position,
            end_yaw: start_yaw,
            start_time,
            duration,
            client,
            minimum_snap_planner: MinimumSnapWaypointPlanner::new(
                drone_waypoints,
                yaws.into(),
                durations,
                start_time,
            )?,
        })
    }
}

/// Implementation of the planner trait for HyRL
#[async_trait]
impl Planner for HyRLPlanner {
    async fn plan(
        &self,
        current_position: Vector3<f32>,
        current_velocity: Vector3<f32>,
        time: f32,
    ) -> (Vector3<f32>, Vector3<f32>, f32) {
        self.minimum_snap_planner
            .plan(current_position, current_velocity, time)
            .await
    }

    /// Returns true once we have both (a) crossed the half‐plane
    /// at the final waypoint and (b) run at least the full duration.
    fn is_finished(
        &self,
        current_position: Vector3<f32>,
        _time: f32,
    ) -> Result<bool, SimulationError> {
        if self.minimum_snap_planner.waypoints.len() < 2 {
            return Err(SimulationError::OtherError(
                "Not enough waypoints to define half‐plane".into(),
            ));
        }

        // Last two waypoints
        let last_idx = self.minimum_snap_planner.waypoints.len() - 1;
        let p_prev = self.minimum_snap_planner.waypoints[last_idx - 1];
        let p_last = self.minimum_snap_planner.waypoints[last_idx];

        // Unit normal pointing along the path direction
        let n = (p_last - p_prev).normalize();

        // Check 1: Have we crossed the plane through p_last with normal n?
        let finished = (current_position - p_last).dot(&n) >= 0.0;
        Ok(finished)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hyrl::DroneState;
    use async_trait::async_trait;
    use nalgebra::Vector3;

    /// A little mock that just returns whatever DroneState vector we give it.
    struct MockHyRLClient {
        responses: Vec<DroneState>,
    }

    #[async_trait]
    impl HyRLObstacleAvoidanceService for MockHyRLClient {
        async fn request_trajectory(
            &mut self,
            _start: Vector3<f32>,
            _target: Vector3<f32>,
            _duration_s: f32,
            _num_waypoints: u32,
        ) -> Result<Vec<DroneState>, SimulationError> {
            Ok(self.responses.clone())
        }
    }

    #[tokio::test]
    async fn hyrl_planner_new_with_mock_returns_correct_waypoints() -> Result<(), SimulationError> {
        // Arrange: choose some fake waypoints
        let fake_states = vec![
            DroneState {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            DroneState {
                x: 0.5,
                y: 0.5,
                z: 0.5,
            },
            DroneState {
                x: 1.0,
                y: 1.0,
                z: 1.0,
            },
        ];
        let mock = MockHyRLClient {
            responses: fake_states.clone(),
        };

        let start_pos = Vector3::new(0.0, 0.0, 0.0);
        let target_pos = Vector3::new(1.0, 1.0, 1.0);
        let start_yaw = 0.0;
        let start_time = 0.0;
        let duration = 5.0;
        let num_waypoints = fake_states.len() as u32;

        // Act: build the HyRLPlanner with our mock client
        let planner = HyRLPlanner::new(
            start_pos,
            start_yaw,
            target_pos,
            start_time,
            duration,
            num_waypoints,
            Box::new(mock),
        )
        .await?;

        // Assert: the inner MinimumSnapWaypointPlanner should have exactly our fake waypoints
        let expected: Vec<Vector3<f32>> = fake_states
            .into_iter()
            .map(|s| Vector3::new(s.x, s.y, s.z) - Vector3::new(1.5, 0.0, 0.0))
            .map(|v| Vector3::new(v.x, v.y, start_pos.z))
            .collect();

        assert_eq!(planner.minimum_snap_planner.waypoints, expected);

        Ok(())
    }
}
