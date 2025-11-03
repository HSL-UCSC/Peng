use crate::Quadrotor;
use crate::SimulationError;
use crate::betaflight_quad::BetaflightQuad;
use crate::config;
use crate::config::QuadrotorConfigurations;
use crate::liftoff_quad::LiftoffQuad;
use nalgebra::{UnitQuaternion, Vector3};
use std::collections::VecDeque;
use std::ops::{Deref, DerefMut};

#[derive(Clone, Debug, Default)]
pub struct State {
    /// Current position of the quadrotor in NED
    pub position: Vector3<f32>,
    /// Current velocity of the quadrotor
    pub velocity: Vector3<f32>,
    /// Current Acceleration of the quadrotor
    pub acceleration: Vector3<f32>,
    /// Current orientation of the quadrotor
    pub orientation: UnitQuaternion<f32>,
    /// Current angular velocity of the quadrotor
    pub angular_velocity: Vector3<f32>,
}

// TODO: consider nested 12 DOFs, one for true, one for measureda
#[derive(Clone, Debug, Default)]
/// Represents the state of a quadrotor
pub struct QuadrotorState {
    /// Time stampe of the state
    pub time: f32,
    /// True State
    pub state: State,
    /// Measured State
    pub measured_state: State,
}

// Implement Deref so QuadrotorState behaves like State
impl Deref for QuadrotorState {
    type Target = State;

    fn deref(&self) -> &Self::Target {
        &self.state
    }
}

// Implement Deref so QuadrotorState behaves like State
impl DerefMut for QuadrotorState {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.state
    }
}

/// Types implementing this trait can be used as a quadrotor in the simulation.
/// The underlying model can be internal or external to the simulator.
pub trait QuadrotorInterface: Send {
    fn control(
        &mut self,
        step_number: usize,
        thrust: f32,
        torque: &Vector3<f32>,
    ) -> Result<Option<(f32, Vector3<f32>)>, SimulationError>;

    fn observe(&mut self, t: f32) -> Result<QuadrotorState, SimulationError>;

    fn parameters(&self) -> crate::config::QuadrotorConfig;

    fn max_thrust(&self) -> f32;

    fn max_torque(&self) -> Vector3<f32>;

    // TODO: integrate this into the observe method, add a
    fn read_imu(&self) -> Result<(Vector3<f32>, Vector3<f32>), SimulationError>;
}

pub fn build_quadrotor(
    config: &config::Config,
    quadrotor_config: &config::QuadrotorConfigurations,
) -> Result<(Box<dyn QuadrotorInterface>, f32), SimulationError> {
    let (quad, mass): (Box<dyn QuadrotorInterface>, f32) = match &quadrotor_config {
        QuadrotorConfigurations::Peng(ref quad_config) => (
            Box::new(Quadrotor::new(
                1.0 / config.simulation.simulation_frequency as f32,
                config.simulation.clone(),
                quad_config.clone(),
                config.imu.clone(),
            )?),
            quad_config.mass,
        ),
        QuadrotorConfigurations::Liftoff(ref liftoff_quad_config) => (
            Box::new(LiftoffQuad::new(
                config.simulation.clone(),
                liftoff_quad_config.clone(),
            )?),
            liftoff_quad_config.quadrotor_config.mass,
        ),
        QuadrotorConfigurations::Betaflight(ref betaflight_config) => (
            Box::new(BetaflightQuad::new(
                config.simulation.clone(),
                betaflight_config.clone(),
            )?),
            betaflight_config.quadrotor_config.mass,
        ),
    };
    Ok((quad, mass))
}

const WINDOW_SIZE: usize = 8;

pub struct OrientationFilter {
    buffer: VecDeque<UnitQuaternion<f32>>,
}

impl OrientationFilter {
    pub fn new() -> Self {
        Self {
            buffer: VecDeque::with_capacity(WINDOW_SIZE),
        }
    }

    pub fn add_sample(&mut self, q: UnitQuaternion<f32>) {
        if self.buffer.len() == WINDOW_SIZE {
            self.buffer.pop_front();
        }
        self.buffer.push_back(q);
    }

    pub fn get_filtered(&self) -> UnitQuaternion<f32> {
        self.quaternion_geodesic_mean(self.buffer.as_slices().0)
    }

    fn quaternion_geodesic_mean(&self, quaternions: &[UnitQuaternion<f32>]) -> UnitQuaternion<f32> {
        assert!(!quaternions.is_empty());

        let mut mean = quaternions[0];

        for _ in 0..10 {
            // iterative refinement
            // Step 1: compute log difference to mean
            let mut sum = Vector3::zeros();
            for q in quaternions {
                let delta = mean.inverse() * *q;

                sum += delta.scaled_axis();
            }

            // Step 2: compute average tangent vector
            sum /= quaternions.len() as f32;

            // Step 3: apply average back to mean
            let delta_mean = nalgebra::UnitQuaternion::from_scaled_axis(sum);
            mean = mean * delta_mean;

            // Early exit if update is very small
            if sum.norm() < 1e-6 {
                break;
            }
        }

        mean
    }
}
