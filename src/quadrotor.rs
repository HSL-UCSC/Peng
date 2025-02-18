use crate::betaflight_quad::BetaflightQuad;
use crate::config;
use crate::config::QuadrotorConfigurations;
use crate::liftoff_quad::LiftoffQuad;
use crate::Quadrotor;
use crate::SimulationError;
use nalgebra::{UnitQuaternion, Vector3};
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
) -> Result<(Box<dyn QuadrotorInterface>, f32), SimulationError> {
    let (quad, mass): (Box<dyn QuadrotorInterface>, f32) = match &config.quadrotor {
        QuadrotorConfigurations::Peng(quad_config) => (
            Box::new(Quadrotor::new(
                1.0 / config.simulation.simulation_frequency as f32,
                config.simulation.clone(),
                config.imu.clone(),
                quad_config.mass,
                config.simulation.gravity,
                quad_config.drag_coefficient,
                quad_config.inertia_matrix,
            )?),
            quad_config.mass,
        ),
        QuadrotorConfigurations::Liftoff(ref liftoff_quad_config) => (
            Box::new(LiftoffQuad::new(
                config.simulation.clone(),
                liftoff_quad_config.clone(),
            )?),
            liftoff_quad_config.mass,
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
