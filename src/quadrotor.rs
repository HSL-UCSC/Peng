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
pub trait QuadrotorInterface {
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
