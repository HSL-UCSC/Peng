use crate::config::Config;
use crate::SimulationError;
use nalgebra::{UnitQuaternion, Vector3};

/// Represents the state of a quadrotor
pub struct QuadrotorState {
    pub position: Vector3<f32>,
    /// Current velocity of the quadrotor
    pub velocity: Vector3<f32>,
    /// Current orientation of the quadrotor
    pub orientation: UnitQuaternion<f32>,
    /// Current angular velocity of the quadrotor
    pub angular_velocity: Vector3<f32>,
}

/// Types implementing this trait can be used as a quadrotor in the simulation.
/// The underlying model can be internal or external to the simulator.
pub trait QuadrotorInterface {
    fn control(&mut self, step_number: usize, config: &Config, thrust: f32, torque: &Vector3<f32>);

    fn observe(&mut self) -> Result<QuadrotorState, SimulationError>;
}
