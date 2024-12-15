use crate::SimulationError;
use nalgebra::{UnitQuaternion, Vector3};

#[derive(Clone, Debug, Default)]
/// Represents the state of a quadrotor
pub struct QuadrotorState {
    /// Time stampe of the state
    pub time: f32,
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

/// Types implementing this trait can be used as a quadrotor in the simulation.
/// The underlying model can be internal or external to the simulator.
pub trait QuadrotorInterface {
    // TODO remove config from control
    fn control(
        &mut self,
        step_number: usize,
        thrust: f32,
        torque: &Vector3<f32>,
    ) -> Result<(), SimulationError>;

    fn observe(&mut self) -> Result<QuadrotorState, SimulationError>;

    fn max_thrust(&self) -> f32;

    fn max_torque(&self) -> Vector3<f32>;

    fn read_imu(&self) -> Result<(Vector3<f32>, Vector3<f32>), SimulationError>;
}
