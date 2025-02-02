use crate::betaflight_quad::BetaflightQuad;
use crate::config;
use crate::liftoff_quad::LiftoffQuad;
use peng_quad::config::QuadrotorConfigurations;
use peng_quad::quadrotor::QuadrotorInterface;
use peng_quad::Quadrotor;
use peng_quad::SimulationError;

pub fn build_quadrotor(
    config: &config::Config,
) -> Result<(Box<dyn QuadrotorInterface>, f32), SimulationError> {
    let (quad, mass): (Box<dyn QuadrotorInterface>, f32) = match &config.quadrotor {
        QuadrotorConfigurations::Peng(quad_config) => (
            Box::new(Quadrotor::new(
                1.0 / config.simulation.simulation_frequency as f32,
                config.simulation.clone(),
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
