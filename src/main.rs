mod betaflight_quad;
mod liftoff_quad;
mod logger;
mod quadrotor_factory;

use nalgebra::Vector3;
use peng_quad::environment::Maze;
use peng_quad::sensors::Camera;
use peng_quad::*;
use rerun::external::log;
use std::time::Duration;
use std::time::Instant;

#[tokio::main]
/// Main function for the simulation
async fn main() -> Result<(), SimulationError> {
    // Configuration
    let mut config_str = "config/quad.yaml";
    let args: Vec<String> = std::env::args().collect();
    let plog = logger::PrintLogger::new(logger::LogLevel::Debug);
    info!(plog, "Starting Peng Quadrotor Simulation");
    if args.len() != 2 {
        warn!(plog, "Usage: {} <config.yaml>.", args[0]);
        info!(plog, "Loading default configuration: config/quad.yaml");
    } else {
        info!(plog, "Loading configuration: {}", args[1]);
        config_str = &args[1];
    }
    let config = config::Config::from_yaml(config_str).expect("Failed to load configuration.");
    info!(plog, "Use rerun.io: {}", config.use_rerun);
    info!(plog, "Using quadrotor: {:?}", config.quadrotor);

    // Initialize Environment
    let mut maze = Maze::new(
        config.maze.lower_bounds,
        config.maze.upper_bounds,
        config.maze.num_obstacles,
        config.maze.obstacles_velocity_bounds,
        config.maze.obstacles_radius_bounds,
    );

    // Instantiate quadrotor
    info!(plog, "Using quadrotor: {:?}", config.quadrotor);
    let (mut quad, mass) = quadrotor_factory::build_quadrotor(&config)?;

    let mut planner_manager = PlannerManager::new(Vector3::zeros(), 0.0);
    let planner_config: Vec<PlannerStepConfig> = config
        .planner_schedule
        .iter()
        .map(|step| PlannerStepConfig {
            step: step.step,
            planner_type: step.planner_type.clone(),
            params: step.params.clone(),
        })
        .collect();
    // Configure logger
    let mut rerun_logger_handle = if config.use_rerun {
        // Set up Rerun
        let rerun_state_logger = logger::RerunLogger::new(&config, &maze, vec![quad.parameters()])?;
        Some(rerun_state_logger)
    } else {
        env_logger::builder()
            .parse_env(env_logger::Env::default().default_filter_or("debug"))
            .init();
        None
    };
    log::info!("Use rerun.io: {}", config.use_rerun);
    log::info!("Quadrotor: {:?} {:?}", mass, config.simulation.gravity);
    let _pos_gains = config.pid_controller.pos_gains;
    let _att_gains = config.pid_controller.att_gains;
    let mut controller = PIDController::new(
        [_pos_gains.kp, _pos_gains.kd, _pos_gains.ki],
        [_att_gains.kp, _att_gains.kd, _att_gains.ki],
        config.pid_controller.pos_max_int,
        config.pid_controller.att_max_int,
        mass,
        config.simulation.gravity,
    );
    log::info!("Starting simulation...");
    let mut i = 0;
    let time_step = 1.0 / config.simulation.simulation_frequency as f32;
    let frame_time = Duration::from_secs_f32(time_step);
    let mut next_frame = tokio::time::Instant::now();
    let mut quad_state = quad.observe(time_step)?;
    // Observe Loop Warmup
    let start_time = Instant::now();
    loop {
        if start_time.elapsed() >= Duration::new(5, 0) {
            let _ = quad.observe(time_step);
            break; // Exit the loop after 3 seconds
        }
    }
    loop {
        // If real-time mode is enabled, sleep until the next frame simulation frame
        if config.real_time {
            tokio::time::sleep_until(next_frame).await;
            next_frame += frame_time;
        }
        let time = time_step * i as f32;
        maze.update_obstacles(time_step);
        update_planner(
            &mut planner_manager,
            i,
            time,
            config.simulation.simulation_frequency,
            &quad_state,
            &maze.obstacles,
            &planner_config,
        )?;
        let (desired_position, desired_velocity, desired_yaw) = planner_manager.update(
            quad_state.position,
            quad_state.orientation,
            quad_state.velocity,
            time,
            &maze.obstacles,
        )?;
        let (thrust, desired_orientation) = controller.compute_position_control(
            &desired_position,
            &desired_velocity,
            desired_yaw,
            &quad_state.position,
            &quad_state.velocity,
            time_step,
        );

        let torque = controller.compute_attitude_control(
            &desired_orientation,
            &quad_state.orientation,
            &quad_state.angular_velocity,
            time_step,
        );
        // TODO: cleanup the return type of the control method
        let (thrust_out, torque_out) = match quad.control(i, thrust, &torque)? {
            Some(values) => values,
            None => (0_f32, Vector3::<f32>::zeros()),
        };
        quad_state = quad.observe(time_step)?;
        if i % (config.simulation.simulation_frequency / config.simulation.log_frequency) == 0 {
            if let Some(logger) = rerun_logger_handle.as_mut() {
                logger.log(
                    time,
                    quad_state.clone(),
                    &maze,
                    logger::DesiredState {
                        position: desired_position,
                        velocity: desired_velocity,
                        orientation: desired_orientation,
                    },
                    logger::ControlOutput {
                        torque,
                        thrust,
                        thrust_out,
                        torque_out,
                    },
                )?;
            }
        }
        i += 1;
        if time >= config.simulation.duration {
            log::info!("Complete Simulation");
            break;
        }
    }
    log::logger().flush();
    Ok(())
}
