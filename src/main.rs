#![feature(thread_sleep_until)]
use liftoff_quad::LiftoffQuad;
use nalgebra::Vector3;
use peng_quad::quadrotor::QuadrotorInterface;
use peng_quad::*;
use std::thread;
use std::time::Duration;
use std::time::Instant;

mod liftoff_quad;

#[tokio::main]
/// Main function for the simulation
async fn main() -> Result<(), SimulationError> {

    let mut config_str = "config/quad.yaml";
    let args: Vec<String> = std::env::args().collect();
    if args.len() != 2 {
        println!(
            "[\x1b[33mWARN\x1b[0m peng_quad] Usage: {} <config.yaml>.",
            args[0]
        );
        println!("[\x1b[33mWARN\x1b[0m peng_quad] Loading default configuration: config/quad.yaml");
    } else {
        println!(
            "[\x1b[32mINFO\x1b[0m peng_quad] Loading configuration: {}",
            args[1]
        );
        config_str = &args[1];
    }
    let config = config::Config::from_yaml(config_str).expect("Failed to load configuration.");
    println!(
        "[\x1b[32mINFO\x1b[0m peng_quad]Use rerun.io: {}",
        config.use_rerun
    );
    let time_step = 1.0 / config.simulation.simulation_frequency as f32;
    // TODO: quadrotor factory
    println!(
        "[\x1b[32mINFO\x1b[0m peng_quad] Using quadrotor: {:?}",
        config.quadrotor
    );
    let mut imu = Imu::new(
        config.imu.accel_noise_std,
        config.imu.gyro_noise_std,
        config.imu.accel_bias_std,
        config.imu.gyro_bias_std,
    )?;
    let mut maze = Maze::new(
        config.maze.lower_bounds,
        config.maze.upper_bounds,
        config.maze.num_obstacles,
        config.maze.obstacles_velocity_bounds,
        config.maze.obstacles_radius_bounds,
    );
    let mut camera = Camera::new(
        config.camera.resolution,
        config.camera.fov_vertical.to_radians(),
        config.camera.near,
        config.camera.far,
    );
    let mut planner_manager = PlannerManager::new(Vector3::zeros(), 0.0);
    let mut trajectory = Trajectory::new(Vector3::new(0.0, 0.0, 0.0));
    let mut depth_buffer: Vec<f32> = vec![0.0; camera.resolution.0 * camera.resolution.1];
    let planner_config: Vec<PlannerStepConfig> = config
        .planner_schedule
        .iter()
        .map(|step| PlannerStepConfig {
            step: step.step,
            planner_type: step.planner_type.clone(),
            params: step.params.clone(),
        })
        .collect();
    let rec = if config.use_rerun {
        let _rec = rerun::RecordingStreamBuilder::new("Peng").spawn()?;
        rerun::Logger::new(_rec.clone())
            .with_path_prefix("logs")
            .with_filter(rerun::default_log_filter())
            .init()
            .unwrap();
        Some(_rec)
    } else {
        env_logger::builder()
            .parse_env(env_logger::Env::default().default_filter_or("debug"))
            .init();
        None
    };
    // log::info!("Use rerun.io: {}", config.use_rerun);
    if let Some(rec) = &rec {
        rec.log_file_from_path(config.rerun_blueprint.clone(), None, false)?;

        rec.set_time_seconds("timestamp", 0);
        log_mesh(rec, config.mesh.division, config.mesh.spacing)?;
        log_maze_tube(rec, &maze)?;
        log_maze_obstacles(rec, &maze)?;
    }
    let (mut quad, mass): (Box<dyn QuadrotorInterface>, f32) = match config.quadrotor {
        config::QuadrotorConfigurations::Peng(quad_config) => (
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
        config::QuadrotorConfigurations::Liftoff(ref liftoff_quad_config) => (
            Box::new(LiftoffQuad::new(
                1.0 / config.simulation.control_frequency as f32,
                config.simulation.clone(),
                liftoff_quad_config.clone(),
            )?),
            liftoff_quad_config.mass,
            liftoff_quad_config.gravity,
        ),
        _ => {
            return Err(SimulationError::OtherError(
                "Unsupported quadrotor type".to_string(),
            ))
        }
    };

    println!(
        "[\x1b[32mINFO\x1b[0m peng_quad] Quadrotor: {:?} {:?}",
        mass, config.simulation.gravity
    );
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
    let frame_time = Duration::from_secs_f32(time_step);
    let mut next_frame = Instant::now();
    let mut quad_state = quad.observe(i)?;
    loop {
        // If real-time mode is enabled, sleep until the next frame simulation frame
        if config.real_time {
            thread::sleep_until(next_frame);
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
        let (mut thrust, mut calculated_desired_orientation) = controller.compute_position_control(
            &desired_position,
            &desired_velocity,
            desired_yaw,
            &quad_state.position,
            &quad_state.velocity,
            time_step,
        );

        let mut torque = controller.compute_attitude_control(
            &calculated_desired_orientation,
            &quad_state.orientation,
            &quad_state.angular_velocity,
            time_step,
        );
        println!("Desired Position: {:?}", desired_position);
        println!("Desired Orientation: {:?}", calculated_desired_orientation);
        println!("Desired Torque: {:?}", torque);
        let first_planner = planner_config.first().unwrap();
        if i >= first_planner.step {
            let _ = quad.control(i, thrust, &torque);
        }
        quad_state = quad.observe(i)?;
        imu.update(time_step)?;
        let (true_accel, true_gyro) = quad.read_imu()?;
        let (measured_accel, measured_gyro) = imu.read(true_accel, true_gyro)?;
        if i % (config.simulation.simulation_frequency / config.simulation.log_frequency) == 0 {
            if config.render_depth {
                camera.render_depth(
                    &quad_state.position,
                    &quad_state.orientation,
                    &maze,
                    config.use_multithreading_depth_rendering,
                )?;
            }

            if let Some(rec) = &rec {
                rec.set_time_seconds("timestamp", time);
                let mut rerun_quad_state = quad_state.clone();
                if let config::QuadrotorConfigurations::Liftoff(_) = config.quadrotor.clone() {
                    rerun_quad_state.position = Vector3::new(
                        quad_state.position.x,
                        -quad_state.position.y,
                        quad_state.position.z,
                    );
                }
                if trajectory.add_point(rerun_quad_state.position) {
                    log_trajectory(rec, &trajectory)?;
                }
                // log_joy(rec, thrust, &torque)?;
                log_data(
                    rec,
                    &rerun_quad_state,
                    &desired_position,
                    &desired_velocity,
                    &measured_accel,
                    &quad_state.angular_velocity,
                    thrust,
                    &torque,
                )?;
                let rotation = nalgebra::UnitQuaternion::from_axis_angle(
                    &Vector3::z_axis(),
                    // std::f32::consts::FRAC_PI_2,
                    0.0,
                );
                if config.render_depth {
                    log_depth_image(rec, &camera, config.use_multithreading_depth_rendering)?;
                    log_pinhole_depth(
                        rec,
                        &camera,
                        rerun_quad_state.position,
                        rotation * quad_state.orientation,
                        config.camera.rotation_transform,
                    )?;
                }
                log_maze_obstacles(rec, &maze)?;
            }
        }
        i += 1;
        // if time >= config.simulation.duration {
        //     log::info!("Complete Simulation");
        //     break;
        // }
    }
    log::logger().flush();
    Ok(())
}
