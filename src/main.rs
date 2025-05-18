mod betaflight_quad;
mod liftoff_quad;
mod logger;
// mod quadrotor_factory;
pub mod planners;
mod sync;

use futures::future::try_join_all;
use logger::{FileLogger, RerunLogger};
use nalgebra::Vector3;
use peng_quad::environment::Maze;
use peng_quad::*;
use quadrotor::build_quadrotor;
use rerun::external::log;
use std::sync::{
    atomic::{AtomicBool, AtomicU64, Ordering},
    Arc,
};
use std::time::Duration;
use std::time::Instant;
use sync::WorkerSync;
use tokio::sync::Barrier;

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
    let config: Arc<config::Config> =
        Arc::new(config::Config::from_yaml(config_str).expect("Failed to load configuration."));
    info!(plog, "Use rerun.io: {}", config.use_rerun);
    info!(plog, "Using quadrotor: {:?}", config.quadrotor);

    let maze = environment::Maze::new(
        config.maze.lower_bounds,
        config.maze.upper_bounds,
        config.maze.num_obstacles,
        config.maze.obstacles_velocity_bounds,
        config.maze.obstacles_radius_bounds,
    );
    // TODO: rename these to be maze specific
    let (tx_maze, rx_maze) = tokio::sync::watch::channel(maze.clone());

    // TODO: num workers for barriers
    let worker_sync = Arc::new(WorkerSync {
        clock: Arc::new(AtomicU64::new(0)), // Shared atomic clock/tick counter
        start_barrier: Arc::new(Barrier::new(3)), // Synchronize thread start
        end_barrier: Arc::new(Barrier::new(3)), // Synchronize thread end
        kill: Arc::new(AtomicBool::new(false)), // Graceful shutdown flag
    });

    // Configure logger
    let (rerun_handle, rerun_logger) = if config.use_rerun {
        let ids: Vec<String> = config
            .quadrotor
            .iter()
            .map(|config| config.get_id())
            .collect();
        if ids.iter().any(|id| id.trim().is_empty()) {
            return Err(SimulationError::OtherError(
                "quadrotor with no ID".to_string(),
            ));
        }

        // Set up Rerun
        // Note, the value of maze passed in is only used for the initial drawing of rerun.
        // Subsequent maze updates are processed on a maze watch channel
        let log_sync_clone = Arc::clone(&worker_sync);
        let (rerun_handle, rerun_logger) =
            logger::RerunLogger::new(config.clone(), rx_maze.clone(), ids, log_sync_clone)?;
        (Some(rerun_handle), Some(rerun_logger))
    } else {
        env_logger::builder()
            .parse_env(env_logger::Env::default().default_filter_or("debug"))
            .init();
        (None, None)
    };
    let (file_logger_handle, file_logger) = if let Some(file_logger) = &config.file_logger {
        let (file_logger_handle, file_logger) =
            logger::FileLogger::new(file_logger.as_ref(), worker_sync.clone())?;
        (Some(file_logger_handle), Some(file_logger))
    } else {
        (None, None)
    };

    let maze_sync_clone = Arc::clone(&worker_sync);
    let maze_handle = maze_worker(
        "maze".to_string(),
        &config.simulation,
        maze_sync_clone,
        tx_maze,
        maze.clone(),
    )?;

    log::info!("Starting simulation...");
    let time_step = 1.0 / config.simulation.simulation_frequency as f32;
    let start_time = tokio::time::Instant::now();
    let _frame_time = Duration::from_secs_f32(time_step);
    let _next_frame = start_time;
    // Observe Loop Warmup
    let start_time = Instant::now();

    let clock_sync = Arc::clone(&worker_sync);
    let clock_handle = clock_handle(config.simulation.clone(), clock_sync);
    // loop {
    //     if start_time.elapsed() >= Duration::new(5, 0) {
    //         break; // Exit the loop after 3 seconds
    //     }
    // }

    let rr = rerun_logger.unwrap();
    let quad_handles: Vec<tokio::task::JoinHandle<()>> = config
        .quadrotor
        .clone()
        .into_iter()
        .map(|quadrotor_config| {
            let quadrotor_sync_clone = Arc::clone(&worker_sync);
            // TODO: need a file logger per quad
            quadrotor_worker(
                Arc::clone(&config),
                quadrotor_config.clone(),
                quadrotor_sync_clone,
                rr.clone(),
                file_logger.clone(),
                rx_maze.clone(),
            )
            .expect("Filed to create quadrotor worker")
        })
        .collect();

    // TODO: just sleep in main thread for simulation duration?
    loop {
        if start_time.elapsed() >= std::time::Duration::from_secs_f32(config.simulation.duration) {
            log::info!("Complete Simulation");
            break;
        }
    }

    worker_sync.kill.store(true, Ordering::Relaxed);
    log::logger().flush();
    clock_handle.await.expect("failed to await clock handle");
    maze_handle.await.expect("failed to await maze handle");
    try_join_all(quad_handles)
        .await
        .expect("failed to await quadrotor handles");

    println!("Quads Joined...");
    rerun_handle
        .unwrap()
        .await
        .expect("failed to await rerun handle");

    println!("Rerun Logger Joined...");

    if file_logger.is_some() {
        drop(file_logger);
    }
    if let Some(handle) = file_logger_handle {
        handle.await.expect("failed to await rerun handle");
    }
    println!("File Logger Joined...");
    Ok(())
}

fn quadrotor_worker(
    config: Arc<config::Config>,
    quadrotor_config: config::QuadrotorConfigurations,
    sync: Arc<WorkerSync>,
    rerun_logger: RerunLogger,
    mut file_logger: Option<FileLogger>,
    maze_watch: tokio::sync::watch::Receiver<Maze>,
) -> Result<tokio::task::JoinHandle<()>, SimulationError> {
    let simulation_period = 1_f32 / config.simulation.simulation_frequency as f32;
    let handle = tokio::spawn(async move {
        let (mut quad, mass) =
            build_quadrotor(&config, &quadrotor_config).expect("failed to build quadrotor");
        let mut planner_manager =
            planners::PlannerManager::new(quad.observe(0.0).unwrap().position, 0.0);
        let planner_config: Vec<planners::PlannerStepConfig> = config
            .planner_schedule
            .iter()
            .map(|step| planners::PlannerStepConfig {
                step: step.step,
                time: step.time,
                planner_type: step.planner_type.clone(),
                params: step.params.clone(),
            })
            .collect();

        // Configure logger
        log::info!("Use rerun.io: {}", config.use_rerun);
        log::info!("Quadrotor: {:?} {:?}", mass, config.simulation.gravity);
        let pos_gains = config.pid_controller.pos_gains;
        let att_gains = config.pid_controller.att_gains;
        let mut controller = peng_quad::PIDController::new(
            [pos_gains.kp, pos_gains.kd, pos_gains.ki],
            [att_gains.kp, att_gains.kd, att_gains.ki],
            config.pid_controller.pos_max_int,
            config.pid_controller.att_max_int,
            mass,
            config.simulation.gravity,
        );
        let mut quad_state = quad
            .observe(0_f32)
            .expect("error getting latest state estimate");

        while !sync.kill.load(Ordering::Relaxed) {
            sync.start_barrier.wait().await;
            // sync.notify.notified().await;
            let step = sync.clock.load(Ordering::Relaxed);

            // Do simulation and control here
            let time = simulation_period * step as f32;
            let maze = maze_watch.borrow().clone();
            // TODO: START HERE - should the update_planner, and planner_manager.update methods be
            // combined?
            // planners::update_planner(
            //     &mut planner_manager,
            //     step as usize,
            //     time,
            //     config.simulation.simulation_frequency,
            //     &quad_state,
            //     &maze.obstacles,
            //     &planner_config,
            // )
            // .expect("failed to update planner");

            // step: usize,
            // time: f32,
            // _simulation_frequency: usize,
            // quad_state: &QuadrotorState,
            // obstacles: &[Obstacle],
            // planner_config: &[PlannerStepConfig],

            // FIXME: why do I need to cast step?
            let (desired_position, desired_velocity, desired_yaw) = planner_manager
                .update(
                    step.try_into().unwrap(),
                    time,
                    &quad_state,
                    &maze.obstacles,
                    &planner_config,
                )
                .await
                .expect("failed to calculate inner loop control");

            let (thrust, desired_orientation) = controller.compute_position_control(
                &desired_position,
                &desired_velocity,
                desired_yaw,
                &quad_state.position,
                &quad_state.velocity,
                simulation_period,
            );

            let torque = controller.compute_attitude_control(
                &desired_orientation,
                &quad_state.orientation,
                &quad_state.angular_velocity,
                simulation_period,
            );
            // TODO: cleanup the return type of the control method
            let (thrust_out, torque_out) = match quad
                .control(step as usize, thrust, &torque)
                .expect("failed to execute control command")
            {
                Some(values) => values,
                None => (0_f32, Vector3::<f32>::zeros()),
            };
            quad_state = quad
                .observe(simulation_period)
                .expect("error getting latest state estimate");

            if let Some(file_logger) = file_logger.as_mut() {
                file_logger
                    .log(
                        time,
                        quadrotor_config.clone(),
                        quad_state.clone(),
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
                    )
                    .expect("Failed to log state with FileLogger");
            }

            if (step as usize)
                % (config.simulation.simulation_frequency / config.simulation.log_frequency)
                == 0
            {
                rerun_logger
                    .log(
                        time,
                        quadrotor_config.clone(),
                        quad_state.clone(),
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
                    )
                    .expect("Failed to log state with RerunLogger");
            }
            sync.end_barrier.wait().await;
        }
    });
    Ok(handle)
}

fn clock_handle(
    config: config::SimulationConfig,
    sync: Arc<WorkerSync>,
) -> tokio::task::JoinHandle<()> {
    tokio::spawn(async move {
        let sim_duration = config.duration;
        let start_time = tokio::time::Instant::now();
        let period = 1_f64 / (config.simulation_frequency as f64);
        let period_duration = tokio::time::Duration::from_secs_f64(period);
        let mut next_frame = tokio::time::Instant::now();

        while !sync.kill.load(Ordering::Relaxed) {
            sync.start_barrier.wait().await;
            let now = tokio::time::Instant::now(); // Capture actual start time
            if now > next_frame + tokio::time::Duration::from_millis(1) {
                let lag = now.duration_since(next_frame);
                log::debug!(
                    "[WARNING] Running behind schedule! Lag: {:.3} ms",
                    lag.as_secs_f64()
                );
            }

            sync.clock.fetch_add(1, Ordering::Relaxed);
            if config.real_time {
                tokio::time::sleep_until(next_frame).await;
                next_frame =
                    start_time + (sync.clock.load(Ordering::Relaxed) as u32 * period_duration);
            }

            if start_time.elapsed().as_secs_f32() >= sim_duration {
                sync.kill.store(true, Ordering::Relaxed);
            }
            sync.end_barrier.wait().await;
        }
    })
}

fn maze_worker(
    _id: String,
    config: &config::SimulationConfig,
    sync: Arc<WorkerSync>,
    tx: tokio::sync::watch::Sender<environment::Maze>,
    maze: environment::Maze,
) -> Result<tokio::task::JoinHandle<()>, SimulationError> {
    let dt = 1_f32 / (config.simulation_frequency as f32);
    let mut maze = maze.clone();
    let handle = tokio::spawn(async move {
        while !sync.kill.load(Ordering::Relaxed) {
            sync.start_barrier.wait().await;
            // let t = sync.clock.load(Ordering::Relaxed) * dt;

            // Do simulation and control here
            // let maze = tx.borrow().clone();
            // let mut maze_lock = maze.lock().await;
            maze.update_obstacles(dt);
            tx.send(maze.clone()).expect("failed to send maze");
            sync.end_barrier.wait().await;
        }
    });
    Ok(handle)
}
