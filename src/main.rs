mod betaflight_quad;
mod liftoff_quad;
mod logger;
mod quadrotor_factory;

use logger::ControlOutput;
use logger::DesiredState;
use logger::RerunLogger;
use nalgebra::Vector3;
use peng_quad::environment::Maze;
use peng_quad::*;
use quadrotor::QuadrotorState;
use rerun::external::log;
use std::time::Duration;
use std::time::Instant;
use std::{
    borrow::Borrow,
    sync::{
        atomic::{AtomicBool, AtomicU64, Ordering},
        Arc,
    },
};
use tokio::sync::{Barrier, Mutex, Notify};

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

    let maze = Maze::new(
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
        notify: Arc::new(Notify::new()), // Notification system for signaling
        clock: Arc::new(AtomicU64::new(0)), // Shared atomic clock/tick counter
        start_barrier: Arc::new(Barrier::new(1)), // Synchronize thread start
        end_barrier: Arc::new(Barrier::new(1)), // Synchronize thread end
        kill: Arc::new(AtomicBool::new(false)), // Graceful shutdown flag
    });

    // Configure logger
    println!("Before rerun spawn");
    let rerun_handle = if config.use_rerun {
        let ids = vec![config.quadrotor.get_id()];
        // Set up Rerun
        // Note, the value of maze passed in is only used for the initial drawing of rerun.
        // Subsequent maze updates are processed on a maze watch channel
        let rerun_state_logger = logger::RerunLogger::new(config.clone(), rx_maze, ids)?;
        println!("Out");
        Some(rerun_state_logger)
    } else {
        env_logger::builder()
            .parse_env(env_logger::Env::default().default_filter_or("debug"))
            .init();
        None
    };
    println!("After rerun spawn");

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
    let frame_time = Duration::from_secs_f32(time_step);
    let start_time = tokio::time::Instant::now();
    let mut next_frame = start_time;
    // Observe Loop Warmup
    let start_time = Instant::now();

    let clock_sync = Arc::clone(&worker_sync);
    let clock_handle = clock_handle(config.simulation.clone(), clock_sync);
    // loop {
    //     if start_time.elapsed() >= Duration::new(5, 0) {
    //         let _ = quad.observe(time_step);
    //         break; // Exit the loop after 3 seconds
    //     }
    // }

    loop {
        // If real-time mode is enabled, sleep until the next frame simulation frame
        // worker_sync.start_barrier.wait().await;
        if (&config.simulation).real_time {
            tokio::time::sleep_until(next_frame).await;
            next_frame += frame_time;
        }
        // worker_sync.end_barrier.wait().await;

        // if let Some(rerun) = &rerun_handle {
        //     _ = rerun.log(
        //         start_time.elapsed().as_secs_f32(),
        //         QuadrotorState::default(),
        //         DesiredState::default(),
        //         ControlOutput::default(),
        //     );
        // };
        if start_time.elapsed() >= std::time::Duration::from_secs_f32((&config.simulation).duration)
        {
            log::info!("Complete Simulation");
            break;
        }
    }

    println!("here....");
    worker_sync.kill.store(true, Ordering::Relaxed);
    log::logger().flush();
    clock_handle.await.expect("failed to await clock handle");
    maze_handle.await.expect("failed to await maze handle");
    if let Some(rerun) = rerun_handle {
        rerun.handle.await.expect("failed to await rerun handle");
    };
    Ok(())
}

struct WorkerSync {
    notify: Arc<Notify>,
    clock: Arc<AtomicU64>,
    start_barrier: Arc<Barrier>,
    end_barrier: Arc<Barrier>,
    kill: Arc<AtomicBool>,
}

fn quadrotor_worker(
    id: usize,
    config: config::Config,
    sync: WorkerSync,
    rerun_logger: RerunLogger,
    maze_watch: tokio::sync::watch::Receiver<Arc<Mutex<Maze>>>,
) -> Result<(), SimulationError> {
    let simulation_period = 1_f32 / config.simulation.simulation_frequency as f32;
    tokio::spawn(async move {
        let (mut quad, mass) =
            quadrotor_factory::build_quadrotor(&config).expect("failed to build quadrotor");
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
        log::info!("Use rerun.io: {}", config.use_rerun);
        log::info!("Quadrotor: {:?} {:?}", mass, config.simulation.gravity);
        let pos_gains = config.pid_controller.pos_gains;
        let att_gains = config.pid_controller.att_gains;
        let mut controller = PIDController::new(
            [pos_gains.kp, pos_gains.kd, pos_gains.ki],
            [att_gains.kp, att_gains.kd, att_gains.ki],
            config.pid_controller.pos_max_int,
            config.pid_controller.att_max_int,
            mass,
            config.simulation.gravity,
        );
        let quad_state = quad
            .observe(0_f32)
            .expect("error getting latest state estimate");
        while !sync.kill.load(Ordering::Relaxed) {
            sync.start_barrier.wait().await;
            sync.notify.notified().await;
            let step = sync.clock.load(Ordering::Relaxed);

            // Do simulation and control here

            let time = simulation_period * step as f32;
            let latest_maze = maze_watch.borrow().clone();
            let mut maze = latest_maze.lock().await;
            // maze.update_obstacles(simulation_period);
            update_planner(
                &mut planner_manager,
                step as usize,
                time,
                config.simulation.simulation_frequency,
                &quad_state,
                &maze.obstacles,
                &planner_config,
            )
            .expect("failed to update planner");
            let (desired_position, desired_velocity, desired_yaw) = planner_manager
                .update(
                    quad_state.position,
                    quad_state.orientation,
                    quad_state.velocity,
                    time,
                    &maze.obstacles,
                )
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
            let quad_state = quad
                .observe(step as f32)
                .expect("error getting latest state estimate");
            if (step as usize)
                % (config.simulation.simulation_frequency / config.simulation.log_frequency)
                == 0
            {
                rerun_logger
                    .log(
                        simulation_period,
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
                    .expect("failed to log state");
            }
            sync.end_barrier.wait().await;
        }
    });
    Ok(())
}

fn clock_handle(
    config: config::SimulationConfig,
    sync: Arc<WorkerSync>,
) -> tokio::task::JoinHandle<()> {
    tokio::spawn(async move {
        let sim_duration = config.duration;
        let start_time = Instant::now();
        let period = 1_f64 / (config.simulation_frequency as f64);
        let period_duration = std::time::Duration::from_secs_f64(period);
        let mut next_frame = tokio::time::Instant::now();

        while !sync.kill.load(Ordering::Relaxed) {
            // sync.start_barrier.wait().await;
            sync.clock.fetch_add(1, Ordering::Relaxed);
            // Notify worker tasks
            sync.notify.notify_waiters();

            // Sleep for the tick interval
            if config.real_time {
                tokio::time::sleep_until(next_frame).await;
                next_frame += period_duration;
            }

            // Stop after simulation time
            // if start_time.elapsed().as_secs_f32() >= sim_duration {
            //     sync.kill.store(true, Ordering::Relaxed);
            //     sync.notify.notify_waiters(); // Ensure workers wake up to exit
            // }
            // sync.end_barrier.wait().await;
        }
    })
}

fn maze_worker(
    id: String,
    config: &config::SimulationConfig,
    sync: Arc<WorkerSync>,
    tx: tokio::sync::watch::Sender<Maze>,
    maze: Maze,
) -> Result<tokio::task::JoinHandle<()>, SimulationError> {
    let dt = 1_f32 / (config.simulation_frequency as f32);
    let mut maze = maze.clone();
    let handle = tokio::spawn(async move {
        while !sync.kill.load(Ordering::Relaxed) {
            // sync.start_barrier.wait().await;
            sync.notify.notified().await;
            let t = sync.clock.load(Ordering::Relaxed);

            // Do simulation and control here
            // let maze = tx.borrow().clone();
            // let mut maze_lock = maze.lock().await;
            maze.update_obstacles(dt);
            _ = tx.send(maze.clone()).expect("failed to send maze");
            // sync.end_barrier.wait().await;
        }
    });
    Ok(handle)
}
