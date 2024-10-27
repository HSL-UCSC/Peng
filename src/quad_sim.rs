use anyhow::Result;
use peng_quad::config::Config;
use peng_quad::Imu;
use peng_quad::Maze;
use peng_quad::PIDController;
use peng_quad::Quadrotor;

pub fn quad_sim(config: Config) -> Result<()> {
    let mut quad = Quadrotor::new(
        1.0 / config.simulation.simulation_frequency as f32,
        config.quadrotor.mass,
        config.quadrotor.gravity,
        config.quadrotor.drag_coefficient,
        config.quadrotor.inertia_matrix,
    )?;
    let _pos_gains = config.pid_controller.pos_gains;
    let _att_gains = config.pid_controller.att_gains;
    let mut controller = PIDController::new(
        [_pos_gains.kp, _pos_gains.kd, _pos_gains.ki],
        [_att_gains.kp, _att_gains.kd, _att_gains.ki],
        config.pid_controller.pos_max_int,
        config.pid_controller.att_max_int,
        config.quadrotor.mass,
        config.quadrotor.gravity,
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
    loop {}
}
