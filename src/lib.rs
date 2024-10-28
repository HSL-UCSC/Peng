//! # Quadrotor Simulation
//! This crate provides a comprehensive simulation environment for quadrotor drones.
//! It includes models for quadrotor dynamics, IMU simulation, various trajectory planners,
//! and a PID controller for position and attitude control.
//! ## Features
//! - Realistic quadrotor dynamics simulation
//! - IMU sensor simulation with configurable noise parameters
//! - Multiple trajectory planners including hover, minimum jerk, Lissajous curves, and circular paths
//! - PID controller for position and attitude control
//! - Integration with the `rerun` crate for visualization
//! ## Example
//! ```
//! use nalgebra::Vector3;
//! use peng_quad::{Quadrotor, SimulationError};
//! let (time_step, mass, gravity, drag_coefficient) = (0.01, 1.3, 9.81, 0.01);
//! let inertia_matrix = [0.0347563, 0.0, 0.0, 0.0, 0.0458929, 0.0, 0.0, 0.0, 0.0977];
//! let quadrotor = Quadrotor::new(time_step, mass, gravity, drag_coefficient, inertia_matrix);
//! ```
pub mod lib_controller;
pub mod lib_planner;
pub mod lib_quadrotor;
pub mod lib_sensors;

pub use lib_controller::*;
pub use lib_planner::*;
pub use lib_quadrotor::*;
pub use lib_sensors::*;

pub mod config;

/// Fast square root function
/// # Arguments
/// * `x` - The input value
/// # Returns
/// * The square root of the input value
#[inline(always)]
fn fast_sqrt(x: f32) -> f32 {
    let i = x.to_bits();
    let i = 0x1fbd1df5 + (i >> 1);
    f32::from_bits(i)
}

#[derive(thiserror::Error, Debug)]
/// Represents errors that can occur during simulation
/// # Example
/// ```
/// use peng_quad::SimulationError;
/// let error = SimulationError::NalgebraError("Matrix inversion failed".to_string());
/// ```
pub enum SimulationError {
    /// Error related to Rerun visualization
    #[error("Rerun error: {0}")]
    RerunError(#[from] rerun::RecordingStreamError),
    /// Error related to Rerun spawn process
    #[error("Rerun spawn error: {0}")]
    RerunSpawnError(#[from] rerun::SpawnError),
    /// Error related to linear algebra operations
    #[error("Nalgebra error: {0}")]
    NalgebraError(String),
    /// Error related to normal distribution calculations
    #[error("Normal error: {0}")]
    NormalError(#[from] rand_distr::NormalError),
    /// Other general errors
    #[error("Other error: {0}")]
    OtherError(String),
}

/// log trajectory data to the rerun recording stream
/// # Arguments
/// * `rec` - The rerun::RecordingStream instance
/// * `trajectory` - The Trajectory instance
/// # Errors
/// * If the data cannot be logged to the recording stream
/// # Example
/// ```no_run
/// use peng_quad::{Trajectory, log_trajectory};
/// use nalgebra::Vector3;
/// let rec = rerun::RecordingStreamBuilder::new("log.rerun").connect().unwrap();
/// let mut trajectory = Trajectory::new(nalgebra::Vector3::new(0.0, 0.0, 0.0));
/// trajectory.add_point(nalgebra::Vector3::new(1.0, 0.0, 0.0));
/// log_trajectory(&rec, &trajectory).unwrap();
/// ```
pub fn log_trajectory(
    rec: &rerun::RecordingStream,
    trajectory: &Trajectory,
) -> Result<(), SimulationError> {
    let path = trajectory
        .points
        .iter()
        .map(|p| (p.x, p.y, p.z))
        .collect::<Vec<(f32, f32, f32)>>();
    rec.log(
        "world/quad/path",
        &rerun::LineStrips3D::new([path]).with_colors([rerun::Color::from_rgb(0, 255, 255)]),
    )?;
    Ok(())
}
/// log mesh data to the rerun recording stream
/// # Arguments
/// * `rec` - The rerun::RecordingStream instance
/// * `division` - The number of divisions in the mesh
/// * `spacing` - The spacing between divisions
/// # Errors
/// * If the data cannot be logged to the recording stream
/// # Example
/// ```no_run
/// use peng_quad::log_mesh;
/// let rec = rerun::RecordingStreamBuilder::new("log.rerun").connect().unwrap();
/// log_mesh(&rec, 10, 0.1).unwrap();
/// ```
pub fn log_mesh(
    rec: &rerun::RecordingStream,
    division: usize,
    spacing: f32,
) -> Result<(), SimulationError> {
    let grid_size: usize = division + 1;
    let half_grid_size: f32 = (division as f32 * spacing) / 2.0;
    let points: Vec<rerun::external::glam::Vec3> = (0..grid_size)
        .flat_map(|i| {
            (0..grid_size).map(move |j| {
                rerun::external::glam::Vec3::new(
                    j as f32 * spacing - half_grid_size,
                    i as f32 * spacing - half_grid_size,
                    0.0,
                )
            })
        })
        .collect();
    let horizontal_lines: Vec<Vec<rerun::external::glam::Vec3>> = (0..grid_size)
        .map(|i| points[i * grid_size..(i + 1) * grid_size].to_vec())
        .collect();
    let vertical_lines: Vec<Vec<rerun::external::glam::Vec3>> = (0..grid_size)
        .map(|j| (0..grid_size).map(|i| points[i * grid_size + j]).collect())
        .collect();
    let line_strips: Vec<Vec<rerun::external::glam::Vec3>> =
        horizontal_lines.into_iter().chain(vertical_lines).collect();
    rec.log(
        "world/mesh",
        &rerun::LineStrips3D::new(line_strips)
            .with_colors([rerun::Color::from_rgb(255, 255, 255)])
            .with_radii([0.02]),
    )?;
    Ok(())
}
/// log depth image data to the rerun recording stream
/// # Arguments
/// * `rec` - The rerun::RecordingStream instance
/// * `depth_image` - The depth image data
/// * `width` - The width of the depth image
/// * `height` - The height of the depth image
/// * `min_depth` - The minimum depth value
/// * `max_depth` - The maximum depth value
/// # Errors
/// * If the data cannot be logged to the recording stream
/// # Example
/// ```no_run
/// use peng_quad::log_depth_image;
/// let rec = rerun::RecordingStreamBuilder::new("log.rerun").connect().unwrap();
/// let depth_image = vec![0.0; 640 * 480];
/// log_depth_image(&rec, &depth_image, 640, 480, 0.0, 1.0).unwrap();
/// ```
pub fn log_depth_image(
    rec: &rerun::RecordingStream,
    depth_image: &[f32],
    width: usize,
    height: usize,
    min_depth: f32,
    max_depth: f32,
) -> Result<(), SimulationError> {
    let mut image = rerun::external::ndarray::Array::zeros((height, width, 3));
    let depth_range = max_depth - min_depth;
    image
        .axis_iter_mut(rerun::external::ndarray::Axis(0))
        .enumerate()
        .for_each(|(y, mut row)| {
            for (x, mut pixel) in row
                .axis_iter_mut(rerun::external::ndarray::Axis(0))
                .enumerate()
            {
                let depth = depth_image[y * width + x];
                let color = if depth.is_finite() {
                    let normalized_depth = ((depth - min_depth) / depth_range).clamp(0.0, 1.0);
                    color_map_fn(normalized_depth * 255.0)
                } else {
                    (0, 0, 0)
                };
                (pixel[0], pixel[1], pixel[2]) = color;
            }
        });
    let rerun_image = rerun::Image::from_color_model_and_tensor(rerun::ColorModel::RGB, image)
        .map_err(|e| SimulationError::OtherError(format!("Failed to create rerun image: {}", e)))?;
    rec.log("world/quad/cam/depth", &rerun_image)?;
    Ok(())
}
/// turbo color map function
/// # Arguments
/// * `gray` - The gray value in the range [0, 255]
/// # Returns
/// * The RGB color value in the range [0, 255]
/// # Example
/// ```
/// use peng_quad::color_map_fn;
/// let color = color_map_fn(128.0);
/// ```
#[inline]
pub fn color_map_fn(gray: f32) -> (u8, u8, u8) {
    let x = gray / 255.0;
    let r = (34.61
        + x * (1172.33 - x * (10793.56 - x * (33300.12 - x * (38394.49 - x * 14825.05)))))
        .clamp(0.0, 255.0) as u8;
    let g = (23.31 + x * (557.33 + x * (1225.33 - x * (3574.96 - x * (1073.77 + x * 707.56)))))
        .clamp(0.0, 255.0) as u8;
    let b = (27.2 + x * (3211.1 - x * (15327.97 - x * (27814.0 - x * (22569.18 - x * 6838.66)))))
        .clamp(0.0, 255.0) as u8;
    (r, g, b)
}
