use crate::config::{self};
use crate::environment::Maze;
use crate::quadrotor::QuadrotorState;
use crate::sensors::Camera;
use crate::{SimulationError, Trajectory};
use colored::Colorize;
use nalgebra::{Matrix3, Rotation3, UnitQuaternion, Vector3};
use rayon::prelude::*;
use rerun::RecordingStream;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Arc;
use std::time::{SystemTime, UNIX_EPOCH};

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub enum LogLevel {
    Error,
    Warn,
    Info,
    Debug,
}

pub struct PrintLogger {
    level: Arc<AtomicUsize>,
}

impl PrintLogger {
    pub fn new(level: LogLevel) -> Self {
        Self {
            level: Arc::new(AtomicUsize::new(level as usize)),
        }
    }

    pub fn set_level(&self, level: LogLevel) {
        self.level.store(level as usize, Ordering::SeqCst);
    }

    pub fn log(&self, level: LogLevel, message: &str) {
        if level.clone() as usize <= self.level.load(Ordering::SeqCst) {
            let timestamp = SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .expect("Time went backwards")
                .as_secs();
            let string = format!("[{:?}] [{}] {}", level, timestamp, message);
            match level {
                LogLevel::Error => println!("{}", string.red()),
                LogLevel::Warn => println!("{}", string.yellow()),
                LogLevel::Info => println!("{}", string.blue()),
                LogLevel::Debug => println!("{}", string.cyan()),
            };
        }
    }
}

// Macros for logging
#[macro_export]
macro_rules! error {
    ($logger:expr, $($arg:tt)*) => {
        $logger.log(logger::LogLevel::Error, &format!($($arg)*));
    };
}

#[macro_export]
macro_rules! warn {
    ($logger:expr, $($arg:tt)*) => {
        $logger.log(logger::LogLevel::Warn, &format!($($arg)*));
    };
}

#[macro_export]
macro_rules! info {
    ($logger:expr, $($arg:tt)*) => {
        $logger.log(logger::LogLevel::Info, &format!($($arg)*));
    };
}

#[macro_export]
macro_rules! debug {
    ($logger:expr, $($arg:tt)*) => {
        $logger.log(LogLevel::Debug, &format!($($arg)*));
    };
}

pub struct DesiredState {
    pub position: Vector3<f32>,
    pub velocity: Vector3<f32>,
    pub orientation: UnitQuaternion<f32>,
}

pub struct ControlOutput {
    pub thrust: f32,
    pub torque: Vector3<f32>,
    // applied control outputs
    pub thrust_out: f32,
    pub torque_out: Vector3<f32>,
}

#[derive(Clone)]
pub struct RerunLogger<'a> {
    pub tx: tokio::sync::mpsc::Sender<String>,
    rec: Arc<RecordingStream>,
    config: &'a config::Config,
    _maze: &'a Maze,
    trajectory_map: std::collections::HashMap<String, Trajectory>,
}

impl<'a> RerunLogger<'a> {
    pub fn new(
        config: &'a config::Config,
        maze: &'a Maze,
        quadrotors: Vec<crate::config::QuadrotorConfig>,
    ) -> Result<Self, SimulationError> {
        let (tx, mut rx) = tokio::sync::mpsc::channel(10);

        let rec = rerun::RecordingStreamBuilder::new("Peng").spawn()?;
        rerun::Logger::new(rec.clone())
            .with_path_prefix("logs")
            .with_filter(rerun::default_log_filter())
            .init()
            .expect("error starting rerun");
        rec.log_file_from_path(config.rerun_blueprint.clone(), None, false)?;
        rec.set_time_seconds("timestamp", 0);
        log_maze_tube(&rec, maze)?;
        log_maze_obstacles(&rec, maze)?;
        let mut trajectory_map = std::collections::HashMap::<String, Trajectory>::new();
        quadrotors.iter().for_each(|quad| {
            // let id = match quad {
            //     config::QuadrotorConfigurations::Peng(c) => c.id.clone(),
            //     config::QuadrotorConfigurations::Liftoff(c) => c.id.clone(),
            //     config::QuadrotorConfigurations::Betaflight(c) => c.quadrotor_config.id.clone(),
            // };
            let trajectory = Trajectory::new(Vector3::new(0.0, 0.0, 0.0));
            trajectory_map.insert(quad.id.clone(), trajectory);
        });

        tokio::spawn(async move {
            loop {
                if let Some(message) = rx.recv().await {
                    println!("Received: {}", message);
                } else {
                    // Exit if the channel is closed and all senders are dropped
                    println!("All senders have been dropped. Exiting receiver loop.");
                    break;
                }
            }
        });
        // let rc = rec.clone();
        Ok(Self {
            tx,
            // rx,
            config,
            rec: Arc::new(rec),
            _maze: maze,
            trajectory_map,
        })
    }

    pub fn log(
        &mut self,
        time: f32,
        quad_state: QuadrotorState,
        maze: Maze,
        camera: &mut Camera,
        // mut trajectory: Trajectory,
        desired_state: DesiredState,
        control_out: ControlOutput,
    ) -> Result<(), SimulationError> {
        let rec = &self.rec;
        rec.set_time_seconds("timestamp", time);
        let mut rerun_quad_state = quad_state.clone();
        match self.config.quadrotor.clone() {
            config::QuadrotorConfigurations::Peng(_) => (),
            config::QuadrotorConfigurations::Liftoff(_) => {
                rerun_quad_state.position = Vector3::new(
                    quad_state.position.x,
                    -quad_state.position.y,
                    quad_state.position.z,
                );
            }
            config::QuadrotorConfigurations::Betaflight(_) => {
                rerun_quad_state.position = Vector3::new(
                    quad_state.position.x,
                    quad_state.position.y,
                    quad_state.position.z,
                );
            }
        };
        // let mut trajectory = self.trajectory_map.get("").as_ref();
        if let Some(trajectory) = self.trajectory_map.get_mut("PengQuad") {
            trajectory.add_point(quad_state.position);
            log_trajectory(rec, trajectory)?;
        }

        let euler_d = desired_state.orientation.euler_angles();
        log_data(
            rec,
            &rerun_quad_state,
            &desired_state.position,
            &Vector3::new(euler_d.0, euler_d.1, euler_d.2),
            &desired_state.velocity,
            &control_out.torque,
        )?;
        log_control(
            rec,
            control_out.thrust,
            control_out.torque,
            Some((control_out.thrust_out, control_out.torque_out)),
        )?;
        if self.config.render_depth {
            log_depth_image(rec, camera, self.config.use_multithreading_depth_rendering)?;
            log_pinhole_depth(
                rec,
                camera,
                rerun_quad_state.position,
                rerun_quad_state.orientation,
                self.config.camera.rotation_transform,
            )?;
            camera.render_depth(
                &quad_state.position,
                &quad_state.orientation,
                &maze,
                self.config.use_multithreading_depth_rendering,
            )?;
        }
        log_maze_obstacles(rec, &maze)?;
        Ok(())
    }
}

/// Logs simulation data to the rerun recording stream
/// # Arguments
/// * `rec` - The rerun::RecordingStream instance
/// * `quad` - The Quadrotor instance
/// * `desired_position` - The desired position vector
/// * `measured_accel` - The measured acceleration vector
/// * `measured_gyro` - The measured angular velocity vector
/// # Errors
/// * If the data cannot be logged to the recording stream
/// # Example
/// ```no_run
/// use peng_quad::config;;
/// use peng_quad::{Quadrotor, quadrotor::QuadrotorInterface};
/// use peng_quad::logger::log_data;
/// use nalgebra::Vector3;
/// let rec = rerun::RecordingStreamBuilder::new("peng").connect().unwrap();
/// let (time_step, mass, gravity, drag_coefficient) = (0.01, 1.3, 9.81, 0.01);
/// let inertia_matrix = [0.0347563, 0.0, 0.0, 0.0, 0.0458929, 0.0, 0.0, 0.0, 0.0977];
/// let mut quadrotor = Quadrotor::new(time_step, config::SimulationConfig::default(), config::ImuConfig::default(), mass, gravity, drag_coefficient, inertia_matrix).unwrap();
/// let quad_state = quadrotor.observe(0.0).unwrap();
/// let desired_position = Vector3::new(0.0, 0.0, 0.0);
/// let desired_orientation = Vector3::new(0.0, 0.0, 0.0);
/// let desired_velocity = Vector3::new(0.0, 0.0, 0.0);
/// let measured_accel = Vector3::new(0.0, 0.0, 0.0);
/// let measured_gyro = Vector3::new(0.0, 0.0, 0.0);
/// let measured_torque = Vector3::new(0.0, 0.0, 0.0);
/// log_data(&rec, &quad_state, &desired_position, &desired_orientation, &desired_velocity, &measured_accel, &measured_gyro, &measured_torque).unwrap();
/// ```
pub fn log_data(
    rec: &rerun::RecordingStream,
    quad_state: &QuadrotorState,
    desired_position: &Vector3<f32>,
    desired_orientation: &Vector3<f32>,
    desired_velocity: &Vector3<f32>,
    torque: &Vector3<f32>,
) -> Result<(), SimulationError> {
    rec.log(
        "world/quad/desired_position",
        &rerun::Points3D::new([(desired_position.x, desired_position.y, desired_position.z)])
            .with_radii([0.1])
            .with_colors([rerun::Color::from_rgb(255, 255, 255)]),
    )?;
    rec.log(
        "world/quad/base_link",
        &rerun::Transform3D::from_translation_rotation(
            rerun::Vec3D::new(
                quad_state.position.x,
                quad_state.position.y,
                quad_state.position.z,
            ),
            rerun::Quaternion::from_xyzw([
                quad_state.orientation.i,
                quad_state.orientation.j,
                quad_state.orientation.k,
                quad_state.orientation.w,
            ]),
        )
        .with_axis_length(0.7),
    )?;
    let (quad_roll, quad_pitch, quad_yaw) = quad_state.orientation.euler_angles();
    let quad_euler_angles: Vector3<f32> = Vector3::new(quad_roll, quad_pitch, quad_yaw);
    for (pre, vec) in [
        ("position", &quad_state.position),
        ("velocity", &quad_state.velocity),
        ("accel", &quad_state.measured_state.acceleration),
        ("orientation", &quad_euler_angles),
        ("desired_orientation", desired_orientation),
        ("gyro", &quad_state.measured_state.angular_velocity),
        ("desired_position", desired_position),
        ("desired_velocity", desired_velocity),
        ("torque", torque),
    ] {
        for (i, a) in ["x", "y", "z"].iter().enumerate() {
            rec.log(format!("{}/{}", pre, a), &rerun::Scalar::new(vec[i] as f64))?;
        }
    }
    Ok(())
}

pub fn log_control(
    rec: &rerun::RecordingStream,
    thrust: f32,
    torque: Vector3<f32>,
    control_out: Option<(f32, Vector3<f32>)>,
) -> Result<(), SimulationError> {
    if let Some((thrust_ppm, torque_ppm)) = control_out {
        rec.log("ppm/throttle", &rerun::Scalar::new(thrust_ppm as f64))?;
        rec.log("control/thrust", &rerun::Scalar::new(thrust as f64))?;
        for (i, a) in ["x", "y", "z"].iter().enumerate() {
            rec.log(
                format!("ppm/{}/{}", "torque", a),
                &rerun::Scalar::new(torque_ppm[i] as f64),
            )?;
        }
        for (i, a) in ["x", "y", "z"].iter().enumerate() {
            rec.log(
                format!("control/{}/{}", "torque", a),
                &rerun::Scalar::new(torque[i] as f64),
            )?;
        }
    };
    Ok(())
}

/// Log the maze tube to the rerun recording stream
/// # Arguments
/// * `rec` - The rerun::RecordingStream instance
/// * `maze` - The maze instance
/// # Errors
/// * If the data cannot be logged to the recording stream
/// # Example
/// ```no_run
/// use peng_quad::environment::Maze;
/// use peng_quad::logger::log_maze_tube;
/// use rerun::RecordingStreamBuilder;
/// let rec = rerun::RecordingStreamBuilder::new("log.rerun").connect().unwrap();
/// let mut maze = Maze::new([-1.0, -1.0, -1.0], [1.0, 1.0, 1.0], 5, [0.1, 0.1, 0.1], [0.1, 0.5]);
/// log_maze_tube(&rec, &maze).unwrap();
/// ```
pub fn log_maze_tube(rec: &rerun::RecordingStream, maze: &Maze) -> Result<(), SimulationError> {
    let (lower_bounds, upper_bounds) = (maze.lower_bounds, maze.upper_bounds);
    let center_position = rerun::external::glam::Vec3::new(
        (lower_bounds[0] + upper_bounds[0]) / 2.0,
        (lower_bounds[1] + upper_bounds[1]) / 2.0,
        (lower_bounds[2] + upper_bounds[2]) / 2.0,
    );
    let half_sizes = rerun::external::glam::Vec3::new(
        (upper_bounds[0] - lower_bounds[0]) / 2.0,
        (upper_bounds[1] - lower_bounds[1]) / 2.0,
        (upper_bounds[2] - lower_bounds[2]) / 2.0,
    );
    rec.log(
        "world/maze/tube",
        &rerun::Boxes3D::from_centers_and_half_sizes([center_position], [half_sizes])
            .with_colors([rerun::Color::from_rgb(128, 128, 255)]),
    )?;
    Ok(())
}
/// Log the maze obstacles to the rerun recording stream
/// # Arguments
/// * `rec` - The rerun::RecordingStream instance
/// * `maze` - The maze instance
/// # Errors
/// * If the data cannot be logged to the recording stream
/// # Example
/// ```no_run
/// use peng_quad::environment::Maze;
/// use peng_quad::logger::log_maze_obstacles;
/// let rec = rerun::RecordingStreamBuilder::new("log.rerun").connect().unwrap();
/// let mut maze = Maze::new([-1.0, -1.0, -1.0], [1.0, 1.0, 1.0], 5, [0.1, 0.1, 0.1], [0.1, 0.5]);
/// log_maze_obstacles(&rec, &maze).unwrap();
/// ```
pub fn log_maze_obstacles(
    rec: &rerun::RecordingStream,
    maze: &Maze,
) -> Result<(), SimulationError> {
    let (positions, radii): (Vec<(f32, f32, f32)>, Vec<f32>) = maze
        .obstacles
        .iter()
        .map(|obstacle| {
            let pos = obstacle.position;
            ((pos.x, pos.y, pos.z), obstacle.radius)
        })
        .unzip();
    rec.log(
        "world/maze/obstacles",
        &rerun::Points3D::new(positions)
            .with_radii(radii)
            .with_colors([rerun::Color::from_rgb(255, 128, 128)]),
    )?;
    Ok(())
}

/// creates pinhole camera
/// # Arguments
/// * `rec` - The rerun::RecordingStream instance
/// * `cam` - The camera object
/// * `cam_position` - The position vector of the camera (aligns with the quad)
/// * `cam_orientation` - The orientation quaternion of quad
/// * `cam_transform` - The transform matrix between quad and camera alignment
/// # Errors
/// * If the data cannot be logged to the recording stream
/// # Example
/// ```no_run
/// use peng_quad::sensors::Camera;
/// use peng_quad::logger::log_pinhole_depth;
/// use nalgebra::{Vector3, UnitQuaternion};
/// let rec = rerun::RecordingStreamBuilder::new("log.rerun").connect().unwrap();
/// let depth_image = vec![ 0.0f32 ; 640 * 480];
/// let cam_position = Vector3::new(0.0,0.0,0.0);
/// let cam_orientation = UnitQuaternion::identity();
/// let cam_transform = [0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0];
/// let camera = Camera::new((800, 600), 60.0, 0.1, 100.0);
/// log_pinhole_depth(&rec, &camera, cam_position, cam_orientation, cam_transform).unwrap();
/// ```
pub fn log_pinhole_depth(
    rec: &rerun::RecordingStream,
    cam: &Camera,
    cam_position: Vector3<f32>,
    cam_orientation: UnitQuaternion<f32>,
    cam_transform: [f32; 9],
) -> Result<(), SimulationError> {
    let depth_image = &cam.depth_buffer;
    let (width, height) = cam.resolution;
    let pinhole_camera = rerun::Pinhole::from_focal_length_and_resolution(
        (cam.horizontal_focal_length, cam.vertical_focal_length),
        (width as f32, height as f32),
    )
    .with_camera_xyz(rerun::components::ViewCoordinates::RDF)
    .with_resolution((width as f32, height as f32))
    .with_principal_point((width as f32 / 2.0, height as f32 / 2.0));
    let rotated_camera_orientation = UnitQuaternion::from_rotation_matrix(
        &(cam_orientation.to_rotation_matrix()
            * Rotation3::from_matrix_unchecked(Matrix3::from_row_slice(&cam_transform))),
    );
    let cam_transform = rerun::Transform3D::from_translation_rotation(
        rerun::Vec3D::new(cam_position.x, cam_position.y, cam_position.z),
        rerun::Quaternion::from_xyzw([
            rotated_camera_orientation.i,
            rotated_camera_orientation.j,
            rotated_camera_orientation.k,
            rotated_camera_orientation.w,
        ]),
    );
    rec.log("world/quad/cam", &cam_transform)?;
    rec.log("world/quad/cam", &pinhole_camera)?;
    let depth_image_rerun =
        rerun::external::ndarray::Array::from_shape_vec((height, width), depth_image.to_vec())
            .unwrap();
    rec.log(
        "world/quad/cam/rerun_depth",
        &rerun::DepthImage::try_from(depth_image_rerun)
            .unwrap()
            .with_meter(1.0),
    )?;

    Ok(())
}

/// log trajectory data to the rerun recording stream
/// # Arguments
/// * `rec` - The rerun::RecordingStream instance
/// * `trajectory` - The Trajectory instance
/// # Errors
/// * If the data cannot be logged to the recording stream
/// # Example
/// ```no_run
/// use peng_quad::{Trajectory};
/// use peng_quad::logger::log_trajectory;
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

/// Log depth image data to the rerun recording stream
///
/// When the depth value is `f32::INFINITY`, the pixel is considered invalid and logged as black
/// When the resolution is larger than 32x24, multi-threading can accelerate the rendering
/// # Arguments
/// * `rec` - The rerun::RecordingStream instance
/// * `cam` - The Camera instance
/// * `use_multi_threading` - Whether to use multithreading to log the depth image
/// # Errors
/// * If the data cannot be logged to the recording stream
/// # Example
/// ```no_run
/// use peng_quad::sensors::Camera;
/// use peng_quad::logger::log_depth_image;
/// let rec = rerun::RecordingStreamBuilder::new("log.rerun").connect().unwrap();
/// let camera = Camera::new((640, 480), 0.1, 100.0, 60.0);
/// let use_multi_threading = false;
/// log_depth_image(&rec, &camera, use_multi_threading).unwrap();
/// let use_multi_threading = true;
/// log_depth_image(&rec, &camera, use_multi_threading).unwrap();
/// ```
pub fn log_depth_image(
    rec: &rerun::RecordingStream,
    cam: &Camera,
    use_multi_threading: bool,
) -> Result<(), SimulationError> {
    let (width, height) = (cam.resolution.0, cam.resolution.1);
    let (min_depth, max_depth) = (cam.near, cam.far);
    let depth_image = &cam.depth_buffer;
    let mut image: rerun::external::ndarray::Array<u8, _> =
        rerun::external::ndarray::Array::zeros((height, width, 3));
    let depth_range = max_depth - min_depth;
    let scale_factor = 255.0 / depth_range;
    if use_multi_threading {
        const CHUNK_SIZE: usize = 32;
        image
            .as_slice_mut()
            .expect("Failed to get mutable slice of image")
            .par_chunks_exact_mut(CHUNK_SIZE * 3)
            .enumerate()
            .for_each(|(chunk_index, chunk)| {
                let start_index = chunk_index * 32;
                for (i, pixel) in chunk.chunks_exact_mut(3).enumerate() {
                    let idx = start_index + i;
                    let depth = depth_image[idx];
                    let color = if depth.is_finite() {
                        let normalized_depth =
                            ((depth - min_depth) * scale_factor).clamp(0.0, 255.0);
                        color_map_fn(normalized_depth)
                    } else {
                        (0, 0, 0)
                    };
                    (pixel[0], pixel[1], pixel[2]) = color;
                }
            });
    } else {
        for (index, pixel) in image
            .as_slice_mut()
            .expect("Failed to get mutable slice of image")
            .chunks_exact_mut(3)
            .enumerate()
        {
            let depth = depth_image[index];
            let color = if depth.is_finite() {
                let normalized_depth = ((depth - min_depth) * scale_factor).clamp(0.0, 255.0);
                color_map_fn(normalized_depth)
            } else {
                (0, 0, 0)
            };
            (pixel[0], pixel[1], pixel[2]) = color;
        }
    }
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
/// use peng_quad::logger::color_map_fn;
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
