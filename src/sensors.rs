use nalgebra::{Matrix3, UnitQuaternion, Vector3};
use rand::SeedableRng;
use rand_chacha::ChaCha8Rng;
use rand_distr::{Distribution, Normal};
use rayon::prelude::*;
use rayon::slice::ParallelSliceMut;

use crate::fast_sqrt;
use crate::Maze;
use crate::{ray_cast, SimulationError};

/// Represents an Inertial Measurement Unit (IMU) with bias and noise characteristics
/// # Example
/// ```
/// use nalgebra::Vector3;
/// use peng_quad::sensors::Imu;
/// let accel_noise_std = 0.0003;
/// let gyro_noise_std = 0.02;
/// let accel_bias_std = 0.0001;
/// let gyro_bias_std = 0.001;
/// let imu = Imu::new(accel_noise_std, gyro_noise_std, accel_bias_std, gyro_bias_std);
/// ```
pub struct Imu {
    /// Accelerometer bias
    pub accel_bias: Vector3<f32>,
    /// Gyroscope bias
    pub gyro_bias: Vector3<f32>,
    /// Standard deviation of accelerometer noise
    pub accel_noise_std: f32,
    /// Standard deviation of gyroscope noise
    pub gyro_noise_std: f32,
    /// Standard deviation of accelerometer bias drift
    pub accel_bias_std: f32,
    /// Standard deviation of gyroscope bias drift
    pub gyro_bias_std: f32,
    /// Accelerometer noise distribution
    accel_noise: Normal<f32>,
    /// Gyroscope noise distribution
    gyro_noise: Normal<f32>,
    /// Accelerometer bias drift distribution
    accel_bias_drift: Normal<f32>,
    /// Gyroscope bias drift distribution
    gyro_bias_drift: Normal<f32>,
    /// Random number generator
    rng: ChaCha8Rng,
}
/// Implements the IMU
impl Imu {
    /// Creates a new IMU with default parameters
    /// # Arguments
    /// * `accel_noise_std` - Standard deviation of accelerometer noise
    /// * `gyro_noise_std` - Standard deviation of gyroscope noise
    /// * `accel_bias_std` - Standard deviation of accelerometer bias drift
    /// * `gyro_bias_std` - Standard deviation of gyroscope bias drift
    /// # Returns
    /// * A new Imu instance
    /// # Example
    /// ```
    /// use peng_quad::sensors::Imu;
    ///
    /// let imu = Imu::new(0.01, 0.01, 0.01, 0.01);
    /// ```
    pub fn new(
        accel_noise_std: f32,
        gyro_noise_std: f32,
        accel_bias_std: f32,
        gyro_bias_std: f32,
    ) -> Result<Self, SimulationError> {
        Ok(Self {
            accel_bias: Vector3::zeros(),
            gyro_bias: Vector3::zeros(),
            accel_noise_std,
            gyro_noise_std,
            accel_bias_std,
            gyro_bias_std,
            accel_noise: Normal::new(0.0, accel_noise_std)?,
            gyro_noise: Normal::new(0.0, gyro_noise_std)?,
            accel_bias_drift: Normal::new(0.0, accel_bias_std)?,
            gyro_bias_drift: Normal::new(0.0, gyro_bias_std)?,
            rng: ChaCha8Rng::from_entropy(),
        })
    }
    /// Updates the IMU biases over time
    /// # Arguments
    /// * `dt` - Time step for the update
    /// # Errors
    /// * Returns a SimulationError if the bias drift cannot be calculated
    /// # Example
    /// ```
    /// use peng_quad::sensors::Imu;
    ///
    /// let mut imu = Imu::new(0.01, 0.01, 0.01, 0.01).unwrap();
    /// imu.update(0.01).unwrap();
    /// ```
    pub fn update(&mut self, dt: f32) -> Result<(), SimulationError> {
        let dt_sqrt = fast_sqrt(dt);
        let accel_drift = self.accel_bias_drift.sample(&mut self.rng) * dt_sqrt;
        let gyro_drift = self.gyro_bias_drift.sample(&mut self.rng) * dt_sqrt;
        self.accel_bias += Vector3::from_iterator((0..3).map(|_| accel_drift));
        self.gyro_bias += Vector3::from_iterator((0..3).map(|_| gyro_drift));
        Ok(())
    }
    /// Simulates IMU readings with added bias and noise
    ///
    /// The added bias and noise are based on normal distributions
    /// # Arguments
    /// * `true_acceleration` - The true acceleration vector
    /// * `true_angular_velocity` - The true angular velocity vector
    /// # Returns
    /// * A tuple containing the measured acceleration and angular velocity
    /// # Errors
    /// * Returns a SimulationError if the IMU readings cannot be calculated
    /// # Example
    /// ```
    /// use nalgebra::Vector3;
    /// use peng_quad::sensors::Imu;
    ///
    /// let mut imu = Imu::new(0.01, 0.01, 0.01, 0.01).unwrap();
    /// let true_acceleration = Vector3::new(0.0, 0.0, 9.81);
    /// let true_angular_velocity = Vector3::new(0.0, 0.0, 0.0);
    /// let (measured_acceleration, measured_ang_velocity) = imu.read(true_acceleration, true_angular_velocity).unwrap();
    /// ```
    pub fn read(
        &mut self,
        true_acceleration: Vector3<f32>,
        true_angular_velocity: Vector3<f32>,
    ) -> Result<(Vector3<f32>, Vector3<f32>), SimulationError> {
        let accel_noise_sample =
            Vector3::from_iterator((0..3).map(|_| self.accel_noise.sample(&mut self.rng)));
        let gyro_noise_sample =
            Vector3::from_iterator((0..3).map(|_| self.gyro_noise.sample(&mut self.rng)));
        let measured_acceleration = true_acceleration + self.accel_bias + accel_noise_sample;
        let measured_ang_velocity = true_angular_velocity + self.gyro_bias + gyro_noise_sample;
        Ok((measured_acceleration, measured_ang_velocity))
    }
}

/// Represents a camera in the simulation which is used to render the depth of the scene
/// # Example
/// ```
/// use peng_quad::sensors::Camera;
/// let camera = Camera::new((800, 600), 60.0, 0.1, 100.0);
/// ```
#[derive(Default)]
pub struct Camera {
    /// The resolution of the camera
    pub resolution: (usize, usize),
    /// The vertical field of view of the camera
    pub fov_vertical: f32,
    /// The horizontal field of view of the camera
    pub fov_horizontal: f32,
    /// The vertical focal length of the camera
    pub vertical_focal_length: f32,
    /// The horizontal focal length of the camera
    pub horizontal_focal_length: f32,
    /// The near clipping plane of the camera
    pub near: f32,
    /// The far clipping plane of the camera
    pub far: f32,
    /// The aspect ratio of the camera
    pub aspect_ratio: f32,
    /// The ray directions of each pixel in the camera
    pub ray_directions: Vec<Vector3<f32>>,
    /// Depth buffer
    pub depth_buffer: Vec<f32>,
}
/// Implementation of the camera
impl Camera {
    /// Creates a new camera with the given resolution, field of view, near and far clipping planes
    /// # Arguments
    /// * `resolution` - The resolution of the camera
    /// * `fov_vertical` - The vertical field of view of the camera
    /// * `near` - The near clipping plane of the camera
    /// * `far` - The far clipping plane of the camera
    /// # Returns
    /// * The new camera instance
    /// # Example
    /// ```
    /// use peng_quad::sensors::Camera;
    /// let camera = Camera::new((800, 600), 1.0, 5.0, 120.0);
    /// ```
    pub fn new(resolution: (usize, usize), fov_vertical: f32, near: f32, far: f32) -> Self {
        let (width, height) = resolution;
        let (aspect_ratio, tan_half_fov) =
            (width as f32 / height as f32, (fov_vertical / 2.0).tan());
        let mut ray_directions = Vec::with_capacity(width * height);
        for y in 0..height {
            for x in 0..width {
                let x_ndc = (2.0 * x as f32 / width as f32 - 1.0) * aspect_ratio * tan_half_fov;
                let y_ndc = (1.0 - 2.0 * y as f32 / height as f32) * tan_half_fov;
                ray_directions.push(Vector3::new(1.0, x_ndc, y_ndc).normalize());
            }
        }
        let fov_horizontal =
            (width as f32 / height as f32 * (fov_vertical / 2.0).tan()).atan() * 2.0;
        let horizontal_focal_length = (width as f32 / 2.0) / ((fov_horizontal / 2.0).tan());
        let vertical_focal_length = (height as f32 / 2.0) / ((fov_vertical / 2.0).tan());
        let depth_buffer = vec![0.0; width * height];

        Self {
            resolution,
            fov_vertical,
            fov_horizontal,
            vertical_focal_length,
            horizontal_focal_length,
            near,
            far,
            aspect_ratio,
            ray_directions,
            depth_buffer,
        }
    }

    /// Renders the depth of the scene from the perspective of the quadrotor
    ///
    /// When the depth value is out of the near and far clipping planes, it is set to infinity
    /// When the resolution is larger than 32x24, multi-threading can accelerate the rendering
    /// # Arguments
    /// * `quad_position` - The position of the quadrotor
    /// * `quad_orientation` - The orientation of the quadrotor
    /// * `maze` - The maze in the scene
    /// * `use_multi_threading` - Whether to use multi-threading to render the depth
    /// # Errors
    /// * If the depth buffer is not large enough to store the depth values
    /// # Example
    /// ```
    /// use peng_quad::sensors::Camera;
    /// use peng_quad::environment::Maze;
    /// use nalgebra::{Vector3, UnitQuaternion};
    /// let mut camera = Camera::new((800, 600), 60.0, 0.1, 100.0);
    /// let quad_position = Vector3::new(0.0, 0.0, 0.0);
    /// let quad_orientation = UnitQuaternion::identity();
    /// let mut maze = Maze::new([-1.0, -1.0, -1.0], [1.0, 1.0, 1.0], 5, [0.1, 0.1, 0.1], [0.1, 0.5]);
    /// let use_multi_threading = true;
    /// camera.render_depth(&quad_position, &quad_orientation, &maze, use_multi_threading);
    /// let use_multi_threading = false;
    /// camera.render_depth(&quad_position, &quad_orientation, &maze, use_multi_threading);
    /// ```
    pub fn render_depth(
        &mut self,
        quad_position: &Vector3<f32>,
        quad_orientation: &UnitQuaternion<f32>,
        maze: &Maze,
        use_multi_threading: bool,
    ) -> Result<(), SimulationError> {
        let (width, height) = self.resolution;
        let total_pixels = width * height;
        let rotation_camera_to_world = quad_orientation.to_rotation_matrix().matrix()
            * Matrix3::new(1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0);
        let rotation_world_to_camera = rotation_camera_to_world.transpose();

        const CHUNK_SIZE: usize = 64;
        if use_multi_threading {
            self.depth_buffer
                .par_chunks_mut(CHUNK_SIZE)
                .enumerate()
                .try_for_each(|(chunk_idx, chunk)| {
                    let start_idx = chunk_idx * CHUNK_SIZE;
                    for (i, depth) in chunk.iter_mut().enumerate() {
                        let ray_idx = start_idx + i;
                        if ray_idx >= total_pixels {
                            break;
                        }
                        *depth = ray_cast(
                            quad_position,
                            &rotation_world_to_camera,
                            &(rotation_camera_to_world * self.ray_directions[ray_idx]),
                            maze,
                            self.near,
                            self.far,
                        )?;
                    }
                    Ok::<(), SimulationError>(())
                })?;
        } else {
            for i in 0..total_pixels {
                self.depth_buffer[i] = ray_cast(
                    quad_position,
                    &rotation_world_to_camera,
                    &(rotation_camera_to_world * self.ray_directions[i]),
                    maze,
                    self.near,
                    self.far,
                )?;
            }
        }
        Ok(())
    }
}
