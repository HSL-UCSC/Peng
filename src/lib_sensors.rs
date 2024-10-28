use crate::fast_sqrt;
use crate::SimulationError;
use nalgebra::Vector3;
use rand::Rng;
use rand::SeedableRng;
use rand_chacha::ChaCha8Rng;
use rand_distr::{Distribution, Normal};
/// Represents an Inertial Measurement Unit (IMU) with bias and noise characteristics
/// # Example
/// ```
/// use nalgebra::Vector3;
/// use peng_quad::Imu;
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

#[derive(Debug)]
struct GaussMarkov {
    tau: f64,       // Mean-reversion rate
    eta: f64,       // Noise scaling factor
    time_step: f64, // Time step
    x: f64,         // Current state of the process
}

impl GaussMarkov {
    // Constructor to initialize the Gauss-Markov process with parameters
    fn new(tau: f64, eta: f64, time_step: f64, initial_state: f64) -> Self {
        Self {
            tau,
            eta,
            time_step,
            x: initial_state,
        }
    }

    // Method to update the state and generate the next value
    fn next_value(&mut self) -> f64 {
        let noise = rand::thread_rng().gen::<f64>() * self.eta * (self.time_step).sqrt();
        self.x = self.x * (-self.tau * self.time_step).exp() + noise;
        self.x // Return the new state
    }
}

/// Represents a Global Positioning System (GPS) with bias and noise characteristics
/// ```
/// use nalgebra::Vector3;
/// use peng_quad::Gps;
/// let tauGPS = 1100.0;
/// let gpsRateHz = 1.0;
/// let etaGPSHorizontal = 0.21;
/// let etaGPSVertical= 0.4;
/// let gps = Gps::new(gpsRateHz, tauGPS, etaGPSHorizontal, etaGPSVertical);
/// ```
/// TODO: normalize comments
/// TODO: use Vector3 for positions
pub struct Gps {
    /// Update rate
    pub update_hz: f64,
    /// GPS tau
    pub tau: f64,
    /// GPS eta horizontal
    pub eta_horizontal: f64,
    /// GPS eta vertical
    pub eta_vertical: f64,
    pub x: f32,   // Current x position
    pub y: f32,   // Current y position
    pub alt: f32, // Current altitude
    pub sog: f32, // Current SOG
    pub cog: f32, // Current COG
    pub measured_x: GaussMarkov,
    pub measured_y: GaussMarkov,
    pub measured_alt: GaussMarkov,
    pub measured_sog: GaussMarkov,
    pub measured_cog: GaussMarkov,
    pub udpate_ticks: i32,
    pub tick_threshhold: i32,
}

/// Implements the GPS
impl Gps {
    /// Creates a new GPS with default parameters
    /// # Arguments
    /// * `update_hz` -
    /// * `tau` -
    /// * `eta_horizontal` -
    /// * `eta_vertical` -
    /// # Returns
    /// * A new Gps instance
    /// # Example
    /// ```
    /// use peng_quad::Gps;
    ///
    /// let mut gps = Gps::new(1.0, 1100.0, 0.21, 0.4).unwrap();
    /// gps.update(0.01).unwrap();
    /// ```
    pub fn new(
        gps_update_hz: f64,
        sim_update_hz: f64,
        tau: f64,
        eta_horizontal: f64,
        eta_vertical: f64,
        initial_position: Vector3<f32>,
        initial_sog: f32,
        initial_cog: f32,
    ) -> Result<Self, SimulationError> {
        let time_step: f64 = 1.0 / gps_update_hz;
        let gps_x = GaussMarkov::new(tau, eta_horizontal, time_step, 0.0);
        let gps_y = GaussMarkov::new(tau, eta_horizontal, time_step, 0.0);
        let gps_alt = GaussMarkov::new(tau, eta_vertical, time_step, 0.0);
        // Speed over ground
        let gps_sog = GaussMarkov::new(tau, 0.0, time_step, 0.0);
        // Course over ground
        let gps_cog = GaussMarkov::new(tau, 0.0, time_step, 0.0);
        Ok(Self {
            update_hz: gps_update_hz,
            tau,
            eta_horizontal,
            eta_vertical,
            // TODO: allow initial conditions from constructor
            x: initial_position[0],
            y: initial_position[1],
            alt: -initial_position[2],
            sog: initial_sog,
            cog: initial_cog,
            measured_x: gps_x,
            measured_y: gps_y,
            measured_alt: gps_alt,
            measured_sog: gps_sog,
            measured_cog: gps_cog,
            udpate_ticks: 0,
            tick_threshhold: (sim_update_hz / gps_update_hz) as i32,
        })
    }

    pub fn update(&mut self, true_position: Vector3<f32>, dt: f32) -> Result<(), SimulationError> {
        self.udpate_ticks += 1;
        if self.udpate_ticks >= self.tick_threshhold {
            self.x = true_position[0];
            self.y = true_position[1];
            self.alt = -true_position[2];
            self.sog = (self.x * self.x + self.y * self.y).sqrt();
            self.cog = (self.y / self.x).atan();
            self.udpate_ticks = 0;
            self.measured_x.next_value();
            self.measured_y.next_value();
            self.measured_alt.next_value();
            self.measured_sog.next_value();
            self.measured_cog.next_value();
        }
        Ok(())
    }

    /// Simulates GPS readings with added bias and noise
    /// # Example
    /// ```
    /// use nalgebra::Vector3;
    /// use peng_quad::Gps;
    ///
    /// use peng_quad::Gps;
    ///
    /// let mut gps = Gps::new(1.0, 1100.0, 0.21, 0.4).unwrap();
    /// gps.update(0.01).unwrap();
    /// let true_position = Vector3::new(0.0, 0.0, 9.81);
    /// let true_sog = 0.0;
    /// let true_cog = 0.0;
    /// let (position, sog, cog) = imu.read(true_position, true_cog, true_sog).unwrap();
    /// ```
    pub fn read(
        &mut self,
        true_position: Vector3<f32>,
        true_sog: f32,
        true_cog: f32,
        noisy: bool,
    ) -> Result<(Vector3<f32>, f32, f32), SimulationError> {
        // TODO: return measured values if noisy, else true values
        let measured_position = true_position;
        let true_sog = true_sog;
        let true_cog = true_cog;
        Ok((measured_position, true_sog, true_cog))
    }
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
    /// use peng_quad::Imu;
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
    /// use peng_quad::Imu;
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
    /// use peng_quad::Imu;
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
