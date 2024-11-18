use binrw::{binrw, BinRead, BinWrite};
use cyber_rc::{cyberrc, Writer};
use nalgebra::{Matrix4, Quaternion, UnitQuaternion, Vector3};
use peng_quad::config::LiftoffQuadrotorConfig;
use peng_quad::quadrotor::{QuadrotorInterface, QuadrotorState};
use peng_quad::SimulationError;
use prost::Message;
use rand;
use serialport::{available_ports, SerialPort, SerialPortBuilder, SerialPortType};
use std::io::{Cursor, Write};
use std::net::UdpSocket;
use std::sync::Arc;
use std::sync::Mutex;
use tempfile::NamedTempFile;
use tokio::time::{sleep, Duration};

/// Represents a quadrotor in the game Liftoff
/// # Example
/// ```
/// let quad = LiftoffQuad{ }
/// ```
pub struct LiftoffQuad {
    /// The serial writer to communicate with the quadrotor
    pub writer: Option<Writer>,
    /// The current state of the quadrotor
    pub state: QuadrotorState,
    /// The last state of the quadrotor
    pub previous_state: QuadrotorState,
    /// Initial State
    pub initial_state: QuadrotorState,
    /// Simulation time step in seconds
    pub time_step: f32,
    /// Configured physical parameters
    pub config: LiftoffQuadrotorConfig,
    /// Previous Thrust
    pub previous_thrust: f32,
    /// Previous Torque
    pub previous_torque: Vector3<f32>,
    /// Quadrotor sample mutex
    pub shared_data: Arc<Mutex<Option<LiftoffPacket>>>,
}

impl LiftoffQuad {
    pub fn new(time_step: f32, config: LiftoffQuadrotorConfig) -> Result<Self, SimulationError> {
        // let shared_data: Arc<Mutex<Option<LiftoffPacket>>> =
        //     Arc::new(Mutex::new(Some(LiftoffPacket::default())));
        // let shared_data_clone = Arc::clone(&shared_data);
        // let config_clone = config.clone();
        // tokio::spawn(async move {
        //     let _ = feedback_loop(config_clone, shared_data_clone).await;
        // });
        // Open a serial port to communicate with the quadrotor if one is specified
        // If not, open a writer to a temp file
        let writer: Option<Writer> = match config.clone().serial_port {
            Some(port) => {
                println!("Port: {:?}", port);
                let mut writer = Writer::new(port.to_string(), config.baud_rate).map_err(|e| {
                    SimulationError::OtherError(format!(
                        "Failed to open SerialPort {:?}",
                        e.to_string()
                    ))
                })?;
                let start_time = std::time::Instant::now();
                // Zero throttle to arm the quadrotor in Liftoff
                println!("Arming Drone");
                while std::time::Instant::now() - start_time < std::time::Duration::from_secs(5) {
                    writer
                        .write(&mut cyberrc::RcData {
                            throttle: 32767,
                            aileron: 0,
                            elevator: 0,
                            rudder: 0,
                            arm: 1,
                            mode: 0,
                        })
                        .map_err(|e| SimulationError::OtherError(e.to_string()))?;
                }
                Some(writer)
            }
            None => None,
        };

        Ok(Self {
            writer,
            state: QuadrotorState::default(),
            previous_state: QuadrotorState::default(),
            initial_state: QuadrotorState::default(),
            config,
            time_step,
            previous_thrust: 0.0,
            previous_torque: Vector3::zeros(),
            shared_data: Arc::new(Mutex::new(None)),
        })
    }

    // Function to calculate body-frame acceleration for NED coordinates
    fn body_acceleration(
        &self,
        q_inertial_to_body: UnitQuaternion<f32>, // Unit quaternion rotation from inertial to body
        omega_body: Vector3<f32>,                // Angular velocity in body frame
        velocity_body: Vector3<f32>,             // Linear velocity in body frame
    ) -> Vector3<f32> {
        // Calculate thrust acceleration (negative Z in NED body frame)
        let thrust_acceleration = Vector3::new(0.0, 0.0, -self.previous_thrust / self.config.mass);

        // Rotate the gravity vector to the body frame
        let gravity_inertial = Vector3::new(0.0, 0.0, self.config.gravity); // NED gravity vector in inertial frame
        let gravity_body = q_inertial_to_body.transform_vector(&gravity_inertial); // Rotate gravity vector to body frame

        // Calculate rotational (Coriolis) effects
        let rotational_acceleration = omega_body.cross(&velocity_body);

        // Combine all terms
        thrust_acceleration + gravity_body - rotational_acceleration
    }
}

impl QuadrotorInterface for LiftoffQuad {
    fn control(
        &mut self,
        _: usize,
        thrust: f32,
        torque: &Vector3<f32>,
    ) -> Result<(), SimulationError> {
        // Given thrust and torque, calculate the control inputs
        // Clamp thrust and torque control inputs
        let max_thrust = self.max_thrust();
        let thrust = thrust.clamp(0.0, max_thrust);
        let max_torque = self.max_torque();
        let torque = Vector3::new(
            torque.x.clamp(-max_torque.x, max_torque.x),
            torque.y.clamp(-max_torque.y, max_torque.y),
            torque.z.clamp(-max_torque.z, max_torque.z),
        );

        // Normalize inputs
        let normalized_thrust = normalize(thrust, 0.0, self.max_thrust());
        let normalized_roll = normalize(torque.x, -max_torque.x, max_torque.x);
        let normalized_pitch = normalize(torque.y, -max_torque.y, max_torque.y);
        let normalized_yaw = normalize(torque.z, -max_torque.z, max_torque.z);

        // Scale to RC commands
        let throttle_command = scale_throttle(normalized_thrust);
        let aileron_command = scale_control(normalized_roll);
        let elevator_command = -scale_control(normalized_pitch);
        let rudder_command = scale_control(normalized_yaw);

        let mut cyberrc_data = cyberrc::RcData {
            throttle: throttle_command,
            aileron: aileron_command,
            elevator: elevator_command,
            rudder: rudder_command,
            arm: 0,
            mode: 0,
        };
        // println!("RC Data: {:?}", cyberrc_data);
        if let Some(writer) = &mut self.writer {
            writer
                .write(&mut cyberrc_data)
                .map_err(|e| SimulationError::OtherError(e.to_string()))?;
        }
        Ok(())
    }

    /// Observe the current state of the quadrotor
    /// Returns a tuple containing the position, velocity, orientation, and angular velocity of the quadrotor.
    fn observe(&mut self) -> Result<QuadrotorState, SimulationError> {
        let mut buf = [0; 128];
        // let mut data_lock = self.shared_data.lock().unwrap();
        let sample = match UdpSocket::bind(self.config.ip_address.to_string()) {
            Ok(socket) => {
                socket
                    .set_read_timeout(Some(Duration::from_millis(1)))
                    .map_err(|e| SimulationError::OtherError(e.to_string()))?;
                match socket.recv_from(&mut buf) {
                    Ok((len, _)) => {
                        let mut cursor = std::io::Cursor::new(&buf[..len]);
                        // TODO: more robust handling of packet parsing errors during resets
                        let sample = if let Ok(sample) = LiftoffPacket::read(&mut cursor) {
                            sample
                        } else {
                            LiftoffPacket::default()
                        };
                        Some(sample)
                    }
                    Err(_) => None,
                }
            }
            Err(e) => {
                return Err(SimulationError::OtherError(format!(
                    "Bind loop exceeded max wait time {}",
                    e.to_string()
                )));
            }
        };
        let state = match sample {
            Some(sample) => {
                // update the last state value
                self.previous_state = self.state.clone();
                let attitude_quaternion = sample.attitude_quaternion();
                // The position is in Unity inertial coordinates, so we transform to NED inertial
                // coordinates.
                let sample_position = sample.position();
                // Calculate the body-frame velocity by rotating the inertial velocity using the
                // attitude quaternion.
                let v_body = attitude_quaternion.transform_vector(
                    &((sample_position - self.previous_state.position) / self.time_step),
                );

                let omega_body = sample.pqr();
                let acceleration_body =
                    self.body_acceleration(attitude_quaternion, omega_body, v_body);
                QuadrotorState {
                    position: sample_position,
                    velocity: v_body,
                    acceleration: acceleration_body,
                    orientation: attitude_quaternion,
                    angular_velocity: omega_body,
                }
            }
            None => return Ok(self.state.clone()),
        };
        Ok(state)
    }

    fn max_thrust(&self) -> f32 {
        self.config.max_thrust_kg
    }

    /// Calculate all maximum torques and return them as a tuple
    fn max_torque(&self) -> Vector3<f32> {
        let motor_thrust = self.max_thrust() / 4.0;
        let max_rp_torque = 2.0 * 0.65 * motor_thrust;
        let yaw_torque = 2.0 * 0.005 * motor_thrust;
        Vector3::new(max_rp_torque, max_rp_torque, yaw_torque)
    }

    fn read_imu(&self) -> Result<(Vector3<f32>, Vector3<f32>), SimulationError> {
        Ok((self.state.acceleration, self.state.angular_velocity))
    }
}

fn try_take_with_timeout<T>(mutex: &Mutex<Option<T>>, timeout: Duration) -> Option<T> {
    let start = std::time::Instant::now();
    while start.elapsed() < timeout {
        if let Ok(mut guard) = mutex.try_lock() {
            if let Some(value) = guard.take() {
                return Some(value); // Successfully took the value
            }
        }
        std::thread::sleep(Duration::from_millis(1)); // Small sleep to avoid busy waiting
    }
    None // Timeout occurred
}

// TODO: configure packet based on the content of the Liftoff config file
#[binrw]
#[brw(little)]
#[derive(Debug, Default)]
pub struct LiftoffPacket {
    timestamp: f32,
    // x, y, z
    position: [f32; 3],
    // x, y, z, w
    attitude: [f32; 4],
    // pitch, roll, yaw - q, p, r
    gyro: [f32; 3],
    motor_num: u8,
    #[br(count = motor_num)]
    motor_rpm: Vec<f32>,
}

impl LiftoffPacket {
    /// Returns the angular velocity p, q, r
    /// These are the roll rate, pitch rate, and yaw rate respectively
    pub fn pqr(&self) -> Vector3<f32> {
        Vector3::new(self.gyro[1], self.gyro[0], self.gyro[2])
    }

    /// Returns the attitude quaternion in the NED frame
    pub fn attitude_quaternion(&self) -> UnitQuaternion<f32> {
        // The quaternion is in the Unity right-up-forward (RUF) frame
        let ruf_quat = UnitQuaternion::from_quaternion(Quaternion::new(
            self.attitude[3],
            self.attitude[0],
            self.attitude[1],
            self.attitude[2],
        ));
        // Flip the handedness by negating the Z component of the RUF quaternion.
        let flipped_quat = Quaternion::new(ruf_quat.w, ruf_quat.i, ruf_quat.j, -ruf_quat.k);

        // Define a 90-degree rotation around the Y-axis to align X (Right) to X (North)
        let rotation_y =
            UnitQuaternion::from_axis_angle(&Vector3::y_axis(), std::f32::consts::FRAC_PI_2);

        // Define a -90-degree rotation around the X-axis to align Z (Forward) to Z (Down)
        let rotation_x =
            UnitQuaternion::from_axis_angle(&Vector3::x_axis(), -std::f32::consts::FRAC_PI_2);

        // Combine the handedness-adjusted quaternion with the rotation transformations
        // Apply the Y rotation first, then the X rotation
        rotation_x * rotation_y * UnitQuaternion::new_normalize(flipped_quat)
    }

    /// Translate Unity coordinates to NED coordinates
    pub fn position(&self) -> Vector3<f32> {
        Vector3::new(self.position[2], self.position[0], -self.position[1])
    }
}

async fn feedback_loop(
    liftoff_config: LiftoffQuadrotorConfig,
    data_lock: Arc<Mutex<Option<LiftoffPacket>>>,
) -> Result<(), SimulationError> {
    let mut current_wait = Duration::from_secs(0);
    let mut delay = Duration::from_secs(2);
    let max_wait = liftoff_config.connection_timeout;
    let max_delay = liftoff_config.max_retry_delay;

    loop {
        let mut buf = [0; 128];
        match UdpSocket::bind(liftoff_config.ip_address.to_string()) {
            Ok(socket) => {
                socket
                    .set_read_timeout(Some(Duration::from_secs(60)))
                    .map_err(|e| SimulationError::OtherError(e.to_string()))?;
                match socket.recv_from(&mut buf) {
                    Ok((len, _)) => {
                        let mut cursor = std::io::Cursor::new(&buf[..len]);
                        // TODO: more robust handling of packet parsing errors during resets
                        if let Ok(sample) = LiftoffPacket::read(&mut cursor) {
                            let mut data_lock = data_lock.lock().unwrap();
                            *data_lock = Some(sample);
                        }
                        current_wait = Duration::from_secs(0);
                        delay = Duration::from_secs(2);
                    }
                    Err(e) => {
                        if current_wait >= max_wait {
                            return Err(SimulationError::OtherError(format!(
                                "Bind loop exceeded max wait time {}",
                                e.to_string(),
                            )));
                        }
                        current_wait += delay;
                        sleep(
                            delay + Duration::from_millis((rand::random::<f64>() * 1000.0) as u64),
                        )
                        .await;
                        delay = (delay * 2).min(max_delay);
                    }
                }
            }
            Err(e) => {
                return Err(SimulationError::OtherError(format!(
                    "Bind loop exceeded max wait time {}",
                    e.to_string()
                )));
            }
        }
    }
}

/// Scale a value from a given range to a new range
/// # Example
/// ```
/// let value = 0.0;
/// let min = -10.0;
/// let max = 10.0;
/// let scaled_value = normalize(value, min, max);
/// assert_eq!(scaled_value, 0.5);
/// ```
fn normalize(value: f32, min: f32, max: f32) -> f32 {
    (value - min) / (max - min)
}

// TODO clean up all the type casts
// TODO assert value is normalized
fn scale_to_rc_command(value: f32, min: i32, max: i32) -> i32 {
    let value = value.clamp(0.0, 1.0);
    // assert value is in the range 0 to 1
    // Scale normalized value to the range between min and max
    let range = max - min;
    let scaled_value = min as f32 + value * range as f32;
    scaled_value as i32
    // // Calculate the offset from the center within the range
    // (center + scaled_value as i32) as i32
}

fn scale_to_rc_command_with_center(value: f32, min: f32, center: f32, max: f32) -> i32 {
    let value = value.clamp(0.0, 1.0);
    let output = if value < 0.5 {
        // Map to the lower half (min to center)
        center - (center - min) * (0.5 - value) * 2.0
    } else {
        // Map to the upper half (center to max)
        center + (max - center) * (value - 0.5) * 2.0
    };
    output as i32
}

/// Scale a thrust value to a throttle command
/// Throttle is inverted from Xinput to Liftoff
/// # Example
/// ```
/// let thrust = 0.0;
/// let throttle = scale_throttle(thrust);
/// assert_eq!(throttle, 0);
/// ```
fn scale_throttle(thrust: f32) -> i32 {
    // thrust is inverted from Xinput to Liftoff
    -scale_to_rc_command(thrust, -32768, 32768)
}

fn scale_control(value: f32) -> i32 {
    scale_to_rc_command_with_center(value, -32768 as f32, 32767 as f32, 0.0)
}

#[rustfmt::skip]
const MOTOR_MIXING_MATRIX: Matrix4<f32> = Matrix4::new(
    1.0, 1.0, 1.0, 1.0,
    -1.0, 1.0, -1.0, 1.0,
    -1.0, -1.0, 1.0, 1.0,
    1.0, -1.0, -1.0, 1.0,
);
