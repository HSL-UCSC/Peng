use binrw::{binrw, BinRead};
use cyber_rc::{cyberrc, Writer};
use nalgebra::{AbstractRotation, Matrix4, Quaternion, Unit, UnitQuaternion, Vector3};
use peng_quad::config::{self, LiftoffQuadrotorConfig, QuadrotorConfig, QuadrotorConfigurations};
use peng_quad::quadrotor::{QuadrotorInterface, QuadrotorState};
use peng_quad::SimulationError;
use rand;
use serialport::{available_ports, SerialPort, SerialPortBuilder, SerialPortType};
use std::net::UdpSocket;
use std::sync::Arc;
use tokio::sync::Mutex;
use tokio::time::{sleep, Duration};

/// Represents a quadrotor in the game Liftoff
/// # Example
/// ```
/// let quad = LiftoffQuad{ }
/// ```
pub struct LiftoffQuad {
    /// The serial writer to communicate with the quadrotor
    pub writer: Writer,
    /// The current state of the quadrotor
    pub state: QuadrotorState,
    /// The last state of the quadrotor
    pub last_state: QuadrotorState,
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

impl QuadrotorInterface for LiftoffQuad {
    fn control(
        &mut self,
        _: usize,
        thrust: f32,
        torque: &Vector3<f32>,
    ) -> Result<(), SimulationError> {
        // Given thrust and torque, calculate the control inputs
        let max_torques = self.config.max_torques();

        // Normalize inputs
        let normalized_thrust = normalize(thrust, 0.0, self.config.max_thrust_kg);
        let normalized_roll = normalize(torque[0], -max_torques.0, max_torques.0);
        let normalized_pitch = normalize(torque[2], -max_torques.1, max_torques.1);
        let normalized_yaw = normalize(torque[3], -max_torques.2, max_torques.2);

        // Scale to RC commands
        let throttle_command = scale_throttle(normalized_thrust);
        let aileron_command = scale_control(normalized_roll);
        let elevator_command = scale_control(normalized_pitch);
        let rudder_command = scale_control(normalized_yaw);

        let cyberrc_data = cyberrc::RcData {
            throttle: throttle_command,
            aileron: aileron_command,
            elevator: elevator_command,
            rudder: rudder_command,
            arm: 0,
            mode: 0,
        };
        self.writer
            .write(cyberrc_data)
            .map_err(|e| SimulationError::OtherError(e.to_string()))?;
        Ok(())
    }

    /// Observe the current state of the quadrotor
    /// Returns a tuple containing the position, velocity, orientation, and angular velocity of the quadrotor.
    fn observe(&mut self) -> Result<QuadrotorState, SimulationError> {
        let mut data_lock = tokio::runtime::Handle::current().block_on(self.shared_data.lock());
        while let Some(sample) = data_lock.take() {
            // update the last state value
            self.last_state = self.state.clone();
            let attitude_quaternion = sample.attitude_quaternion();
            // The position is in Unity inertial coordinates, so we transform to NED inertial
            // coordinates.
            let sample_position = sample.position();
            // Calculate the body-frame velocity by rotating the inertial velocity using the
            // attitude quaternion.
            let v_body = attitude_quaternion
                .transform_vector(&((sample_position - self.last_state.position) / self.time_step));

            let omega_body = sample.pqr();
            let acceleration_body = self.body_acceleration(attitude_quaternion, omega_body, v_body);
            self.state = QuadrotorState {
                position: sample_position,
                velocity: v_body,
                acceleration: acceleration_body,
                orientation: attitude_quaternion,
                angular_velocity: omega_body,
            };
        }
        Ok(self.state.clone())
    }

    fn read_imu(&self) -> Result<(Vector3<f32>, Vector3<f32>), SimulationError> {
        todo!("")
    }
}

impl LiftoffQuad {
    pub fn new(time_step: f32, config: LiftoffQuadrotorConfig) -> Result<Self, SimulationError> {
        let shared_data: Arc<Mutex<Option<LiftoffPacket>>> = Arc::new(Mutex::new(None));
        let shared_data_clone = Arc::clone(&shared_data);
        let config_clone = config.clone();
        // tokio::spawn(async move {
        //     let _ = feedback_loop(config_clone, shared_data_clone).await;
        // });
        let writer = Writer::new("COM3".to_string(), 115200)
            .map_err(|e| SimulationError::OtherError(e.to_string()))?;
        Ok(Self {
            writer: writer,
            state: QuadrotorState::default(),
            last_state: QuadrotorState::default(),
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
        // Step 1: Calculate thrust acceleration (negative Z in NED body frame)
        let thrust_acceleration = Vector3::new(0.0, 0.0, -self.previous_thrust / self.config.mass);

        // Step 2: Rotate the gravity vector to the body frame using the quaternion `apply`
        let gravity_inertial = Vector3::new(0.0, 0.0, self.config.gravity); // NED gravity vector in inertial frame
        let gravity_body = q_inertial_to_body.transform_vector(&gravity_inertial); // Rotate gravity vector to body frame

        // Step 3: Calculate rotational (Coriolis) effects
        let rotational_acceleration = omega_body.cross(&velocity_body);

        // Step 4: Combine all terms
        thrust_acceleration + gravity_body - rotational_acceleration
    }
}

// TODO: configure packet based on the content of the Liftoff config file
#[binrw]
#[br(little)]
#[derive(Debug)]
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
                    .set_read_timeout(Some(Duration::from_secs(15)))
                    .map_err(|e| SimulationError::OtherError(e.to_string()))?;
                match socket.recv_from(&mut buf) {
                    Ok((len, _)) => {
                        let mut cursor = std::io::Cursor::new(&buf[..len]);
                        // TODO: more robust handling of packet parsing errors during resets
                        if let Ok(sample) = LiftoffPacket::read(&mut cursor) {
                            let mut data_lock = data_lock.lock().await;
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
fn scale_to_rc_command(value: f32, min: i32, max: i32, center: i32) -> i32 {
    // assert value is in the range 0 to 1
    // Scale normalized value to the range between min and max
    let range = max - min;
    let scaled_value = min as f32 + value * range as f32;

    // Calculate the offset from the center within the range
    let center_offset = scaled_value - (min as f32 + (range as f32) / 2.0);

    // Adjust the result around the center
    (center as f32 + center_offset) as i32
    // scaled_value.clamp(max_output, min_output) as i32
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
    scale_to_rc_command(thrust, 32768, -32768, 32768)
}

fn scale_control(value: f32) -> i32 {
    scale_to_rc_command(value, -32768, 32767, 0)
}

#[rustfmt::skip]
const MOTOR_MIXING_MATRIX: Matrix4<f32> = Matrix4::new(
    1.0, 1.0, 1.0, 1.0,
    -1.0, 1.0, -1.0, 1.0,
    -1.0, -1.0, 1.0, 1.0,
    1.0, -1.0, -1.0, 1.0,
);
