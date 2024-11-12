use binrw::{binrw, BinRead};
use cyber_rc::{cyberrc, Writer};
use nalgebra::{Matrix4, Quaternion, Unit, UnitQuaternion, Vector3};
use peng_quad::config::{LiftoffConfiguration, QuadrotorConfig};
use peng_quad::quadrotor::{QuadrotorInterface, QuadrotorState};
use peng_quad::SimulationError;
use rand;
use serialport::{available_ports, SerialPort, SerialPortBuilder, SerialPortType};
use std::net::UdpSocket;
use std::sync::Arc;
use tokio::sync::Mutex;
use tokio::time::{sleep, Duration};

#[rustfmt::skip]
const MOTOR_MIXING_MATRIX: Matrix4<f32> = Matrix4::new(
    1.0, 1.0, 1.0, 1.0,
    -1.0, 1.0, -1.0, 1.0,
    -1.0, -1.0, 1.0, 1.0,
    1.0, -1.0, -1.0, 1.0,
);

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
    pub config: QuadrotorConfig,
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
        step_number: usize,
        config: &peng_quad::config::Config,
        thrust: f32,
        torque: &Vector3<f32>,
    ) -> Result<(), SimulationError> {
        // Given thrust and torque, calculate the control inputs
        self.writer
            .write(control_inputs_to_rc_commands(
                config.quadrotor.clone(),
                thrust,
                torque,
            ))
            .map_err(|e| SimulationError::OtherError(e.to_string()))?;
        Ok(())
    }

    /// Observe the current state of the quadrotor
    /// # Returns
    /// A tuple containing the position, velocity, orientation, and angular velocity of the quadrotor
    fn observe(&mut self) -> Result<QuadrotorState, SimulationError> {
        let mut data_lock = tokio::runtime::Handle::current().block_on(self.shared_data.lock());
        while let Some(sample) = data_lock.take() {
            // update the last state value
            self.last_state = self.state.clone();
            // store the sample values into templ variables
            let position = Vector3::from_row_slice(&sample.position);
            let velocity = (position - self.last_state.position) / self.time_step;
            let orientation = sample.attitude_quaternion();
            let delta_q = self.last_state.orientation * orientation.conjugate();
            let angle = 2.0 * delta_q.scalar().acos();
            let axis = delta_q
                .axis()
                .unwrap_or(Unit::new_normalize(Vector3::<f32>::zeros()));
            let angular_velocity = axis.scale(angle / self.time_step);
            self.state = QuadrotorState {
                position,
                velocity,
                orientation,
                angular_velocity,
            };
            return Ok(QuadrotorState {
                position,
                velocity,
                orientation,
                angular_velocity,
            });
        }
        todo!("implement state feedback from Liftoff UDP")
    }
}

impl LiftoffQuad {
    fn new(
        time_step: f32,
        vehicle_config: QuadrotorConfig,
        liftoff_config: LiftoffConfiguration,
    ) -> Result<Self, SimulationError> {
        let shared_data = Arc::new(Mutex::new(None));
        let shared_data_clone = Arc::clone(&shared_data);
        tokio::spawn(async move {
            let _ = feedback_loop(
                &liftoff_config.ip_address.to_string(),
                liftoff_config.connection_timeout,
                liftoff_config.max_retry_delay,
                shared_data_clone,
            )
            .await;
        });
        let writer = Writer::new("COM3".to_string(), 115200)
            .map_err(|e| SimulationError::OtherError(e.to_string()))?;
        Ok(Self {
            writer: writer,
            state: QuadrotorState::default(),
            last_state: QuadrotorState::default(),
            config: vehicle_config.clone(),
            time_step,
            previous_thrust: 0.0,
            previous_torque: Vector3::zeros(),
            shared_data: Arc::new(Mutex::new(None)),
        })
    }
}

// TODO: configure packet based on the content of the Liftoff config file
#[binrw]
#[br(little)]
#[derive(Debug)]
struct LiftoffPacket {
    timestamp: f32,
    position: [f32; 3],
    // TODO: binrw read to quaternion
    attitude: [f32; 4],
    motor_num: u8,
    #[br(count = motor_num)]
    motor_rpm: Vec<f32>,
}

impl LiftoffPacket {
    pub fn attitude_quaternion(&self) -> UnitQuaternion<f32> {
        UnitQuaternion::from_quaternion(Quaternion::new(
            self.attitude[0],
            self.attitude[1],
            self.attitude[2],
            self.attitude[3],
        ))
    }
}

fn quaternion_ruf_to_ned(ruf_quat: UnitQuaternion<f32>) -> UnitQuaternion<f32> {
    // Step 1: Flip the handedness by negating the Z component of the RUF quaternion.
    let flipped_quat = Quaternion::new(ruf_quat.w, ruf_quat.i, ruf_quat.j, -ruf_quat.k);

    // Step 2: Define a 90-degree rotation around the Y-axis to align X (Right) to X (North)
    let rotation_y =
        UnitQuaternion::from_axis_angle(&Vector3::y_axis(), std::f32::consts::FRAC_PI_2);

    // Step 3: Define a -90-degree rotation around the X-axis to align Z (Forward) to Z (Down)
    let rotation_x =
        UnitQuaternion::from_axis_angle(&Vector3::x_axis(), -std::f32::consts::FRAC_PI_2);

    // Step 4: Combine the handedness-adjusted quaternion with the rotation transformations
    // Apply the Y rotation first, then the X rotation
    rotation_x * rotation_y * UnitQuaternion::new_normalize(flipped_quat)
}

/// Translate Unity coordinates to NED coordinates
pub fn vector3_ruf_to_ned(unity_position: Vector3<f32>) -> Vector3<f32> {
    Vector3::new(unity_position[2], unity_position[0], -unity_position[1])
}

async fn feedback_loop(
    address: &str,
    data_lock: Arc<Mutex<Option<LiftoffPacket>>>,
) -> Result<(), SimulationError> {
    let mut current_wait = Duration::from_secs(0);
    let mut delay = Duration::from_secs(2);
    let max_wait = Duration::from_secs(60 * 5);
    let max_delay = Duration::from_secs(30);

    loop {
        let mut buf = [0; 128];
        match UdpSocket::bind(address) {
            Ok(socket) => {
                socket
                    .set_read_timeout(Some(Duration::from_secs(15)))
                    .map_err(|e| SimulationError::OtherError(e.to_string()))?;
                match socket.recv_from(&mut buf) {
                    Ok((len, _)) => {
                        let mut cursor = std::io::Cursor::new(&buf[..len]);
                        // TODO: more robust handling of packet parsing errors during resets
                        if let Ok(sample) = LiftoffPacket::read(&mut cursor) {
                            // TODO: RUF to NED orientation
                            // TODO: apply transformation to position vector
                            let mut data_lock = data_lock.lock().await;
                            *data_lock = Some(sample);
                        }
                        current_wait = Duration::from_secs(0);
                        delay = Duration::from_secs(2);
                    }
                    Err(e) => {
                        if current_wait >= max_wait {
                            return Err(SimulationError::OtherError(
                                "Bind loop exceeded max wait time".into(),
                            ));
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
                return Err(SimulationError::OtherError(
                    "Bind loop exceeded max wait time".into(),
                ));
            }
        }
    }
    Ok(())
}

fn normalize(value: f32, min: f32, max: f32) -> f32 {
    (value - min) / (max - min)
}

fn scale_to_rc_command(value: f32, min_output: f32, max_output: f32, center: f32) -> i32 {
    let scaled_value = value * (max_output - center) + center;
    scaled_value.clamp(min_output, max_output) as i32
}

fn scale_throttle(thrust: f32) -> i32 {
    scale_to_rc_command(thrust, 1000.0, 2000.0, 1500.0)
}

fn scale_control(value: f32) -> i32 {
    scale_to_rc_command(value, 1000.0, 2000.0, 1500.0)
}

fn control_inputs_to_rc_commands(
    quadrotor_config: QuadrotorConfig,
    thrust: f32,
    torque: &Vector3<f32>,
) -> cyberrc::RcData {
    let roll_torque = torque[0];
    let pitch_torque = torque[1];
    let yaw_torque = torque[2];

    // TODO: what cases will we have this be non zero
    let max_torques = quadrotor_config.max_torques();

    // Normalize inputs
    let normalized_thrust = normalize(thrust, 0.0, quadrotor_config.max_thrust_kg);
    let normalized_roll = normalize(roll_torque, -max_torques.0, max_torques.0);
    let normalized_pitch = normalize(pitch_torque, -max_torques.1, max_torques.1);
    let normalized_yaw = normalize(yaw_torque, -max_torques.2, max_torques.2);

    // Scale to RC commands
    let throttle_command = scale_throttle(normalized_thrust);
    let aileron_command = scale_control(normalized_roll);
    let elevator_command = scale_control(normalized_pitch);
    let rudder_command = scale_control(normalized_yaw);

    cyberrc::RcData {
        throttle: throttle_command,
        aileron: aileron_command,
        elevator: elevator_command,
        rudder: rudder_command,
        arm: 0,
        mode: 0,
    }
}
