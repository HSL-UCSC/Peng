use binrw::{binrw, BinRead, BinWrite};
use cyber_rc::{cyberrc, Writer};
use nalgebra::{Matrix4, Quaternion, UnitQuaternion, Vector3};
use peng_quad::config::{LiftoffQuadrotorConfig, SimulationConfig};
use peng_quad::quadrotor::{QuadrotorInterface, QuadrotorState};
use peng_quad::SimulationError;
use std::net::UdpSocket;
use tokio::sync::watch;
use tokio::time::Duration;

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
    pub initial_state: Option<QuadrotorState>,
    /// Simulation time step in seconds
    pub time_step: f32,
    /// Config
    pub simulation_config: SimulationConfig,
    /// Configured physical parameters
    pub config: LiftoffQuadrotorConfig,
    /// Previous Thrust
    pub previous_thrust: f32,
    /// Previous Torque
    pub previous_torque: Vector3<f32>,
    /// Quadrotor sample mutex
    pub producer: watch::Sender<Option<Vec<u8>>>,
    /// Quadrotor sample mutex
    pub consumer: watch::Receiver<Option<Vec<u8>>>,
}

impl LiftoffQuad {
    pub fn new(
        time_step: f32,
        simulation_config: SimulationConfig,
        config: LiftoffQuadrotorConfig,
    ) -> Result<Self, SimulationError> {
        let (producer, mut consumer) = watch::channel(None::<Vec<u8>>);
        let config_clone = config.clone();
        let producer_clone = producer.clone();
        tokio::spawn(async move {
            let _ = feedback_loop_fast(&config_clone.ip_address, producer_clone).await;
        });
        // Open a serial port to communicate with the quadrotor if one is specified
        let writer: Option<Writer> = match config.clone().serial_port {
            Some(port) => {
                let mut writer = Writer::new(port.to_string(), config.baud_rate).map_err(|e| {
                    SimulationError::OtherError(format!(
                        "Failed to open SerialPort {:?}",
                        e.to_string()
                    ))
                })?;
                let start_time = std::time::Instant::now();
                // Zero throttle to arm the quadrotor in Liftoff
                println!("Arming Drone");
                writer
                    .serial_port
                    .set_timeout(Duration::from_millis(50))
                    .map_err(|e| {
                        SimulationError::OtherError(format!(
                            "Failed to set timeout {:?}",
                            e.to_string()
                        ))
                    })?;
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
                    std::thread::sleep(Duration::from_millis(100));
                }

                Some(writer)
            }
            None => {
                println!("No serial port specified, writing to temp file");
                None
            }
        };
        Ok(Self {
            writer,
            state: QuadrotorState::default(),
            previous_state: QuadrotorState::default(),
            initial_state: None,
            simulation_config,
            config,
            time_step,
            previous_thrust: 0.0,
            previous_torque: Vector3::zeros(),
            consumer,
            producer,
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

    fn ensure_initial_is_set(&mut self, state: QuadrotorState) {
        self.initial_state.get_or_insert(state);
    }
}

impl QuadrotorInterface for LiftoffQuad {
    fn control(
        &mut self,
        step_number: usize,
        thrust: f32,
        torque: &Vector3<f32>,
    ) -> Result<(), SimulationError> {
        if step_number
            % (self.simulation_config.simulation_frequency
                / self.simulation_config.control_frequency)
            == 0
        {
            // Given thrust and torque, calculate the control inputs
            // Clamp thrust and torque control inputs
            let max_thrust = self.max_thrust();
            // let thrust = thrust.clamp(0.0, 5.0);
            let max_torque = self.max_torque();
            println!("Pitch Torque: {:?}", torque.y);
            let torque = Vector3::new(
                torque.x.clamp(-max_torque.x, max_torque.x),
                torque.y.clamp(-10.0, 10.0),
                // torque.z.clamp(-max_torque.z, max_torque.z),
                torque.z.clamp(-6.0, 6.0),
            );

            // Normalize inputs
            let normalized_thrust = normalize(thrust, 0.0, 5.0);
            let normalized_roll = normalize(torque.x, -max_torque.x, max_torque.x);
            let normalized_pitch = normalize(torque.y, -12.0, 12.0);
            let normalized_yaw = normalize(torque.z, -6.0, 6.0);
            println!(
                "Thrust: {:?}, Normalized Thrust: {:?} Timestamp: {:?}",
                thrust, normalized_thrust, self.state.time
            );

            // Scale to RC commands
            let throttle_command = scale_throttle(normalized_thrust);
            let aileron_command = scale_control(normalized_roll);
            let elevator_command = -scale_control(normalized_pitch);
            let rudder_command = scale_control(normalized_yaw);

            let mut cyberrc_data = cyberrc::RcData {
                throttle: throttle_command,
                // elevator: 0, // elevator_command,
                aileron: aileron_command,
                elevator: elevator_command,
                rudder: rudder_command,
                arm: 0,
                mode: 0,
            };
            if let Some(writer) = &mut self.writer {
                writer
                    .write(&mut cyberrc_data)
                    .map_err(|e| SimulationError::OtherError(e.to_string()))?;
            }
        }
        Ok(())
    }

    /// Observe the current state of the quadrotor
    /// Returns a tuple containing the position, velocity, orientation, and angular velocity of the quadrotor.
    fn observe(&mut self) -> Result<QuadrotorState, SimulationError> {
        // TODO: if there is not a new packet, return the old state
        if !self
            .consumer
            .has_changed()
            .map_err(|e| SimulationError::OtherError(e.to_string()))?
        {
            return Ok(self.state.clone());
        }
        self.previous_state = self.state.clone();
        let packet = self.consumer.borrow_and_update().clone();
        let sample = match packet {
            Some(packet) => {
                let mut cursor = std::io::Cursor::new(packet);
                LiftoffPacket::read(&mut cursor)
                    .map_err(|e| SimulationError::OtherError(e.to_string()))
            }
            None => Err(SimulationError::OtherError("No packet".to_string())),
        }?;

        let initial_state = self.initial_state.get_or_insert_with(|| {
            QuadrotorState {
                time: sample.timestamp,
                position: sample.position(),
                velocity: Vector3::zeros(),
                acceleration: Vector3::zeros(),
                // Store the initial yaw as the initial attitude so that we can adjust for it on
                // subsequent samples
                orientation: (|| -> UnitQuaternion<f32> {
                    let attitude = sample.attitude_quaternion();
                    println!("Initial Yaw: {:?}", attitude.euler_angles().2.to_degrees());
                    UnitQuaternion::new_normalize(Quaternion::new(
                        (1.0 - (attitude.k * attitude.k)).sqrt(),
                        0.0,
                        0.0,
                        -attitude.k,
                    ))
                })(),
                // orientation: sample.attitude_quaternion(),
                angular_velocity: Vector3::zeros(),
            }
        });

        // Adjust the sample by the initial state
        // Adjust for the initial yaw - we take the starting yaw of the vehicle to be 0
        let attitude_quaternion =
            initial_state.orientation.clone().conjugate() * sample.attitude_quaternion();
        // Asjut fot the initial position - we take the starting location of the vehicle to be the
        // origin
        let mut sample_position = sample.position();
        sample_position[0] -= initial_state.position[0];
        sample_position[1] -= initial_state.position[1];
        sample_position[2] -= initial_state.position[2];

        // Calculate the body-frame velocity by rotating the inertial velocity using the
        // attitude quaternion.
        let v_body = attitude_quaternion.transform_vector(
            &((sample_position - self.previous_state.position)
                / (sample.timestamp - self.previous_state.time)),
        );
        // sample_position[2] = -sample_position[2];

        let omega_body = sample.pqr();
        let acceleration_body = self.body_acceleration(attitude_quaternion, omega_body, v_body);
        self.state = QuadrotorState {
            time: sample.timestamp as f32,
            position: sample_position,
            velocity: v_body,
            acceleration: acceleration_body,
            orientation: attitude_quaternion,
            angular_velocity: omega_body,
        };
        Ok(self.state.clone())
    }

    fn max_thrust(&self) -> f32 {
        self.config.max_thrust_kg
    }

    /// Calculate all maximum torques and return them as a tuple
    fn max_torque(&self) -> Vector3<f32> {
        let motor_thrust = self.max_thrust() / 4.0;
        let max_rp_torque = 2.0 * 0.65 * motor_thrust;
        let yaw_torque = 8.0 * motor_thrust;
        Vector3::new(max_rp_torque, max_rp_torque, yaw_torque)
    }

    fn read_imu(&self) -> Result<(Vector3<f32>, Vector3<f32>), SimulationError> {
        Ok((self.state.acceleration, self.state.angular_velocity))
    }
}

// TODO: configure packet based on the content of the Liftoff config file
#[binrw]
#[brw(little)]
#[derive(Debug, Default, Clone)]
pub struct LiftoffPacket {
    timestamp: f32,
    // x, y, z
    position: [f32; 3],
    // x, y, z, w
    attitude: [f32; 4],
    // pitch, roll, yaw - q, p, r
    gyro: [f32; 3],
    // throttle, yaw, pitch, roll
    input: [f32; 4],
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
        // Cast the attitude as a unit quaternion, and flip the handedness by inverting the z
        // axis.
        let r = UnitQuaternion::from_quaternion(
            Quaternion::new(
                self.attitude[3],
                self.attitude[0],
                self.attitude[1],
                -self.attitude[2],
            )
            .normalize(),
        );
        let euler = r.euler_angles();
        UnitQuaternion::from_euler_angles(euler.2, euler.0, -euler.1)
    }

    /// TODO: this is NEU
    /// Translate Unity coordinates in RUF to NED coordinates
    pub fn position(&self) -> Vector3<f32> {
        Vector3::new(self.position[2], self.position[0], self.position[1])
    }
}

async fn feedback_loop_fast(
    address: &str,
    tx: watch::Sender<Option<Vec<u8>>>,
) -> Result<(), SimulationError> {
    let socket = tokio::net::UdpSocket::bind(address.to_string())
        .await
        .map_err(|e| SimulationError::OtherError(e.to_string()))?;
    let mut buf = [0; 2048];
    loop {
        match socket.recv_from(&mut buf).await {
            Ok((len, _)) => {
                let _ = tx
                    .send(Some(buf[..len].to_vec()))
                    .map_err(|e| SimulationError::OtherError(e.to_string()))?;
            }
            Err(_) => (),
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
    scale_to_rc_command_with_center(value, -32768 as f32, 0.0, 32737 as f32)
}

#[rustfmt::skip]
const MOTOR_MIXING_MATRIX: Matrix4<f32> = Matrix4::new(
    1.0, 1.0, 1.0, 1.0,
    -1.0, 1.0, -1.0, 1.0,
    -1.0, -1.0, 1.0, 1.0,
    1.0, -1.0, -1.0, 1.0,
);

#[cfg(test)]
mod tests {

    use super::*;
    use std::f32::consts::FRAC_PI_2;

    /// Checks if two `Vector3` instances are element-wise close within the given tolerances.
    fn is_close(this: &Vector3<f32>, other: &Vector3<f32>, rel_tol: f32, abs_tol: f32) -> bool {
        fn is_close_scalar(a: f32, b: f32, rel_tol: f32, abs_tol: f32) -> bool {
            (a - b).abs() <= (rel_tol * b.abs()).max(abs_tol)
        }

        is_close_scalar(this.x, other.x, rel_tol, abs_tol)
            && is_close_scalar(this.y, other.y, rel_tol, abs_tol)
            && is_close_scalar(this.z, other.z, rel_tol, abs_tol)
    }

    #[test]
    fn test_acceleration_body() {
        let quad = LiftoffQuad::new(
            0.01,
            SimulationConfig::default(),
            LiftoffQuadrotorConfig::default(),
        )
        .unwrap();
        let omega = Vector3::zeros();
        let v = Vector3::zeros();

        let q = UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0);
        let a = quad.body_acceleration(q, omega, v);
        assert_eq!(a, Vector3::new(0.0, 0.0, quad.config.gravity));

        // Roll onto the right side, gravity should align with y direction to the "east" side of
        // the drone
        let q = UnitQuaternion::from_euler_angles(-FRAC_PI_2, 0.0, 0.0);
        let a = quad.body_acceleration(q, omega, v);
        assert!(is_close(
            &a,
            &Vector3::new(0.0, quad.config.gravity, 0.0),
            1e-6,
            1e-6
        ));

        // Pitch forward 90 degrees, gravity should align with x direction down the nose
        let q = UnitQuaternion::from_euler_angles(0.0, FRAC_PI_2, 0.0);
        let a = quad.body_acceleration(q, omega, v);
        assert!(is_close(
            &a,
            &Vector3::new(quad.config.gravity, 0.0, 0.0),
            1e-6,
            1e-6
        ));
    }
}
