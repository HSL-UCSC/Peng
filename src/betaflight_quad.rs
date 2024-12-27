use binrw::{binrw, BinRead};
use cyber_rc::{cyberrc, CyberRCMessageType, Writer};
use nalgebra::{Matrix4, Quaternion, Rotation3, UnitQuaternion, Vector3};
use peng_quad::config;
use peng_quad::quadrotor::{QuadrotorInterface, QuadrotorState};
use peng_quad::SimulationError;
use std::time::Duration;
use tokio::sync::watch;

/// Represents a physical quadrotor running a Betaflight controller.
/// # Example
/// ```
/// let quad = BetaflightQuad{ }
/// ```
pub struct BetaflightQuad {
    /// The serial writer to communicate with the quadrotor
    pub writer: Option<Writer>,
    /// The current state of the quadrotor
    pub state: QuadrotorState,
    /// The last state of the quadrotor
    pub previous_state: QuadrotorState,
    /// Initial State
    pub initial_state: Option<QuadrotorState>,
    /// Config
    pub simulation_config: config::SimulationConfig,
    /// Configured physical parameters
    pub config: config::Betaflight,
    /// Previous Thrust
    pub previous_thrust: f32,
    /// Quadrotor sample mutex
    pub consumer: watch::Receiver<Option<Vec<u8>>>,
}

impl BetaflightQuad {
    pub fn new(
        simulation_config: config::SimulationConfig,
        config: config::Betaflight,
    ) -> Result<Self, SimulationError> {
        let (producer, consumer) = watch::channel(None::<Vec<u8>>);
        let config_clone = config.clone();
        let producer_clone = producer.clone();
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
                writer
                    .serial_port
                    .set_timeout(Duration::from_millis(50))
                    .map_err(|e| {
                        SimulationError::OtherError(format!(
                            "Failed to set timeout {:?}",
                            e.to_string()
                        ))
                    })?;
                Some(writer)
            }
            None => {
                println!("No serial port specified, writing to temp file");
                None
            }
        };
        tokio::spawn(async move {
            let _ = feedback_loop(&config_clone.vicon_address, producer_clone).await;
        });
        // Open a serial port to communicate with the quadrotor if one is specified
        Ok(Self {
            writer,
            state: QuadrotorState::default(),
            previous_state: QuadrotorState::default(),
            initial_state: None,
            simulation_config,
            config,
            previous_thrust: 0.0,
            consumer,
        })
    }

    fn euler_body_velocity(
        &self,
        p1: Vector3<f32>, // Inertial position a t_n
        p2: Vector3<f32>, // Inertial position at t_n+1
        q_ib: UnitQuaternion<f32>,
        dt: f32,
    ) -> Vector3<f32> {
        // Inertial velocity
        let v_inertial = (p2 - p1) / dt;

        // Transform to body frame
        let q_conjugate = q_ib.conjugate();
        let v_body = q_conjugate
            * UnitQuaternion::from_quaternion(Quaternion::from_parts(0.0, v_inertial))
            * q_ib;

        // Extract the vector part (x, y, z)
        Vector3::new(v_body.i, v_body.j, v_body.k)
    }

    fn rk4_body_velocity(
        &self,
        q_ib: UnitQuaternion<f32>,
        p1: Vector3<f32>, // Inertial position a t_n
        p2: Vector3<f32>, // Inertial position at t_n+1
        dt: f32,
    ) -> Vector3<f32> {
        let k1 = self.euler_body_velocity(p1, p2, q_ib, dt);

        // k2: velocity at t + 0.5 * dt (assume midpoint position for simplicity)
        let p_mid = p1 + 0.5 * (p2 - p1);
        let k2 = self.euler_body_velocity(p1, p_mid, q_ib, dt * 0.5);

        // k3: velocity at t + 0.5 * dt (another midpoint evaluation)
        let k3 = self.euler_body_velocity(p_mid, p2, q_ib, dt * 0.5);

        // k4: velocity at t + dt
        let k4 = self.euler_body_velocity(p2, p2 + (p2 - p1), q_ib, dt);

        // RK4 weighted sum
        (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0
    }

    // Function to calculate body-frame acceleration for NED coordinates
    fn rk4_body_acceleration(
        &self,
        v1: Vector3<f32>, // Linear velocity in body frame
        v2: Vector3<f32>, // Linear velocity in body frame
    ) -> Vector3<f32> {
        // Calculate thrust acceleration (negative Z in NED body frame)
        let thrust_acceleration = Vector3::new(
            0.0,
            0.0,
            -self.previous_thrust / self.config.quadrotor_config.mass,
        );
        let dt = (self.state.time - self.previous_state.time) as f32;
        let k1 = (v2 - v1) / dt;
        let k2 = (v2 - (v1 + 0.5 * k1 * dt)) / dt;
        let k3 = (v2 - (v1 + 0.5 * k2 * dt)) / dt;
        let k4 = (v2 - (v1 + k3 * dt)) / dt;

        (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0
    }

    fn angular_velocity(
        &self,
        q1: UnitQuaternion<f32>,
        q2: UnitQuaternion<f32>,
        dt: f32,
    ) -> Vector3<f32> {
        let delta_q = q2 * q1.conjugate();
        let imag = [delta_q.i, delta_q.j, delta_q.k];
        let scale = 2.0 / dt;

        // Angular velocity in body frame
        Vector3::new(imag[0] * scale, imag[1] * scale, imag[2] * scale)
    }

    fn rk4_angular_velocity(
        &self,
        q_t: UnitQuaternion<f32>,
        q_t_next: UnitQuaternion<f32>,
        dt: f32,
    ) -> Vector3<f32> {
        // k1: angular velocity at t
        let k1 = self.angular_velocity(q_t, q_t_next, dt);

        // k2: midpoint evaluation
        let q_mid = q_t.slerp(&q_t_next, 0.5); // Slerp to find midpoint quaternion
        let k2 = self.angular_velocity(q_t, q_mid, dt * 0.5);

        // k3: another midpoint evaluation
        let k3 = self.angular_velocity(q_mid, q_t_next, dt * 0.5);

        // k4: final evaluation
        let k4 = self.angular_velocity(q_t_next, q_t_next, dt);

        // Combine with RK4 formula
        Vector3::<f32>::new(
            (k1[0] + 2.0 * k2[0] + 2.0 * k3[0] + k4[0]) / 6.0,
            (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1]) / 6.0,
            (k1[2] + 2.0 * k2[2] + 2.0 * k3[2] + k4[2]) / 6.0,
        )
    }
}

struct Vec3(Vector3<f32>);

impl TryFrom<(f32, f32, f32)> for Vec3 {
    type Error = SimulationError;
    fn try_from(value: (f32, f32, f32)) -> Result<Self, Self::Error> {
        Ok(Vec3(Vector3::new(value.0, value.1, value.2)))
    }
}

impl QuadrotorInterface for BetaflightQuad {
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
            let _max_thrust = self.max_thrust();
            let thrust = thrust.clamp(-5.0, 10.0);
            let _max_torque = self.max_torque();
            // println!("Roll Torque: {:?}", torque.x);
            let torque = Vector3::new(
                torque.x.clamp(-10.0, 10.0),
                torque.y.clamp(-10.0, 10.0),
                torque.z.clamp(-20.0, 20.0),
            );

            // Normalize inputs
            let normalized_thrust = normalize(thrust, 0.0, 20.0);
            let normalized_roll = normalize(torque.x, -20.0, 20.0);
            let normalized_pitch = normalize(torque.y, -6.0, 6.0);
            let normalized_yaw = normalize(torque.z, -20.0, 20.0);

            // TODO: scale to PPM commands
            self.previous_thrust = normalized_thrust * self.max_thrust();
            if let Some(writer) = &mut self.writer {
                writer
                    .write(CyberRCMessageType::PpmUpdate(cyberrc::PpmUpdateAll {
                        line: 1,
                        channel_values: vec![1500, 1500, 1000, 1500],
                    }))
                    .map_err(|e| SimulationError::OtherError(e.to_string()))?;
            }
        }
        Ok(())
    }

    /// Observe the current state of the quadrotor
    /// Returns a tuple containing the position, velocity, orientation, and angular velocity of the quadrotor.
    fn observe(&mut self, step: usize) -> Result<QuadrotorState, SimulationError> {
        if !self
            .consumer
            .has_changed()
            .map_err(|e| SimulationError::OtherError(e.to_string()))?
        {
            return Ok(self.state.clone());
        }
        self.previous_state = self.state.clone();

        let sample = match self.consumer.borrow_and_update().clone() {
            Some(packet) => {
                let mut cursor = std::io::Cursor::new(packet);
                ViconPacket::read(&mut cursor)
                    .map_err(|e| SimulationError::OtherError(e.to_string()))
            }
            None => Err(SimulationError::OtherError("No packet".to_string())),
        }?;

        // Calculate the body-frame velocity by rotating the inertial velocity using the
        // attitude quaternion.
        let dt = 1.0 / self.simulation_config.simulation_frequency as f32;
        let v_body = self.rk4_body_velocity(
            sample.attitude_quaternion(),
            sample.position(),
            self.previous_state.position,
            dt,
        );
        let omega_body = self.rk4_angular_velocity(
            self.previous_state.orientation,
            sample.attitude_quaternion(),
            dt,
        );
        // Low-pass filter the angular velocity
        let alpha = 0.5;
        let omega_body = alpha * omega_body + (1.0 - alpha) * self.previous_state.angular_velocity;
        let acceleration_body = self.rk4_body_acceleration(self.previous_state.velocity, v_body);
        self.state = QuadrotorState {
            time: step as f32 * dt,
            position: sample.position(),
            velocity: v_body,
            acceleration: acceleration_body,
            orientation: sample.attitude_quaternion(),
            angular_velocity: omega_body,
        };
        Ok(self.state.clone())
    }

    fn max_thrust(&self) -> f32 {
        self.config.quadrotor_config.max_thrust_kg
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

    fn vehicle_configuration(&self) -> peng_quad::config::QuadrotorConfig {
        peng_quad::config::QuadrotorConfig {
            mass: self.config.quadrotor_config.mass,
            max_thrust_kg: self.config.quadrotor_config.max_thrust_kg,
            drag_coefficient: 0.0,
            inertia_matrix: self.config.quadrotor_config.inertia_matrix,
            arm_length_m: self.config.quadrotor_config.arm_length_m,
            yaw_torque_constant: self.config.quadrotor_config.yaw_torque_constant,
        }
    }
}

#[binrw]
#[brw(little)]
#[derive(Debug, Default, Clone)]
pub struct ViconPacket {
    pub frame: u32,            // Matches `I`: Unsigned int (4 bytes)
    pub num_items: u8,         // Matches `B`: Unsigned char (1 byte)
    pub header_id: u8,         // Matches `B`: Unsigned char (1 byte)
    pub header_data_size: u16, // Matches `H`: Unsigned short (2 bytes)
    #[br(count = 24)] // Matches `24s`: Fixed-length string
    pub item_name: Vec<u8>, // Use `String` if you know it's UTF-8 encoded
    pub x: f64,                // Matches `d`: Double (8 bytes)
    pub y: f64,                // Matches `d`: Double (8 bytes)
    pub z: f64,                // Matches `d`: Double (8 bytes)
    pub rx: f64,               // Matches `d`: Double (8 bytes)
    pub ry: f64,               // Matches `d`: Double (8 bytes)
    pub rz: f64,               // Matches `d`: Double (8 bytes)
}

impl ViconPacket {
    /// Returns the attitude quaternion in the NED frame
    pub fn attitude_quaternion(&self) -> UnitQuaternion<f32> {
        // FIXME: the rotation from Vicon to NED frame
        #[rustfmt::skip]
        let r_vicon_to_ned = Rotation3::from_matrix_unchecked({
            nalgebra::Matrix3::new(
        1.0, 0.0, 0.0,  // First row
        0.0, 1.0, 0.0,  // Second row
        0.0, 0.0, 1.0,  // Third row (Identity rotation)
            )
        });
        let orientation_v = Vector3::new(self.rx as f32, self.ry as f32, self.rz as f32);
        let orientation_ned = r_vicon_to_ned * orientation_v;
        UnitQuaternion::from_euler_angles(orientation_ned.x, orientation_ned.y, orientation_ned.z)
    }

    pub fn position(&self) -> Vector3<f32> {
        // FIXME: Get Vicon position mapping to NED
        Vector3::new(self.x as f32, self.y as f32, self.z as f32)
    }
}

async fn feedback_loop(
    address: &str,
    tx: watch::Sender<Option<Vec<u8>>>,
) -> Result<(), SimulationError> {
    let socket = tokio::net::UdpSocket::bind(address.to_string())
        .await
        .map_err(|e| SimulationError::OtherError(e.to_string()))?;
    let mut buf = [0; 2048];
    loop {
        if let Ok((len, _)) = socket.recv_from(&mut buf).await {
            tx.send(Some(buf[..len].to_vec()))
                .map_err(|e| SimulationError::OtherError(e.to_string()))?;
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
    -scale_to_rc_command(thrust, -32768, 32767)
}

fn scale_control(value: f32) -> i32 {
    scale_to_rc_command_with_center(value, -32768_f32, 0.0, 32767_f32)
}

#[rustfmt::skip]
const _MOTOR_MIXING_MATRIX: Matrix4<f32> = Matrix4::new(
    1.0, 1.0, 1.0, 1.0,
    -1.0, 1.0, -1.0, 1.0,
    -1.0, -1.0, 1.0, 1.0,
    1.0, -1.0, -1.0, 1.0,
);
