use crate::config::{LiftoffQuadrotorConfig, SimulationConfig};
use crate::quadrotor::{QuadrotorInterface, QuadrotorState, State};
use crate::SimulationError;
use binrw::{binrw, BinRead};
use cyber_rc::{cyberrc, Writer};
use nalgebra::{Quaternion, UnitQuaternion, Vector3};
use tokio::sync::watch;
use tokio::time::Duration;

/// Represents a quadrotor in the game Liftoff
pub struct LiftoffQuad {
    /// The serial writer to communicate with the quadrotor
    pub writer: Option<Writer>,
    /// The current state of the quadrotor
    pub state: QuadrotorState,
    /// The last state of the quadrotor
    pub previous_state: QuadrotorState,
    /// Initial State
    pub initial_state: Option<QuadrotorState>,
    /// Config
    pub simulation_config: SimulationConfig,
    /// Configured physical parameters
    pub config: LiftoffQuadrotorConfig,
    /// Previous Thrust
    pub previous_thrust: f32,
    /// Quadrotor sample mutex
    pub consumer: watch::Receiver<Option<Vec<u8>>>,
}

impl LiftoffQuad {
    #[allow(dead_code)]
    pub fn new(
        simulation_config: SimulationConfig,
        config: LiftoffQuadrotorConfig,
    ) -> Result<Self, SimulationError> {
        let (producer, consumer) = watch::channel(None::<Vec<u8>>);
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
                        .write(cyber_rc::CyberRCMessageType::RcData(cyberrc::RcData {
                            throttle: 32767,
                            aileron: 0,
                            elevator: 0,
                            rudder: 0,
                            arm: 1,
                            mode: 0,
                        }))
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
            previous_thrust: 0.0,
            consumer,
        })
    }

    /// Calculate the body-frame velocity of the quadrotor
    /// Given q_ib, the quaternion that rotates from the inertial frame to the body frame,
    /// p1, the position of the quadrotor at time t_n, p2, the position of the quadrotor at time t_n+1,
    /// and the time step dt.
    fn velocity_body(
        &self,
        q_ib: UnitQuaternion<f32>,
        p1: Vector3<f32>, // Inertial position a t_n
        p2: Vector3<f32>, // Inertial position at t_n+1
        dt: f32,
    ) -> Vector3<f32> {
        // Inertial velocity
        let v_inertial = (p2 - p1) / dt;
        // Transform to body frame
        // let q_inv = q_ib.inverse();
        q_ib.transform_vector(&v_inertial)
    }

    // Function to calculate body-frame acceleration for NED coordinates
    fn body_acceleration(
        &self,
        v_current: Vector3<f32>,  // Linear velocity in body frame
        v_previous: Vector3<f32>, // Linear velocity in body frame
        dt: f32,
    ) -> Vector3<f32> {
        (v_current - v_previous) / dt
    }

    fn angular_velocity(
        &self,
        q1: UnitQuaternion<f32>,
        q2: UnitQuaternion<f32>,
        dt: f32,
        threshold: f32,
    ) -> Vector3<f32> {
        let delta_q = q2 * q1.conjugate();
        let delta_q = UnitQuaternion::from_quaternion(delta_q.normalize());
        let (axis, angle) = delta_q.axis_angle().unwrap_or((Vector3::x_axis(), 0.0));
        if angle.abs() < threshold {
            return Vector3::zeros(); // Return zero angular velocity
        }
        axis.into_inner() * (angle / dt)
    }
}

impl QuadrotorInterface for LiftoffQuad {
    fn control(
        &mut self,
        step_number: usize,
        thrust: f32,
        torque: &Vector3<f32>,
    ) -> Result<Option<(f32, Vector3<f32>)>, SimulationError> {
        if step_number
            % (self.simulation_config.simulation_frequency
                / self.simulation_config.control_frequency)
            == 0
        {
            println!(
                "Thrust: {:?} Roll {:?} Pitch {:?} Yaw {:?} Timestamp: {:?}",
                thrust,
                torque.x,
                torque.y,
                torque.z,
                step_number as f32 * 1.0 / self.simulation_config.simulation_frequency as f32
            );
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

            // Scale to RC commands
            let throttle_command = scale_throttle(normalized_thrust);
            let aileron_command = scale_control(normalized_roll);
            let elevator_command = -scale_control(normalized_pitch);
            let rudder_command = scale_control(normalized_yaw);

            self.previous_thrust = normalized_thrust * self.max_thrust();
            if let Some(writer) = &mut self.writer {
                writer
                    .write(cyber_rc::CyberRCMessageType::RcData(cyberrc::RcData {
                        throttle: throttle_command,
                        // elevator: 0, // elevator_command,
                        aileron: aileron_command,
                        elevator: elevator_command,
                        rudder: rudder_command,
                        arm: 0,
                        mode: 0,
                    }))
                    .map_err(|e| SimulationError::OtherError(e.to_string()))?;
            }
        }
        Ok(None)
    }

    /// Observe the current state of the quadrotor
    /// Returns a tuple containing the position, velocity, orientation, and angular velocity of the quadrotor.
    fn observe(&mut self, t: f32) -> Result<QuadrotorState, SimulationError> {
        if !self
            .consumer
            .has_changed()
            .map_err(|e| SimulationError::OtherError(e.to_string()))?
        {
            return Ok(self.state.clone());
        }

        let dt = 1.0 / self.simulation_config.simulation_frequency as f32;
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
            let get_initial_attitude = || -> UnitQuaternion<f32> {
                let attitude = sample.rotation();
                UnitQuaternion::new_normalize(Quaternion::new(
                    (1.0 - (attitude.k * attitude.k)).sqrt(),
                    0.0,
                    0.0,
                    -attitude.k,
                ))
            };
            QuadrotorState {
                time: t,
                state: State {
                    position: sample.position(),
                    velocity: Vector3::zeros(),
                    acceleration: Vector3::zeros(),
                    // Store the initial yaw as the initial attitude so that we can adjust for it on
                    // subsequent samples
                    orientation: get_initial_attitude(),
                    // orientation: UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0),
                    angular_velocity: Vector3::zeros(),
                },
                ..Default::default()
            }
        });

        // Adjust the sample by the initial state
        // Adjust for the initial yaw - we take the starting yaw of the vehicle to be 0
        let rotation = sample.rotation();
        // Adjust fot the initial position - we take the starting location of the vehicle to be the
        // origin
        let mut position = sample.position();
        position[0] -= initial_state.position[0];
        position[1] -= initial_state.position[1];
        position[2] -= initial_state.position[2];

        let v_body = self.velocity_body(rotation, self.previous_state.position, position, dt);
        let omega_body =
            self.angular_velocity(self.previous_state.orientation, sample.rotation(), dt, 0.1);
        // println!("Unfiltered Omega");
        // dbg!(omega_body);
        // Low-pass filter the angular velocity
        let alpha = 0.8;
        let omega_body = alpha * omega_body + (1.0 - alpha) * self.previous_state.angular_velocity;
        let acceleration_body = self.body_acceleration(v_body, self.previous_state.velocity, dt);

        self.state = QuadrotorState {
            time: t,
            state: State {
                position,
                velocity: v_body,
                acceleration: acceleration_body,
                orientation: rotation,
                angular_velocity: omega_body,
            },
            ..Default::default()
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

    fn parameters(&self) -> crate::config::QuadrotorConfig {
        self.config.quadrotor_config.clone()
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
    #[allow(dead_code)]
    pub fn pqr(&self) -> Vector3<f32> {
        Vector3::new(self.gyro[1], self.gyro[0], self.gyro[2])
    }

    /// Returns the attitude quaternion in the NED frame
    pub fn rotation(&self) -> UnitQuaternion<f32> {
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
        if let Ok((len, _)) = socket.recv_from(&mut buf).await {
            tx.send(Some(buf[..len].to_vec()))
                .map_err(|e| SimulationError::OtherError(e.to_string()))?;
        }
    }
}

/// Scale a value from a given range to a new range
/// # Example
/// ```
/// use peng_quad::liftoff_quad::normalize;
/// let value = 0.0;
/// let min = -10.0;
/// let max = 10.0;
/// let scaled_value = normalize(value, min, max);
/// assert_eq!(scaled_value, 0.5);
/// ```
pub fn normalize(value: f32, min: f32, max: f32) -> f32 {
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
/// use peng_quad::liftoff_quad::scale_throttle;
/// let thrust = 0.0;
/// let throttle = scale_throttle(thrust);
/// assert_eq!(throttle, 32768);
/// ```
pub fn scale_throttle(thrust: f32) -> i32 {
    // thrust is inverted from Xinput to Liftoff
    -scale_to_rc_command(thrust, -32768, 32767)
}

fn scale_control(value: f32) -> i32 {
    scale_to_rc_command_with_center(value, -32768_f32, 0.0, 32767_f32)
}

#[cfg(test)]
mod tests {

    use super::*;

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
    #[ignore = "only runs in tokio runtime"]
    fn test_acceleration_body() {
        let quad = LiftoffQuad::new(
            SimulationConfig::default(),
            LiftoffQuadrotorConfig::default(),
        )
        .unwrap();

        let a = quad.body_acceleration(
            Vector3::<f32>::new(0_f32, 0_f32, 0_f32),
            Vector3::<f32>::new(0_f32, 0_f32, 0_f32),
            0.01,
        );
        assert_eq!(a, Vector3::new(0.0, 0.0, 9.81));

        let a = quad.body_acceleration(
            Vector3::<f32>::new(0_f32, 0_f32, 0_f32),
            Vector3::<f32>::new(0_f32, 0_f32, 9.81),
            1.0,
        );
        assert!(is_close(&a, &Vector3::new(0.0, 0.0, -9.81), 1e-6, 1e-6));
    }
}
