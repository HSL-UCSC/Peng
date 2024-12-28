use binrw::{binrw, BinRead};
use cyber_rc::{cyberrc, CyberRCMessageType, Writer};
use nalgebra::{AbstractRotation, Matrix4, Quaternion, Rotation3, UnitQuaternion, Vector3};
use peng_quad::config;
use peng_quad::quadrotor::{QuadrotorInterface, QuadrotorState};
use peng_quad::SimulationError;
use std::time::Duration;
use tokio::sync::watch;
use vicon_sys::HasViconHardware;

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
    pub consumer: watch::Receiver<Option<vicon_sys::ViconSubject>>,
}

impl BetaflightQuad {
    pub fn new(
        simulation_config: config::SimulationConfig,
        config: config::Betaflight,
    ) -> Result<Self, SimulationError> {
        let (producer, consumer) = watch::channel(None::<vicon_sys::ViconSubject>);
        let config_clone = config.clone();
        let producer_clone = producer.clone();
        // Open a serial port to communicate with the quadrotor if one is specified
        let writer: Option<Writer> = match config.clone().serial_port {
            Some(port) => {
                // let mut writer = Writer::new(port.to_string(), config.baud_rate).map_err(|e| {
                //     SimulationError::OtherError(format!(
                //         "Failed to open SerialPort {:?}",
                //         e.to_string()
                //     ))
                // })?;
                // let start_time = std::time::Instant::now();
                // writer
                //     .serial_port
                //     .set_timeout(Duration::from_millis(50))
                //     .map_err(|e| {
                //         SimulationError::OtherError(format!(
                //             "Failed to set timeout {:?}",
                //             e.to_string()
                //         ))
                //     })?;
                // Some(writer)
                None
            }
            None => {
                println!("No serial port specified, writing to temp file");
                None
            }
        };
        let subject_name = config.clone().subject_name;
        tokio::spawn(async move {
            let _ = feedback_loop(&config_clone.vicon_address, &subject_name, producer_clone).await;
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
        let q_conjugate = q_ib.conjugate();
        q_conjugate.transform_vector(&v_inertial)
    }

    // Function to calculate body-frame acceleration for NED coordinates
    fn body_acceleration(
        &self,
        v1: Vector3<f32>, // Linear velocity in body frame
        v2: Vector3<f32>, // Linear velocity in body frame
        dt: f32,
    ) -> Vector3<f32> {
        (v2 - v1) / dt
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
            Some(packet) => Ok(ViconPacket(packet)),
            None => Err(SimulationError::OtherError("No packet".to_string())),
        }?;

        // TODO: use sample time for dt?
        let dt = 1.0 / self.simulation_config.simulation_frequency as f32;
        let (position, rotation) = if sample.occluded() {
            // Extrapolate the position
            let position = extrapolate_position(
                self.previous_state.position,
                self.previous_state.velocity,
                dt,
            );
            // Extrapolate the rotation
            let rotation = extrapolate_orientation(
                self.previous_state.orientation,
                self.previous_state.angular_velocity,
                dt,
            );
            (position, rotation)
        } else {
            // Low-pass filter the position
            let alpha_position = 0.8;
            let position = alpha_position * sample.position()
                + (1.0 - alpha_position) * self.previous_state.position;
            // Low-pass filter the orientation
            let alpha_rotation = 0.8;
            let rotation = match self.previous_state.orientation.try_slerp(
                &sample.rotation(),
                alpha_rotation,
                1e-6,
            ) {
                Some(rotation) => rotation,
                None => sample.rotation(),
            };
            (position, rotation)
        };

        let v_body = self.velocity_body(rotation, position, self.previous_state.position, dt);
        let omega_body =
            self.angular_velocity(self.previous_state.orientation, sample.rotation(), dt);
        // Low-pass filter the angular velocity
        let alpha = 0.5;
        let omega_body = alpha * omega_body + (1.0 - alpha) * self.previous_state.angular_velocity;
        let acceleration_body = self.body_acceleration(self.previous_state.velocity, v_body, dt);
        self.state = QuadrotorState {
            time: step as f32 * dt,
            position: position,
            velocity: v_body,
            acceleration: acceleration_body,
            orientation: sample.rotation(),
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

fn extrapolate_position(
    last_position: Vector3<f32>, // Previous position
    velocity: Vector3<f32>,      // Velocity vector
    dt: f32,                     // Time step
) -> Vector3<f32> {
    last_position + velocity * dt
}

fn extrapolate_orientation(
    last_orientation: UnitQuaternion<f32>, // Previous orientation (unit quaternion)
    angular_velocity: Vector3<f32>,        // Angular velocity in radians per second
    dt: f32,                               // Time step (seconds)
) -> UnitQuaternion<f32> {
    // Compute the magnitude of the angular velocity
    let omega_magnitude = angular_velocity.norm();

    // If there's no significant angular velocity, return the previous orientation
    if omega_magnitude.abs() < 1e-6 {
        return last_orientation;
    }

    // Compute the axis of rotation (normalize angular velocity)
    let rotation_axis = nalgebra::Unit::new_normalize(angular_velocity);

    // Compute the angle of rotation
    let rotation_angle = omega_magnitude * dt;

    // Create the delta rotation as a unit quaternion
    let delta_quaternion = UnitQuaternion::from_axis_angle(&rotation_axis.into(), rotation_angle);

    // Apply the delta quaternion to the previous orientation
    last_orientation * delta_quaternion
}

struct ViconPacket(vicon_sys::ViconSubject);

impl ViconPacket {
    // Rotation from Vicon frame to NED frame
    // Does a rotation of pi radians about the x-axis
    const R: Quaternion<f32> = Quaternion::<f32>::new(0.0, 1.0, 0.0, 0.0);

    pub fn occluded(&self) -> bool {
        self.0.occluded
    }

    /// Returns the attitude quaternion in the NED frame
    pub fn rotation(&self) -> UnitQuaternion<f32> {
        let rquat = nalgebra::UnitQuaternion::<f32>::from_quaternion(ViconPacket::R);
        // TODO: support other rotation types
        let rotation = match self.0.rotation {
            vicon_sys::RotationType::Quaternion(quat) => {
                nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                    quat.w as f32,
                    quat.i as f32,
                    quat.j as f32,
                    quat.k as f32,
                ))
            }
            _ => panic!("Unsupported rotation type"),
        };
        rquat * rotation * rquat.conjugate()
    }

    pub fn position(&self) -> Vector3<f32> {
        // convert from Vicon NWU to NED frame
        let position = self.0.origin;
        nalgebra::Vector3::<f32>::new(position[0] as f32, -position[1] as f32, -position[2] as f32)
    }
}

// FIXME: address for sdk
async fn feedback_loop(
    address: &str,
    subject_name: &str,
    tx: watch::Sender<Option<vicon_sys::ViconSubject>>,
) -> Result<(), SimulationError> {
    let mut vicon = vicon_sys::sys::ViconSystem::new("localhost")
        .map_err(|e| SimulationError::OtherError(e.to_string()))?;
    loop {
        // FIXME: vicon operating mode check block
        if let Ok(subjects) = vicon.read_frame_subjects(vicon_sys::OutputRotation::Quaternion) {
            // TODO: add search for all subjects
            if let Some(sample) = subjects.first() {
                if sample.name == subject_name {
                    tx.send(Some(sample.clone()))
                        .map_err(|e| SimulationError::OtherError(e.to_string()))?;
                }
            }
        };
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
