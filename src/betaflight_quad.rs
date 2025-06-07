#![allow(clippy::all, dead_code, unused_variables)]
use crate::config::{Betaflight, QuadrotorConfig, SimulationConfig};
use crate::quadrotor::{OrientationFilter, QuadrotorInterface, QuadrotorState};
use crate::SimulationError;
use cyber_rc::{cyberrc, CyberRCMessageType, Writer};
use nalgebra::{UnitQuaternion, Vector3};
use std::f32::consts::PI;
use std::time::Duration;
use std::time::Instant;
use tokio::sync::watch;
#[cfg(feature = "vicon")]
use vicon_sys::HasViconHardware;

/// Represents a physical quadrotor running a Betaflight controller.
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
    pub simulation_config: SimulationConfig,
    /// Configured physical parameters
    pub config: Betaflight,
    /// Previous Thrust
    pub previous_thrust: f32,
    /// The last vicon sample
    #[cfg(feature = "vicon")]
    pub last_sample: Option<ViconPacket>,
    /// Orientation filter to smooth out the orientation readings
    pub orientation_filter: OrientationFilter,
    /// Quadrotor sample mutex
    #[cfg(feature = "vicon")]
    pub consumer: watch::Receiver<Option<ViconPacket>>,
}

impl BetaflightQuad {
    pub fn new(
        simulation_config: SimulationConfig,
        config: Betaflight,
    ) -> Result<Self, SimulationError> {
        #[cfg(feature = "vicon")]
        let consumer = if cfg!(feature = "vicon") {
            let (producer, consumer) = watch::channel(None::<ViconPacket>);
            let producer_clone = producer.clone();
            let config_clone = config.clone();
            let subject_name = config.clone().quadrotor_config.id;
            tokio::spawn(async move {
                let _ =
                    feedback_loop(&config_clone.vicon_address, &subject_name, producer_clone).await;
            });
            consumer
        } else {
            let (_, consumer) = watch::channel(None::<ViconPacket>);
            consumer
        };
        // Open a serial port to communicate with the quadrotor if one is specified
        let writer: Option<Writer> = match config.clone().serial_port {
            Some(port) => {
                let mut writer = Writer::new(port.to_string(), config.baud_rate).map_err(|e| {
                    SimulationError::OtherError(format!(
                        "Failed to open SerialPort {:?}",
                        e.to_string()
                    ))
                })?;
                let _start_time = std::time::Instant::now();
                writer
                    .serial_port
                    .set_timeout(Duration::from_millis(25))
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
        // Open a serial port to communicate with the quadrotor if one is specified
        Ok(Self {
            writer,
            state: QuadrotorState::default(),
            previous_state: QuadrotorState::default(),
            initial_state: None,
            simulation_config,
            config,
            previous_thrust: 0.0,
            #[cfg(feature = "vicon")]
            last_sample: None,
            orientation_filter: OrientationFilter::new(),
            #[cfg(feature = "vicon")]
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

impl QuadrotorInterface for BetaflightQuad {
    fn control(
        &mut self,
        _step_number: usize,
        thrust: f32,
        torque: &Vector3<f32>,
    ) -> Result<Option<(f32, Vector3<f32>)>, SimulationError> {
        {
            // Given thrust and torque, calculate the control inputs
            // Clamp thrust and torque control inputs
            let _max_thrust = self.max_thrust();
            let _max_torque = self.max_torque();
            self.previous_thrust = thrust;
            let thrust_ppm = scale_control(thrust, 0.0_f32, 1.0_f32);
            let aileron_ppm = scale_control(torque.x, -5_f32, 5_f32);
            let elevator_ppm = scale_control(torque.y, -5_f32, 5_f32);
            let rudder_ppm = scale_control(-torque.z, -60_f32, 60_f32);
            if let Some(writer) = &mut self.writer {
                writer
                    .write(CyberRCMessageType::PpmUpdate(cyberrc::PpmUpdateAll {
                        line: 1,
                        channel_values: vec![
                            aileron_ppm as i32,
                            elevator_ppm as i32,
                            thrust_ppm as i32,
                            rudder_ppm as i32,
                        ],
                    }))
                    .map_err(|e| SimulationError::OtherError(e.to_string()))?;
            }
            let control_out = (
                thrust_ppm,
                Vector3::new(aileron_ppm, elevator_ppm, rudder_ppm),
            );
            Ok(Some(control_out))
        }
    }

    /// Observe the current state of the quadrotor
    /// Returns a tuple containing the position, velocity, orientation, and angular velocity of the quadrotor.
    #[cfg(feature = "vicon")]
    fn observe(&mut self, dt: f32) -> Result<QuadrotorState, SimulationError> {
        if !self
            .consumer
            .has_changed()
            .map_err(|e| SimulationError::OtherError(e.to_string()))?
        {
            return Ok(self.state.clone());
        }

        self.previous_state = self.state.clone();

        // Get the latest sample
        let sample = match self.consumer.borrow_and_update().clone() {
            Some(stamped) => stamped,
            None => return Err(SimulationError::OtherError("No packet".to_string())),
        };

        // Compute dt from last_sample if available
        let dt = match self.last_sample {
            Some(ref last) => (sample.timestamp.duration_since(last.timestamp))
                .as_secs_f32()
                .max(0.0),
            None => dt,
        };
        // TODO: consider a dt clamp
        // let dt = dt.clamp(1e-4, 0.1); // limit to 0.1s max for safety

        // Save current stamped sample for next iteration
        self.last_sample = Some(sample.clone());

        // let sample = ViconPacket(stamped.subject.clone());

        // TODO: use sample time for dt?
        // let dt = 1.0 / self.simulation_config.simulation_frequency as f32;
        // let dt = 1.0 / 120.0;
        // let dt = (step as f32) * (1.0 / self.simulation_config.simulation_frequency as f32)
        //     - self.previous_state.time;
        // dbg!(step, self.previous_state.time);
        // dbg!(dt);
        // This needs to match the sample rate of the Vicon system.
        // Ideally, this will use the time steps or real time
        let (position, rotation) = if sample.occluded() {
            // TODO: failsafe for condition where oclusion lasts longer than N frames
            println!("Warning! Vicon sample is occluded, extrapolating position and rotation");
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
            let alpha_position = 0.8;
            let position = alpha_position * sample.position()
                + (1.0 - alpha_position) * self.previous_state.position;

            // Uncomment this block to go back to old filter
            // Low-pass filter the orientation
            let alpha_rotation = 0.15;
            let rotation = match self.previous_state.orientation.try_slerp(
                &sample.rotation(),
                alpha_rotation,
                1e-6,
            ) {
                Some(rotation) => rotation,
                None => sample.rotation(),
            };
            // self.orientation_filter.add_sample(sample.rotation());
            (position, rotation);

            // Geodesic mean filter
            self.orientation_filter.add_sample(sample.rotation());
            // (position, self.orientation_filter.get_filtered())
            (position, rotation)
        };

        
        let v_body = self.velocity_body(rotation, self.previous_state.position, position, dt);
        let omega_body =
            self.angular_velocity(self.previous_state.orientation, sample.rotation(), dt, 0.1);
        // println!("Unfiltered Omega");
        // dbg!(omega_body);
        // Low-pass filter the angular velocity
        let alpha = 0.8;
        let omega_body = alpha * omega_body + (1.0 - alpha) * self.previous_state.angular_velocity;
        let acceleration_body = self.body_acceleration(self.previous_state.velocity, v_body, dt);
        self.state = QuadrotorState {
            time: dt,
            state: crate::quadrotor::State {
                position: position,
                velocity: v_body,
                acceleration: acceleration_body,
                orientation: rotation,
                angular_velocity: omega_body,
            },
            ..Default::default()
        };
        Ok(self.state.clone())
    }

    #[cfg(not(feature = "vicon"))]
    fn observe(&mut self, t: f32) -> Result<QuadrotorState, SimulationError> {
        Ok(QuadrotorState::default())
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

    fn parameters(&self) -> QuadrotorConfig {
        self.config.quadrotor_config.clone()
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
    let delta_quaternion = UnitQuaternion::from_axis_angle(&rotation_axis, rotation_angle);

    // Apply the delta quaternion to the previous orientation
    last_orientation * delta_quaternion
}

#[cfg(feature = "vicon")]
#[derive(Clone)]
pub struct ViconPacket {
    pub subject: vicon_sys::ViconSubject,
    pub timestamp: Instant,
}

#[cfg(feature = "vicon")]
impl ViconPacket {
    pub fn occluded(&self) -> bool {
        self.subject.occluded
    }

    /// Returns the attitude quaternion in the NED frame
    pub fn rotation(&self) -> UnitQuaternion<f32> {
        // let rquat = nalgebra::UnitQuaternion::<f32>::from_axis_angle(&Vector3::x_axis(), PI);
        let rotation = match self.subject.rotation {
            vicon_sys::RotationType::Quaternion(q) => {
                let mut q = q.normalize(); // Ensure unit quaternion
                if q.w < 0.0 {
                    q = -q; // Force consistent quaternion sign
                }
                nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                    q.w as f32, q.i as f32, q.j as f32, q.k as f32,
                ))
            }
            _ => panic!("Unsupported rotation type"),
        };
        rotation
    }

    pub fn position(&self) -> Vector3<f32> {
        // convert from Vicon NWU to NED frame
        let position = self.subject.origin;
        nalgebra::Vector3::<f32>::new(position[0] as f32, position[1] as f32, position[2] as f32)
    }
}

// FIXME: address for sdk
#[cfg(feature = "vicon")]
async fn feedback_loop(
    address: &str,
    id: &str,
    tx: watch::Sender<Option<ViconPacket>>,
) -> Result<(), SimulationError> {
    let mut vicon = vicon_sys::sys::ViconSystem::new(address)
        .map_err(|e| SimulationError::OtherError(e.to_string()))?;

    loop {
        if let Ok(subjects) = vicon.read_frame_subjects(vicon_sys::OutputRotation::Quaternion) {
            if let Some(sample) = subjects.iter().find(|s| s.name == id) {
                let stamped = ViconPacket {
                    subject: sample.clone(),
                    timestamp: Instant::now(),
                };

                tx.send(Some(stamped))
                    .map_err(|e| SimulationError::OtherError(e.to_string()))?;
            }
        }

        // optional: sleep to avoid 100% CPU
        tokio::time::sleep(std::time::Duration::from_millis(1)).await;
    }
}

/// Scale a control value to a stick command, given a minimum and maximum input range
/// # Example
/// ```
/// use peng_quad::betaflight_quad::scale_control;
/// let throttle = scale_control(0.0, 0_f32, 20_f32);
/// assert_eq!(throttle, 1000_f32);
/// let throttle = scale_control(250.0, 0_f32, 200_f32);
/// assert_eq!(throttle, 2000_f32);
/// ```
pub fn scale_control(value: f32, min_torque: f32, max_torque: f32) -> f32 {
    let torque = value.clamp(min_torque, max_torque);
    let min_ppm = 1000_f32;
    let max_ppm = 2000_f32;

    let ppm_range = max_ppm - min_ppm;
    min_ppm + ((torque - min_torque) * ppm_range) / (max_torque - min_torque)
}
