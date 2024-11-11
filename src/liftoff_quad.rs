use binrw::{binrw, BinRead};
use nalgebra::{zero, Matrix3, Quaternion, Rotation3, SMatrix, Unit, UnitQuaternion, Vector3};
use peng_quad::config::{Config, LiftoffConfiguration, QuadrotorConfig};
use peng_quad::quadrotor::{QuadrotorInterface, QuadrotorState};
use peng_quad::SimulationError;
use rand;
use std::net::UdpSocket;
use std::sync::Arc;
use tokio::sync::Mutex;
use tokio::time::{sleep, Duration};

/// Represents an RC quadrotor
/// # Example
/// ```
/// use nalgebra::Vector3;
/// use peng_quad::Quadrotor;
/// let (time_step, mass, gravity, drag_coefficient) = (0.01, 1.3, 9.81, 0.01);
/// let inertia_matrix = [0.0347563, 0.0, 0.0, 0.0, 0.0458929, 0.0, 0.0, 0.0, 0.0977];
/// let quadrotor = RCQuad::new(time_step, mass, inertia_matrix);
/// ```
pub struct LiftoffQuad {
    /// Current position of the quadrotor in 3D space
    pub position: Vector3<f32>,
    pub last_position: Vector3<f32>,
    /// Current velocity of the quadrotor
    pub velocity: Vector3<f32>,
    /// Current orientation of the quadrotor
    pub orientation: UnitQuaternion<f32>,
    pub last_orientation: UnitQuaternion<f32>,
    /// Current angular velocity of the quadrotor
    pub angular_velocity: Vector3<f32>,
    /// Simulation time step in seconds
    pub time_step: f32,
    /// Mass of the quadrotor in kg
    pub mass: f32,
    /// Inertia matrix of the quadrotor
    pub inertia_matrix: Matrix3<f32>,
    /// Inverse of the inertia matrix
    pub inertia_matrix_inv: Matrix3<f32>,
    /// Previous Thrust
    pub previous_thrust: f32,
    /// Previous Torque
    pub previous_torque: Vector3<f32>,
    /// Quadrotor sample mutex
    pub shared_data: Arc<Mutex<Option<LiftoffPacket>>>,
}

impl QuadrotorInterface for LiftoffQuad {
    fn control(&mut self, step_number: usize, config: &Config, thrust: f32, torque: &Vector3<f32>) {
        // TODO: implement control outputs to CyberRC - gamepad mode
        // todo!("implement control outputs to CyberRC - gamepad mode")
    }

    /// Observe the current state of the quadrotor
    /// # Returns
    /// A tuple containing the position, velocity, orientation, and angular velocity of the quadrotor
    fn observe(&mut self) -> Result<QuadrotorState, SimulationError> {
        let mut data_lock = tokio::runtime::Handle::current().block_on(self.shared_data.lock());
        while let Some(sample) = data_lock.take() {
            self.position = Vector3::from_row_slice(&sample.position);
            self.velocity = (self.position - self.last_position) / self.time_step;
            self.orientation = sample.attitude_quaternion();
            let delta_q =
                UnitQuaternion::new(self.last_orientation * self.last_position.conjugate());
            let angle = 2.0 * delta_q.scalar().acos();
            let angular_velocity = angle / self.time_step;
            let axis = delta_q
                .axis()
                .unwrap_or(Unit::new_normalize(Vector3::<f32>::zeros()));
            self.angular_velocity = axis.scale(angular_velocity);
            // Update last values for future velocity and angular velocity calculations
            self.last_position = self.position;
            self.last_orientation = self.orientation;
            return Ok(QuadrotorState {
                position: self.position,
                velocity: self.velocity,
                orientation: self.orientation,
                angular_velocity: self.angular_velocity,
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
        let inertia_matrix = Matrix3::from_row_slice(&vehicle_config.inertia_matrix);
        let inertia_matrix_inv =
            inertia_matrix
                .try_inverse()
                .ok_or(SimulationError::NalgebraError(
                    "Failed to invert inertia matrix".to_string(),
                ))?;
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
        Ok(Self {
            position: Vector3::zeros(),
            last_position: Vector3::zeros(),
            velocity: Vector3::zeros(),
            orientation: UnitQuaternion::identity(),
            last_orientation: UnitQuaternion::identity(),
            angular_velocity: Vector3::zeros(),
            time_step,
            mass: vehicle_config.mass,
            inertia_matrix,
            inertia_matrix_inv,
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

async fn feedback_loop(
    address: &str,
    timeout: Duration,
    max_retry_delay: Duration,
    data_lock: Arc<Mutex<Option<LiftoffPacket>>>,
) -> Result<(), SimulationError> {
    let mut buf = [0; 256];
    let mut current_wait = Duration::from_secs(0);
    let mut delay = Duration::from_secs(2);

    loop {
        match UdpSocket::bind(address) {
            // Bind successful
            Ok(socket) => {
                socket
                    .set_read_timeout(Some(Duration::from_secs(15)))
                    .map_err(|e| SimulationError::OtherError(e.to_string()))?;
                current_wait = Duration::from_secs(0);
                delay = Duration::from_secs(1);
                // Read loop
                loop {
                    match socket.recv_from(&mut buf) {
                        Ok((len, _)) => {
                            let mut cursor = std::io::Cursor::new(&buf[..len]);
                            // TODO: more robust handling of packet parsing errors during resets
                            let sample = LiftoffPacket::read_be(&mut cursor)
                                .expect("Failed to read LiftoffPacket");
                            let mut data_lock = data_lock.lock().await;
                            *data_lock = Some(sample);
                        }
                        Err(_) => {
                            if current_wait >= timeout {
                                return Err(SimulationError::OtherError(
                                    "Bind loop exceeded max wait time".into(),
                                ));
                            }
                            current_wait += delay;
                            sleep(
                                delay
                                    + Duration::from_millis(
                                        (rand::random::<f64>() * 1000.0) as u64,
                                    ),
                            )
                            .await;
                            delay = (delay * 2).min(max_retry_delay);
                            // break;
                        }
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
