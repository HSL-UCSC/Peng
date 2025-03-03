use nalgebra::Vector3;
use rand::SeedableRng;
use rand_chacha::ChaCha8Rng;

/// Represents an obstacle in the simulation
/// # Example
/// ```
/// use peng_quad::environment::Obstacle;
/// use nalgebra::Vector3;
/// let obstacle = Obstacle::new(Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 0.0), 1.0);
/// ```
#[derive(Clone, Debug, Default)]
pub struct Obstacle {
    /// The position of the obstacle
    pub position: Vector3<f32>,
    /// The velocity of the obstacle
    pub velocity: Vector3<f32>,
    /// The radius of the obstacle
    pub radius: f32,
}

/// Implementation of the Obstacle
impl Obstacle {
    /// Creates a new obstacle with the given position, velocity, and radius
    /// # Arguments
    /// * `position` - The position of the obstacle
    /// * `velocity` - The velocity of the obstacle
    /// * `radius` - The radius of the obstacle
    /// # Returns
    /// * The new obstacle instance
    /// # Example
    /// ```
    /// use peng_quad::environment::Obstacle;
    /// use nalgebra::Vector3;
    /// let obstacle = Obstacle::new(Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 0.0), 1.0);
    /// ```
    pub fn new(position: Vector3<f32>, velocity: Vector3<f32>, radius: f32) -> Self {
        Self {
            position,
            velocity,
            radius,
        }
    }
}

/// Represents a maze in the simulation
/// # Example
/// ```
/// use peng_quad::environment::{Maze, Obstacle};
/// use rand_chacha::ChaCha8Rng;
/// use rand::SeedableRng;
/// use nalgebra::Vector3;
/// let maze = Maze {
///     lower_bounds: [0.0, 0.0, 0.0],
///     upper_bounds: [1.0, 1.0, 1.0],
///     obstacles: vec![Obstacle::new(Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 0.0), 1.0)],
///     obstacles_velocity_bounds: [0.0, 0.0, 0.0],
///     obstacles_radius_bounds: [0.0, 0.0],
///     rng: ChaCha8Rng::from_entropy(),
///     ..Default::default()
/// };
/// ```
#[derive(Clone, Debug)]
pub struct Maze {
    /// The simulation time for this maze
    pub t: f32,
    /// The lower bounds of the maze in the x, y, and z directions
    pub lower_bounds: [f32; 3],
    /// The upper bounds of the maze in the x, y, and z directions
    pub upper_bounds: [f32; 3],
    /// The obstacles in the maze
    pub obstacles: Vec<Obstacle>,
    /// The bounds of the obstacles' velocity
    pub obstacles_velocity_bounds: [f32; 3],
    /// The bounds of the obstacles' radius
    pub obstacles_radius_bounds: [f32; 2],
    /// Rng for generating random numbers
    pub rng: ChaCha8Rng,
}

// Implement `Default`
impl Default for Maze {
    fn default() -> Self {
        Self {
            t: 0_f32,
            lower_bounds: [0.0, 0.0, 0.0], // Default maze starts at origin
            upper_bounds: [10.0, 10.0, 10.0], // Default size 10x10x10
            obstacles: Vec::new(),         // No obstacles by default
            obstacles_velocity_bounds: [0.0, 0.0, 0.0], // No movement
            obstacles_radius_bounds: [0.5, 2.0], // Default obstacle size range
            rng: ChaCha8Rng::seed_from_u64(0), // Deterministic default RNG seed
        }
    }
}

/// Implementation of the maze
impl Maze {
    /// Creates a new maze with the given bounds and number of obstacles
    /// # Arguments
    /// * `lower_bounds` - The lower bounds of the maze
    /// * `upper_bounds` - The upper bounds of the maze
    /// * `num_obstacles` - The number of obstacles in the maze
    /// # Returns
    /// * The new maze instance
    /// # Example
    /// ```
    /// use peng_quad::environment::Maze;
    /// let maze = Maze::new([-1.0, -1.0, -1.0], [1.0, 1.0, 1.0], 5, [0.1, 0.1, 0.1], [0.1, 0.5]);
    /// ```
    pub fn new(
        lower_bounds: [f32; 3],
        upper_bounds: [f32; 3],
        num_obstacles: usize,
        obstacles_velocity_bounds: [f32; 3],
        obstacles_radius_bounds: [f32; 2],
    ) -> Self {
        let mut maze = Maze {
            t: 0_f32,
            lower_bounds,
            upper_bounds,
            obstacles: Vec::new(),
            obstacles_velocity_bounds,
            obstacles_radius_bounds,
            rng: ChaCha8Rng::from_entropy(),
        };
        maze.generate_obstacles(num_obstacles);
        maze
    }
    /// Generates the obstacles in the maze
    /// # Arguments
    /// * `num_obstacles` - The number of obstacles to generate
    /// # Example
    /// ```
    /// use peng_quad::environment::Maze;
    /// let mut maze = Maze::new([-1.0, -1.0, -1.0], [1.0, 1.0, 1.0], 5, [0.1, 0.1, 0.1], [0.1, 0.5]);
    /// maze.generate_obstacles(5);
    /// ```
    pub fn generate_obstacles(&mut self, num_obstacles: usize) {
        self.obstacles = (0..num_obstacles)
            .map(|_| {
                let position = Vector3::new(
                    rand::Rng::gen_range(&mut self.rng, self.lower_bounds[0]..self.upper_bounds[0]),
                    rand::Rng::gen_range(&mut self.rng, self.lower_bounds[1]..self.upper_bounds[1]),
                    rand::Rng::gen_range(&mut self.rng, self.lower_bounds[2]..self.upper_bounds[2]),
                );
                let v_bounds = self.obstacles_velocity_bounds;
                let r_bounds = self.obstacles_radius_bounds;
                let velocity = Vector3::new(
                    rand::Rng::gen_range(&mut self.rng, -v_bounds[0]..v_bounds[0]),
                    rand::Rng::gen_range(&mut self.rng, -v_bounds[1]..v_bounds[1]),
                    rand::Rng::gen_range(&mut self.rng, -v_bounds[2]..v_bounds[2]),
                );
                let radius = rand::Rng::gen_range(&mut self.rng, r_bounds[0]..r_bounds[1]);
                Obstacle::new(position, velocity, radius)
            })
            .collect();
    }
    /// Updates the obstacles in the maze, if an obstacle hits a boundary, it bounces off
    /// # Arguments
    /// * `dt` - The time step
    /// # Example
    /// ```
    /// use peng_quad::environment::Maze;
    /// let mut maze = Maze::new([-1.0, -1.0, -1.0], [1.0, 1.0, 1.0], 5, [0.1, 0.1, 0.1], [0.1, 0.5]);
    /// maze.update_obstacles(0.1);
    /// ```
    pub fn update_obstacles(&mut self, dt: f32) {
        self.t += dt;
        self.obstacles.iter_mut().for_each(|obstacle| {
            obstacle.position += obstacle.velocity * dt;
            for i in 0..3 {
                if obstacle.position[i] - obstacle.radius < self.lower_bounds[i]
                    || obstacle.position[i] + obstacle.radius > self.upper_bounds[i]
                {
                    obstacle.velocity[i] *= -1.0;
                }
            }
        });
    }
}
