use crate::ball::Ball;
use crate::environment::Environment;
use crate::impact::Impact;
use nalgebra::{Vector3, Vector2};
use std::f32::consts::PI;

// in s
const TAU: f32 = 25.0;

// Fitted coefficients (from Nathan)
const STATIC_DRAG_COEFFICIENT: f32 = 0.3008;
const SPIN_DRAG_COEFFICIENT: f32 = 0.0292;

struct Constants {
    c_0: f32,
    initial_position: Vector3<f32>,
    initial_velocity: Vector3<f32>,
    initial_spin: Vector3<f32>,
    cartesian_spin: Vector3<f32>,
    omega: f32,
    omega_r: f32,
    wind_height: f32,
    wind_velocity: Vector2<f32>,
}

impl Constants {
    pub fn from_conditions(ball: &Ball, environment: &Environment, impact: &Impact) -> Self {
        let rho = environment.calculate_rho();
        let c_0 = calculate_c_0(ball.mass, ball.circumference, rho);
        let initial_position = Vector3::new(impact.x, impact.y, impact.z);
        let initial_velocity = impact.calculate_initial_velocity();
        let initial_spin = Vector3::new(impact.back_spin, impact.side_spin, impact.gyro_spin);
        let cartesian_spin = impact.calculate_cartesian_spin(initial_velocity);
        let omega = cartesian_spin.magnitude();
        let omega_r = omega_to_omega_r(ball.circumference, omega);
        let wind_velocity = environment.calculate_wind_velocity();

        Self {
            c_0,
            initial_position,
            initial_velocity,
            initial_spin,
            cartesian_spin,
            omega,
            omega_r,
            wind_velocity,
            wind_height: environment.wind_height,
        }
    }

    fn get_initial_state(&self) -> State {
        let position = self.initial_position;
        let velocity = self.initial_velocity;
        let spin = self.initial_spin;

        State {
            position,
            velocity,
            spin,
            hang_time: 0.0,
        }
    }
}

fn omega_to_omega_r(circumference: f32, omega: f32) -> f32 {
    (circumference / 2.0 / PI) * omega / 12.0
}

// coefficient used for drag and magnus acceleration related to the properties of the air around the ball
fn calculate_c_0(mass: f32, circumference: f32, rho: f32) -> f32 {
    0.07182 * rho * (5.125 / mass) * (circumference / 9.125).powi(2)
}

pub struct State {
    pub position: Vector3<f32>,
    pub velocity: Vector3<f32>,
    pub spin: Vector3<f32>,
    pub hang_time: f32,
}

impl State {
    pub fn step(&self, trajectory: &Trajectory, delta: f32) -> State {
        // update the position and velocity, then the acceleration with this new approximated position and velocity
        let acceleration = trajectory.calculate_acceleration(self);
        let position = self.position + self.velocity * delta + acceleration * (delta * delta) / 2.0;
        let velocity = self.velocity + acceleration * delta;

        State {
            position,
            velocity,
            spin: self.spin,
            hang_time: self.hang_time + delta,
        }
    }
}

pub struct Trajectory {
    pub ball: Ball,
    pub environment: Environment,
    pub impact: Impact,
    constants: Constants,
}

impl Trajectory {
    pub fn new(ball: Ball, environment: Environment, impact: Impact) -> Self {
        let constants = Constants::from_conditions(&ball, &environment, &impact);

        Self {
            ball,
            environment,
            impact,
            constants,
        }
    }

    pub fn get_initial_state(&self) -> State {
        self.constants.get_initial_state()
    }

    fn calculate_acceleration(&self, state: &State) -> Vector3<f32> {
        // Calculate drag + magnus acceleration, then subtracts for gravity to get the total acceleration
        let wind_velocity = self.constants.wind_velocity;
        let wind_height = self.constants.wind_height;
        let omega_r = self.constants.omega_r;
        let velocity = state.velocity;

        let relative_wind_speed = calculate_relative_wind_speed(wind_velocity, wind_height, velocity, state.position[2]);

        let drag_decay = (-state.hang_time / (TAU * 146.7 / velocity.magnitude())).exp();
        let s = (omega_r / relative_wind_speed) * drag_decay;

        let drag_coefficient = calculate_drag_coefficient(state.spin, drag_decay);

        let drag_acceleration = self.calculate_drag_acceleration(velocity, wind_velocity, relative_wind_speed, drag_coefficient);
        let magnus_acceleration = self.calculate_magnus_acceleration(s, velocity, relative_wind_speed);

        let mut acceleration = drag_acceleration + magnus_acceleration;
        // Acceleration due to gravity in ft/s^2
        acceleration[2] -= 32.17404855643;

        acceleration
    }


    fn calculate_drag_acceleration(&self, velocity: Vector3<f32>, wind_velocity: Vector2<f32>, relative_wind_speed: f32, drag_coefficient: f32) -> Vector3<f32> {
        // Calculate the contribution to acceleration from drag which changes wrt velocity
        let c_0 = self.constants.c_0;
        let wind_height = self.constants.wind_height;

        // Max function is used to only consider wind if above wind height
        Vector3::new(
            -c_0 * drag_coefficient * relative_wind_speed * (velocity[0] - f32::max(wind_velocity[0] - wind_height, 0.0)),
            -c_0 * drag_coefficient * relative_wind_speed * (velocity[1] - f32::max(wind_velocity[1] - wind_height, 0.0)),
            -c_0 * drag_coefficient * relative_wind_speed * (velocity[2]),
        )
    }

    fn calculate_magnus_acceleration(&self, s: f32, velocity: Vector3<f32>, relative_wind_speed: f32) -> Vector3<f32> {
        // Calculate the contribution to acceleration from magnus effects which changes wrt velocity (lift)
        let lift_coefficient = 1.0 / (2.32 + 0.4 / s);
        let c_0 = self.constants.c_0;
        let omega = self.constants.omega;
        let cartesian_spin = self.constants.cartesian_spin;
        let wind_velocity = self.constants.wind_velocity;
        let wind_height = self.constants.wind_height;

        Vector3::new(
            0.0,
            c_0
                * (lift_coefficient / omega)
                * relative_wind_speed
                * (cartesian_spin[2]
                * (velocity[0] - 0.0)
                - cartesian_spin[0] * velocity[2]),
            c_0 *
                (lift_coefficient / omega)
                * relative_wind_speed
                * (cartesian_spin[0]
                * (velocity[1] - f32::max(wind_velocity[1] - wind_height, 0.0))
                - cartesian_spin[1] * (velocity[0] - 0.0)),
        )
    }
}

// drag coefficient that "inflects" at exit velocity around 100
fn calculate_drag_coefficient(spin: Vector3<f32>, drag_decay: f32) -> f32 {
    STATIC_DRAG_COEFFICIENT + SPIN_DRAG_COEFFICIENT * (spin.magnitude() / 1000.0) * drag_decay
}

pub fn calculate_relative_wind_speed(wind_velocity: Vector2<f32>, wind_height: f32, velocity: Vector3<f32>, z: f32) -> f32 {
    // velocity of the wind "felt" by the ball
    if z >= wind_height {
        return Vector3::new(velocity[0] - wind_velocity[0], velocity[1] - wind_velocity[1], velocity[2]).magnitude();
    }

    return velocity.magnitude();
}

