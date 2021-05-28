mod util;

mod environment;
mod impact;
mod ball;
mod trajectory;

pub use crate::environment::Environment;
pub use crate::impact::Impact;
pub use crate::ball::Ball;
pub use crate::trajectory::Trajectory;
pub use crate::trajectory::State;

#[cfg(test)]
mod tests {
    use nalgebra::Vector3;
    use crate::{Trajectory, Ball, Environment, Impact, State};

    #[test]
    fn it_works() {
        let ball = Ball::default();
        let environment = Environment::default();
        let impact = Impact::default();

        let trajectory = Trajectory::new(ball, environment, impact);
        let mut state = trajectory.get_initial_state();

        let delta = 1.0 / 100.0;

        loop {
            state = state.step(&trajectory, delta);

            if state.position[2] <= 0.0 {
                break;
            }
        }

        println!("{}", state.position);

        assert_eq!(state.get_position(), (0.0, 421.509, -0.25));
    }
}
