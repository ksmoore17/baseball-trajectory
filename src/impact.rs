use nalgebra::Vector3;
use std::f32::consts::PI;

pub struct Impact {
    // ft/s
    pub exit_speed: f32,
    // deg up from ground
    pub launch_angle: f32,
    // clockwise deg from 0 = cf
    pub direction: f32,
    // back spin, side spin, gyro spin in rpm
    pub back_spin: f32,
    pub side_spin: f32,
    pub gyro_spin: f32,
    // +ft to the right of the plate
    pub x: f32,
    // +ft toward center field from the plate
    pub y: f32,
    // +ft above the plate
    pub z: f32,
}

impl Impact {
    pub fn calculate_velocity(exit_speed: f32, launch_angle: f32, direction: f32) -> (f32, f32, f32) {
        // getting these as unit vector
        let x = (launch_angle * PI / 180.0).cos() * (direction * PI / 180.0).sin();
        let y = (launch_angle * PI / 180.0).cos() * (direction * PI / 180.0).cos();
        let z = (launch_angle * PI / 180.0).sin();

        // scaling to magnitude
        let magnitude = 1.467 * exit_speed;
        (x * magnitude, y * magnitude, z * magnitude)
    }

    pub fn calculate_cartesian_spin(&self, velocity: &Vector3<f32>) -> Vector3<f32> {
        // spins are constant from the initial velocity

        let x = (
            self.back_spin * (self.direction * PI / 180.0).cos()
                - self.side_spin * (self.launch_angle * PI / 180.0).sin() * (self.direction * PI / 180.0).sin()
                + self.gyro_spin * velocity[0] / velocity.magnitude()
        ) * PI / 30.0;

        let y = (
            self.back_spin * (self.direction * PI / 180.0).sin()
                - self.side_spin * (self.launch_angle * PI / 180.0).sin() * (self.direction * PI / 180.0).cos()
                + self.gyro_spin * velocity[1] / velocity.magnitude()
        ) * PI / 30.0;

        let z = (
            self.side_spin * (self.launch_angle * PI / 180.0).cos()
                + self.gyro_spin * velocity[2] / velocity.magnitude()
        ) * PI / 30.0;

        // scaling to magnitude
        Vector3::new(x, y, z)
    }
}

impl Default for Impact {
    fn default() -> Self {
        Self {
            // mph
            exit_speed: 103.0,
            // deg up from ground
            launch_angle: 27.5,
            // clockwise deg from 0 = cf
            direction: 0.0,
            back_spin: 2500.0,
            side_spin: 0.0,
            gyro_spin: 0.0,
            x: 0.0,
            y: 2.0,
            z: 3.0,
        }
    }
}
