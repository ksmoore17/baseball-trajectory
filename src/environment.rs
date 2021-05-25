use crate::util::{f_to_c, in_hg_to_mm_hg};
use nalgebra::{Vector2};

const BETA: f32 = 0.0001217;

pub struct Environment {
    // F
    pub temperature: f32,
    // ft
    pub elevation: f32,
    // 29.92
    pub pressure: f32,
    // %
    pub relative_humidity: f32,
    // mph
    pub wind_speed: f32,
    // clockwise deg from center
    pub wind_direction: f32,
    // ft above which there is wind
    pub wind_height: f32,
}

impl Default for Environment {
    fn default() -> Environment {
        Environment {
            temperature: 70.0,
            elevation: 15.0,
            pressure: 29.92,
            relative_humidity: 50.0,
            wind_speed: 0.0,
            wind_direction: 0.0,
            wind_height: 0.0
        }
    }
}

impl Environment {
    pub fn calculate_rho(&self) -> f32 {
        0.06261 * 1.2929 * (
            273.0
            / (f_to_c(self.temperature) + 273.0)
            * (
                in_hg_to_mm_hg(self.pressure)
                * (-BETA * self.elevation).exp()
                - 0.3783 * self.relative_humidity * self.calculate_svp() / 100.0
            )
            / 760.0
        )
    }

    fn calculate_svp(&self) -> f32 {
        let temperature_c = f_to_c(self.temperature);

        4.5841 * (
            (18.687 - temperature_c / 234.5)
            * temperature_c
            / (257.14 + temperature_c)
        ).exp()
    }

    pub fn calculate_wind_velocity(&self) -> Vector2<f32> {
        const PI: f32 = std::f32::consts::PI;
        let x = self.wind_speed * 1.467 * (self.wind_direction * PI / 180.0).sin();
        let y = self.wind_speed * 1.467 * (self.wind_direction * PI / 180.0).cos();

        Vector2::new(x, y)
    }
}