pub struct Ball {
    // oz
    pub mass: f32,
    // in
    pub circumference: f32,
}

impl Default for Ball {
    fn default() -> Ball {
        Ball { mass: 5.125, circumference: 9.125 }
    }
}