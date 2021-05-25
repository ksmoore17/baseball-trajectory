pub fn f_to_c(fahrenheit: f32) -> f32 {
    (5.0 / 9.0) * (fahrenheit - 32.0)
}

pub fn in_hg_to_mm_hg(in_hg: f32) -> f32 {
    in_hg * 1000.0 / 39.37
}