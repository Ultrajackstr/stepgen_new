use micromath::F32Ext;
use crate::utils::enums::Error;

pub fn sigmoid_delay_us(current_delay_accumulator_us: f32, start_delay_us: f32, end_delay_us: f32, alpha: f32, accel_duration_us: f32) -> f32 {
    start_delay_us + (end_delay_us - start_delay_us) / (1.0 + (-alpha * (current_delay_accumulator_us - accel_duration_us / 2.0)).exp())
}

pub fn find_alpha_value(start_delay_us: f32, end_delay_us: f32, accel_duration_us: f32, tolerance_us: f32) -> Result<f32, Error> {
    let mut start_alpha = 0.000001;
    let tolerance_range = end_delay_us - tolerance_us..=end_delay_us + tolerance_us;
    while !tolerance_range.contains(&sigmoid_delay_us(accel_duration_us, start_delay_us, end_delay_us, start_alpha, accel_duration_us).round()) {
        start_alpha *= 1.1;
        if start_alpha > 1.0 {
            return Err(Error::InvalidAlpha)
        }
    }
    Ok(start_alpha)
}