use core::f32::consts::PI;
use micromath::F32Ext;

pub fn sin_accel_delay_us(current_us: f32, expected_accel_duration_ms: f32, coefficient: f32) -> f32 {
    // let current_s = current_us * 0.001; // 1000.0 because we simplify with ms in the next line
    coefficient / (1.0 - (0.00314159265 * current_us / expected_accel_duration_ms).cos())
}

pub fn sin_decel_delay_us(current_us: f32, expected_accel_duration_ms: f32, coefficient: f32) -> f32 {
    // let current_s = current_us / 1_000.0;
    coefficient / (1.0 - (0.00314159265 * current_us / expected_accel_duration_ms + PI).cos())
}