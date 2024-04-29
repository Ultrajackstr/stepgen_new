use core::f32::consts::PI;
use micromath::F32Ext;

pub fn sin_accel_delay_us(current_us: f32, expected_accel_duration_ms: f32, coefficient: f32) -> f32 {
    let current_s = current_us / 1_000_000.0;
    coefficient / (1.0 - (PI * current_s / (expected_accel_duration_ms / 1000.0)).cos()) * 1_000_000.0
}

pub fn sin_decel_delay_us(current_us: f32, expected_accel_duration_ms: f32, coefficient: f32) -> f32 {
    let current_s = current_us / 1_000_000.0;
    coefficient / (1.0 - (PI * current_s / (expected_accel_duration_ms / 1000.0) + PI).cos()) * 1_000_000.0
}