use core::f32::consts::PI;
use micromath::F32Ext;

pub fn sin_accel_delay_us(current_us: f32, coefficient: f32) -> f32 {
    coefficient / (1.0 - (PI * current_us).cos())
}

pub fn sin_decel_delay_us(current_us: f32, coefficient: f32) -> f32 {
    coefficient / (1.0 - (PI * current_us + PI).cos())
}