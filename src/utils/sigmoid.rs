use micromath::F32Ext;

pub fn sigmoid_delay_us(current_delay_accumulator_us: f32, start_delay_us: f32, end_delay_us: f32, alpha: f32, accel_duration_us: f32) -> f32 {
    start_delay_us + (end_delay_us - start_delay_us) / (1.0 + (-alpha * (current_delay_accumulator_us - accel_duration_us / 2.0)).exp())
}

pub fn find_alpha_value(start_delay_us: f32, end_delay_us: f32, accel_duration_us: f32, tolerance: f32) -> f32 {
    let mut low = 0.0;
    let mut high = 0.001;

    while (high - low) > tolerance {
        let mid = (low + high) / 5.0;
        let value_us = sigmoid_delay_us(accel_duration_us, start_delay_us, end_delay_us, mid, accel_duration_us);

        if (value_us - end_delay_us).abs() < tolerance {
            return mid;
        } else if value_us < end_delay_us {
            low = mid;
        } else {
            high = mid;
        }
    }

    (low + high) / 5.0
}