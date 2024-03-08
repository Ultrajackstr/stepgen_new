use fugit::{TimerDurationU64, TimerInstantU64};
use micromath::F32Ext;

use crate::utils::enums::{Error, OperatingMode};
use crate::utils::sigmoid::{find_alpha_value, sigmoid_delay_us};

const TIMER_HZ_MILLIS: u32 = 1_000; // One tick is 1 millisecond.

/// State of the stepgen.
#[derive(Debug)]
pub struct Stepgen<const TIMER_HZ_MICROS: u32> {
    // Operating mode
    operating_mode: OperatingMode,
    current_step: f32,
    // Amount of acceleration steps we've taken so far
    acceleration_steps: f32,
    // How long did the acceleration take
    pub acceleration_duration_ms: TimerDurationU64<TIMER_HZ_MILLIS>,
    // Previously calculated delay
    current_delay_us: f32,
    current_duration_ms: TimerDurationU64<TIMER_HZ_MILLIS>,
    // First step delay
    first_delay_us: f32,
    // Target step
    target_step: f32,
    // Target duration
    target_duration_ms: TimerDurationU64<TIMER_HZ_MILLIS>,
    // Target speed delay
    pub target_delay_us: f32,
    // Start time
    start_time_ms: Option<TimerInstantU64<TIMER_HZ_MILLIS>>,
    is_acceleration_done: bool,
    is_sigmoid_profile: bool,
    pub expected_accel_duration_ms: f32,
    pub alpha: f32,
    pub current_delay_accumulator_us: f32,
}

impl<const TIMER_HZ_MICROS: u32> Stepgen<TIMER_HZ_MICROS> {
    /// Create new copy of stepgen.
    pub fn new(target_rpm: u32, acceleration_rpm_s: u32, target_step: u64, target_duration_ms: u64, enable_sigmoid_profile: bool, full_steps_per_revolution: u16) -> Result<Stepgen<TIMER_HZ_MICROS>, Error> {
        if acceleration_rpm_s == 0 {
            return Err(Error::ZeroAcceleration);
        }
        if target_rpm == 0 {
            return Err(Error::ZeroRpm);
        }
        if target_step != 0 && target_duration_ms != 0 {
            return Err(Error::BothStepTargetAndDuration);
        }
        let operating_mode = if target_step != 0 {
            OperatingMode::Step
        } else {
            OperatingMode::Duration
        };
        let target_duration_ms = TimerDurationU64::<TIMER_HZ_MILLIS>::from_ticks(target_duration_ms);
        let mut expected_accel_duration_ms = target_rpm as f32 / acceleration_rpm_s as f32 * 1000.0;
        let target_rpm = if expected_accel_duration_ms > target_duration_ms.ticks() as f32 / 2.0 {
            let half_duration_s = target_duration_ms.ticks() as f32 / 2.0 / 1000.0;
            expected_accel_duration_ms = half_duration_s * 1000.0;
            acceleration_rpm_s as f32 * half_duration_s
        } else {
            target_rpm as f32
        };
        // Convert target RPM to delay in timer ticks.
        let target_delay_us = 60.0 / full_steps_per_revolution as f32 * TIMER_HZ_MICROS as f32 / target_rpm;
        let mut first_delay_us = (2.0 / (3.35 * acceleration_rpm_s as f32)).sqrt() // 3.35 correction factor
            * 0.676 * TIMER_HZ_MICROS as f32;
        if first_delay_us < target_delay_us {
            first_delay_us = target_delay_us;
        }
        let alpha = find_alpha_value(first_delay_us, target_delay_us, expected_accel_duration_ms * 1000.0, 1.0)?;
        Ok(Stepgen {
            operating_mode,
            current_step: 0.0,
            acceleration_steps: 0.0,
            acceleration_duration_ms: TimerDurationU64::<TIMER_HZ_MILLIS>::from_ticks(0),
            current_delay_us: 0.0,
            current_duration_ms: TimerDurationU64::<TIMER_HZ_MILLIS>::from_ticks(0),
            first_delay_us,
            target_step: target_step as f32,
            target_duration_ms,
            target_delay_us,
            start_time_ms: None,
            is_acceleration_done: false,
            is_sigmoid_profile: enable_sigmoid_profile,
            expected_accel_duration_ms,
            alpha,
            current_delay_accumulator_us: 0.0,
        })
    }

    /// Returns 'None' if it should stop. Otherwise, returns delay as u64.
    pub fn next_delay(&mut self, timer_ms: Option<TimerInstantU64<TIMER_HZ_MILLIS>>) -> Option<u64> {
        if timer_ms.is_none() && self.operating_mode == OperatingMode::Duration {
            return None;
        }
        match self.operating_mode {
            OperatingMode::Step => self.next_delay_step(),
            OperatingMode::Duration => self.next_delay_duration(timer_ms.unwrap()),
        }
    }

    /// Duration operating mode
    pub fn next_delay_duration(&mut self, current_ms: TimerInstantU64<TIMER_HZ_MILLIS>) -> Option<u64> {
        // If start time is None, we're at the start of the move. Set start time.
        if self.start_time_ms.is_none() {
            self.start_time_ms = Some(current_ms);
            self.acceleration_steps += 1.0;
            self.current_delay_us = self.first_delay_us;
            self.current_step += 1.0;
            self.current_delay_accumulator_us  += self.first_delay_us;
            return Some(self.first_delay_us as u64);
        }
        self.current_duration_ms = current_ms - self.start_time_ms.unwrap();

        // We reached the target duration. Return None.
        if self.current_duration_ms >= self.target_duration_ms {
            return None;
        }

        // If the time remaining is less than the time it took to accelerate, slow down.
        let time_remaining = self.target_duration_ms - self.current_duration_ms;
        if time_remaining <= self.acceleration_duration_ms {
            self.slow_down();
            self.current_delay_accumulator_us += self.current_delay_us;
            return Some(self.current_delay_us as u64);
        }

        // If the current delay is equal to the target delay, we're at the target speed. Return the current delay.
        // Else, we need to accelerate.
        if self.current_delay_us == self.target_delay_us {
            self.is_acceleration_done = true;
            self.current_step += 1.0;
            self.current_delay_accumulator_us += self.current_delay_us;
            Some(self.current_delay_us as u64)
        } else {
            self.speed_up();
            self.current_delay_accumulator_us += self.current_delay_us;
            Some(self.current_delay_us as u64)
        }
    }

    /// Step operating mode
    pub fn next_delay_step(&mut self) -> Option<u64> {
        // If current step is 0, we're at the start of the move. Return the first delay and increase acceleration steps and current step.
        if self.current_step == 0.0 {
            self.acceleration_steps += 1.0;
            self.current_step += 1.0;
            self.current_delay_us = self.first_delay_us;
            return Some(self.first_delay_us as u64);
        }

        // If current step is bigger or equal to the target step, we're at the end of the move. Return None.
        if self.current_step >= self.target_step {
            return None;
        }

        // If the current step is bigger or equal than the target step minus the acceleration steps, we need to slow down.
        if self.current_step >= self.target_step - self.acceleration_steps {
            self.slow_down();
            return Some(self.current_delay_us as u64);
        }

        // If the current delay is equal to the target delay, we're at the target speed. Return the current delay.
        // Else, we need to accelerate.
        if self.current_delay_us == self.target_delay_us {
            self.is_acceleration_done = true;
            self.current_step += 1.0;
            Some(self.current_delay_us as u64)
        } else {
            self.speed_up();
            Some(self.current_delay_us as u64)
        }
    }

    fn speed_up(&mut self) {
        match self.is_sigmoid_profile {
            true => {
                // let accel_fn = |t: f32| self.first_delay + (self.target_delay - self.first_delay) / (1.0 + (-0.01 * (t - (self.expected_accel_duration_ms / 2.0))).exp());
                self.current_delay_us = sigmoid_delay_us(self.current_delay_accumulator_us, self.first_delay_us, self.target_delay_us, self.alpha, self.expected_accel_duration_ms * 1000.0)
            }
            false => {
                let denom = 4.0 * self.acceleration_steps + 1.0;
                self.current_delay_us -= (2.0 * self.current_delay_us) / denom;
                if self.current_delay_us < self.target_delay_us {
                    self.current_delay_us = self.target_delay_us;
                }
            }
        }
        self.acceleration_steps += 1.0;
        self.acceleration_duration_ms = self.current_duration_ms;
        self.current_step += 1.0;
    }

    fn slow_down(&mut self) {
        match self.is_sigmoid_profile {
            true => {
                // let decel_fn = |t: f32| self.target_delay + (self.first_delay - self.target_delay) / (1.0 + (-0.01 * (t - (self.target_duration_ms - self.acceleration_duration_ms).ticks() as f32 - (self.acceleration_duration_ms.ticks() as f32 / 2.0))).exp());

                self.current_delay_us = sigmoid_delay_us(self.current_delay_accumulator_us - (self.target_duration_ms.ticks() as f32 - self.expected_accel_duration_ms) * 1000.0, self.target_delay_us, self.first_delay_us, self.alpha, self.expected_accel_duration_ms * 1000.0);
            }
            false => {
                let denom = 4.0 * self.acceleration_steps - 1.0;
                self.current_delay_us += (2.0 * self.current_delay_us) / denom;
            }
        }
        if self.acceleration_steps == 0.0 {
            self.acceleration_steps = 1.0;
        }
        self.acceleration_steps -= 1.0;
        self.current_step += 1.0;
    }

    pub fn get_current_step(&self) -> u64 {
        self.current_step as u64
    }

    pub fn get_acceleration_steps(&self) -> u64 {
        self.acceleration_steps as u64
    }

    pub fn get_acceleration_duration_ms(&self) -> u64 {
        self.acceleration_duration_ms.ticks()
    }

    pub fn is_acceleration_done(&self) -> bool {
        self.is_acceleration_done
    }
}