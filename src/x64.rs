use fugit::{TimerDurationU64, TimerInstantU64};
use micromath::F32Ext;

use crate::utils::enums::{Error, OperatingMode};

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
    acceleration_duration_ms: TimerDurationU64<TIMER_HZ_MILLIS>,
    // Previously calculated delay
    current_delay: f32,
    current_duration_ms: TimerDurationU64<TIMER_HZ_MILLIS>,
    // First step delay
    first_delay: f32,
    // Target step
    target_step: f32,
    // Target duration
    target_duration_ms: TimerDurationU64<TIMER_HZ_MILLIS>,
    // Target speed delay
    target_delay: f32,
    // Start time
    start_time_ms: Option<TimerInstantU64<TIMER_HZ_MILLIS>>,
    is_acceleration_done: bool,
    // Sigmoid parameter
    alpha_sigmoid: Option<f32>,
}

impl<const TIMER_HZ_MICROS: u32> Stepgen<TIMER_HZ_MICROS> {
    /// Create new copy of stepgen.
    pub fn new(target_rpm: u32, acceleration: u32, target_step: u64, target_duration_ms: u64, alpha_sigmoid: Option<u16>, full_steps_per_revolution: u16) -> Result<Stepgen<TIMER_HZ_MICROS>, Error> {
        if acceleration == 0 {
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
        // Convert target RPM to delay in timer ticks.
        let target_delay = 60.0 /full_steps_per_revolution as f32 * TIMER_HZ_MICROS as f32 / target_rpm as f32;
        let mut first_delay = (2.0 / acceleration as f32 * 3.35).sqrt() // 3.35 correction factor
            * 0.676 * TIMER_HZ_MICROS as f32;
        if first_delay < target_delay {
            first_delay = target_delay;
        }
        let alpha_sigmoid = if let Some(a) = alpha_sigmoid {
            if a > 1000 {
                return Err(Error::InvalidAlpha);
            }
            Some(a as f32 / 1000.0)
        } else {
            None
        };
        let target_duration_ms = TimerDurationU64::<TIMER_HZ_MILLIS>::from_ticks(target_duration_ms);
        Ok(Stepgen {
            operating_mode,
            current_step: 0.0,
            acceleration_steps: 0.0,
            acceleration_duration_ms: TimerDurationU64::<TIMER_HZ_MILLIS>::from_ticks(0),
            current_delay: 0.0,
            current_duration_ms: TimerDurationU64::<TIMER_HZ_MILLIS>::from_ticks(0),
            first_delay,
            target_step: target_step as f32,
            target_duration_ms,
            target_delay,
            start_time_ms: None,
            is_acceleration_done: false,
            alpha_sigmoid,
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
            self.current_delay = self.first_delay;
            self.current_step += 1.0;
            return Some(self.first_delay as u64);
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
            return Some(self.current_delay as u64);
        }

        // If the current delay is equal to the target delay, we're at the target speed. Return the current delay.
        // Else, we need to accelerate.
        if self.current_delay == self.target_delay {
            self.is_acceleration_done = true;
            self.current_step += 1.0;
            Some(self.current_delay as u64)
        } else {
            self.speed_up();
            Some(self.current_delay as u64)
        }
    }

    /// Step operating mode
    pub fn next_delay_step(&mut self) -> Option<u64> {
        // If current step is 0, we're at the start of the move. Return the first delay and increase acceleration steps and current step.
        if self.current_step == 0.0 {
            self.acceleration_steps += 1.0;
            self.current_step += 1.0;
            self.current_delay = self.first_delay;
            return Some(self.first_delay as u64);
        }

        // If current step is bigger or equal to the target step, we're at the end of the move. Return None.
        if self.current_step >= self.target_step {
            return None;
        }

        // If the current step is bigger or equal than the target step minus the acceleration steps, we need to slow down.
        if self.current_step >= self.target_step - self.acceleration_steps {
            self.slow_down();
            return Some(self.current_delay as u64);
        }

        // If the current delay is equal to the target delay, we're at the target speed. Return the current delay.
        // Else, we need to accelerate.
        if self.current_delay == self.target_delay {
            self.is_acceleration_done = true;
            self.current_step += 1.0;
            Some(self.current_delay as u64)
        } else {
            self.speed_up();
            Some(self.current_delay as u64)
        }
    }

    fn speed_up(&mut self) {
        match self.alpha_sigmoid {
            Some(alpha) => {
                let progress = self.acceleration_steps / self.get_acceleration_steps() as f32;
                let sigmoid = 1.0 / (1.0 + (-alpha * progress).exp());
                self.current_delay = self.first_delay + self.target_delay - self.first_delay * sigmoid;
            }
            None => {
                let denom = 4.0 * self.acceleration_steps + 1.0;
                self.current_delay -= (2.0 * self.current_delay) / denom;
                if self.current_delay < self.target_delay {
                    self.current_delay = self.target_delay;
                }
            }
        }
        self.acceleration_steps += 1.0;
        self.acceleration_duration_ms = self.current_duration_ms;
        self.current_step += 1.0
    }

    fn slow_down(&mut self) {
        match self.alpha_sigmoid {
            Some(alpha) => {
                let progress = self.acceleration_steps / self.get_acceleration_steps() as f32;
                let sigmoid = 1.0 / (1.0 + (-alpha * progress).exp());
                self.current_delay = self.target_delay + self.first_delay - self.target_delay * (1.0 - sigmoid);
            }
            None => {
                let denom = 4.0 * self.acceleration_steps - 1.0;
                self.current_delay += (2.0 * self.current_delay) / denom;
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