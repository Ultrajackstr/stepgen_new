use fixed::FixedU64;
use fixed::types::extra::U32;
use fixed::types::U32F32;
use fixed_macro::fixed;
use fixed_sqrt::FixedSqrt;
use fugit::{TimerDurationU64, TimerInstantU64};

use crate::utils::enums::{Error, OperatingMode};

type Fix = FixedU64<U32>;

const TWO: U32F32 = fixed!(2: U32F32);
const FOUR: U32F32 = fixed!(4: U32F32);

const TIMER_HZ_MILLIS: u32 = 1_000; // One tick is 1 millisecond.

/// State of the stepgen.
#[derive(Debug)]
pub struct Stepgen<const TIMER_HZ_MICROS: u32> {
    // Operating mode
    operating_mode: OperatingMode,
    current_step: Fix,
    // Amount of acceleration steps we've taken so far
    acceleration_steps: Fix,
    // How long did the acceleration take
    acceleration_duration_ms: Option<TimerDurationU64<TIMER_HZ_MILLIS>>,
    // Previously calculated delay
    current_delay: Fix,
    // First step delay
    first_delay: Fix,
    // Target step
    target_step: Option<Fix>,
    // Target duration
    target_duration_ms: Option<TimerDurationU64<TIMER_HZ_MILLIS>>,
    // Target speed delay
    target_delay: Fix,
    // Start time
    start_time_ms: Option<TimerInstantU64<TIMER_HZ_MILLIS>>,
}

impl<const TIMER_HZ_MICROS: u32> Stepgen<TIMER_HZ_MICROS> {
    /// Create new copy of stepgen.
    pub fn new(target_rpm: u32, accel: u32, target_step: Option<u32>, target_duration_ms: Option<u32>) -> Result<Stepgen<TIMER_HZ_MICROS>, Error> {
        if target_step.is_none() && target_duration_ms.is_none() {
            return Err(Error::NoStepTargetAndNoDuration);
        } else if target_step.is_some() && target_duration_ms.is_some() {
            return Err(Error::BothStepTargetAndDuration);
        }
        let operating_mode = if target_step.is_some() {
            OperatingMode::Step
        } else {
            OperatingMode::Duration
        };
        // Convert target RPM to delay in timer ticks.
        let target_delay: Fix = Fix::from_num(60) / Fix::from_num(200) * Fix::from_num(TIMER_HZ_MICROS) / Fix::from_num(target_rpm);
        let mut first_delay: Fix = (TWO / (Fix::from_num(accel) * Fix::from_num(3.35))).sqrt() // 3.35 correction factor
            * Fix::from_num(0.676) * Fix::from_num(TIMER_HZ_MICROS);
        if first_delay < target_delay {
            first_delay = target_delay;
        }
        let target_step = target_step.map(Fix::from_num);
        let target_duration_ms = target_duration_ms.map(|duration_ms| TimerDurationU64::<TIMER_HZ_MILLIS>::millis(duration_ms as u64));
        Ok(Stepgen {
            operating_mode,
            current_step: Fix::ZERO,
            acceleration_steps: Fix::from_num(0),
            acceleration_duration_ms: None,
            current_delay: Fix::from_num(0),
            first_delay,
            target_step,
            target_duration_ms,
            target_delay,
            start_time_ms: None,
        })
    }

    /// Returns 'None' if should stop. Otherwise, returns delay as u32.
    pub fn next_delay(&mut self, current_ms: Option<u64>) -> Option<u32> {
        if current_ms.is_none() && self.operating_mode == OperatingMode::Duration {
            return None;
        }
        match self.operating_mode {
            OperatingMode::Step => self.next_delay_step(),
            OperatingMode::Duration => self.next_delay_duration(current_ms.unwrap()),
        }
    }

    /// Duration operating mode
    fn next_delay_duration(&mut self, current_ms: u64) -> Option<u32> {
        let millis_instant = TimerInstantU64::<TIMER_HZ_MILLIS>::from_ticks(current_ms);
        // If start time is None, we're at the start of the move. Set start time.
        if self.start_time_ms.is_none() {
            self.start_time_ms = Some(millis_instant);
        }
        let current_duration = millis_instant - self.start_time_ms.unwrap();
        // We reached the target duration. Return None.
        if current_duration >= self.target_duration_ms.unwrap() {
            return None;
        }
        // If current step is 0, we're at the start of the move. Return the first delay and increase acceleration steps and current step.
        if self.current_step == Fix::ZERO {
            self.acceleration_steps += Fix::ONE;
            self.current_step += Fix::ONE;
            self.current_delay = self.first_delay;
            return Some(self.first_delay.to_num::<u32>());
        }

        // If the time remaining is less than the time it took to accelerate, slow down.
        if let Some(acceleration_duration_ms) = &self.acceleration_duration_ms {
            let time_remaining = self.target_duration_ms.unwrap() - current_duration;
            if time_remaining <= *acceleration_duration_ms {
                self.slow_down();
                return Some(self.current_delay.to_num::<u32>());
            }
        }

        // If the current delay is equal to the target delay, we're at the target speed. Return the current delay.
        // Else, we need to accelerate.
        if self.current_delay == self.target_delay {
            if self.acceleration_duration_ms.is_none() {
                self.acceleration_duration_ms = Some(current_duration);
            }
            self.current_step += Fix::ONE;
            Some(self.current_delay.to_num::<u32>())
        } else {
            self.speed_up();
            Some(self.current_delay.to_num::<u32>())
        }
    }

    /// Step operating mode
    fn next_delay_step(&mut self) -> Option<u32> {
        // If current step is 0, we're at the start of the move. Return the first delay and increase acceleration steps and current step.
        if self.current_step == Fix::ZERO {
            self.acceleration_steps += Fix::ONE;
            self.current_step += Fix::ONE;
            self.current_delay = self.first_delay;
            return Some(self.first_delay.to_num::<u32>());
        }

        // If current step is bigger or equal to the target step, we're at the end of the move. Return None.
        if self.current_step >= self.target_step.unwrap() {
            return None;
        }

        // If the current step is bigger are equal than the target step minus the acceleration steps, we need to slow down.
        if self.current_step >= self.target_step.unwrap() - self.acceleration_steps {
            self.slow_down();
            return Some(self.current_delay.to_num::<u32>());
        }

        // If the current delay is equal to the target delay, we're at the target speed. Return the current delay.
        // Else, we need to accelerate.
        if self.current_delay == self.target_delay {
            self.current_step += Fix::ONE;
            Some(self.current_delay.to_num::<u32>())
        } else {
            self.speed_up();
            Some(self.current_delay.to_num::<u32>())
        }
    }

    /// Speed up function
    fn speed_up(&mut self) {
        let denom: Fix = FOUR * self.acceleration_steps + Fix::ONE;
        self.current_delay -= (TWO * self.current_delay) / denom;
        // if the calculated delay is less than the target delay, we are done speeding up.
        if self.current_delay < self.target_delay {
            self.current_delay = self.target_delay;
        }
        self.acceleration_steps += Fix::ONE;
        self.current_step += Fix::ONE;
    }


    /// Slow down function
    fn slow_down(&mut self) {
        let denom: Fix = FOUR * self.acceleration_steps - Fix::ONE;
        self.current_delay += (TWO * self.current_delay) / denom;
        self.acceleration_steps -= Fix::ONE;
        self.current_step += Fix::ONE;
    }
}