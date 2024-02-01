use fixed::FixedU64;
use fixed::types::U32F32;
use fixed_macro::fixed;
use fixed_sqrt::FixedSqrt;
use fugit::{TimerDurationU64, TimerInstantU64};

use crate::utils::enums::{Error, OperatingMode};

type Fix = FixedU64<32>;

const TWO: U32F32 = fixed!(2: U32F32);
const FOUR: U32F32 = fixed!(4: U32F32);

const TIMER_HZ_MILLIS: u32 = 1_000; // One tick is 1 millisecond.

/// State of the stepgen.
#[derive(Debug)]
pub struct Stepgen<const TIMER_HZ_MICROS: u32> {
    // Operating mode
    operating_mode: OperatingMode,
    current_step: u64,
    // Amount of acceleration steps we've taken so far
    acceleration_steps: Fix,
    // How long did the acceleration take
    acceleration_duration_ms: TimerDurationU64<TIMER_HZ_MILLIS>,
    // Previously calculated delay
    current_delay: Fix,
    current_duration_ms: TimerDurationU64<TIMER_HZ_MILLIS>,
    // First step delay
    first_delay: Fix,
    // Target step
    target_step: u64,
    // Target duration
    target_duration_ms: TimerDurationU64<TIMER_HZ_MILLIS>,
    // Target speed delay
    target_delay: Fix,
    // Start time
    start_time_ms: Option<TimerInstantU64<TIMER_HZ_MILLIS>>,
}

impl<const TIMER_HZ_MICROS: u32> Stepgen<TIMER_HZ_MICROS> {
    /// Create new copy of stepgen.
    pub fn new(target_rpm: u32, acceleration: u32, target_step: u64, target_duration_ms: u64, full_steps_per_revolution: u16) -> Result<Stepgen<TIMER_HZ_MICROS>, Error> {
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
        let target_delay: Fix = Fix::from_num(60) / Fix::from_num(full_steps_per_revolution) * Fix::from_num(TIMER_HZ_MICROS) / Fix::from_num(target_rpm);
        let mut first_delay: Fix = (TWO / (Fix::from_num(acceleration) * Fix::from_num(3.35))).sqrt() // 3.35 correction factor
            * Fix::from_num(0.676) * Fix::from_num(TIMER_HZ_MICROS);
        if first_delay < target_delay {
            first_delay = target_delay;
        }
        let target_duration_ms = TimerDurationU64::<TIMER_HZ_MILLIS>::from_ticks(target_duration_ms);
        Ok(Stepgen {
            operating_mode,
            current_step: 0,
            acceleration_steps: Fix::from_num(0),
            acceleration_duration_ms: TimerDurationU64::<TIMER_HZ_MILLIS>::from_ticks(0),
            current_delay: Fix::from_num(0),
            current_duration_ms: TimerDurationU64::<TIMER_HZ_MILLIS>::from_ticks(0),
            first_delay,
            target_step,
            target_duration_ms,
            target_delay,
            start_time_ms: None,
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
            self.acceleration_steps += Fix::ONE;
            self.current_delay = self.first_delay;
            self.current_step += 1;
            return Some(self.first_delay.to_num::<u64>());
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
                return Some(self.current_delay.to_num::<u64>());
            }

        // If the current delay is equal to the target delay, we're at the target speed. Return the current delay.
        // Else, we need to accelerate.
        if self.current_delay == self.target_delay {
            self.current_step += 1;
            Some(self.current_delay.to_num::<u64>())
        } else {
            self.speed_up();
            Some(self.current_delay.to_num::<u64>())
        }
    }

    /// Step operating mode
    pub fn next_delay_step(&mut self) -> Option<u64> {
        // If current step is 0, we're at the start of the move. Return the first delay and increase acceleration steps and current step.
        if self.current_step == 0 {
            self.acceleration_steps += Fix::ONE;
            self.current_step += 1;
            self.current_delay = self.first_delay;
            return Some(self.first_delay.to_num::<u64>());
        }

        // If current step is bigger or equal to the target step, we're at the end of the move. Return None.
        if self.current_step >= self.target_step {
            return None;
        }

        // If the current step is bigger are equal than the target step minus the acceleration steps, we need to slow down.
        if self.current_step >= self.target_step - self.acceleration_steps.to_num::<u64>() {
            self.slow_down();
            return Some(self.current_delay.to_num::<u64>());
        }

        // If the current delay is equal to the target delay, we're at the target speed. Return the current delay.
        // Else, we need to accelerate.
        if self.current_delay == self.target_delay {
            self.current_step += 1;
            Some(self.current_delay.to_num::<u64>())
        } else {
            self.speed_up();
            Some(self.current_delay.to_num::<u64>())
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
        self.acceleration_duration_ms = self.current_duration_ms;
        self.current_step += 1;
    }


    /// Slow down function
    fn slow_down(&mut self) {
        let denom: Fix = FOUR * self.acceleration_steps - Fix::ONE;
        self.current_delay += (TWO * self.current_delay) / denom;
        if self.acceleration_steps == Fix::ZERO { // Prevent underflow
            self.acceleration_steps = Fix::ONE;
        }
        self.acceleration_steps -= Fix::ONE;
        self.current_step += 1;
    }

    pub fn get_current_step(&self) -> u64 {
        self.current_step
    }
}