use fixed::FixedU64;
use fixed::types::extra::U32;
use fixed::types::U32F32;
use fixed_macro::fixed;
use fixed_sqrt::FixedSqrt;

type Fix = FixedU64<U32>;

const TWO: U32F32 = fixed!(2: U32F32);
const FOUR: U32F32 = fixed!(4: U32F32);

/// State of the stepgen.
#[derive(Debug)]
pub struct Stepgen<const TIMER_HZ_MICROS: u32> {
    current_step: Fix,
    // Amount of acceleration steps we've taken so far
    acceleration_steps: Fix,
    // Previously calculated delay
    current_delay: Fix,
    // First step delay
    first_delay: Fix,
    // Target step
    target_step: Fix,
    // Target speed delay
    target_delay: Fix,
}

impl<const TIMER_HZ_MICROS: u32> Stepgen<TIMER_HZ_MICROS> {
    /// Create new copy of stepgen.
    pub fn new(target_rpm: u32, accel: u32, target_step: u32) -> Stepgen<TIMER_HZ_MICROS> {
        // Convert target RPM to delay in timer ticks.
        let target_delay: Fix = Fix::from_num(60) / Fix::from_num(200) * Fix::from_num(TIMER_HZ_MICROS) / Fix::from_num(target_rpm);
        // Calculate first delay based on acceleration.
        let mut first_delay: Fix = (TWO / (Fix::from_num(accel) * Fix::from_num(3.35))).sqrt() // 3.35 correction factor
            * Fix::from_num(0.676) * Fix::from_num(TIMER_HZ_MICROS);
        // If first_delay is smaller than target_delay, first_delay = target_delay
        if first_delay < target_delay {
            first_delay = target_delay;
        }
        Stepgen {
            current_step: Fix::ZERO,
            acceleration_steps: Fix::from_num(0),
            current_delay: Fix::from_num(0),
            first_delay,
            target_step: Fix::from_num(target_step),
            target_delay,
        }
    }

    /// Returns 'None' if should stop. Otherwise, returns delay as u32.
    pub fn next_delay(&mut self) -> Option<u32> {
        // If current step is 0, we're at the start of the move. Return the first delay and increase acceleration steps and current step.
        if self.current_step == Fix::ZERO {
            self.acceleration_steps += Fix::ONE;
            self.current_step += Fix::ONE;
            self.current_delay = self.first_delay;
            return Some(self.first_delay.to_num::<u32>());
        }

        // If current step is bigger or equal to the target step, we're at the end of the move. Return None.
        if self.current_step >= self.target_step {
            return None;
        }

        // If the current step is bigger are equal than the target step minus the acceleration steps, we need to slow down.
        if self.current_step >= self.target_step - self.acceleration_steps {
            self.slow_down();
            return Some(self.current_delay.to_num::<u32>());
        }

        // If the current delay is equal to the target delay, we're at the target speed. Return the current delay.
        // Else, we need to accelerate.
        return if self.current_delay == self.target_delay {
            self.current_step += Fix::ONE;
            Some(self.current_delay.to_num::<u32>())
        } else {
            self.speed_up();
            Some(self.current_delay.to_num::<u32>())
        };
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