#![no_std]

use fixed::FixedU32;
use fixed::types::extra::{U0, U12, U18, U32};
use fixed::types::U20F12;
use fixed_macro::fixed;
use fixed_sqrt::FixedSqrt;

type Fix = FixedU32<U12>;
type Fix18 = FixedU32<U18>;
// Higher precision for the sqrt function.
type Fix32 = FixedU32<U32>;
// Higher precision for the sqrt function.
type Fix0 = FixedU32<U0>; // Equivalent to u32.

const ZERO_POINT_TWENTY_SIX: U20F12 = fixed!(0.26: U20F12);
const TWO: U20F12 = fixed!(2: U20F12);
const FOUR: U20F12 = fixed!(4: U20F12);

/// State of the stepgen.
#[derive(Debug)]
pub struct Stepgen<const TIMER_HZ_MICROS: u32> {
    current_step: Fix0,
    // Amount of acceleration steps we've taken so far
    acceleration_steps: Fix,
    // Previously calculated delay
    current_delay: Fix,
    // If slewing, this will be the slewing delay. Switched to this mode once
    // we overshoot target speed.
    slewing_delay: Fix,
    // First step delay
    first_delay: Fix,
    // Target step
    target_step: Fix0,
    // Target speed delay
    target_delay: Fix,
}

impl<const TIMER_HZ_MICROS: u32> Stepgen<TIMER_HZ_MICROS> {
    /// Create new copy of stepgen.
    pub fn new(target_rpm: u16, accel: u16, target_step: u32) -> Stepgen<TIMER_HZ_MICROS> {
        if !(150..=4_800).contains(&accel) || target_rpm < 32 {
            return Stepgen {
                current_step: Fix0::ZERO,
                acceleration_steps: Fix::from_num(150),
                current_delay: Fix::ZERO,
                slewing_delay: Fix::ZERO,
                first_delay: Fix::ZERO,
                target_step: Fix0::ZERO,
                target_delay: Fix::ZERO,
            };
        }
        // Convert target RPM to delay in timer ticks.
        let target_delay: Fix = Fix::from_num(60) / Fix::from_num(200) * Fix::from_num(TIMER_HZ_MICROS) / Fix::from_num(target_rpm);
        // Calculate first delay based on acceleration.
        let first_delay: Fix = Fix::from_num(Fix32::from_num(Fix18::from_num(2u8) / (Fix18::from_num(accel) * Fix18::from_num(3))).sqrt()
            * Fix32::from_num(0.676)) * Fix::from_num(TIMER_HZ_MICROS);
        Stepgen {
            current_step: Fix0::ZERO,
            acceleration_steps: Fix::from_num(0),
            current_delay: Fix::from_num(0),
            slewing_delay: Fix::from_num(0),
            first_delay,
            target_step: Fix0::from_num(target_step),
            target_delay,
        }
    }

    /// Returns '0' if should stop. Otherwise, returns timer delay in 24.8 format
    pub fn next_delay(&mut self) -> Option<u32> {
        // We are at the stop point and speed is zero -- return "stopped" (delay of 0)
        if self.current_step >= self.target_step && self.acceleration_steps <= Fix::ONE {
            self.acceleration_steps = Fix::ZERO;
            return None;
        }

        // Stop slewing if target delay was changed
        if self.slewing_delay != Fix::ZERO && self.slewing_delay != self.target_delay {
            self.slewing_delay = Fix::ZERO;
        }

        // Steps made so far
        self.current_step += Fix0::ONE;

        if self.acceleration_steps == Fix::ZERO {
            return if self.target_delay > self.first_delay {
                // No acceleration is necessary -- just return the target delay
                Some(self.target_delay.to_num::<u32>())
            } else {
                // First step: load first delay, count as one acceleration step
                self.current_delay = self.first_delay;
                self.acceleration_steps = Fix::ONE;
                Some(self.current_delay.to_num::<u32>())
            };
        }

        // Calculate the projected step we would stop at if we start decelerating right now
        let estimated_stop_step = self.current_step + self.acceleration_steps.to_num::<Fix0>();
        if estimated_stop_step == self.target_step {
            // We would stop one step earlier than we want, so let's just
            // return the same delay as the current one and start deceleration
            // on the next step.
        } else if estimated_stop_step > self.target_step {
            // We need to stop at target step, slow down
            self.slowdown();

            // We are not slewing even though we could have slowed down below the slewing speed
            self.slewing_delay = Fix::ZERO;
        } else if self.slewing_delay == Fix::ZERO && self.current_delay < self.target_delay {
            // Not slewing and running too fast, slow down
            self.slowdown();

            // Switch to slewing if we slowed down enough
            if self.current_delay >= self.target_delay {
                self.slewing_delay = self.target_delay;
            }
        } else if self.slewing_delay == Fix::ZERO && self.current_delay > self.target_delay {
            // Not slewing and running too slow, speed up
            self.speedup();

            // Switch to slewing if we have accelerated enough
            if self.current_delay <= self.target_delay {
                self.slewing_delay = self.target_delay;
            }
        }

        // If slewing, return slew delay. delay should be close enough, but could
        // be different due to the accumulated rounding errors
        if self.slewing_delay != Fix::ZERO { Some(self.slewing_delay.to_num::<u32>()) } else { Some(self.current_delay.to_num::<u32>()) }
    }


    fn speedup(&mut self) {
        let denom = FOUR * self.acceleration_steps + Fix::ONE;
        self.current_delay -= (TWO * self.current_delay) / denom;
        self.acceleration_steps += Fix::ONE;
    }

    fn slowdown(&mut self) {
        if self.acceleration_steps < ZERO_POINT_TWENTY_SIX { // Prevent underflow.
            self.acceleration_steps = ZERO_POINT_TWENTY_SIX
        }
        let denom = FOUR * self.acceleration_steps - Fix::ONE;
        self.current_delay += (TWO * self.current_delay) / denom;
        self.acceleration_steps -= Fix::ONE;
    }
}