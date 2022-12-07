#![no_std]
#![deny(warnings)]

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Error {
    TooSlow,
    TooFast,
    SpeedAccelerationNotSet,
}

pub type Result = core::result::Result<(), Error>;

/// State of the stepgen.
#[derive(Debug)]
pub struct Stepgen {
    // Current step
    current_step: u32,

    // Amount of acceleration steps we've taken so far
    speed: u32,
    // Previously calculated delay, in 16.16 format
    delay: u32,

    // If slewing, this will be the slewing delay. Switched to this mode once
    // we overshoot target speed. 16.16 format.
    slewing_delay: u32,

    // Timer frequency
    ticks_per_second: u32,
    // First step delay, in 16.16 format
    first_delay: u32,
    // Target step
    target_step: u32,
    // Target speed delay, in 16.16 format
    target_delay: u32,
}

impl Stepgen {
    /// Create new copy of stepgen. `ticks_per_second` defines size of each tick stepgen operates.
    /// All settings (acceleration, speed) and current parameters (speed) are defined in terms of
    /// these ticks.
    pub const fn new(ticks_per_second: u32) -> Stepgen {
        Stepgen {
            current_step: 0,
            speed: 0,
            delay: 0,
            slewing_delay: 0,
            ticks_per_second,
            first_delay: 0,
            target_step: 0,
            target_delay: 0,
        }
    }

    pub fn set_acceleration(&mut self, acceleration: f32) -> Result {
        let delay = self.ticks_per_second as f32 * micromath::F32Ext::sqrt(2.0 / acceleration) * 0.676;
        self.first_delay = delay as u32;
        Ok(())
    }

    pub fn set_target_step(&mut self, target_step: u32) -> Result {
        if self.target_delay == 0 || self.first_delay == 0 {
            return Err(Error::SpeedAccelerationNotSet);
        }
        self.target_step = target_step;
        Ok(())
    }

    pub fn set_target_speed(&mut self, target_speed: f32) -> Result {
        // if target_speed == 0_f32 {
        //     // Too slow, speed is zero
        //     return Err(Error::TooSlow);
        // }
        let delay = self.ticks_per_second as f32 / target_speed;
        self.target_delay = delay as u32;
        Ok(())
    }

    /// Current step stepgen is at.
    pub fn current_step(&self) -> u32 {
        self.current_step
    }

    /// Target step stepgen should stop at. Note that if stepper is running too fast, it might not
    /// be able to stop exactly at this step. This could happen when target step is updated after
    /// stepper motor accelerated to certain speed.
    pub fn target_step(&self) -> u32 {
        self.target_step
    }

    /// Get estimated current speed, in 24.8 format
    pub fn current_speed(&self) -> u32 {
        let delay = if self.slewing_delay != 0 { self.slewing_delay } else { self.delay };
        if delay != 0 {
            self.ticks_per_second / delay
        } else {
            0
        }
    }

    /// If we are running at target speed
    pub fn is_at_speed(&self) -> bool {
        self.slewing_delay != 0
    }

    /// Returns '0' if should stop. Otherwise, returns timer delay in 24.8 format
    fn next_delay(&mut self) -> u32 {
        let target_step = self.target_step;
        let target_delay = self.target_delay;
        let st = self.current_step;

        // We are at the stop point and speed is zero -- return "stopped" (delay of 0)
        if st >= target_step && self.speed <= 1 {
            self.speed = 0;
            return 0;
        }

        // Stop slewing if target delay was changed
        if self.slewing_delay != 0 && self.slewing_delay != target_delay {
            self.slewing_delay = 0;
        }

        // Steps made so far
        self.current_step += 1;

        if self.speed == 0 {
            if target_delay > self.first_delay {
                // No acceleration is necessary -- just return the target delay
                target_delay
            } else {
                // First step: load first delay, count as one acceleration step
                self.delay = self.first_delay;
                self.speed = 1;
                self.delay
            };
        }

        // Calculate the projected step we would stop at if we start decelerating right now
        let est_stop = st + self.speed;
        if est_stop == target_step {
            // We would stop one step earlier than we want, so let's just
            // return the same delay as the current one and start deceleration
            // on the next step.
        } else if est_stop > target_step {
            // We need to stop at target step, slow down
            self.slowdown();

            // We are not slewing even though we could have slowed down below the slewing speed
            self.slewing_delay = 0;
        } else if self.slewing_delay == 0 && self.delay < target_delay {
            // Not slewing and running too fast, slow down
            self.slowdown();

            // Switch to slewing if we slowed down enough
            if self.delay >= target_delay {
                self.slewing_delay = target_delay;
            }
        } else if self.slewing_delay == 0 && self.delay > target_delay {
            // Not slewing and running too slow, speed up
            self.speedup();

            // Switch to slewing if we have accelerated enough
            if self.delay <= target_delay {
                self.slewing_delay = target_delay;
            }
        }

        // If slewing, return slew delay. delay should be close enough, but could
        // be different due to the accumulated rounding errors
        if self.slewing_delay != 0 { self.slewing_delay } else { self.delay }
    }


    fn speedup(&mut self) {
        let denom = 4 * self.speed + 1;
        self.delay -= (2 * self.delay + denom / 2) / denom;
        self.speed += 1;
    }

    fn slowdown(&mut self) {
        self.speed -= 1;
        let denom = 4 * self.speed - 1;
        self.delay += (2 * self.delay + denom / 2) / denom;
    }
}

impl Iterator for Stepgen {
    type Item = u32;

    fn next(&mut self) -> Option<Self::Item> {
        match Stepgen::next_delay(self) {
            0 => None,
            v => Some(v)
        }
    }
}