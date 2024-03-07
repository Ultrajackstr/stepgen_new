use core::fmt::Display;

// Error Enum
#[derive(Debug)]
pub enum Error {
    NoStepTargetAndNoDuration,
    BothStepTargetAndDuration,
    ZeroAcceleration,
    ZeroRpm,
    InvalidState,
    InvalidAlpha,
}

impl Error {
    pub fn description(&self) -> &str {
        match self {
            Error::NoStepTargetAndNoDuration => "No targets: No step target and no duration target",
            Error::BothStepTargetAndDuration => "Multiple targets: Step target and duration target",
            Error::ZeroAcceleration => "Zero acceleration",
            Error::ZeroRpm => "Zero RPM",
            Error::InvalidState => "Invalid state",
            Error::InvalidAlpha => "Invalid alpha",
        }
    }
}

impl Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
        write!(f, "{}", self.description())
    }
}

// Step target or duration target
#[derive(Debug, PartialEq)]
pub enum OperatingMode {
    Step,
    Duration,
}