// Error Enum
#[derive(Debug)]
pub enum Error {
    NoStepTargetAndNoDuration,
    BothStepTargetAndDuration,
    InvalidState,
}

impl Error {
    pub fn description(&self) -> &str {
        match self {
            Error::NoStepTargetAndNoDuration => "No targets: No step target and no duration target",
            Error::BothStepTargetAndDuration => "Multiple targets: Step target and duration target",
            Error::InvalidState => "Invalid state",
        }
    }
}

// Step target or duration target
#[derive(Debug, PartialEq)]
pub enum OperatingMode {
    Step,
    Duration,
}