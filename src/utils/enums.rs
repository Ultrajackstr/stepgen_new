// Error Enum
#[derive(Debug)]
pub enum Error {
    NoTargetAndNoDuration,
    InvalidState,
}

impl Error {
    pub fn description(&self) -> &str {
        match self {
            Error::NoTargetAndNoDuration => "No target and no duration",
            Error::InvalidState => "Invalid state",
        }
    }
}