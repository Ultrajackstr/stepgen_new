#![no_std]
#[cfg(not(feature = "x64"))]
pub mod x32;
#[cfg(feature = "x64")]
pub mod x64;