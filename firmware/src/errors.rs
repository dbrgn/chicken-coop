//! Error handling.

use heapless::spsc::Queue;

/// All possible error types
#[derive(PartialEq, Copy, Clone)]
pub enum Error {
    VemlGainSetFailed,
    VemlIntegrationTimeSetFailed,
    VemlEnableFailed,
    UfmtSerialWriteError,
    MotorGpioWriteError,
    RtcReadTimeError,
}

impl Error {
    pub fn log<const N: usize>(&self, queue: &mut Queue<Self, N>) {
        match queue.enqueue(*self) {
            Ok(()) => { /* Enqueued */ }
            Err(e) => {
                // Queue full, drop the oldest value and try again
                queue.dequeue();
                queue.enqueue(e).ok();
            }
        }
    }

    pub fn to_bytes(&self) -> &'static [u8] {
        // TODO: Maybe implement uDisplay instead?
        match self {
            Self::VemlGainSetFailed => b"VEML7700: Setting gain failed",
            Self::VemlIntegrationTimeSetFailed => b"VEML7700: Setting integration time failed",
            Self::VemlEnableFailed => b"VEML7700: Enabling failed",
            Self::UfmtSerialWriteError => b"Write serial log using ufmt failed",
            Self::MotorGpioWriteError => b"Motor GPIO write error",
            Self::RtcReadTimeError => b"RTC: Reading time failed",
        }
    }
}

