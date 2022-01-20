//! The door sensors that can detect whether the door is open or closed.

use stm32f4xx_hal::gpio::{EPin, Input, PullUp};
use ufmt::derive::uDebug;

pub struct DoorSensors {
    switch_open: EPin<Input<PullUp>>,
    switch_closed: EPin<Input<PullUp>>,
}

#[derive(uDebug)]
pub enum DoorStatus {
    Open,
    Closed,
    Unknown,
    Error,
}

impl DoorSensors {
    pub fn new(switch_open: EPin<Input<PullUp>>, switch_closed: EPin<Input<PullUp>>) -> Self {
        Self {
            switch_open,
            switch_closed,
        }
    }

    /// Query the current door opening status.
    pub fn query(&self) -> DoorStatus {
        // The reed switches are connected with a pull-up resistor, so
        // triggering the switch will result in a low input.
        let open = self.switch_open.is_low();
        let closed = self.switch_closed.is_low();
        match (open, closed) {
            (true, false) => DoorStatus::Open,
            (false, true) => DoorStatus::Closed,
            (false, false) => DoorStatus::Unknown,
            (true, true) => DoorStatus::Error,
        }
    }
}
