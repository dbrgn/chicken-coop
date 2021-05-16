//! The door sensors that can detect whether the door is open or closed.

use stm32f4xx_hal::{
    gpio::{gpioa, Input, PullUp},
    prelude::*,
};

pub struct DoorSensors {
    switch_open: gpioa::PA<Input<PullUp>>,
    switch_closed: gpioa::PA<Input<PullUp>>,
}

pub enum DoorStatus {
    Open,
    Closed,
    Unknown,
    Error,
}

impl DoorSensors {
    pub fn new(
        switch_open: gpioa::PA<Input<PullUp>>,
        switch_closed: gpioa::PA<Input<PullUp>>,
    ) -> Self {
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
            (Ok(true), Ok(false)) => DoorStatus::Open,
            (Ok(false), Ok(true)) => DoorStatus::Closed,
            (Ok(false), Ok(false)) => DoorStatus::Unknown,
            (Ok(true), Ok(true)) => DoorStatus::Error,
            (Err(_), _) | (_, Err(_)) => DoorStatus::Error,
        }
    }
}
