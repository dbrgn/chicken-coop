//! State machine.

use ufmt::{derive::uDebug, uwrite};

use crate::{
    door_sensors::DoorStatus,
    serial::{SerialPortType, SerialWriter},
};

#[derive(uDebug)]
pub enum State {
    PreOpening,
    Open,
    PreClosing,
    Closed,
    Error,
}

impl State {
    /// Determine the initial state
    pub fn init(door_status: DoorStatus, serial: &mut SerialPortType) -> Self {
        let state = match door_status {
            DoorStatus::Open => State::Open,
            DoorStatus::Closed => State::Closed,
            DoorStatus::Unknown | DoorStatus::Error => State::Error,
        };
        uwrite!(SerialWriter(serial), ":: Initial state: {:?}\n", state).ok();
        state
    }

    /// Determine the initial state
    pub fn transition(&mut self, to: Self, serial: &mut SerialPortType) {
        uwrite!(
            SerialWriter(serial),
            ":: State transition: {:?} -> {:?}\n",
            self,
            to
        )
        .ok();

        // TODO: Validate transition
        // TOOD: Return side effect

        *self = to;
    }
}
