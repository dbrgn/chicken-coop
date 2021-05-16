//! State machine.

use crate::door_sensors::DoorStatus;

pub enum State {
    Initial,
    PreOpening,
    Open,
    PreClosing,
    Closed,
}

//impl State {
//    /// Determine the initial state
//    pub fn init(door_status: DoorStatus, brightness: Brightness) -> Self {
//        Self::Initial // TODO
//    }
//}
