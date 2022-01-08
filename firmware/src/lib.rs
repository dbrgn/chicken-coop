// This module is only used as an entry point for unit testing.
#![cfg_attr(not(test), no_std)]

#[cfg(test)]
mod ambient_light;
#[cfg(test)]
mod door_sensors;
#[cfg(test)]
mod errors;
#[cfg(test)]
mod serial;
#[cfg(test)]
mod states;
