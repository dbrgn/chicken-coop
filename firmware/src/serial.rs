//! Helpers for using the serial port.

use stm32f4xx_hal::otg_fs::UsbBusType;
use usbd_serial::SerialPort;

use crate::errors::Error;

// Configure serial buffer
pub const SERIAL_READ_BUFFER_BYTES: usize = 256;
pub const SERIAL_WRITE_BUFFER_BYTES: usize = 512;

/// Type alias for the serial port type
pub type SerialPortType = SerialPort<
    'static,
    UsbBusType,
    [u8; SERIAL_READ_BUFFER_BYTES],
    [u8; SERIAL_WRITE_BUFFER_BYTES],
>;

/// Wrapper for a `SerialPort` that supports ufmt
pub struct SerialWriter<'a>(pub &'a mut SerialPortType);

impl<'a> ufmt::uWrite for SerialWriter<'a> {
    type Error = Error;
    fn write_str(&mut self, s: &str) -> Result<(), Self::Error> {
        self.0
            .write(s.as_bytes())
            .map(|_| ())
            .map_err(|_| Error::UfmtSerialWriteError)
    }
}
