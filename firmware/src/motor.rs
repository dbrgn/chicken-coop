use stm32f4xx_hal::{
    gpio::{gpioa::PA, Output, PushPull},
    prelude::*,
};

pub struct Motor {
    pub forwards: PA<Output<PushPull>>,
    pub backwards: PA<Output<PushPull>>,
}

impl Motor {
    pub fn forwards(&mut self) {
        self.backwards.set_low().unwrap();
        self.forwards.set_high().unwrap();
    }

    pub fn backwards(&mut self) {
        self.forwards.set_low().unwrap();
        self.backwards.set_high().unwrap();
    }

    pub fn off(&mut self) {
        self.forwards.set_low().unwrap();
        self.backwards.set_low().unwrap();
    }
}
