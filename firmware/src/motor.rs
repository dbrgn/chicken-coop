use stm32f4xx_hal::gpio::{EPin, Output, PushPull};

pub struct Motor {
    pub forwards: EPin<Output<PushPull>>,
    pub backwards: EPin<Output<PushPull>>,
}

impl Motor {
    pub fn forwards(&mut self) {
        self.backwards.set_low();
        self.forwards.set_high();
    }

    pub fn backwards(&mut self) {
        self.forwards.set_low();
        self.backwards.set_high();
    }

    pub fn off(&mut self) {
        self.forwards.set_low();
        self.backwards.set_low();
    }
}
