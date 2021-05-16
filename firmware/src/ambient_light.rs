//! Ambient light detector with hysteresis.
//!
//! Initial values between the two thresholds will be considered "night".
pub struct AmbientLight {
    /// Lux values above this value are considered "day"
    threshold_high: f32,
    /// Lux values below this value are considered "night"
    threshold_low: f32,
    /// Current daylight status, used to implement hysteresis
    is_day: bool,
}

impl AmbientLight {
    pub fn new(threshold_high: f32, threshold_low: f32) -> Self {
        Self {
            threshold_high,
            threshold_low,
            is_day: false,
        }
    }

    pub fn update(&mut self, lux: f32) -> Brightness {
        if !self.is_day && lux > self.threshold_high {
            self.is_day = true;
        } else if self.is_day && lux < self.threshold_low {
            self.is_day = false;
        }
        Brightness {
            lux,
            is_day: self.is_day,
        }
    }
}

pub struct Brightness {
    /// Current brightness level
    pub lux: f32,
    /// Whether daylight was detected
    is_day: bool,
}

impl Brightness {
    pub fn is_day(&self) -> bool {
        self.is_day
    }

    pub fn is_night(&self) -> bool {
        !self.is_day
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    macro_rules! assert_day {
        ($brightness:expr) => {{
            assert!($brightness.is_day());
            assert!(!$brightness.is_night());
        }};
    }

    macro_rules! assert_night {
        ($brightness:expr) => {{
            assert!(!$brightness.is_day());
            assert!($brightness.is_night());
        }};
    }

    #[test]
    fn test_initial_state() {
        let al = AmbientLight::new(200.0, 100.0);
        assert!(!al.is_day);
    }

    #[test]
    fn test_night() {
        let mut al = AmbientLight::new(200.0, 100.0);

        // Initial value below low threshold
        assert_night!(al.update(10.0));
    }

    #[test]
    fn test_day() {
        let mut al = AmbientLight::new(200.0, 100.0);

        // Initial value above high threshold
        assert_day!(al.update(220.0));
    }

    #[test]
    fn test_hysteresis() {
        let mut al = AmbientLight::new(200.0, 100.0);

        // Initially dark
        assert_night!(al.update(10.0));

        // Value above low threshold but below high threshold
        assert_night!(al.update(150.0));

        // Back below low threshold
        assert_night!(al.update(20.0));

        // Jump above high threshold
        assert_night!(al.update(180.0));
        assert_day!(al.update(220.0));

        // Back to hysteresis zone, no change
        assert_day!(al.update(180.0));
        assert_day!(al.update(220.0));

        // And back to night
        assert_night!(al.update(1.0));
    }
}
