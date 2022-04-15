#![no_std]
#![no_main]
#![cfg(target_arch = "arm")] // Main module will only build for ARM. For testing, use lib.rs.

use panic_halt as _;
use rtic::app;

mod ambient_light;
mod door_sensors;
mod errors;
mod motor;
mod serial;
mod states;

const AMBIENT_LIGHT_THRESHOLD_LOW: f32 = 20.0; // Close threshold
const AMBIENT_LIGHT_THRESHOLD_HIGH: f32 = 100.0; // Open threshold
const EARLIEST_OPENING_TIME: (u32, u32) = (8, 30);
const LATEST_OPENING_TIME: (u32, u32) = (9, 30);
const EARLIEST_CLOSING_TIME: (u32, u32) = (17, 00);
const LATEST_CLOSING_TIME: (u32, u32) = (22, 00);

#[app(device = stm32f4xx_hal::pac, dispatchers = [SPI1, SPI2], peripherals = true)]
mod app {
    use ds323x::{ic::DS3231, interface::I2cInterface, Ds323x, Rtcc, Timelike};
    use heapless::spsc::Queue;
    use panic_halt as _;
    use rtt_target::{rprintln, rtt_init_print};
    use shared_bus_rtic::SharedBus;
    use stm32f4xx_hal::{
        self as hal,
        gpio::{gpioa, gpiob, gpioc, Alternate, Edge, Input, OpenDrain, Output, PullUp, PushPull},
        i2c::I2c,
        otg_fs::{UsbBus, UsbBusType, USB},
        pac,
        prelude::*,
        timer::{CountDownTimer, Event, Timer},
    };
    use ufmt::{uwrite, uwriteln};
    use usb_device::{
        bus::UsbBusAllocator,
        prelude::{UsbDevice, UsbDeviceBuilder, UsbVidPid},
    };
    use usbd_serial::{SerialPort, USB_CLASS_CDC};
    use veml6030::Veml6030;

    use crate::{
        ambient_light::AmbientLight,
        door_sensors::{DoorSensors, DoorStatus},
        errors::Error,
        motor::Motor,
        serial::{self, SerialPortType, SerialWriter},
        states::State,
    };

    type SharedBusType = I2c<
        pac::I2C1,
        (
            gpiob::PB6<Alternate<OpenDrain, 4>>,
            gpiob::PB7<Alternate<OpenDrain, 4>>,
        ),
    >;

    pub struct SharedBusResources<T: 'static> {
        // Ambient light sensor (VEML7700)
        lightsensor: Veml6030<SharedBus<T>>,

        // Real-time clock (DS3231)
        rtc: Ds323x<I2cInterface<SharedBus<T>>, DS3231>,
    }

    #[shared]
    struct SharedResources {
        // State machine
        #[lock_free]
        state: State,

        // Debugging
        #[lock_free]
        errors: Queue<Error, 16>, // Allow logging up to 16 errors

        // Door sensors
        #[lock_free]
        door_sensors: DoorSensors,

        // Serial
        #[lock_free]
        serial: SerialPortType,

        // I²C devices
        #[lock_free]
        i2c: SharedBusResources<SharedBusType>,
    }

    #[local]
    struct LocalResources {
        // Debugging
        button: gpioa::PA0<Input<PullUp>>,
        led: gpioc::PC13<Output<PushPull>>,

        // Periodic timer
        timer: CountDownTimer<pac::TIM2>,
        timer_counter: u8,

        // Ambient light state tracker
        ambient_light: AmbientLight,

        // Motor control
        motor: Motor,

        // USB
        usb_dev: UsbDevice<'static, UsbBusType>,
    }

    #[init(local = [
       ep_memory: [u32; 1024] = [0; 1024],
       usb_bus: Option<UsbBusAllocator<UsbBusType>> = None,
    ])]
    fn init(mut ctx: init::Context) -> (SharedResources, LocalResources, init::Monotonics) {
        rtt_init_print!();

        let mut errors = Queue::new();

        rprintln!("Initializing");

        let mut syscfg = ctx.device.SYSCFG.constrain();

        // Clock setup
        let rcc = ctx.device.RCC.constrain();
        let clocks = rcc
            .cfgr
            .use_hse(25.mhz())
            .sysclk(48.mhz())
            .require_pll48clk()
            .freeze();

        rprintln!("Clock setup done");

        // GPIO setup
        let gpioa = ctx.device.GPIOA.split();
        let gpiob = ctx.device.GPIOB.split();
        let gpioc = ctx.device.GPIOC.split();

        // USB setup: We're using USB FS for serial output
        let usb = USB {
            usb_global: ctx.device.OTG_FS_GLOBAL,
            usb_device: ctx.device.OTG_FS_DEVICE,
            usb_pwrclk: ctx.device.OTG_FS_PWRCLK,
            pin_dm: gpioa.pa11.into_alternate::<10>(),
            pin_dp: gpioa.pa12.into_alternate::<10>(),
            hclk: clocks.hclk(),
        };
        ctx.local
            .usb_bus
            .replace(UsbBus::new(usb, ctx.local.ep_memory));
        let mut serial = SerialPort::new_with_store(
            ctx.local.usb_bus.as_ref().unwrap(),
            [0u8; serial::SERIAL_READ_BUFFER_BYTES],
            [0u8; serial::SERIAL_WRITE_BUFFER_BYTES],
        );
        let usb_dev = UsbDeviceBuilder::new(
            ctx.local.usb_bus.as_ref().unwrap(),
            UsbVidPid(0x16c0, 0x27dd),
        )
        .manufacturer("Bargen Software")
        .product("ChickenDoor")
        .serial_number("0001")
        .device_class(USB_CLASS_CDC)
        .build();

        // Motor driver
        let motor = Motor {
            forwards: gpioa.pa8.into_push_pull_output().erase(),
            backwards: gpioa.pa9.into_push_pull_output().erase(),
        };

        // LED and button for debugging purposes
        let mut led = gpioc.pc13.into_push_pull_output();
        let mut button = gpioa.pa0.into_pull_up_input();
        led.set_high();

        // Reed switches (A1 = opened, A2 = closed)
        let door_sensors = DoorSensors::new(
            gpioa.pa1.into_pull_up_input().erase(),
            gpioa.pa2.into_pull_up_input().erase(),
        );

        // I2C setup. SCL is PB6 and SDA is PB7 (both with AF04).
        let scl = gpiob.pb6.into_alternate::<4>().set_open_drain();
        let sda = gpiob.pb7.into_alternate::<4>().set_open_drain();
        let i2c = I2c::new(ctx.device.I2C1, (scl, sda), 400.khz(), &clocks);

        // Create shared bus
        let bus_manager = shared_bus_rtic::new!(i2c, SharedBusType);

        rprintln!("I2C and GPIO setup done");

        // VEML7700 ambient light sensor
        //
        // Note: After enabling the sensor, a startup time of 4 ms plus the
        // integration time must be awaited.
        //
        // TODO: Timeout!
        let mut lightsensor = Veml6030::new(bus_manager.acquire(), veml6030::SlaveAddr::default());
        let ambient_light = AmbientLight::new(
            super::AMBIENT_LIGHT_THRESHOLD_LOW,
            super::AMBIENT_LIGHT_THRESHOLD_HIGH,
        );
        if let Err(e) = lightsensor.set_gain(veml6030::Gain::OneQuarter) {
            rprintln!("Could not set VEML gain: {:?}", e);
            Error::VemlGainSetFailed.log(&mut errors);
        }
        if let Err(e) = lightsensor.set_integration_time(veml6030::IntegrationTime::Ms800) {
            rprintln!("Could not set VEML integration time: {:?}", e);
            Error::VemlIntegrationTimeSetFailed.log(&mut errors);
        }
        if let Err(e) = lightsensor.enable() {
            rprintln!("Could not enable VEML7700: {:?}", e);
            Error::VemlEnableFailed.log(&mut errors);
        }

        rprintln!("Light sensor setup done");

        // RTC
        let rtc = Ds323x::new_ds3231(bus_manager.acquire());

        // Periodic timer
        let mut timer = Timer::new(ctx.device.TIM2, &clocks).start_count_down(1.hz());
        timer.listen(Event::TimeOut);
        unsafe {
            // Enable TIM2 interrupt in NVIC
            pac::NVIC::unmask(pac::Interrupt::TIM2);
        }

        // Wire up button interrupt
        button.make_interrupt_source(&mut syscfg);
        button.enable_interrupt(&mut ctx.device.EXTI);
        button.trigger_on_edge(&mut ctx.device.EXTI, Edge::Rising);

        // Get door status
        let door_status = door_sensors.query();

        // Initial state
        let state = State::init(door_status, &mut serial);

        rprintln!("Done initializing");

        let shared = SharedResources {
            state,
            errors,
            door_sensors,
            serial,
            i2c: SharedBusResources { lightsensor, rtc },
        };
        let local = LocalResources {
            button,
            led,
            timer,
            timer_counter: 10,
            ambient_light,
            motor,
            usb_dev,
        };
        (shared, local, init::Monotonics())
    }

    #[task(binds = EXTI0, shared = [serial, door_sensors, state], local = [button])]
    fn button_click(mut ctx: button_click::Context) {
        ctx.local.button.clear_interrupt_pending_bit();

        let serial = &mut ctx.shared.serial;

        serial.write(b":: Button pressed, resetting state\n").ok();
        let door_status = ctx.shared.door_sensors.query();
        uwrite!(SerialWriter(serial), ":: Door status: {:?}\n", door_status).ok();
        ctx.shared.state.reset(door_status, serial);
    }

    /// This task runs every second.
    #[task(binds = TIM2, local = [led, timer, timer_counter])]
    fn every_second(ctx: every_second::Context) {
        // Toggle LED
        ctx.local.led.toggle();

        // Decrement timer counter to work around the fact that the HAL
        // currently doesn't support wait times >1s (because sleeping is
        // specified using a frequency).
        *ctx.local.timer_counter -= 1;
        if *ctx.local.timer_counter == 0 {
            *ctx.local.timer_counter = 10;

            // Spawn update task
            update::spawn().unwrap();
        }

        // Clear interrupt
        ctx.local.timer.clear_interrupt(Event::TimeOut);
    }

    /// This task is run every 10s.
    #[task(shared = [state, i2c, serial, errors], local = [ambient_light])]
    fn update(mut ctx: update::Context) {
        // Error collector
        let errors = &mut ctx.shared.errors;

        // Resource aliases
        let serial = &mut ctx.shared.serial;
        let state = &mut ctx.shared.state;
        let rtc = &mut ctx.shared.i2c.rtc;

        // Get current time
        let time = match rtc.time() {
            Ok(time) => time,
            Err(_) => {
                Error::RtcReadTimeError.log(errors);
                serial.write(b":: Read time error\n").ok();
                return;
            }
        };

        // Log update to serial
        serial.write(b":: ").ok();
        print_time(&time, serial);
        uwrite!(SerialWriter(serial), " Update [State={:?}]\n", *state).ok();

        // Get current timestamp.
        // For easier calculations, use day-minutes as timestamps
        let timestamp = time.hour() * 60 + time.minute();
        let earliest_opening_timestamp =
            super::EARLIEST_OPENING_TIME.0 * 60 + super::EARLIEST_OPENING_TIME.1;
        let latest_opening_timestamp =
            super::LATEST_OPENING_TIME.0 * 60 + super::LATEST_OPENING_TIME.1;
        let earliest_closing_timestamp =
            super::EARLIEST_CLOSING_TIME.0 * 60 + super::EARLIEST_CLOSING_TIME.1;
        let latest_closing_timestamp =
            super::LATEST_CLOSING_TIME.0 * 60 + super::LATEST_CLOSING_TIME.1;

        // Read lux if in Pre* state
        let brightness_opt = match state {
            State::PreOpening | State::PreClosing => match ctx.shared.i2c.lightsensor.read_lux() {
                Ok(lux) => {
                    uwriteln!(
                        SerialWriter(serial),
                        "Current brightness level: {} lux",
                        lux as usize
                    )
                    .ok();
                    Some(ctx.local.ambient_light.update(lux))
                }
                Err(e) => {
                    let reason = match e {
                        veml6030::Error::I2C(_) => "I2C bus error",
                    };
                    uwriteln!(SerialWriter(serial), "Could not read lux: {}", reason).ok();
                    None
                }
            },
            _ => None,
        };

        // Act depending on state
        let new_state = match state {
            State::Closed => {
                if timestamp >= earliest_opening_timestamp {
                    Some(State::PreOpening)
                } else {
                    None
                }
            }
            State::PreOpening => {
                // Read brightness
                if brightness_opt
                    .map(|brightness| brightness.is_day())
                    .unwrap_or(false)
                {
                    uwriteln!(SerialWriter(serial), "Daylight level reached, opening",).ok();
                    Some(State::Open)
                } else if timestamp > latest_opening_timestamp {
                    // If brightness couldn't be read, or if latest opening time is reached, open
                    Some(State::Open)
                } else {
                    None
                }
            }
            State::Open => {
                if timestamp >= earliest_closing_timestamp {
                    Some(State::PreClosing)
                } else {
                    None
                }
            }
            State::PreClosing => {
                // Read brightness
                if brightness_opt
                    .map(|brightness| brightness.is_night())
                    .unwrap_or(false)
                {
                    uwriteln!(SerialWriter(serial), "Nighttime level reached, closing",).ok();
                    Some(State::Closed)
                } else if timestamp > latest_closing_timestamp {
                    // If brightness couldn't be read, or if latest closing time is reached, close
                    Some(State::Closed)
                } else {
                    None
                }
            }
            State::Error => {
                // TODO: Indicate error state somehow
                // TODO: Try to recover
                None
            }
        };

        if let Some(new_state) = new_state {
            state.transition(new_state, serial);
            // TODO: Side effect
        }
    }

    /// Task that binds to the "USB OnTheGo FS global interrupt" (OTG_FS).
    #[task(binds=OTG_FS, shared = [serial, i2c, errors, door_sensors], local = [motor, usb_dev])]
    fn on_usb(mut ctx: on_usb::Context) {
        // Poll USB device for events
        if !ctx.local.usb_dev.poll(&mut [ctx.shared.serial]) {
            return;
        }

        // Handle serial input
        let mut buf = [0u8; 8];
        match ctx.shared.serial.read(&mut buf) {
            // Data received
            Ok(count) if count > 0 => {
                for byte in &buf {
                    handle_command(*byte, &mut ctx);
                }
            }

            // No data received
            _ => {}
        }
    }

    /// Handle single-byte commands sent via serial
    fn handle_command(byte: u8, ctx: &mut on_usb::Context) {
        let serial = &mut ctx.shared.serial;
        match byte {
            b'?' => {
                serial.write(b"   \\\\\n").ok();
                serial.write(b"   (o>\n").ok();
                serial.write(b"\\\\_//) CHICKEN DOOR STATUS REPORT\n").ok();
                serial.write(b" \\_/_)\n").ok();
                serial.write(b"  _|_\n").ok();
                serial.write(b"\n").ok();
                serial.write(b"Status:\n\n").ok();
                serial.write(b" Door status: ").ok();
                match ctx.shared.door_sensors.query() {
                    DoorStatus::Open => serial.write(b"open\n").ok(),
                    DoorStatus::Closed => serial.write(b"closed\n").ok(),
                    DoorStatus::Unknown => serial.write(b"unknown\n").ok(),
                    DoorStatus::Error => serial.write(b"error\n").ok(),
                };
                serial.write(b" RTC running: ").ok();
                match ctx.shared.i2c.rtc.running() {
                    Ok(true) => serial.write(b"yes\n").ok(),
                    Ok(false) => serial.write(b"no\n").ok(),
                    Err(ds323x::Error::Comm(hal::i2c::Error::BUS)) => {
                        serial.write(b"bus error\n").ok()
                    }
                    Err(ds323x::Error::Comm(hal::i2c::Error::OVERRUN)) => {
                        serial.write(b"overrun error\n").ok()
                    }
                    Err(ds323x::Error::Comm(hal::i2c::Error::NACK)) => serial.write(b"nack\n").ok(),
                    Err(ds323x::Error::Comm(hal::i2c::Error::TIMEOUT)) => {
                        serial.write(b"timeout\n").ok()
                    }
                    Err(ds323x::Error::Comm(hal::i2c::Error::CRC)) => serial.write(b"crc\n").ok(),
                    Err(ds323x::Error::Comm(hal::i2c::Error::ARBITRATION)) => {
                        serial.write(b"arbitration lost\n").ok()
                    }
                    Err(ds323x::Error::Pin(_)) => serial.write(b"pin error\n").ok(),
                    Err(ds323x::Error::InvalidInputData) => serial.write(b"data error\n").ok(),
                    Err(ds323x::Error::InvalidDeviceState) => {
                        serial.write(b"invalid device state\n").ok()
                    }
                };
                serial.write(b"\n").ok();
                serial.write(b"Logged errors:\n\n").ok();
                for error in ctx.shared.errors.iter() {
                    serial.write(b"- ").ok();
                    serial.write(error.as_bytes()).ok();
                    serial.write(b"\n").ok();
                }
                if ctx.shared.errors.is_empty() {
                    serial.write(b" None\n").ok();
                }
                serial.write(b"\n").ok();
                serial.write(b"Available commands:\n\n").ok();
                serial.write(b" ? - Show this report\n").ok();
                serial.write(b" l - Measure ambient light level\n").ok();
                serial.write(b" c - Output RTC clock info\n").ok();
                serial.write(b" t - Output RTC temperature\n").ok();
                serial.write(b" f - Motor FW\n").ok();
                serial.write(b" b - Motor BW\n").ok();
                serial.write(b" o - Motor off\n").ok();
                serial.write(b"\n").ok();
            }
            b'l' | b'L' => match ctx.shared.i2c.lightsensor.read_lux() {
                Ok(lux) => {
                    uwriteln!(
                        SerialWriter(serial),
                        "Current brightness level: {} lux",
                        lux as usize
                    )
                    .ok();
                }
                Err(_e) => {
                    serial.write(b"Could not measure brightness level\n").ok();
                }
            },
            b'c' | b'C' => match ctx.shared.i2c.rtc.time() {
                Ok(time) => {
                    serial.write(b"Current time: ").ok();
                    print_time(&time, serial);
                    serial.write(b"\n").ok();
                }
                Err(_) => {
                    Error::RtcReadTimeError.log(ctx.shared.errors);
                    serial.write(b"Could not determine time\n").ok();
                }
            },
            b't' | b'T' => match ctx.shared.i2c.rtc.temperature() {
                Ok(temp) => {
                    uwriteln!(
                        SerialWriter(serial),
                        "Current temperature: {}",
                        temp as usize
                    )
                    .ok();
                }
                Err(_) => {
                    serial.write(b"Could not determine temperature\n").ok();
                }
            },
            b'f' | b'F' => {
                ctx.local.motor.forwards();
                serial.write(b"Motor: Move forwards\n").ok();
            }
            b'b' | b'B' => {
                ctx.local.motor.backwards();
                serial.write(b"Motor: Move backwards\n").ok();
            }
            b'o' | b'O' => {
                ctx.local.motor.off();
                serial.write(b"Motor: Off\n").ok();
            }
            _ => { /* Unknown command, ignore */ }
        }
    }

    /// Print the 24h time with proper 0-prefixing.
    fn print_time(time: &impl Timelike, serial: &mut SerialPortType) {
        fn print_part(value: u32, serial: &mut SerialPortType) {
            if value < 10 {
                uwrite!(SerialWriter(serial), "0").ok();
            }
            uwrite!(SerialWriter(serial), "{}", value).ok();
        }

        print_part(time.hour(), serial);
        serial.write(b":").ok();
        print_part(time.minute(), serial);
        serial.write(b":").ok();
        print_part(time.second(), serial);
    }
}
