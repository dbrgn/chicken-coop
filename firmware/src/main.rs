#![no_std]
#![no_main]

use ds323x::{ic::DS3231, interface::I2cInterface, Ds323x};
use panic_halt as _;
use rtic::app;
use rtt_target::{rprintln, rtt_init_print};
use shared_bus_rtic::SharedBus;
use stm32f4xx_hal::{
    gpio::{gpioa, gpiob, gpioc, AlternateOD, Edge, Input, Output, PullUp, PushPull, AF4},
    i2c::I2c,
    otg_fs::{UsbBus, UsbBusType, USB},
    pac,
    prelude::*,
};
use usb_device::{
    bus::UsbBusAllocator,
    prelude::{UsbDevice, UsbDeviceBuilder, UsbVidPid},
};
use usbd_serial::{SerialPort, USB_CLASS_CDC};
use veml6030::Veml6030;

// VEML ambient light sensor integration time
const VEML_INTEGRATION_TIME: veml6030::IntegrationTime = veml6030::IntegrationTime::Ms100;

type SharedBusType = I2c<pac::I2C1, (gpiob::PB6<AlternateOD<AF4>>, gpiob::PB7<AlternateOD<AF4>>)>;

pub struct SharedBusResources<T: 'static> {
    // Ambient light sensor (VEML7700)
    lightsensor: Veml6030<SharedBus<T>>,

    // Real-time clock (DS3231)
    rtc: Ds323x<I2cInterface<SharedBus<T>>, DS3231>,
}

#[app(device = stm32f4xx_hal::stm32, peripherals = true)]
const APP: () = {
    struct Resources {
        // Debugging
        button: gpioa::PA0<Input<PullUp>>,
        led: gpioc::PC13<Output<PushPull>>,

        // USB
        usb_dev: UsbDevice<'static, UsbBusType>,

        // Serial
        serial: SerialPort<'static, UsbBusType>,

        // I²C devices
        i2c: SharedBusResources<SharedBusType>,
    }

    #[init]
    fn init(mut ctx: init::Context) -> init::LateResources {
        static mut EP_MEMORY: [u32; 1024] = [0; 1024];
        static mut USB_BUS: Option<UsbBusAllocator<UsbBusType>> = None;

        rtt_init_print!();

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
            pin_dm: gpioa.pa11.into_alternate_af10(),
            pin_dp: gpioa.pa12.into_alternate_af10(),
            hclk: clocks.hclk(),
        };
        USB_BUS.replace(UsbBus::new(usb, EP_MEMORY));
        let serial = usbd_serial::SerialPort::new(USB_BUS.as_ref().unwrap());
        let usb_dev = UsbDeviceBuilder::new(USB_BUS.as_ref().unwrap(), UsbVidPid(0x16c0, 0x27dd))
            .manufacturer("Bargen Software")
            .product("ChickenDoor")
            .serial_number("0001")
            .device_class(USB_CLASS_CDC)
            .build();

        // I2C setup. SCL is PB6 and SDA is PB7 (both with AF04).
        let scl = gpiob.pb6.into_alternate_af4().set_open_drain();
        let sda = gpiob.pb7.into_alternate_af4().set_open_drain();
        let i2c = I2c::new(ctx.device.I2C1, (scl, sda), 400.khz(), clocks);

        // Create shared bus
        let bus_manager = shared_bus_rtic::new!(i2c, SharedBusType);

        // LED and button for debugging purposes
        let led = gpioc.pc13.into_push_pull_output();
        let mut button = gpioa.pa0.into_pull_up_input();

        rprintln!("I2C and GPIO setup done");

        // Light sensor
        //
        // TODO: Timeout!
        let mut lightsensor = Veml6030::new(bus_manager.acquire(), veml6030::SlaveAddr::default());
        if let Err(e) = lightsensor.set_gain(veml6030::Gain::OneQuarter) {
            rprintln!("Could not set VEML gain: {:?}", e);
        }
        if let Err(e) = lightsensor.set_integration_time(VEML_INTEGRATION_TIME) {
            rprintln!("Could not set VEML integration time: {:?}", e);
        }
        rprintln!("Light sensor setup done");

        // RTC
        let rtc = Ds323x::new_ds3231(bus_manager.acquire());

        // Wire up button interrupt
        button.make_interrupt_source(&mut syscfg);
        button.enable_interrupt(&mut ctx.device.EXTI);
        button.trigger_on_edge(&mut ctx.device.EXTI, Edge::RISING);

        rprintln!("Done initializing");

        init::LateResources {
            button,
            led,
            usb_dev,
            serial,
            i2c: SharedBusResources { lightsensor, rtc },
        }
    }

    #[task(binds = EXTI0, resources = [button, led])]
    fn button_click(ctx: button_click::Context) {
        rprintln!("Button pressed");
        ctx.resources.button.clear_interrupt_pending_bit();
        ctx.resources.led.toggle().unwrap();
    }

    /// Task that binds to the "USB OnTheGo FS global interrupt" (OTG_FS).
    #[task(binds=OTG_FS, resources = [usb_dev, serial])]
    fn on_usb(ctx: on_usb::Context) {
        let usb_dev = ctx.resources.usb_dev;
        let serial = ctx.resources.serial;

        // Poll USB device for events
        if !usb_dev.poll(&mut [serial]) {
            return;
        }

        // Handle serial input
        let mut buf = [0u8; 8];
        match serial.read(&mut buf) {
            // Data received
            Ok(count) if count > 0 => {
                for byte in &buf {
                    handle_command(*byte, serial);
                }
            }

            // No data received
            _ => {}
        }
    }
};

/// Handle single-byte commands sent via serial
fn handle_command<B: usb_device::bus::UsbBus>(byte: u8, serial: &mut SerialPort<'static, B>) {
    match byte {
        b'?' => {
            serial.write(b"   \\\\\n").ok();
            serial.write(b"   (o>\n").ok();
            serial.write(b"\\\\_//) CHICKEN DOOR STATUS REPORT\n").ok();
            serial.write(b" \\_/_)\n").ok();
            serial.write(b"  _|_\n\n").ok();
            serial.write(b"Available commands:\n\n").ok();
            serial.write(b" ? - Show this status / help\n").ok();
            serial.write(b"\n").ok();
        }
        _ => { /* Unknown command, ignore */ }
    }
}
