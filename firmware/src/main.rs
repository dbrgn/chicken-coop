#![no_std]
#![no_main]

use panic_halt as _;
use rtic::app;
use rtt_target::{rprintln, rtt_init_print};
use stm32f4xx_hal::{
    gpio::{gpioa, gpiob, gpioc, AlternateOD, Edge, Input, Output, PullUp, PushPull, AF4},
    i2c::I2c,
    pac,
    prelude::*,
};
use veml6030::Veml6030;

// VEML ambient light sensor integration time
const VEML_INTEGRATION_TIME: veml6030::IntegrationTime = veml6030::IntegrationTime::Ms100;

#[app(device = stm32f4xx_hal::stm32, peripherals = true)]
const APP: () = {
    struct Resources {
        // Debugging
        button: gpioa::PA0<Input<PullUp>>,
        led: gpioc::PC13<Output<PushPull>>,

        // Ambient light sensor
        lightsensor: Veml6030<I2c<pac::I2C1, (
            gpiob::PB6<AlternateOD<AF4>>,
            gpiob::PB7<AlternateOD<AF4>>,
        )>>,
    }

    #[init]
    fn init(mut ctx: init::Context) -> init::LateResources {
        rtt_init_print!();

        rprintln!("Initializing");

        let mut syscfg = ctx.device.SYSCFG.constrain();

        // Clock setup
        let rcc = ctx.device.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(48.mhz()).freeze();

        rprintln!("Clock setup done");

        // GPIO setup
        let gpioa = ctx.device.GPIOA.split();
        let gpiob = ctx.device.GPIOB.split();
        let gpioc = ctx.device.GPIOC.split();

        // I2C setup. SCL is PB6 and SDA is PB7 (both with AF04).
        let scl = gpiob.pb6.into_alternate_af4().set_open_drain();
        let sda = gpiob.pb7.into_alternate_af4().set_open_drain();
        let i2c = I2c::new(ctx.device.I2C1, (scl, sda), 400.khz(), clocks);

        // LED and button for debugging purposes
        let led = gpioc.pc13.into_push_pull_output();
        let mut button = gpioa.pa0.into_pull_up_input();

        rprintln!("I2C and GPIO setup done");

        // Light sensor
        //
        // TODO: Timeout!
        let mut lightsensor = Veml6030::new(i2c, veml6030::SlaveAddr::default());
        if let Err(e) = lightsensor.set_gain(veml6030::Gain::OneQuarter) {
            rprintln!("Could not set VEML gain: {:?}", e);
        }
        if let Err(e) = lightsensor.set_integration_time(VEML_INTEGRATION_TIME) {
            rprintln!("Could not set VEML integration time: {:?}", e);
        }
        rprintln!("Light sensor setup done");

        // Wire up button interrupt
        button.make_interrupt_source(&mut syscfg);
        button.enable_interrupt(&mut ctx.device.EXTI);
        button.trigger_on_edge(&mut ctx.device.EXTI, Edge::RISING);

        rprintln!("Done initializing");

        init::LateResources { button, led, lightsensor }
    }

    #[task(binds = EXTI0, resources = [button, led])]
    fn button_click(ctx: button_click::Context) {
        rprintln!("Button pressed");
        ctx.resources.button.clear_interrupt_pending_bit();
        ctx.resources.led.toggle().unwrap();
    }
};
