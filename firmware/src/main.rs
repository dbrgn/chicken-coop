#![no_std]
#![no_main]

use panic_halt as _;
use rtic::app;
use stm32f4xx_hal::{
    gpio::{gpioa, gpioc, Edge, Input, Output, PullUp, PushPull},
    prelude::*,
};

#[app(device = stm32f4xx_hal::stm32, peripherals = true)]
const APP: () = {
    struct Resources {
        // Debugging
        button: gpioa::PA0<Input<PullUp>>,
        led: gpioc::PC13<Output<PushPull>>,
    }

    #[init]
    fn init(mut ctx: init::Context) -> init::LateResources {
        let mut syscfg = ctx.device.SYSCFG.constrain();

        let gpioa = ctx.device.GPIOA.split();
        let gpioc = ctx.device.GPIOC.split();

        let led = gpioc.pc13.into_push_pull_output();
        let mut button = gpioa.pa0.into_pull_up_input();

        button.make_interrupt_source(&mut syscfg);
        button.enable_interrupt(&mut ctx.device.EXTI);
        button.trigger_on_edge(&mut ctx.device.EXTI, Edge::RISING);

        init::LateResources { button, led }
    }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        loop {}
    }

    #[task(binds = EXTI0, resources = [button, led])]
    fn button_click(ctx: button_click::Context) {
        ctx.resources.button.clear_interrupt_pending_bit();
        ctx.resources.led.toggle().unwrap();
    }
};
