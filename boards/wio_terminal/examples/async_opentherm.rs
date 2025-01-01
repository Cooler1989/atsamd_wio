#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;
use heapless::Vec;
use core::time::Duration;

use bsp::pac;
use bsp::{
    hal, 
    hal::gpio::{Pins, Output, PushPullOutput, PushPull, OutputConfig},
    hal::gpio::{PB09},
    pin_alias,
};

use hal::{
    clock::{ClockGenId, ClockSource, GenericClockController},
    ehal::digital::StatefulOutputPin,
    eic::{Eic, Sense},
    gpio::{Pin, PullUpInterrupt},
};

use bsp::pins::UserLed;
use wio_terminal as bsp;

use opentherm_boiler_controller_lib as boiler;
use boiler::opentherm_interface::{
    Temperature, CHState,
    OpenThermEdgeTriggerBus,
    edge_trigger_capture_interface::{
        CaptureError, EdgeCaptureInterface, EdgeTriggerInterface, TriggerError,
        InitLevel, 
    }
};
use boiler::{TimeBaseRef, Instant, BoilerControl};

atsamd_hal::bind_interrupts!(struct Irqs {
    EIC_EXTINT_5 => atsamd_hal::eic::InterruptHandler;
});

#[embassy_executor::task]
async fn boiler_task()
{
}

const VEC_SIZE_CAPTURE: usize = 128;
struct AtsamdEdgeTriggerCapture<const N: usize = VEC_SIZE_CAPTURE> {
    output_pin: Pin<PB09, PushPullOutput>,
}

impl AtsamdEdgeTriggerCapture {
    pub fn new(pin_tx: Pin<PB09,PushPullOutput>) -> Self
    {
        Self{output_pin:pin_tx}
    }
}

impl EdgeTriggerInterface for AtsamdEdgeTriggerCapture {
    async fn trigger(
        &mut self,
        iterator: impl Iterator<Item = bool>,
        period: core::time::Duration,
    ) -> Result<(), TriggerError> {
            todo!()
    }
}

impl<const N: usize> EdgeCaptureInterface<N> for AtsamdEdgeTriggerCapture<N> {
    async fn start_capture(
        &mut self,
        timeout_inactive_capture: core::time::Duration,
        timeout_till_active_capture: core::time::Duration,
    ) -> Result<(InitLevel, Vec<core::time::Duration, N>), CaptureError> {
        todo!()
    }
}

impl OpenThermEdgeTriggerBus for AtsamdEdgeTriggerCapture { }

struct AtsamdTimeDriver {
}

impl AtsamdTimeDriver {
    fn new() -> Self {
        Self{}
    }
}

impl TimeBaseRef for AtsamdTimeDriver {
    fn now(&self) -> Instant
    {
        todo!()
    }
}

#[embassy_executor::main]
async fn main(spawner: embassy_executor::Spawner) {
    let mut peripherals = pac::Peripherals::take().unwrap();
    let _core = pac::CorePeripherals::take().unwrap();

    spawner.spawn(boiler_task()).unwrap();

    let mut clocks = GenericClockController::with_external_32kosc(
        peripherals.gclk,
        &mut peripherals.mclk,
        &mut peripherals.osc32kctrl,
        &mut peripherals.oscctrl,
        &mut peripherals.nvmctrl,
    );
    let pins = Pins::new(peripherals.port);
    //  let mut user_led: bsp::UserLed = pin_alias!(pins.user_led).into();
    let mut user_led: UserLed = pins.pa15.into();

    let _internal_clock = clocks
        .configure_gclk_divider_and_source(ClockGenId::Gclk2, 1, ClockSource::Osculp32k, false)
        .unwrap();
    clocks.configure_standby(ClockGenId::Gclk2, true);

    // Configure a clock for the EIC peripheral
    let gclk2 = clocks.get_gclk(ClockGenId::Gclk2).unwrap();
    let eic_clock = clocks.eic(&gclk2).unwrap();

    let eic_channels = Eic::new(&mut peripherals.mclk, eic_clock, peripherals.eic).split();

    let _ot_rx: Pin<_, PullUpInterrupt> = pins.pb08.into(); // D0
    let pb_09_ot_tx: Pin<_, PushPullOutput> = pins.pb09.into(); // D1
    //  let capture_device = RpEdgeCapture::new(async_input);
    let open_therm_bus = AtsamdEdgeTriggerCapture::new(pb_09_ot_tx);

    let time_driver = AtsamdTimeDriver::new();
    let mut boiler_controller = BoilerControl::new(open_therm_bus, time_driver);
    let _ = boiler_controller.set_point(Temperature::Celsius(16));
    let _ = boiler_controller.enable_ch(CHState::Enable(true));

    let button: Pin<_, PullUpInterrupt> = pins.pd10.into();
    let mut extint = eic_channels.5.with_pin(button).into_future(Irqs);
    extint.enable_interrupt();

    loop {
        // Here we show straight falling edge detection without
        extint.wait(Sense::Fall).await;
        defmt::info!("Falling edge detected");
        user_led.toggle().unwrap();
    }
}
