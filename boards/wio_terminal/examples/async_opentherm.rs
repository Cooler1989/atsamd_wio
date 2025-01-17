#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;
use heapless::Vec;
use core::time::Duration;
use bsp::hal::time::Hertz;

use bsp::pac;
use bsp::{
    hal, 
    hal::gpio::{Pins, Output, PushPullOutput, PushPull, OutputConfig},
    hal::gpio::{E, PB09},
    pin_alias,
};
use wio_terminal::prelude::_embedded_hal_PwmPin;

use hal::{
    clock::{ClockGenId, ClockSource, GenericClockController},
    dmac::{DmaController, PriorityLevel, TriggerAction, TriggerSource},
    ehal::digital::StatefulOutputPin,
    eic::{Eic, Sense},
    gpio::{Pin, PullUpInterrupt},
    pwm::{Pwm4, TC4Pinout},
    pwm_wg::{PwmWg4},
    delay::Delay,
};
use wio_terminal::prelude::_embedded_hal_blocking_delay_DelayMs;

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

atsamd_hal::bind_multiple_interrupts!(struct DmacIrqs {
    DMAC: [DMAC_0, DMAC_1, DMAC_2, DMAC_OTHER] => atsamd_hal::dmac::InterruptHandler;
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
    let core = pac::CorePeripherals::take().unwrap();
    //  let core = CorePeripherals::take().unwrap();

    spawner.spawn(boiler_task()).unwrap();

    let mut clocks = GenericClockController::with_external_32kosc(
        peripherals.gclk,
        &mut peripherals.mclk,
        &mut peripherals.osc32kctrl,
        &mut peripherals.oscctrl,
        &mut peripherals.nvmctrl,
    );
    let gclk0 = clocks.gclk0();
    let mut delay = Delay::new(core.SYST, &mut clocks);

    // Initialize DMA Controller
    let dmac = DmaController::init(peripherals.dmac, &mut peripherals.pm);
    // Turn dmac into an async controller
    let mut dmac = dmac.into_future(DmacIrqs);
    // Get individual handles to DMA channels
    let channels = dmac.split();
    // Initialize DMA Channels 0 and 1
    let mut channel0 = channels.0.init(PriorityLevel::Lvl0);
    let channel1 = channels.1.init(PriorityLevel::Lvl0);

    let mut source = [0xffu8; 100];
    let mut dest = [0x0u8; 100];

    channel0
        .transfer_future(
            &mut source,
            &mut dest,
            TriggerSource::Tc4Ovf,
            TriggerAction::Burst,
        )
        .await
        .unwrap();

    let pins = Pins::new(peripherals.port);
    let pwm_pin = pins.pb09.into_alternate::<E>();

    let mut pwm4 = PwmWg4::<PB09>::new(
        &clocks.tc4_tc5(&gclk0).unwrap(),
        Hertz::from_raw(1000),
        peripherals.tc4,
        TC4Pinout::Pb9(pwm_pin),
        &mut peripherals.mclk,
    );

    let max_duty = pwm4.get_max_duty();

    loop {
        pwm4.set_duty(max_duty / 2);
        delay.delay_ms(2000u16);
        pwm4.set_duty(max_duty / 8);
        delay.delay_ms(2000u16);
    }

    //  let mut user_led: bsp::UserLed = pin_alias!(pins.user_led).into();
    let mut user_led: UserLed = pins.pa15.into();

    let _internal_clock = clocks
        .configure_gclk_divider_and_source(ClockGenId::Gclk2, 1, ClockSource::Osculp32k, false)
        .unwrap();
    clocks.configure_standby(ClockGenId::Gclk2, true);

    //  let edge_trigger_capture_dev = AtsamdEdgeTriggerCapture::new();

    // Configure a clock for the EIC peripheral
    let gclk2 = clocks.get_gclk(ClockGenId::Gclk2).unwrap();
    let eic_clock = clocks.eic(&gclk2).unwrap();

    let eic_channels = Eic::new(&mut peripherals.mclk, eic_clock, peripherals.eic).split();

    let _ot_rx: Pin<_, PullUpInterrupt> = pins.pb08.into(); // D0
    //  let pb_09_ot_tx: Pin<_, PushPullOutput> = pins.pb09.into(); // D1
    //  let capture_device = RpEdgeCapture::new(async_input);
    //  let mut open_therm_bus = AtsamdEdgeTriggerCapture::new(pb_09_ot_tx);
    let example_vector = heapless::Vec::<bool, 13>::from_slice(&[true, true, false, true, false, true, false, false, true, false, false]).unwrap();
    //  let _ = open_therm_bus.trigger(example_vector.into_iter(), Duration::from_millis(100));

    let time_driver = AtsamdTimeDriver::new();
    //  let mut boiler_controller = BoilerControl::new(open_therm_bus, time_driver);
    //  let _ = boiler_controller.set_point(Temperature::Celsius(16));
    //  let _ = boiler_controller.enable_ch(CHState::Enable(true));

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
