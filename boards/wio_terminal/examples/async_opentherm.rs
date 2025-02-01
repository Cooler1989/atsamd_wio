#![no_std]
#![no_main]

use bsp::hal::time::Hertz;
use core::time::Duration;
use defmt_rtt as _;
use hal::fugit::Hertz as FugitHertz;
use hal::fugit::MillisDuration;
use heapless::Vec;
use panic_probe as _;

use bsp::pac;
use bsp::{
    hal,
    hal::gpio::{Output, OutputConfig, Pins, PushPull, PushPullOutput},
    hal::gpio::{E, PB09},
    pin_alias,
};
use wio_terminal::prelude::_embedded_hal_PwmPin;

use hal::{
    clock::{ClockGenId, ClockSource, GenericClockController},
    delay::Delay,
    dmac::{Beat, Buffer},
    dmac::{DmaController, PriorityLevel, TriggerAction, TriggerSource},
    ehal::digital::StatefulOutputPin,
    eic::{Eic, Sense},
    gpio::{Pin as GpioPin, PullUpInterrupt},
    pwm::{Pwm4, TC4Pinout},
    pwm_wg::PwmWg4,
};
use wio_terminal::prelude::_embedded_hal_blocking_delay_DelayMs;

use bsp::pins::UserLed;
use wio_terminal as bsp;

use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};
use rtic_monotonics::Monotonic;

use modular_bitfield::prelude::*;

rtic_monotonics::systick_monotonic!(Mono, 10000);

//  use boiler::{BoilerControl, Instant, TimeBaseRef};
//  use opentherm_boiler_controller_lib as boiler;
//  use boiler::opentherm_interface::{
//      edge_trigger_capture_interface::{
//          CaptureError, EdgeCaptureInterface, EdgeTriggerInterface, InitLevel, TriggerError,
//      },
//      CHState, OpenThermEdgeTriggerBus, Temperature,
//  };

#[cfg(feature = "use_semihosting")]
use panic_semihosting as _;

use cortex_m_semihosting::hprintln;

atsamd_hal::bind_interrupts!(struct Irqs {
    EIC_EXTINT_5 => atsamd_hal::eic::InterruptHandler;
});

atsamd_hal::bind_multiple_interrupts!(struct DmacIrqs {
    DMAC: [DMAC_0, DMAC_1, DMAC_2, DMAC_OTHER] => atsamd_hal::dmac::InterruptHandler;
});

//  #[derive(Clone)]
//  pub(crate) struct PwmWaveformGeneratorPtr<T: Beat>(pub *mut T);
//
//  unsafe impl<T: Beat> Buffer for PwmWaveformGeneratorPtr<T> {
//      type Beat = T;
//
//      #[inline]
//      fn dma_ptr(&mut self) -> *mut Self::Beat {
//          self.0
//      }
//
//      #[inline]
//      fn incrementing(&self) -> bool {
//          false
//      }
//
//      #[inline]
//      fn buffer_len(&self) -> usize {
//          1
//      }
//  }

#[cfg(feature = "use_opentherm")]
mod boiler_implementation {

    #[embassy_executor::task]
    async fn boiler_task() {}

    const VEC_SIZE_CAPTURE: usize = 128;
    struct AtsamdEdgeTriggerCapture<const N: usize = VEC_SIZE_CAPTURE> {
        output_pin: GpioPin<PB09, PushPullOutput>,
    }

    impl AtsamdEdgeTriggerCapture {
        pub fn new(pin_tx: GpioPin<PB09, PushPullOutput>) -> Self {
            Self { output_pin: pin_tx }
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

    struct AtsamdTimeDriver {}

    impl AtsamdTimeDriver {
        fn new() -> Self {
            Self {}
        }
    }

    impl TimeBaseRef for AtsamdTimeDriver {
        fn now(&self) -> Instant {
            todo!()
        }
    }
    impl OpenThermEdgeTriggerBus for AtsamdEdgeTriggerCapture {}
}

#[inline]
pub fn check_and_clear_interrupts(flags: InterruptFlags) -> InterruptFlags {
    let mut cleared = 0;
    let tc4 = unsafe { crate::pac::Peripherals::steal().tc4 };
    tc4.count8().intflag().modify(|r, w| {
        cleared = r.bits() & flags.into_bytes()[0];
        unsafe { w.bits(flags.into_bytes()[0]) }
    });
    InterruptFlags::from_bytes([cleared])
}

#[embassy_executor::task]
async fn print_timer_state_task(/*mut uart_tx: UartFutureTxDuplexDma<Config<bsp::UartPads>, Ch1>*/)
{
    let tc4_readonly = unsafe { crate::pac::Peripherals::steal().tc4 };
    let dmac_readonly = unsafe { crate::pac::Peripherals::steal().dmac };
    let mut value_cc1 = 0x00u8;
    loop {
        //  Read this value:
        let vcc1 = tc4_readonly.count8().cc(1).read().bits();
        if vcc1 != value_cc1 {
            hprintln!("tc4.cc1:0x{:08X}", vcc1).ok();
            value_cc1 = vcc1;
        }

        //  uart_tx.write(b"Hello, world!").await.unwrap();
        //  defmt::info!("Sent 10 bytes");

        //  let mut delay = Delay::new(core.SYST, &mut clocks);
        let _ = tc4_readonly
            .count8()
            .ctrlbset()
            .write(|w| w.cmd().readsync());
        let cnt_value = tc4_readonly.count8().count().read().bits();
        let _ = tc4_readonly
            .count8()
            .ctrlbset()
            .write(|w| w.cmd().readsync());
        let cn2_value = tc4_readonly.count8().count().read().bits();

        hprintln!("cnt:0x{:08X}, 0x{:08X}", cnt_value, cn2_value).ok();
        //  hprintln!(
        //      "tc4int:0x{:08X}, cc1:0x{:08X}",
        //      tc4_readonly.count8().intflag().read().bits(),
        //      tc4_readonly.count8().cc(1).read().bits()
        //  )
        //  .ok();
        //  hprintln!("tc4per:0x{:08X}", tc4_readonly.count8().per().read().bits()).ok();
        //  hprintln!("dma:0x{:08X}", dmac_readonly.active().read().bits()).ok();

        let flags_to_check = InterruptFlags::new().with_ovf(true).with_err(true);
        if check_and_clear_interrupts(flags_to_check).ovf() {
            //  hprintln!("Overflow detected").ok();
        }

        //  delay.delay_ms(200u16);
        Mono::delay(MillisDuration::<u32>::from_ticks(500).convert()).await;
    }
}

#[embassy_executor::main]
async fn main(spawner: embassy_executor::Spawner) {
    let mut peripherals = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    //  let core = CorePeripherals::take().unwrap();

    #[cfg(feature = "use_opentherm")]
    spawner.spawn(boiler_task()).unwrap();

    let mut clocks = GenericClockController::with_external_32kosc(
        peripherals.gclk,
        &mut peripherals.mclk,
        &mut peripherals.osc32kctrl,
        &mut peripherals.oscctrl,
        &mut peripherals.nvmctrl,
    );
    let gclk0 = clocks.gclk0();
    //  let mut delay = Delay::new(core.SYST, &mut clocks);
    let freq: FugitHertz<u32> = clocks.gclk0().into();
    Mono::start(core.SYST, freq.to_Hz());

    // Initialize DMA Controller
    let dmac = DmaController::init(peripherals.dmac, &mut peripherals.pm);
    // Turn dmac into an async controller
    let mut dmac = dmac.into_future(DmacIrqs);
    // Get individual handles to DMA channels
    let channels = dmac.split();
    // Initialize DMA Channels 0 and 1
    let mut channel0 = channels.0.init(PriorityLevel::Lvl0);
    let channel1 = channels.1.init(PriorityLevel::Lvl0);

    let pins = Pins::new(peripherals.port);
    let pwm_pin = pins.pb09.into_alternate::<E>();

    let tc4_readonly = unsafe { crate::pac::Peripherals::steal().tc4 };

    // let dmac_readonly = unsafe { crate::pac::Peripherals::steal().dmac };
    //  self.regs.chctrla.modify(|_, w| w.swrst().set_bit());
    //  .chctrla.read().swrst().bit_is_set() {}
    //  while self.regs.chctrla.read().swrst().bit_is_set() {}

    let mut tc4_timer = peripherals.tc4;

    let mut pwm4 = PwmWg4::<PB09>::new_waveform_generator(
        &clocks.tc4_tc5(&gclk0).unwrap(),
        Hertz::from_raw(32),
        tc4_timer,
        TC4Pinout::Pb9(pwm_pin),
        &mut peripherals.mclk,
    )
    .with_dma_channel(channel0); // TODO: Channel shall be changed to channel0 later on. This is
                                 // just for prototyping

    hprintln!(
        "TC4.perbuf:0x{:08X}",
        tc4_readonly.count8().perbuf().read().bits()
    )
    .ok();
    hprintln!(
        "TC4.ctrla:0x{:08X}",
        tc4_readonly.count8().ctrla().read().bits()
    )
    .ok();
    hprintln!(
        "TC4.ctrlbSet:0x{:08X}",
        tc4_readonly.count8().ctrlbset().read().bits()
    )
    .ok();
    hprintln!(
        "TC4.ctrlbClr:0x{:08X}",
        tc4_readonly.count8().ctrlbclr().read().bits()
    )
    .ok();
    hprintln!(
        "TC4.wave:0x{:08X}",
        tc4_readonly.count8().wave().read().bits()
    )
    .ok();
    hprintln!(
        "TC4.cc0:0x{:08X}",
        tc4_readonly.count8().cc(0).read().bits()
    )
    .ok();
    hprintln!(
        "TC4.cc1:0x{:08X}",
        tc4_readonly.count8().cc(1).read().bits()
    )
    .ok();

    spawner.spawn(print_timer_state_task()).unwrap();

    //  Control duty using using value between 0 and PER register which is set to 233
    //  233 / 2 = 116 in hex = 0x74
    //  pwm4.start_regular_pwm(0x74u8);

    hprintln!("main:: waiting Timer start").ok();
    Mono::delay(MillisDuration::<u32>::from_ticks(2000).convert()).await;
    hprintln!("main:: starting Timer").ok();
    pwm4.start_regular_pwm(0x10u8);
    hprintln!("main:: waiting DMA transfer").ok();
    Mono::delay(MillisDuration::<u32>::from_ticks(200).convert()).await;
    pwm4.start_regular_pwm(0xffu8);
    hprintln!("main:: waiting DMA transfer").ok();
    Mono::delay(MillisDuration::<u32>::from_ticks(200).convert()).await;
    pwm4.start_regular_pwm(210u8);
    hprintln!("main:: waiting DMA transfer").ok();
    Mono::delay(MillisDuration::<u32>::from_ticks(200).convert()).await;
    hprintln!("main:: starting DMA transfer").ok();
    pwm4.start_regular_pwm(0xffu8);
    hprintln!("main:: timer ff").ok();
    Mono::delay(MillisDuration::<u32>::from_ticks(10).convert()).await;

    //  DMA setup:
    let mut source = [0xffu8; 65];
    //  for (index, value) in source.iter_mut().enumerate() {
    //      *value = index as u8;
    //  }
    source
        .iter_mut()
        .enumerate()
        .filter(|(i, _)| i % 2 == 0) //  take every second element
        .for_each(|(i, v)| {
            *v = 0x00u8;
        });

    //  This will start timer, but not yet the DMA transfer

    let dma_future = pwm4.start_timer_prepare_dma_transfer(0xffu8, &mut source);

    Mono::delay(MillisDuration::<u32>::from_ticks(5).convert()).await;
    //  This will start the DMA transfer
    dma_future.await.unwrap();

    //  pwm4.start_regular_pwm(0x10u8); // To see on the analzer
    hprintln!("main:: DMA transfer finished").ok();

    let dma_future = pwm4.start_timer_prepare_dma_transfer(0xffu8, &mut source);
    Mono::delay(MillisDuration::<u32>::from_ticks(5).convert()).await;
    dma_future.await.unwrap();

    loop {
        //      hprintln!("main:: loop{} has finished").ok();
        //      //  delay.delay_ms(5000u16);
        Mono::delay(MillisDuration::<u32>::from_ticks(5000).convert()).await;
        //      //  pwm4.set_duty(max_duty / 8);
        //      //  delay.delay_ms(2000u16);
    }

    //  //  let mut user_led: bsp::UserLed = pin_alias!(pins.user_led).into();
    //  let mut user_led: UserLed = pins.pa15.into();

    //  let _internal_clock = clocks
    //      .configure_gclk_divider_and_source(ClockGenId::Gclk2, 1, ClockSource::Osculp32k, false)
    //      .unwrap();
    //  clocks.configure_standby(ClockGenId::Gclk2, true);

    //  //  let edge_trigger_capture_dev = AtsamdEdgeTriggerCapture::new();

    //  // Configure a clock for the EIC peripheral
    //  let gclk2 = clocks.get_gclk(ClockGenId::Gclk2).unwrap();
    //  let eic_clock = clocks.eic(&gclk2).unwrap();

    //  let eic_channels = Eic::new(&mut peripherals.mclk, eic_clock, peripherals.eic).split();

    //  let _ot_rx: GpioPin<_, PullUpInterrupt> = pins.pb08.into(); // D0
    //                                                              //  let pb_09_ot_tx: GpioPin<_, PushPullOutput> = pins.pb09.into(); // D1
    //                                                              //  let capture_device = RpEdgeCapture::new(async_input);
    //                                                              //  let mut open_therm_bus = AtsamdEdgeTriggerCapture::new(pb_09_ot_tx);
    //  let example_vector = heapless::Vec::<bool, 13>::from_slice(&[
    //      true, true, false, true, false, true, false, false, true, false, false,
    //  ])
    //  .unwrap();
    //  //  let _ = open_therm_bus.trigger(example_vector.into_iter(), Duration::from_millis(100));

    //  #[cfg(feature = "use_opentherm")]
    //  let time_driver = AtsamdTimeDriver::new();
    //  //  let mut boiler_controller = BoilerControl::new(open_therm_bus, time_driver);
    //  //  let _ = boiler_controller.set_point(Temperature::Celsius(16));
    //  //  let _ = boiler_controller.enable_ch(CHState::Enable(true));

    //  let button: GpioPin<_, PullUpInterrupt> = pins.pd10.into();
    //  let mut extint = eic_channels.5.with_pin(button).into_future(Irqs);
    //  extint.enable_interrupt();

    //  loop {
    //      // Here we show straight falling edge detection without
    //      extint.wait(Sense::Fall).await;
    //      defmt::info!("Falling edge detected");
    //      user_led.toggle().unwrap();
    //  }
}

#[bitfield]
#[repr(u8)]
#[derive(Clone, Copy)]
pub struct InterruptFlags {
    /// Overflow
    pub ovf: bool,
    /// Err
    pub err: bool,
    #[skip]
    _reserved: B6,
}

impl Default for InterruptFlags {
    fn default() -> Self {
        Self::new()
    }
}

// Dummy Waker implementation for no_std environment.
static RAW_WAKER_VTABLE: core::task::RawWakerVTable = core::task::RawWakerVTable::new(
    |ptr: *const ()| RawWaker::new(ptr, &RAW_WAKER_VTABLE),
    |_: *const ()| {}, // Do nothing, for simulation
    |_: *const ()| {}, // Do nothing, for simulation
    |_: *const ()| {},
);

unsafe fn raw_waker(waker_ptr: *const ()) -> Waker {
    Waker::from_raw(RawWaker::new(waker_ptr, &RAW_WAKER_VTABLE))
}
