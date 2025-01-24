#![no_std]
#![no_main]

use bsp::hal::time::Hertz;
use core::time::Duration;
use defmt_rtt as _;
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

    let pins = Pins::new(peripherals.port);
    let pwm_pin = pins.pb09.into_alternate::<E>();

    let mut pwm4 = PwmWg4::<PB09>::new(
        &clocks.tc4_tc5(&gclk0).unwrap(),
        Hertz::from_raw(8),
        peripherals.tc4,
        TC4Pinout::Pb9(pwm_pin),
        &mut peripherals.mclk,
    );

    let max_duty = pwm4.get_max_duty();

    //  DMA setup:
    let mut source = [0xffu8; 1000];
    for (index, value) in source.iter_mut().enumerate() {
        *value = index as u8;
    }

    let dmac_for_cnt = unsafe { crate::pac::Peripherals::steal().dmac };
    //  self.regs.chctrla.modify(|_, w| w.swrst().set_bit());
    //  .chctrla.read().swrst().bit_is_set() {}
    //  while self.regs.chctrla.read().swrst().bit_is_set() {}

    loop {
        pwm4.set_duty(max_duty / 2);
        let pwm_dma_address = pwm4.get_dma_ptr();
        let dma_future = channel0.transfer_future(
            &mut source,
            pwm_dma_address,
            TriggerSource::Tc4Ovf,
            TriggerAction::Burst,
        );
        //  .await
        //  .unwrap();
        /* Now, how to poll the future manually? */
        //  let _result = future.poll();

        // Waker is just a simple stub in this example.
        let waker = unsafe {
            core::task::Waker::from_raw(core::task::RawWaker::new(
                core::ptr::null(),
                &RAW_WAKER_VTABLE,
            ))
        };

        // Pin the future to prevent it from being moved
        //  let future_pin: Pin<&mut MyFuture> = unsafe { Pin::new_unchecked(&mut FUTURE) };

        let mut cx = Context::from_waker(&waker);
        // Poll the future manually in the `no_std` environment
        let mut dma_future_pin = core::pin::pin!(dma_future);
        //  , &waker);

        // Manually calling poll shall trigger the DMA transfer:
        let result = match dma_future_pin.as_mut().poll(&mut cx) {
            Poll::Pending => unsafe {
                //  core::hint::unreachable_unchecked()
                hprintln!("Pending").ok();
                Ok(())
            },
            Poll::Ready(output) => {
                hprintln!("Ready").ok();
                output
            }
        };

        //  hprintln!{"{}", }.ok();
        //  pwm4.set_duty(max_duty / 2);
        loop {
            let dma_status = dmac_for_cnt.active().read().bits();
            hprintln!("{}", dma_status).ok();
        }
        delay.delay_ms(10000u16);
        //  pwm4.set_duty(max_duty / 8);
        //  delay.delay_ms(2000u16);
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

    let _ot_rx: GpioPin<_, PullUpInterrupt> = pins.pb08.into(); // D0
                                                                //  let pb_09_ot_tx: GpioPin<_, PushPullOutput> = pins.pb09.into(); // D1
                                                                //  let capture_device = RpEdgeCapture::new(async_input);
                                                                //  let mut open_therm_bus = AtsamdEdgeTriggerCapture::new(pb_09_ot_tx);
    let example_vector = heapless::Vec::<bool, 13>::from_slice(&[
        true, true, false, true, false, true, false, false, true, false, false,
    ])
    .unwrap();
    //  let _ = open_therm_bus.trigger(example_vector.into_iter(), Duration::from_millis(100));

    #[cfg(feature = "use_opentherm")]
    let time_driver = AtsamdTimeDriver::new();
    //  let mut boiler_controller = BoilerControl::new(open_therm_bus, time_driver);
    //  let _ = boiler_controller.set_point(Temperature::Celsius(16));
    //  let _ = boiler_controller.enable_ch(CHState::Enable(true));

    let button: GpioPin<_, PullUpInterrupt> = pins.pd10.into();
    let mut extint = eic_channels.5.with_pin(button).into_future(Irqs);
    extint.enable_interrupt();

    loop {
        // Here we show straight falling edge detection without
        extint.wait(Sense::Fall).await;
        defmt::info!("Falling edge detected");
        user_led.toggle().unwrap();
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
