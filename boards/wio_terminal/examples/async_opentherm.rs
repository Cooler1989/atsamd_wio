#![no_std]
#![no_main]

use bsp::hal::time::Hertz;
use core::time::Duration;
use defmt_rtt as _;
use hal::fugit::Hertz as FugitHertz;
use hal::fugit::MillisDuration;
use heapless::Vec;
use panic_probe as _;

use crate::pac::Mclk;
use bsp::pac;
use bsp::{
    hal,
    hal::gpio::{Output, OutputConfig, Pins, PushPull, PushPullOutput},
    hal::gpio::{E, PB08, PB09},
    pin_alias,
};
use wio_terminal::prelude::_embedded_hal_PwmPin;

use hal::{
    clock::{ClockGenId, ClockSource, GenericClockController, Tc4Tc5Clock},
    delay::Delay,
    dmac,
    dmac::{Beat, Buffer},
    dmac::{DmaController, PriorityLevel, TriggerAction, TriggerSource},
    ehal::digital::{OutputPin, StatefulOutputPin},
    eic::{Eic, Sense},
    gpio::{Pin as GpioPin, PullUpInterrupt},
    pwm::{Pwm4, TC4Pinout},
    timer_capture_waveform::{TimerCapture4,TimerCapture4Future},
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

#[cfg(feature = "use_opentherm")]
use boiler::opentherm_interface::{
    edge_trigger_capture_interface::{
        CaptureError, EdgeCaptureInterface, EdgeTriggerInterface, InitLevel, TriggerError,
    },
    open_therm_message::{CHState, Temperature},
    OpenThermEdgeTriggerBus,
};
#[cfg(feature = "use_opentherm")]
use boiler::{BoilerControl, Instant, TimeBaseRef};
#[cfg(feature = "use_opentherm")]
use opentherm_boiler_controller_lib as boiler;

#[cfg(feature = "use_semihosting")]
use panic_semihosting as _;

use cortex_m_semihosting::hprintln;

atsamd_hal::bind_interrupts!(struct Irqs {
    EIC_EXTINT_5 => atsamd_hal::eic::InterruptHandler;
});

atsamd_hal::bind_multiple_interrupts!(struct DmacIrqs {
    DMAC: [DMAC_0, DMAC_1, DMAC_2, DMAC_OTHER] => atsamd_hal::dmac::InterruptHandler;
});
 
trait OtMode{}
struct NoneT{}
struct OtTx{}
struct OtRx{}
impl OtMode for OtTx{}
impl OtMode for OtRx{}
impl OtMode for NoneT{}

#[cfg(feature = "use_opentherm")]
#[embassy_executor::task]
async fn boiler_task() {}

#[cfg(feature = "use_opentherm")]
mod boiler_implementation {
    use crate::dmac::ReadyFuture;
    use crate::hal::pwm_wg::PwmWg4Future;
    use atsamd_hal::gpio::Alternate;
    use atsamd_hal::pac::gclk::genctrl::OeR;
    use core::marker::PhantomData;

    use super::*;

    struct ResourceHolder<D> {
        pin_rx: GpioPin<PB08, PushPullOutput>,
        tc4_timer: pac::Tc4,
        tc4_tc5_clock: Tc4Tc5Clock,
        dma_channel: D,
    }

    const VEC_SIZE_CAPTURE: usize = 128;
    pub(super) struct AtsamdEdgeTriggerCapture<
        'a,
        D: dmac::AnyChannel<Status = ReadyFuture>,
        M: OtMode = NoneT,
        const N: usize = VEC_SIZE_CAPTURE,
    > {
        tx_pin: Option<GpioPin<PB09, Alternate<E>>>,
        rx_pin: Option<GpioPin<PB08, PushPullOutput>>,
        tx_init_duty_value: u8,
        pwm: Option<PwmWg4Future<PB09, D>>, // one alternative when TX operation
        capture_device:Option<TimerCapture4Future<PB08, D>>, // one alternative when RX operation
        dma: PhantomData<D>,
        rx_resource_holder: Option<ResourceHolder<D>>,
        mclk: &'a mut Mclk,
        mode: PhantomData<M>,
    }

    impl<'a, D> AtsamdEdgeTriggerCapture<'a, D, OtTx, VEC_SIZE_CAPTURE>
    where 
        D: dmac::AnyChannel<Status = ReadyFuture>,
    {
        pub fn new_with_default(
            pin_tx: GpioPin<PB09, PushPullOutput>,
            pin_rx: GpioPin<PB08, PushPullOutput>,
            tc4_timer: pac::Tc4,
            mclk: &'a mut Mclk,
            tc4_tc5_clock: &Tc4Tc5Clock,
            dma_channel: D,
        ) -> AtsamdEdgeTriggerCapture<'a, D, OtTx, VEC_SIZE_CAPTURE> {
            let pwm_tx_pin = pin_tx.into_alternate::<E>();

            let pwm4 = PwmWg4::<PB09>::new_waveform_generator(
                tc4_tc5_clock,
                Hertz::from_raw(32),
                tc4_timer,
                TC4Pinout::Pb9(pwm_tx_pin),
                mclk,
            )
            .with_dma_channel(dma_channel); // TODO: Channel shall be changed to channel0 later on. This is
                                            // just for prototyping
            Self {
                tx_pin: None,
                rx_pin: Some(pin_rx),
                tx_init_duty_value: 0xff, // This determines idle bus state level. TODO: add configuration
                pwm: Some(pwm4),
                capture_device: None,
                dma: PhantomData,
                rx_resource_holder: None,
                mclk: mclk,
                mode: PhantomData,
            }
        }
    }

    //  impl<'a, D, const N: usize> AtsamdEdgeTriggerCapture<'a, D, NoneT, N>
    //  where 
    //      D: dmac::AnyChannel<Status = ReadyFuture>,
    //  {
    //      //  pub fn transition_to_rx(self, tc4_tc5_clock: &Tc4Tc5Clock) -> AtsamdEdgeTriggerCapture<'a, D, OtRx, N> {
    //      //      let (pin_tx, pin_rx, pwm) = (
    //      //          self.tx_pin.unwrap(),
    //      //          self.rx_pin.unwrap(),
    //      //          self.pwm.unwrap(),
    //      //      );
    //      //      let (dma, tc4_timer, _d) = pwm.decompose();
    //      //      Some(Self::new(
    //      //          pin_tx.into(),
    //      //          pin_rx,
    //      //          tc4_timer,
    //      //          self.mclk,
    //      //          tc4_tc5_clock,
    //      //          dma,
    //      //      ))
    //      //  }
    //  }

    impl <'a, D, const N: usize> AtsamdEdgeTriggerCapture<'a, D, OtTx, N>
    where 
        D: dmac::AnyChannel<Status = ReadyFuture>,
    {
        //  Starting with TX as the boiler controller is more common and uses the TX command first
        pub fn new(
            pin_tx: GpioPin<PB09, PushPullOutput>,
            pin_rx: GpioPin<PB08, PushPullOutput>,
            tc4_timer: pac::Tc4,
            mclk: &'a mut Mclk,
            tc4_tc5_clock: &Tc4Tc5Clock,
            dma_channel: D,
        ) -> AtsamdEdgeTriggerCapture<'a, D, OtTx, N> {
            let pwm_tx_pin = pin_tx.into_alternate::<E>();

            let pwm4 = PwmWg4::<PB09>::new_waveform_generator(
                tc4_tc5_clock,
                Hertz::from_raw(32),
                tc4_timer,
                TC4Pinout::Pb9(pwm_tx_pin),
                mclk,
            )
            .with_dma_channel(dma_channel); // TODO: Channel shall be changed to channel0 later on. This is
                                            // just for prototyping
            Self {
                tx_pin: None,
                rx_pin: Some(pin_rx),
                tx_init_duty_value: 0xff, // This determines idle bus state level. TODO: add configuration
                pwm: Some(pwm4),
                capture_device: None,
                dma: PhantomData,
                rx_resource_holder: None,
                mclk: mclk,
                mode: PhantomData,
            }
        }
        pub fn transition_to_rx(self, tc4_tc5_clock: &Tc4Tc5Clock ) -> Option<AtsamdEdgeTriggerCapture<'a, D, OtRx>> {
            let (pin_tx, pin_rx, pwm) = (
                self.tx_pin.unwrap(),
                self.rx_pin.unwrap(),
                self.pwm.unwrap(),
            );
            let (dma, tc4_timer, _d) = pwm.decompose();
            Some(AtsamdEdgeTriggerCapture::<'a, D, OtRx>::new(
                pin_tx.into(),
                pin_rx,
                tc4_timer,
                self.mclk,
                tc4_tc5_clock,
                dma,
            ))
        }

        pub fn transition(self, tc4_tc5_clock: &Tc4Tc5Clock,) -> Option<AtsamdEdgeTriggerCapture<'a, D, OtTx>> {
            let (pin_tx, pin_rx, pwm) = (
                self.tx_pin.unwrap(),
                self.rx_pin.unwrap(),
                self.pwm.unwrap(),
            );
            let (dma, tc4_timer, _d) = pwm.decompose();
            todo!()
            //  Some(Self::new(
            //      pin_tx.into(),
            //      pin_rx,
            //      tc4_timer,
            //      self.mclk,
            //      tc4_tc5_clock,
            //      dma,
            //  ))
        }

    }

    impl<'a, D, const N: usize> AtsamdEdgeTriggerCapture<'a, D, OtRx, N>
    where
        D: dmac::AnyChannel<Status = ReadyFuture>,
    {
        pub fn new(
            pin_tx: GpioPin<PB09, PushPullOutput>,
            pin_rx: GpioPin<PB08, PushPullOutput>,
            tc4_timer: pac::Tc4,
            mclk: &'a mut Mclk,
            tc4_tc5_clock: &Tc4Tc5Clock,
            dma_channel: D,
        ) -> Self {
            let pwm_rx_pin = pin_rx.into_alternate::<E>();

            let timer_capture_4 = TimerCapture4::<PB08>::new_timer_capture(
                tc4_tc5_clock,
                Hertz::from_raw(32),
                tc4_timer,
                TC4Pinout::Pb8(pwm_rx_pin),
                mclk,
            )
            .with_dma_channel(dma_channel); // TODO: Channel shall be changed to channel0 later on. This is
                                            // just for prototyping
            Self {
                tx_pin: Some(pin_tx.into()),
                rx_pin: None,
                tx_init_duty_value: 0xff, // This determines idle bus state level. TODO: add configuration
                pwm: None,
                capture_device: Some(timer_capture_4),  //  TODO: Implement the capture device
                dma: PhantomData,
                rx_resource_holder: None,
                mclk: mclk,
                mode: PhantomData,
            }
        }
        pub fn transition_to_tx(self, tc4_tc5_clock: &Tc4Tc5Clock ) -> Option<AtsamdEdgeTriggerCapture<'a, D, OtTx>> {
            let (pin_tx, pin_rx, pwm) = (
                self.tx_pin.unwrap(),
                self.rx_pin.unwrap(),
                self.pwm.unwrap(),
            );
            let (dma, tc4_timer, _d) = pwm.decompose();
            
            Some(AtsamdEdgeTriggerCapture::<'a, D, OtTx>::new(
                pin_tx.into(),
                pin_rx,
                tc4_timer,
                self.mclk,
                tc4_tc5_clock,
                dma,
            ))
        }
    }

    impl<'a, D, const N: usize> AtsamdEdgeTriggerCapture<'a, D, NoneT, N>
    where
        D: dmac::AnyChannel<Status = ReadyFuture>,
    {
        fn decompose(self) -> (D, pac::Tc4, TC4Pinout<PB09>) {
            todo!()
            //  let pwm = self.pwm;
            //  let (pin_tx, pin_rx, tc4_timer, pwm) = (
            //      self.tx_pin.unwrap(),
            //      self.rx_pin.unwrap(),
            //      self.tc4_timer,
            //      self.pwm.unwrap(),
            //  );
        }

    }

    // impl<'a, D, M, const N: usize> AtsamdEdgeTriggerCapture<'a, D, NoneT, N>
    impl<D, const N: usize> EdgeTriggerInterface for AtsamdEdgeTriggerCapture<'_, D, OtTx, N>
    where
        D: dmac::AnyChannel<Status = ReadyFuture>,
    {
        async fn trigger(
            mut self,
            iterator: impl Iterator<Item = bool>,
            period: core::time::Duration,
        ) -> (Self, Result<(), TriggerError>) {
            let response = match self.pwm.as_mut() {
                Some(pwm) => {
                    let mut source: [u8; N] = [self.tx_init_duty_value; N];
                    for (idx, value) in iterator.enumerate() {
                        if idx >= N {
                            break;
                        }
                        //  TODO: Implement configurable idle bus state level
                        let level = if value { 0xffu8 } else { 0x00u8 };
                        source[idx] = level;
                    }
                    //  return:
                    //  TODO: Actually use the period to set the PWM frequency
                    pwm.start_regular_pwm(self.tx_init_duty_value);
                    let dma_future = self
                        .pwm
                        .as_mut()
                        .unwrap() /* TODO: remove runtime panic */
                        .start_timer_prepare_dma_transfer(self.tx_init_duty_value, &mut source);
                    dma_future.await.map_err(|_| TriggerError::GenericError)
                }
                None => {
                    Err(TriggerError::GenericError)
                }
            };
            (self, response)
        }

    }

    impl<D, const N: usize> EdgeCaptureInterface<N> for AtsamdEdgeTriggerCapture<'_, D, OtRx, N>
    where
        D: dmac::AnyChannel<Status = ReadyFuture>,
    {
        async fn start_capture(
            mut self,
            timeout_inactive_capture: core::time::Duration,
            timeout_till_active_capture: core::time::Duration,
        ) -> (Self, Result<(InitLevel, Vec<core::time::Duration, N>), CaptureError>) {
            todo!()
        }
    }

    pub(super) struct AtsamdTimeDriver {}

    impl AtsamdTimeDriver {
        pub(super) fn new() -> Self {
            Self {}
        }
    }

    impl TimeBaseRef for AtsamdTimeDriver {
        fn now(&self) -> Instant {
            //  Instant { ticks: 0 }
            todo!()
        }
    }

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
    let mut pwm_tx_pin = pins.pb09.into_push_pull_output();
    //  Set initial level of OpenTherm bus to high:
    pwm_tx_pin.set_high().unwrap();

    let pwm_rx_pin = pins.pb08.into_push_pull_output();
    // let pwm_tx_pin = pins.pb09.into_alternate::<E>();
    let mut tc4_timer = peripherals.tc4;

    Mono::delay(MillisDuration::<u32>::from_ticks(50).convert()).await;

    //  let mut user_led: bsp::UserLed = pin_alias!(pins.user_led).into();
    let mut user_led: UserLed = pins.pa15.into();
    user_led.toggle().unwrap();

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

    #[cfg(feature = "use_opentherm")]
    let mut edge_trigger_capture_dev =
        boiler_implementation::AtsamdEdgeTriggerCapture::new_with_default(
            pwm_tx_pin,
            pwm_rx_pin,
            tc4_timer,
            &mut peripherals.mclk,
            &clocks.tc4_tc5(&gclk0).unwrap(),
            channel0,
    );

    let _time_driver = boiler_implementation::AtsamdTimeDriver::new();
    //  let mut boiler_controller = BoilerControl::new(edge_trigger_capture_dev, time_driver);
    //  let _ = boiler_controller.set_point(Temperature::Celsius(16));

    let (device, result) = edge_trigger_capture_dev
        .trigger(
            [
                true, true, true, true, false, true, false, true, false, false, true, false, false,
                true, true, true,
            ]
            .iter()
            .copied(),
            Duration::from_millis(100),
        )
        .await;
    result.unwrap();

    let device = device.transition_to_rx(&clocks.tc4_tc5(&gclk0).unwrap()).unwrap();

    let (device, result) = device.start_capture(Duration::from_millis(100), Duration::from_millis(100)).await;
    if let Ok((level, vector)) = result {
        hprintln!("Capture finished with: {}", vector.len()).ok();
    }

    let device = device.transition_to_tx(&clocks.tc4_tc5(&gclk0).unwrap()).unwrap();

    let (device, _result ) = device
        .trigger(
            [
                true, true, true, true, false, true, false, true, false, false, true, false, false,
                true, true, true,
            ]
            .iter()
            .copied(),
            Duration::from_millis(100),
        )
        .await;

    hprintln!("main:: loop{} start:").ok();

    loop {
        //  let _ = boiler_controller.process().await.unwrap();

        user_led.toggle().unwrap();
        Mono::delay(MillisDuration::<u32>::from_ticks(500).convert()).await;
        //      //  pwm4.set_duty(max_duty / 8);
        //      //  delay.delay_ms(2000u16);
    }

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
