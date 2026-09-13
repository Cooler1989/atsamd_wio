#![allow(non_snake_case)]

/// Timer capture waveform driver for D5x series.
/// TODO: Evaluate different TC mode: 48.6.2.8.2. Period and Pulse-Width (PPW/PWP) Capture Action on Events
/// TODO: write documentation for the D5x timer capture waveform driver.



use core::future::Future;
use core::sync::atomic;
use core::iter;

use atsamd_hal_macros::hal_cfg;

use crate::typelevel;
use crate::pac;
use crate::async_hal::interrupts::{Handler, Interrupt};
#[cfg(feature = "async")]
use crate::dmac::ReadyFuture;
use crate::dmac::{AnyChannel, Beat, Buffer, Error as DmacError, TriggerAction, TriggerSource};
use crate::gpio::*;
use crate::pac::Mclk;
use crate::time::Hertz;

use pin_project::pin_project;

use paste::paste;

///  Channel 0 is used to capture the counter value while channel 1 is used to trigger interrupt
///  used for timeout handling

const TIMER_CHANNEL: usize = 0;
//  Second channel may be used for timeout detection. CCx can thus be set to some value
//  that will be compared with current value of the counter and trigger interrupt when the value matches.
const TIMER_TIMEOUT_CHANNEL: usize = 1;
//  OpenTherm allows a long wait for the first request, but once an edge has
//  arrived the receiver should finish after a short idle gap. The TC compare
//  is first armed with the active wait and then refreshed by the MC0 ISR.
const CAPTURE_ACTIVE_WINDOW_MS: u32 = 20_000;
const CAPTURE_INACTIVITY_MS: u32 = 5;

#[hal_cfg("tc1-d11")]
type RegBlock = pac::tc1::RegisterBlock;

#[hal_cfg("tc3-d21")]
type RegBlock = pac::tc3::RegisterBlock;

#[hal_cfg("tc1-d5x")]
type RegBlock = pac::tc0::RegisterBlock;

#[derive(Clone)]
pub struct TimerCaptureWaveformSourcePtr<T: Beat>(pub(in super::super) *mut T);

unsafe impl<T: Beat> Buffer for TimerCaptureWaveformSourcePtr<T> {
    type Beat = T;

    #[inline]
    fn dma_ptr(&mut self) -> *mut Self::Beat {
        self.0
    }

    #[inline]
    fn incrementing(&self) -> bool {
        false
    }

    #[inline]
    fn buffer_len(&self) -> usize {
        1
    }
}

#[pin_project]
struct TimerCaptureDmaWrapper<'a, DmaFut, T> {
    #[pin]
    _dma_future: DmaFut,
    timer_started: bool,
    tc_waker_index: usize,
    _timer: &'a T,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CounterValueAtTermination(u32);

impl CounterValueAtTermination {
    pub fn get_raw_value(self) -> u32 {
        self.0
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TimerCaptureData<Container: iter::Extend<core::time::Duration>> {
    data: Container,
}

impl<Container> TimerCaptureData<Container>
where
    Container: iter::Extend<core::time::Duration>
{
    pub fn get_data(self) -> Container {
        self.data
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TimerCaptureResultAvailable<Container: iter::Extend<core::time::Duration>> {
    DmaPollReady(TimerCaptureData<Container>),
    TimerTimeout(TimerCaptureData<Container>),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct ExtendWithFirstZeroAndTerminationValueIterator<'a>{
    data: &'a [u32],
    first_done: bool,
    depleted: bool,
    last_value: CounterValueAtTermination,
}

impl<'a> ExtendWithFirstZeroAndTerminationValueIterator<'a> {
    fn new(data: &'a [u32], counter_at_termination: CounterValueAtTermination) -> Self {
        Self { data, first_done: false, depleted: false, last_value: counter_at_termination }
    }
}

//  And only non-zero values are returned. Otherwise the iterator will return `None`
impl<'a> iter::Iterator for ExtendWithFirstZeroAndTerminationValueIterator<'a> {
    type Item = u32;
    fn next(&mut self) -> Option<Self::Item> {
        if self.data.len() == 0 {
            if self.depleted == false {
                self.depleted = true;
                if let CounterValueAtTermination(value) = self.last_value {
                    return Some(value);
                } else {
                    panic!("Invalid value of the counter at termination");
                }
            }
            return None;
        }
        if self.first_done == false {
            self.first_done = true;
            return Some(0u32);
        }
        if self.depleted == true {
            return None;
        }

        let value = self.data[0];
        self.data = &self.data[1..];
        if value == 0 {
            self.depleted = true;
            return None;
        }
        Some(value)
    }
}

#[derive(Debug, PartialEq, Eq)]
struct TimerCaptureRawData<'a> {
    counter_at_termination: CounterValueAtTermination,
    data: Option<&'a mut [u32]>,
}
// TODO: DmaPollReady and TimerTimeout should be merged into one trait / type to avoid processing code duplication:
#[derive(Debug, PartialEq, Eq)]
pub enum TimerCaptureRawResultAvailable<'a> {
    DmaPollReady(TimerCaptureRawData<'a>),
    TimerTimeout(TimerCaptureRawData<'a>),
}

impl<'a> AsMut<TimerCaptureRawData<'a>> for TimerCaptureRawResultAvailable<'a> {
    fn as_mut(&mut self) -> &mut TimerCaptureRawData<'a> {
        match self {
            TimerCaptureRawResultAvailable::DmaPollReady(data) => data,
            TimerCaptureRawResultAvailable::TimerTimeout(data) => data,
        }
    }
}

impl<'a> TimerCaptureRawResultAvailable<'a> {
    pub fn get_public_data<Container: iter::Extend<core::time::Duration>>(self, mut container: Container, max_items: usize) -> TimerCaptureResultAvailable<Container> {
        let adjust_ratio = |x: u64| (x * 4173 * 2) / 1000; //  TODO: fix this by some ratio compund type

        //  Convert the raw 32-bit edge-counter snapshots into inter-edge
        //  Durations. The capture counter is a free-running 32-bit timer that
        //  wraps at 2^32, so consecutive snapshots (and the termination value)
        //  are NOT guaranteed to be strictly increasing. Use wrapping
        //  subtraction so a single wrap is handled correctly. Previously a
        //  non-increasing pair hit `todo!()` and HardFaulted the firmware,
        //  which manifested as random crashes right after a frame (the counter
        //  keeps running into the idle timeout) and much more often at higher
        //  frame rates, where a wrap during the capture window is more likely.
        //  Intervals longer than a full 32-bit period cannot be represented and
        //  would alias, but those are far longer than any real OpenTherm gap.
        match self {
            TimerCaptureRawResultAvailable::DmaPollReady(data) => {
                let counter_value_at_termination = data.counter_at_termination;
                if let Some(data) = data.data {
                    let data_with_preceeding_zero = ExtendWithFirstZeroAndTerminationValueIterator::new(data, counter_value_at_termination);
                    //  `take(max_items)` guards against a full DMA capture
                    //  buffer: this iterator yields one Duration per edge PAIR,
                    //  i.e. up to (samples + 1) items, which can exceed the
                    //  fixed-capacity destination container and previously
                    //  panicked inside heapless `Vec::extend` (push().unwrap()).
                    //  Truncating the harmless tail is far better than a HardFault.
                    container.extend(data_with_preceeding_zero.clone().
                        zip(data_with_preceeding_zero.skip(1)).
                        map(|(x, y)| {
                            let delta = y.wrapping_sub(x);
                            core::time::Duration::from_nanos(adjust_ratio(delta as u64))
                        }
                    ).take(max_items));
                }
                TimerCaptureResultAvailable::DmaPollReady(TimerCaptureData {
                    data: container,
                })
            }
            TimerCaptureRawResultAvailable::TimerTimeout(data) => {
                let counter_value_at_termination = data.counter_at_termination;
                if let Some(data) = data.data {
                    let data_with_preceeding_zero = ExtendWithFirstZeroAndTerminationValueIterator::new(data, counter_value_at_termination);
                    //  See the DmaPollReady arm: cap the produced Durations to
                    //  the container capacity so a full buffer never overflows.
                    container.extend(data_with_preceeding_zero.clone().
                        zip(data_with_preceeding_zero.skip(1)).
                        map(|(x, y)| {
                            let delta = y.wrapping_sub(x);
                            core::time::Duration::from_nanos(adjust_ratio(delta as u64))
                        }
                    ).take(max_items));
                }
                TimerCaptureResultAvailable::TimerTimeout(TimerCaptureData {
                    data: container,
                })
            }
        }
    }
    pub fn fill_the_capture_memory<'b>(&mut self, capture_memory: &'b mut [u32])
    where
        'b: 'a,
    {
        self.as_mut().data = Some(capture_memory);
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TimerCaptureFailure {
    DmaFailed(DmacError),
    TimerFailed,
}

impl<'a, DmaFut, T> TimerCaptureDmaWrapper<'a, DmaFut, T> {
    fn new(dma_future: DmaFut, timer: &'a T, tc_index: usize) -> Self {
        Self {
            _dma_future: dma_future,
            timer_started: false,
            tc_waker_index:tc_index,
            _timer: timer,
         }
    }
}

impl<'a, DmaFut, T> core::future::Future for TimerCaptureDmaWrapper<'a, DmaFut, T>
where
    DmaFut: core::future::Future<Output = Result<(), DmacError>>,
    T: TimerCounterStart,
{
    type Output = Result<TimerCaptureRawResultAvailable<'a>, TimerCaptureFailure>;

    fn poll(
        mut self: core::pin::Pin<&mut Self>,
        cx: &mut core::task::Context<'_>,
    ) -> core::task::Poll<Self::Output> {
        use waker::MC1_INTERRUPT_FIRED;

        //  let mut pinned = core::pin::pin!(self);
        let this = self.as_mut().project();

        //  First time the future is polled, it sets enable bit of DMA. Only after that the timer shall be started.
        let result = this._dma_future.poll(cx);
        if result == core::task::Poll::Ready(Ok(())) {
            let counter_value = this._timer.counter_value();
            this._timer.stop();
            this._timer.disable_interrupt();
            let raw_result = TimerCaptureRawData {
                counter_at_termination: CounterValueAtTermination(counter_value),
                data: None,
            };
            //  TODO: add check on the mc1 flag related to timeout
            return core::task::Poll::Ready(Ok(TimerCaptureRawResultAvailable::DmaPollReady(raw_result)));
        }

        if *this.timer_started == false {
            *this.timer_started = true;
            MC1_INTERRUPT_FIRED[*this.tc_waker_index].store(false, atomic::Ordering::Relaxed);
            //  TODO: Enable interrupts of the timer
            this._timer.start();
            // Enable the NVIC interrupt:
            this._timer.enable_interrupt();
        }

        //  Check if the interrupt was fired:
        let result = if
            MC1_INTERRUPT_FIRED[*this.tc_waker_index].load(atomic::Ordering::Relaxed) {
            //  counter_value
            let counter_value = this._timer.counter_value();
            this._timer.stop();
            this._timer.disable_interrupt();
            let raw_result = TimerCaptureRawData {
                counter_at_termination: CounterValueAtTermination(counter_value),
                data: None,
            };
            core::task::Poll::Ready(Ok(TimerCaptureRawResultAvailable::TimerTimeout(raw_result)))
        } else {
            use waker::WAKERS;
            //  TODO: Do I have to disable interrupt/ put registartion into critical section?
            WAKERS[self.tc_waker_index].register(cx.waker());
            //  TODO: poll must be called so we need to register Waker for the future.
            //  The problem is that we have two source of the wake-up events: DMA and Timer.
            //  We need to combine them into one or as simple alternative we can use the Timer to wake up at the timeout.
            //  This is not very efficient but it is simpler.
            core::task::Poll::Pending
        };
        result
    }
}

/// Trait enabling the use of a Timer/Counter in async mode. Specifically, this
/// trait enables us to register a `TC*` interrupt as a waker for timer futures.
///
/// **⚠️ Warning** This trait should not be implemented outside of this crate!
pub trait TimerSpecificInterruptAndRegisters/* : Sealed*/ {
    /// Index of this TC in the `STATE` tracker
    const WAKER_ID: usize;

    /// Get a reference to the timer's register block
    fn reg_block(peripherals: &pac::Peripherals) -> &RegBlock;

    /// Interrupt type for this timer
    type Interrupt: Interrupt;
}

// Timer/Counter (TCx)
//
trait TimerCounterInterrupt {
    type Interrupt: Interrupt;
}

struct TimerCounterTimeoutInterruptHandler<I: TimerSpecificInterruptAndRegisters> {
    _private: (),
    _tc: core::marker::PhantomData<I>,
}

trait TimerTraitToDo {
}

pub struct InterruptHandlerAsdf<T: TimerTraitToDo> {
    _private: (),
    _tc: core::marker::PhantomData<T>,
}

//  impl<T: TimerSpecificInterruptAndRegisters> typelevel::Sealed for TimerCounterTimeoutInterruptHandler<T> {}

//  impl<T: TimerSpecificInterruptAndRegisters> Handler<T::Interrupt> for TimerCounterTimeoutInterruptHandler<T> {
//      unsafe fn on_interrupt() {
//          use waker::WAKERS;
//          //  WAKERS[extract_number!(I)].wake();
//          let peripheral = unsafe{ crate::pac::Peripherals::steal() };
//      }
//  }

const fn extract_number(tc_name: &str) -> usize {
    let bytes = tc_name.as_bytes();
    let last_byte = bytes[bytes.len() - 1];
    let tc_number = (last_byte - b'0') as usize;
    assert!(tc_number < MAX_TIMER_COUNT);
    tc_number
}

trait TimerCaptureCapable {
    type Interrupt: Interrupt;
    const WAKER_IDX: usize;
    fn reg_block(peripherals: &pac::Peripherals) -> &RegBlock;
}

pub struct TimerCaptureInterruptHandler<T: TimerCaptureCapable> {
    _private: (),
    _tc: core::marker::PhantomData<T>
}

/// EIC EXTINT0 handler for the SAMD51 TC2 event-input capture path.
///
/// In event-input mode the TC2 MC0 DMA request is generated by the capture
/// event, but the TC MC0 interrupt flag is not a dependable interrupt source
/// for refreshing the inactivity compare. EXTINT0 is the same filtered edge
/// source feeding EVSYS, so use it to refresh TC2.CC1 instead.
pub struct TimerCaptureEventInputInterruptHandler;

impl typelevel::Sealed for TimerCaptureEventInputInterruptHandler {}
impl Handler<crate::async_hal::interrupts::EIC_EXTINT_0>
    for TimerCaptureEventInputInterruptHandler
{
    unsafe fn on_interrupt() {
        let peripherals = unsafe { crate::pac::Peripherals::steal() };
        let eic = &peripherals.eic;
        if eic.intflag().read().bits() & 1 == 0 {
            return;
        }
        eic.intflag().write(|w| unsafe { w.bits(1) });

        let count = peripherals.tc2.count32();
        let _ = count.ctrlbset().write(|w| w.cmd().readsync());
        let current = count.count().read().bits();
        let timeout_ticks = waker::CAPTURE_INACTIVITY_TICKS[2]
            .load(atomic::Ordering::Relaxed);
        count
            .cc(1)
            .write(|w| unsafe { w.bits(current.wrapping_add(timeout_ticks)) });
        while count.syncbusy().read().cc1().bit_is_set() {}
    }
}

/// EIC EXTINT8 handler for a TC4 event-input capture.
pub struct TimerCaptureEventInputInterruptHandler8;

impl typelevel::Sealed for TimerCaptureEventInputInterruptHandler8 {}
impl Handler<crate::async_hal::interrupts::EIC_EXTINT_8>
    for TimerCaptureEventInputInterruptHandler8
{
    unsafe fn on_interrupt() {
        let peripherals = unsafe { crate::pac::Peripherals::steal() };
        let eic = &peripherals.eic;
        if eic.intflag().read().bits() & (1 << 8) == 0 {
            return;
        }
        eic.intflag().write(|w| unsafe { w.bits(1 << 8) });

        let count = peripherals.tc4.count32();
        let _ = count.ctrlbset().write(|w| w.cmd().readsync());
        let current = count.count().read().bits();
        let timeout_ticks = waker::CAPTURE_INACTIVITY_TICKS[4]
            .load(atomic::Ordering::Relaxed);
        count
            .cc(1)
            .write(|w| unsafe { w.bits(current.wrapping_add(timeout_ticks)) });
        while count.syncbusy().read().cc1().bit_is_set() {}
    }
}

impl <T: TimerCaptureCapable> typelevel::Sealed for TimerCaptureInterruptHandler<T> {}
impl <T: TimerCaptureCapable> Handler<T::Interrupt> for TimerCaptureInterruptHandler<T> {
    unsafe fn on_interrupt() {
        let periph = unsafe { crate::pac::Peripherals::steal() };
        let tc = T::reg_block(&periph);
        let count = tc.count32();
        let intflag = &tc.count32().intflag();

        // Each capture edge restarts the short inactivity deadline. Before
        // the first edge CC1 contains the long active-capture deadline; after
        // MC0, CC1 is moved relative to the current counter value so the DMA
        // future completes shortly after the frame goes idle.
        if intflag.read().mc0().bit_is_set() {
            intflag.modify(|_, w| w.mc0().set_bit());
            let _ = count.ctrlbset().write(|w| w.cmd().readsync());
            let current = count.count().read().bits();
            let timeout_ticks = waker::CAPTURE_INACTIVITY_TICKS
                [T::WAKER_IDX]
                .load(atomic::Ordering::Relaxed);
            count
                .cc(1)
                .write(|w| unsafe { w.bits(current.wrapping_add(timeout_ticks)) });
            while count.syncbusy().read().cc1().bit_is_set() {}
        }

        //  Wake only on the compare match channel 1, which is used for the timeout detection.
        if intflag.read().mc1().bit_is_set() {
            // Clear the flag
            intflag.modify(|_, w| w.mc1().set_bit());
            use waker::{WAKERS, MC1_INTERRUPT_FIRED};
            MC1_INTERRUPT_FIRED[T::WAKER_IDX].store(true, atomic::Ordering::Relaxed);
            WAKERS[T::WAKER_IDX].wake();
        }
    }
}

pub use crate::pwm::{PinoutCollapse, PinoutReadLevel};

pub trait TimerCaptureBaseTrait {
    type TC;
    type Pinout: PinoutCollapse;
    type ConvertibleToFuture<D>: TimerCaptureFutureTrait<TC = Self::TC, Pinout = Self::Pinout, DmaChannel = D>
    where
        D: AnyChannel<Status=ReadyFuture>;

    // fn get_dma_ptr(&self) -> TimerCaptureWaveformSourcePtr<u32>;
    fn new_timer_capture(
        clock_freq: Hertz,
        freq: Hertz,
        tc: Self::TC,
        pinout: Self::Pinout,
        mclk: Option<&mut Mclk>,
        //  timeout: MillisDurationU32,
    ) -> Self;
    /// Construct an RX capture driven by the TC event input (`EVCTRL.TCEI`)
    /// rather than the TC's own pin, so the RX line can be routed through the
    /// EIC (both-edge + glitch filter) and EVSYS. No capture pin is owned.
    fn new_timer_capture_event_input(
        clock_freq: Hertz,
        freq: Hertz,
        tc: Self::TC,
        mclk: Option<&mut Mclk>,
    ) -> Self;
    fn enable_mclk_clocks(mclk: &mut Mclk);
    /// Select which edge of the RX input the capture unit timestamps.
    ///
    /// * `false` – timestamp **rising** edges (`DRVCTRL.INVEN0` cleared, the
    ///   reset default).
    /// * `true`  – timestamp **falling** edges (`DRVCTRL.INVEN0` set): the
    ///   input is inverted before the capture logic. Use this when the signal
    ///   reaching the pin is inverted (e.g. an inverting OpenTherm bus adapter)
    ///   so the captured edges still line up with the logical bus signal.
    ///
    /// Builder-style: returns `self`, so chain it between
    /// [`Self::new_timer_capture`] and [`Self::with_dma_channel`].
    fn set_capture_edge_inverted(self, inverted: bool) -> Self;
    fn with_dma_channel<CH>(self, channel: CH) -> Self::ConvertibleToFuture<CH>
    where
        CH: AnyChannel<Status=ReadyFuture>;
}

pub trait TimerCaptureFutureTrait {
    type DmaChannel;
    type TC;
    type Pinout: PinoutCollapse;
    fn decompose(self) -> (Self::DmaChannel, Self::TC, Option<Self::Pinout>);
    //  fn start_regular_pwm(&mut self, ccx_value: u8);
    async fn start_timer_execute_dma_transfer<Container: iter::Extend<core::time::Duration>, const N: usize>(&mut self, timestamps_capture: Container)
        -> Result<TimerCaptureResultAvailable<Container>, TimerCaptureFailure>;
    fn read_pin_level(&mut self) -> bool;
}

macro_rules! create_timer_capture {
    ($($TYPE:ident: ($TC:ident, $pinout:ident, $clock:ident, $apmask:ident, $apbits:ident, $wrapper:ident, $event:ident)),+) => {
        $(

macro_rules! extract_number_macro {
    ($tc_name:ident) => {
        {
            extract_number(stringify!{$tc_name})
        }
    };
}

use crate::pwm::$pinout;

pub struct $TYPE<I: PinId> {
    /// The frequency of the attached clock, not the period of the pwm.
    /// Used to calculate the period of the pwm.
    clock_freq: Hertz,
    tc: crate::pac::$TC,
    //  `None` in event-input mode: the RX edges arrive over the TC event input
    //  (EVCTRL.TCEI) from the EIC/EVSYS, so the TC owns no capture pin.
    #[allow(dead_code)]
    pinout: Option<$pinout<I>>,
    //  _channel: Option<DmaCh>,
}

impl<I: PinId> TimerCaptureBaseTrait for $TYPE<I> {
    type TC = crate::pac::$TC;
    type Pinout = $pinout<I>;
    type ConvertibleToFuture<D> = paste!{ [<$TYPE Future>]<I, D> }
    where
        D: AnyChannel<Status=ReadyFuture>;

    //  fn get_dma_ptr(&self) -> TimerCaptureWaveformSourcePtr<u32> {
    //      TimerCaptureWaveformSourcePtr(self.tc.count32().cc(TIMER_CHANNEL).as_ptr() as *mut _)
    //  }

    fn enable_mclk_clocks(mclk: &mut Mclk)
    {
        Self::enable_mclk_clocks(mclk);
    }
    fn new_timer_capture(
        clock_freq: Hertz,
        freq: Hertz,
        tc: crate::pac::$TC,
        pinout: $pinout<I>,
        mclk: Option<&mut Mclk>,
        //  timeout: MillisDurationU32,
    ) -> Self {
        Self::new_timer_capture(clock_freq, freq, tc, pinout, mclk)
    }

    fn new_timer_capture_event_input(
        clock_freq: Hertz,
        freq: Hertz,
        tc: crate::pac::$TC,
        mclk: Option<&mut Mclk>,
    ) -> Self {
        Self::new_timer_capture_event_input(clock_freq, freq, tc, mclk)
    }

    fn set_capture_edge_inverted(self, inverted: bool) -> Self {
        self.set_capture_edge_inverted(inverted)
    }

    fn with_dma_channel<CH>(self, channel: CH) -> Self::ConvertibleToFuture<CH>
    where
        CH: AnyChannel<Status=ReadyFuture>,
    {
        self.with_dma_channel(channel)
    }
}

impl<I: PinId> typelevel::Sealed for $TYPE<I> {}

paste!{
    pub struct [<$TYPE InterruptData >] {
    }
    impl TimerCaptureCapable for [< $TYPE InterruptData >] {
        const WAKER_IDX: usize = extract_number_macro!($TC);

        type Interrupt = crate::async_hal::interrupts::[< $TC:upper >];

        fn reg_block(peripherals: &pac::Peripherals) -> &RegBlock {
            &*peripherals.[< $TC:lower >]
        }
    }
}

//  type Interrupt = crate::async_hal::interrupts::[< $TC:upper >];
//  paste!{
//  impl<I:PinId> Handler<crate::async_hal::interrupts::[< $TC:upper >]> for $TYPE<I> {
//      unsafe fn on_interrupt() {
//          use waker::WAKERS;
//          WAKERS[0].wake();
//      }
//  }
//  }

impl<I: PinId> TimerSpecificInterruptAndRegisters for $TYPE<I> {
    const WAKER_ID: usize = extract_number_macro!($TC);

    paste!{
        fn reg_block(peripherals: &pac::Peripherals) -> &RegBlock {
            &*peripherals.[< $TC:lower >]
        }

        type Interrupt = crate::async_hal::interrupts::[< $TC:upper >];
    }
}

paste!{
pub struct [<$TYPE Future>]<I: PinId, DmaCh: AnyChannel<Status=ReadyFuture>>{
    base_pwm: $TYPE<I>,
    _channel: DmaCh
}

// Implement Interrupt traits for basic timer struct:
impl<I: PinId> TimerCounterInterrupt for $TYPE<I> {
    type Interrupt = crate::async_hal::interrupts::[< $TC:upper >];
}

impl<I: PinId, DmaCh: AnyChannel<Status=ReadyFuture>> [<$TYPE Future>]<I, DmaCh> {

    fn start_capture_timer(&mut self) {
        let count = self.base_pwm.tc.count32();

        count.ctrla().modify(|_, w| w.enable().set_bit());
        while count.syncbusy().read().enable().bit_is_set() {}
    }

}
impl<I: PinId, DmaCh: AnyChannel<Status=ReadyFuture>> TimerCaptureFutureTrait for [<$TYPE Future>]<I, DmaCh> {
    type DmaChannel = DmaCh;
    type TC = crate::pac::$TC;
    type Pinout = $pinout<I>;
    fn decompose(self) -> (Self::DmaChannel, Self::TC, Option<Self::Pinout>){
        let $TYPE{clock_freq: _clock_freq, tc, pinout} = self.base_pwm;
        (self._channel, tc, pinout)
    }
    fn read_pin_level(&mut self) -> bool {
        //  let count = self.base_pwm.tc.count32();
        //  let pinout = self.base_pwm.pinout;
        //  let pin = pinout.get_pin();
        //  let level = pin.is_high().unwrap();
        //  level
        self.base_pwm.read_pin_level()
    }
    /// The capture_memorys first element will be the value of the counter at the moment of the first event. The timer starts counting from zero, which mean the first period can be assumed as the value of the first element in the memory.
    async fn start_timer_execute_dma_transfer<ContainerData: core::iter::Extend<core::time::Duration>, const N: usize>
        (&mut self, mut capture_container: ContainerData)
            -> Result<TimerCaptureResultAvailable<ContainerData>, TimerCaptureFailure> {

        let count = self.base_pwm.tc.count32();

        let pwm_dma_address = self.base_pwm.get_dma_ptr();

        let mut capture_memory: [u32; N] = [0; N];
        // TODO:  core::pin::pin_mut!(capture_memory);
        //  TODO: make transfer_future method to return real number of transferred items
        let dma_future = self._channel.as_mut().transfer_future(
            pwm_dma_address,
            &mut capture_memory,
            TriggerSource::$event,
            TriggerAction::Burst,
        );

        //  dma_future.as_mut().poll()

        let dma_wrapped_future = TimerCaptureDmaWrapper::new(dma_future, &self.base_pwm, extract_number_macro!($TC));

        //  TODO: Change the implementation of the DMA channel so that the timer can be started before the DMA gets enabled
        //  First poll the future starts the DMA transfer. It sets enable bit of the DMA.
         let result = dma_wrapped_future.await;
        //  Right after the DMA transfer is started, we can start the timer.

        if let Ok(mut result) = result {
            result.fill_the_capture_memory(&mut capture_memory);
            //  Cap conversion output at the DMA buffer size `N`, which is also
            //  the destination container's capacity, so a completely full
            //  capture can never overflow the container (would HardFault).
            Ok(result.get_public_data(capture_container, N))
        }else {
            //  A DMA transfer error must be surfaced to the caller as a
            //  recoverable failure, not `todo!()` (which panics -> HardFault on
            //  the target with no stack trace). The listen loop then reports it
            //  as a capture error and simply tries the next frame.
            Err(TimerCaptureFailure::DmaFailed(DmacError::TransferError))
        }
        //  type Output = Result<TimerCaptureResultAvailable<ContainerData>, TimerCaptureFailure>;
        //  Rest of the setup shall go into poll method: i.e. enabling interrupts and the counter
        //  of the timer.
        //  self.start_capture_timer();
        //  count.ctrla().modify(|_, w| w.enable().set_bit());
        //  while count.syncbusy().read().enable().bit_is_set() {}

        // wait for the settings to be applied
        //  while count.syncbusy().read().cc0().bit_is_set() {}
        //  result
    }

}

}  //  paste macrto

/*
///
/// To resolve the issue of the end condition for DMA transfer we probably need to use second interrupt signal from the timer
/// One possibility is to use the overflow, but considering 32 bits counter it is not very practical.
/// Another possibility is to use the compare match on timer channel 1. The compare match can be set to the maximum value of the counter.
///
*/
impl<I: PinId> $TYPE<I> {
    pub fn new_timer_capture(
        clock_freq: Hertz,
        _freq: Hertz,
        tc: crate::pac::$TC,
        pinout: $pinout<I>,
        mclk: Option<&mut Mclk>,
        //  timeout: MillisDurationU32,
    ) -> Self {
        Self::new_timer_capture_inner(clock_freq, tc, Some(pinout), mclk, false)
    }

    /// Construct an RX capture that timestamps edges delivered on the TC event
    /// input (`EVCTRL.TCEI`) instead of the TC's own pin. This lets the RX pin
    /// be routed through the EIC (both-edge sense + glitch filter) and the event
    /// system (EVSYS) into this timer, so the capture stream contains **both**
    /// bus edges. No capture pin is owned by the TC in this mode.
    pub fn new_timer_capture_event_input(
        clock_freq: Hertz,
        _freq: Hertz,
        tc: crate::pac::$TC,
        mclk: Option<&mut Mclk>,
    ) -> Self {
        Self::new_timer_capture_inner(clock_freq, tc, None, mclk, true)
    }

    fn new_timer_capture_inner(
        clock_freq: Hertz,
        tc: crate::pac::$TC,
        pinout: Option<$pinout<I>>,
        mclk: Option<&mut Mclk>,
        event_input: bool,
    ) -> Self {
        //  CC1 starts with the long active-capture deadline. The MC0 interrupt
        // changes it to the short inactivity deadline after the first edge.
        let capture_compare_value_timeout =
            clock_freq.to_Hz() / 1000 * CAPTURE_ACTIVE_WINDOW_MS;
        waker::CAPTURE_INACTIVITY_TICKS[extract_number_macro!($TC)].store(
            clock_freq.to_Hz() / 1000 * CAPTURE_INACTIVITY_MS,
            atomic::Ordering::Relaxed,
        );
        let count = tc.count32();
        //  let tc_ccbuf_dma_data_register_address = count.cc(TIMER_CHANNEL).as_ptr() as *const ();
        //  let TimerCaptureWaveformSourcePtr()(pub(in super::super) *mut T);

        //  write(|w| w.ccbuf().bits(duty as u8));
        //  let params = TimerParams::new(freq.convert(), clock_freq);

        match mclk {
            Some(mclk) => {
                mclk.$apmask().modify(|_, w| w.$apbits().set_bit());
                //  TODO: Dirty hack to allow TC4 + TC5 timers work in 32 bits. This is somewhat against
                //  datasheet declarations so be cerful.
                mclk.apbcmask().modify(|_, w| w.tc5_().set_bit());
                //  TODO: Dirty hack to allow TC2 + TC3 timers work in 32 bits. This is somewhat against
                //  datasheet declarations so be cerful.In the future I expect it will be added to
                //  take over the ownership of the other timer as well so that it is blocked to be
                //  used for other purposes.
                mclk.apbbmask().modify(|_, w| w.tc3_().set_bit());
            }
            None => {}
        }

        // First disable the timer, only after that we should set SWRST bit.
        count.ctrla().modify(|_, w| w.enable().clear_bit());
        while count.syncbusy().read().enable().bit_is_set() {}

        count.ctrla().write(|w| w.swrst().set_bit());
        while count.ctrla().read().bits() & 1 != 0 {}

        //  Set the timer to 32-bit mode:
        count.ctrla().modify(|_, w| w.mode().count32());
        count.ctrla().modify(|_, w| {
            w.prescaler().div1()
            //  match params.divider {
            //      1 => w.prescaler().div1(),
            //      2 => w.prescaler().div2(),
            //      4 => w.prescaler().div4(),
            //      8 => w.prescaler().div8(),
            //      16 => w.prescaler().div16(),
            //      64 => w.prescaler().div64(),
            //      256 => w.prescaler().div256(),
            //      1024 => w.prescaler().div1024(),
            //      _ => unreachable!(),
            //  }
        });
        if event_input {
            count.ctrla().modify(|_, w| w.capten0().set_bit());
        } else {
            count
                .ctrla()
                .modify(|_, w| w.capten0().set_bit().copen0().set_bit());
        }

        //  clear all interrupt flags:
        count.intflag().write(|w| w.mc1().set_bit().mc0().set_bit().ovf().set_bit().err().set_bit());

        count.evctrl().write(|w| {
            w.evact().stamp().mceo0().set_bit();
            //  Event-input mode: capture on edges arriving via EVSYS (from the
            //  EIC) rather than the TC's own pin. See `new_timer_capture_event_input`.
            if event_input {
                w.tcei().set_bit();
            }
            w
        });
        // enable interrupt on the timeout side channel:
        // INTENSET is a write-one-to-set register; use a direct write so the
        // capture (MC0) interrupt is enabled reliably as well as the timeout
        // (MC1) interrupt.
        count.intenset().write(|w| w.mc1().set_bit().mc0().set_bit());

        //  Capture edge selection (rising vs falling) is applied separately via
        //  `set_capture_edge_inverted()` right after construction; here we leave
        //  `DRVCTRL.INVEN0` at its reset default (cleared = capture on rising
        //  edge). See `set_capture_edge_inverted` / `CreatePwmPinout::INVERT_SIGNAL`.

        //  count.ccbuf(0).write(|w| unsafe { w.bits(0x00) });
        //  count.ccbuf(1).write(|w| unsafe { w.bits(0x00) });
        count.cc(0).write(|w| unsafe { w.bits(0x00) });
        while count.syncbusy().read().cc0().bit_is_set() {}
        count.cc(1).write(|w| unsafe { w.bits(capture_compare_value_timeout) });
        while count.syncbusy().read().cc1().bit_is_set() {}

        Self {
            clock_freq: clock_freq,
            tc,
            pinout,
        }
    }

    pub fn read_pin_level(&self) -> bool {
        //  In event-input mode there is no capture pin owned by the TC, so there
        //  is no level to sample; report `false` (unused by the both-edge decoder).
        self.pinout.as_ref().map(|p| p.read_level()).unwrap_or(false)
    }

    /// Set (`inverted = true`) or clear (`false`) the `DRVCTRL.INVEN0` bit,
    /// switching the timer capture between falling and rising edges of the RX
    /// input. See [`TimerCaptureBaseTrait::set_capture_edge_inverted`].
    pub fn set_capture_edge_inverted(self, inverted: bool) -> Self {
        let count = self.tc.count32();
        count.drvctrl().modify(|_, w| {
            if inverted {
                w.inven0().set_bit()
            } else {
                w.inven0().clear_bit()
            }
        });
        self
    }

    paste!{
        //  pub fn with_dma_channels<R, T>(self, rx: R, tx: T) -> Spi<C, D, R, T>
        pub fn with_dma_channel<CH>(self, channel: CH ) -> [<$TYPE Future>]<I, CH>
            where
            CH: AnyChannel<Status=ReadyFuture>
        {
            [<$TYPE Future>] {
                base_pwm: self,
                _channel: channel,
            }
        }
    }

    pub fn enable_mclk_clocks(mclk: &mut Mclk) {
        mclk.$apmask().modify(|_, w| w.$apbits().set_bit());
        //  TODO: Dirty hack to allow TC4 + TC5 timers work in 32 bits. This is somewhat against
        //  datasheet declarations so be cerful.
        mclk.apbcmask().modify(|_, w| w.tc5_().set_bit());
        //  TODO: Dirty hack to allow TC2 + TC3 timers work in 32 bits. This is somewhat against
        //  datasheet declarations so be cerful.In the future I expect it will be added to
        //  take over the ownership of the other timer as well so that it is blocked to be
        //  used for other purposes.
        mclk.apbbmask().modify(|_, w| w.tc3_().set_bit());
    }

    pub fn start(&self) {
        //  Rest of the setup shall go into poll method: i.e. enabling interrupts and the counter
        //  of the timer.
        let count = self.tc.count32();
        count.ctrla().modify(|_, w| w.enable().set_bit());
        while count.syncbusy().read().enable().bit_is_set() {}
    }

    pub fn stop(&self){
        let count = self.tc.count32();
        count.ctrla().modify(|_, w| w.enable().clear_bit());
        while count.syncbusy().read().enable().bit_is_set() {}
    }

    pub fn GetDmaPtr(tc: crate::pac::$TC) -> TimerCaptureWaveformSourcePtr<u32> {
        TimerCaptureWaveformSourcePtr(tc.count32().cc(TIMER_CHANNEL).as_ptr() as *mut _)
    }
    pub fn get_dma_ptr(&self) -> TimerCaptureWaveformSourcePtr<u32> {
        TimerCaptureWaveformSourcePtr(self.tc.count32().cc(TIMER_CHANNEL).as_ptr() as *mut _)
    }
}

impl<I: PinId> TimerCounterStart for $TYPE<I> {
    paste!{
        type Interrupt = crate::async_hal::interrupts::[< $TC:upper >];
    }
    fn start(&self)
    {
        self.start();
    }
    fn stop(&self)
    {
        self.stop();
    }
    fn is_mc1_interrupt_active(&self) -> bool {
        let count = self.tc.count32();
        count.intflag().read().mc1().bit_is_set()
        //  if intflag.read().mc1().bit_is_set() {
    }
    fn enable_interrupt(&self)
    {
        unsafe {
            Self::Interrupt::enable();
        }
    }
    fn disable_interrupt(&self)
    {
        Self::Interrupt::disable();
    }
    fn counter_value(&self) -> u32
    {
        //  trigger counter value read:
        let _ = self.tc.count32().ctrlbset().write(|w| w.cmd().readsync());
        // return counter value:
        self.tc.count32().count().read().bits()
    }
}

impl<I: PinId> $crate::ehal::pwm::ErrorType for $TYPE<I> {
    type Error = ::core::convert::Infallible;
}

)+}}

#[hal_cfg("tc0")]
create_timer_capture! { TimerCapture0: (Tc0, TC0Pinout, Tc0Tc1Clock, apbamask, tc0_, TimerCapture0Wrapper, Tc0Mc0) }
#[hal_cfg("tc1")]
create_timer_capture! { TimerCapture1: (Tc1, TC1Pinout, Tc0Tc1Clock, apbamask, tc1_, TimerCapture1Wrapper, Tc1Mc0) }
#[hal_cfg("tc2")]
create_timer_capture! { TimerCapture2: (Tc2, TC2Pinout, Tc2Tc3Clock, apbbmask, tc2_, TimerCapture2Wrapper, Tc2Mc0) }
#[hal_cfg("tc3")]
create_timer_capture! { TimerCapture3: (Tc3, TC3Pinout, Tc2Tc3Clock, apbbmask, tc3_, TimerCapture3Wrapper, Tc3Mc0) }
#[hal_cfg("tc4")]
create_timer_capture! { TimerCapture4: (Tc4, TC4Pinout, Tc4Tc5Clock, apbcmask, tc4_, TimerCapture4Wrapper, Tc4Mc0) }
#[hal_cfg("tc5")]
create_timer_capture! { TimerCapture5: (Tc5, TC5Pinout, Tc4Tc5Clock, apbcmask, tc5_, TimerCapture5Wrapper, Tc5Mc0) }
#[hal_cfg("tc6")]
create_timer_capture! { TimerCapture6: (Tc6, TC6Pinout, Tc6Tc7Clock, apbdmask, tc6_, TimerCapture6Wrapper, Tc6Mc0) }
#[hal_cfg("tc7")]
create_timer_capture! { TimerCapture7: (Tc7, TC7Pinout, Tc6Tc7Clock, apbdmask, tc7_, TimerCapture7Wrapper, Tc7Mc0) }

trait TimerCounterStart {
    type Interrupt: Interrupt;
    fn start(&self);
    fn stop(&self);
    fn is_mc1_interrupt_active(&self) -> bool;
    fn enable_interrupt(&self);
    fn disable_interrupt(&self);
    fn counter_value(&self) -> u32;
}

const MAX_TIMER_COUNT: usize = 8;

#[cfg(feature = "async")]
mod waker {
    use core::sync::atomic::{AtomicBool, AtomicU32};

    use embassy_sync::waitqueue::AtomicWaker;

    use super::MAX_TIMER_COUNT;

    #[allow(clippy::declare_interior_mutable_const)]
    const NEW_WAKER: AtomicWaker = AtomicWaker::new();
    pub(super) static WAKERS: [AtomicWaker; MAX_TIMER_COUNT] =
        [NEW_WAKER; MAX_TIMER_COUNT];
    const NEW_INTERRUP_FIRED: AtomicBool = AtomicBool::new(false);
    pub(super) static MC1_INTERRUPT_FIRED: [AtomicBool; MAX_TIMER_COUNT] =
        [NEW_INTERRUP_FIRED; MAX_TIMER_COUNT];
    const NEW_CAPTURE_INACTIVITY_TICKS: AtomicU32 = AtomicU32::new(0);
    pub(super) static CAPTURE_INACTIVITY_TICKS: [AtomicU32; MAX_TIMER_COUNT] =
        [NEW_CAPTURE_INACTIVITY_TICKS; MAX_TIMER_COUNT];
}

