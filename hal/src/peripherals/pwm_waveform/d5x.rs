//! # DMA-driven PWM waveform generator for SAMD5x/E5x Timer/Counter peripherals
//!
//! This module provides a PWM waveform generator that uses the TC peripheral
//! in 8-bit NPWM mode to produce arbitrary binary waveforms (e.g. Manchester-
//! encoded OpenTherm signals) via DMA transfers.
//!
//! ## How it works
//!
//! A TC peripheral is configured with a fixed period ([`TIMER_PERIOD`] = 233
//! at prescaler /256, yielding ~1 kHz from a 60 MHz MCLK). Each timer
//! overflow cycle, the DMA writes a new compare value into the CC1 buffer
//! register:
//!
//! - `0xFF` → output stays **high** for the entire cycle
//! - `0x00` → output stays **low** for the entire cycle
//!
//! By filling a DMA buffer with a sequence of these values, the driver
//! produces a bitstream at ~1 kHz per symbol — matching the OpenTherm
//! Manchester encoding rate.
//!
//! ## Architecture
//!
//! The module is organised around two traits and their macro-generated
//! implementations for each TC instance (TC0–TC7):
//!
//! | Type | Purpose |
//! |------|---------|
//! | [`PwmWaveformGenerator`] | Builder: creates + configures the TC, attaches a DMA channel |
//! | [`DmaPwmWaveform`] | Runtime: sends waveforms, sets idle level, decomposes |
//! | `PwmWgN<I>` | Concrete base struct (e.g. `PwmWg4<PB09>`) |
//! | `PwmWgNFuture<I, DmaCh>` | Concrete DMA-attached struct returned by `with_dma_channel()` |
//!
//! ## Feature gates
//!
//! - **No features** — the base `PwmWgN` struct is available with sync
//!   `embedded-hal` PWM traits (`SetDutyCycle`, `PwmPin`).
//! - **`dma`** — enables [`PwmWaveformGeneratorPtr`] and `get_dma_ptr*` methods.
//! - **`dma` + `async`** — enables the full async DMA pipeline:
//!   [`PwmWaveformGenerator`], [`DmaPwmWaveform`], `PwmWgNFuture`.
//!
//! ## Usage
//!
//! ```rust,ignore
//! use atsamd_hal::pwm_wg::{PwmWaveformGenerator, DmaPwmWaveform, PwmWg4};
//!
//! // 1. Create the base generator (configures TC in NPWM mode)
//! let pwm_base = PwmWg4::new(clock_freq, freq, tc4, pinout, Some(&mut mclk));
//!
//! // 2. Attach a DMA channel
//! let mut pwm = pwm_base.with_dma_channel(dma_ch);
//!
//! // 3. Set idle output level
//! pwm.set_idle_level(0xFF);
//!
//! // 4. Send a waveform pattern (async)
//! let pattern = [true, false, true, true, false].into_iter();
//! pwm.send_waveform::<64>(0xFF, pattern, /* invert */ false).await?;
//!
//! // 5. Decompose to reclaim peripherals
//! let (dma_ch, tc4, pinout) = pwm.decompose();
//! ```

use atsamd_hal_macros::hal_cfg;

#[cfg(feature = "dma")]
use crate::dmac::{Beat, Buffer};
#[cfg(all(feature = "dma", feature = "async"))]
use crate::dmac::{
    AnyChannel, Error as DmacError, ReadyFuture, TriggerAction, TriggerSource,
};
use crate::gpio::PinId;
#[cfg(all(feature = "dma", feature = "async"))]
use crate::pac::Mclk;
use crate::time::Hertz;
use crate::timer_params::TimerParams;

use paste::paste;

/// Timer period for ~1kHz manchester signal: mclk / 256 / 233 ≈ 1000 Hz
const TIMER_PERIOD: u8 = 233;

/// DMA pattern value for full-cycle high signal.
const SIGNAL_HIGH: u8 = 0xFF;

/// DMA pattern value for full-cycle low signal.
const SIGNAL_LOW: u8 = 0x00;

/// Offset into the DMA pattern buffer where bit encoding begins.
/// The first entries are kept at idle level to allow the timer to settle.
const DMA_PATTERN_OFFSET: usize = 2;

#[cfg(feature = "dma")]
#[derive(Clone)]
pub struct PwmWaveformGeneratorPtr<T: Beat>(pub(in super::super) *mut T);

#[cfg(feature = "dma")]
unsafe impl<T: Beat> Buffer for PwmWaveformGeneratorPtr<T> {
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

pub use crate::pwm::PinoutCollapse;

/// Async DMA-driven PWM waveform interface.
///
/// Provides methods to send a waveform pattern via DMA, set the idle output
/// level, and decompose back into the underlying peripheral and DMA channel.
#[cfg(all(feature = "dma", feature = "async"))]
pub trait DmaPwmWaveform {
    type DmaChannel: AnyChannel<Status = ReadyFuture>;
    type TC;
    type Pinout: PinoutCollapse;

    fn decompose(self) -> (Self::DmaChannel, Self::TC, Self::Pinout);
    fn set_idle_level(&mut self, ccx_value: u8);
    async fn send_waveform<const N: usize>(
        &mut self,
        ccx_value: u8,
        generation_pattern_iter: impl Iterator<Item = bool>,
        invert: bool,
    ) -> Result<(), DmacError>;
}

/// Timer/Counter based PWM waveform generator.
///
/// Configures a TC peripheral in 8-bit NPWM mode and provides a builder
/// method to attach a DMA channel for async waveform output.
#[cfg(all(feature = "dma", feature = "async"))]
pub trait PwmWaveformGenerator {
    type TC;
    type Pinout: PinoutCollapse;
    type WithDma<D>: DmaPwmWaveform<TC = Self::TC, Pinout = Self::Pinout, DmaChannel = D>
    where
        D: AnyChannel<Status = ReadyFuture>;

    /// Create a new PWM waveform generator.
    fn new(
        clock_freq: Hertz,
        freq: Hertz,
        tc: Self::TC,
        pinout: Self::Pinout,
        mclk: Option<&mut Mclk>,
    ) -> Self;

    fn with_dma_channel<CH>(self, channel: CH) -> Self::WithDma<CH>
    where
        CH: AnyChannel<Status = ReadyFuture>;
}
macro_rules! pwm_wg {
    ($($TYPE:ident: ($TC:ident, $pinout:ident, $apmask:ident, $apbits:ident, $event:ident)),+) => {
        $(

use crate::pwm::$pinout;

pub struct $TYPE<I: PinId> {
    /// The frequency of the attached clock, not the period of the pwm.
    /// Used to calculate the period of the pwm.
    clock_freq: Hertz,
    tc: crate::pac::$TC,
    #[allow(dead_code)]
    pinout: $pinout<I>,
}

paste! {
#[cfg(all(feature = "dma", feature = "async"))]
pub struct [<$TYPE Future>]<I: PinId, DmaCh: AnyChannel<Status = ReadyFuture>> {
    base_pwm: $TYPE<I>,
    channel: DmaCh,
    init_level: u8,
}

#[cfg(all(feature = "dma", feature = "async"))]
impl<I: PinId, DmaCh: AnyChannel<Status = ReadyFuture>> DmaPwmWaveform for [<$TYPE Future>]<I, DmaCh> {
    type DmaChannel = DmaCh;
    type TC = crate::pac::$TC;
    type Pinout = $pinout<I>;

    async fn send_waveform<const N: usize>(
        &mut self,
        ccx_value: u8,
        generation_pattern_iter: impl Iterator<Item = bool>,
        invert: bool,
    ) -> Result<(), DmacError> {
        let mut generation_pattern_dma: [u8; N] = [self.init_level; N];
        for (idx, value) in generation_pattern_iter.enumerate() {
            // TODO: resolve the initial driver state — before the first TX it
            // is low instead of high.
            let idx = idx + DMA_PATTERN_OFFSET;
            if idx >= N {
                break;
            }
            let value = value != invert;
            // TODO: Implement configurable idle bus state level
            let level = if value { SIGNAL_HIGH } else { SIGNAL_LOW };
            generation_pattern_dma[idx] = level;
        }

        let count = self.base_pwm.tc.count8();

        count.cc(0).write(|w| unsafe { w.bits(SIGNAL_LOW) });
        while count.syncbusy().read().cc0().bit_is_set() {}
        count.cc(1).write(|w| unsafe { w.bits(ccx_value) });
        while count.syncbusy().read().cc1().bit_is_set() {}
        count.ccbuf(0).write(|w| unsafe { w.bits(SIGNAL_LOW) });
        count.ccbuf(1).write(|w| unsafe { w.bits(ccx_value) });

        let pwm_dma_address = self.base_pwm.get_dma_ptr();
        let dma_future = self.channel.as_mut().transfer_future(
            &mut generation_pattern_dma,
            pwm_dma_address,
            TriggerSource::$event,
            TriggerAction::Burst,
        );
        count.ctrla().modify(|_, w| w.enable().set_bit());
        while count.syncbusy().read().enable().bit_is_set() {}

        let result = dma_future.await;

        count.cc(1).write(|w| unsafe { w.bits(ccx_value) });
        count.ccbuf(1).write(|w| unsafe { w.bits(ccx_value) });

        result
    }

    fn set_idle_level(&mut self, ccx_value: u8) {
        self.init_level = ccx_value;
        let count = self.base_pwm.tc.count8();

        count.cc(0).write(|w| unsafe { w.bits(SIGNAL_LOW) });
        while count.syncbusy().read().cc0().bit_is_set() {}
        count.cc(1).write(|w| unsafe { w.bits(ccx_value) });
        while count.syncbusy().read().cc1().bit_is_set() {}

        count.ccbuf(0).write(|w| unsafe { w.bits(SIGNAL_LOW) });
        count.ccbuf(1).write(|w| unsafe { w.bits(ccx_value) });

        count.ctrla().modify(|_, w| w.enable().set_bit());
        while count.syncbusy().read().enable().bit_is_set() {}
    }

    fn decompose(self) -> (Self::DmaChannel, Self::TC, Self::Pinout) {
        let $TYPE { tc, pinout, .. } = self.base_pwm;
        (self.channel, tc, pinout)
    }
}

/// Configures a TC peripheral in 8-bit NPWM mode for manchester signal
/// generation.
///
/// The timer produces a constant-period signal via the PER register. The
/// DMA overwrites the CCx register with either `SIGNAL_HIGH` or
/// `SIGNAL_LOW` each cycle, yielding full-cycle high or low output.
#[cfg(all(feature = "dma", feature = "async"))]
impl<I: PinId> PwmWaveformGenerator for $TYPE<I> {
    type TC = crate::pac::$TC;
    type Pinout = $pinout<I>;
    type WithDma<D> = [<$TYPE Future>]<I, D> where
        D: AnyChannel<Status = ReadyFuture>;

    fn new(
        clock_freq: Hertz,
        _freq: Hertz,
        tc: Self::TC,
        pinout: Self::Pinout,
        mclk: Option<&mut Mclk>,
    ) -> Self {
        let count = tc.count8();

        if let Some(mclk) = mclk {
            mclk.$apmask().modify(|_, w| w.$apbits().set_bit());
        }

        count.ctrla().write(|w| w.swrst().set_bit());
        while count.ctrla().read().bits() & 1 != 0 {}
        count.ctrla().modify(|_, w| w.enable().clear_bit());
        while count.syncbusy().read().enable().bit_is_set() {}
        count.ctrla().modify(|_, w| w.mode().count8());
        count.ctrla().modify(|_, w| w.prescaler().div256());

        count.count().write(|w| unsafe { w.bits(TIMER_PERIOD) });
        count.per().write(|w| unsafe { w.bits(TIMER_PERIOD) });
        count.perbuf().write(|w| unsafe { w.bits(TIMER_PERIOD) });

        count.wave().write(|w| w.wavegen().npwm());

        count.cc(0).write(|w| unsafe { w.bits(SIGNAL_LOW) });
        while count.syncbusy().read().cc0().bit_is_set() {}
        count.cc(1).write(|w| unsafe { w.bits(SIGNAL_HIGH) });
        while count.syncbusy().read().cc1().bit_is_set() {}

        Self {
            clock_freq,
            tc,
            pinout,
        }
    }

    fn with_dma_channel<CH>(self, channel: CH) -> Self::WithDma<CH>
    where
        CH: AnyChannel<Status = ReadyFuture>,
    {
        [<$TYPE Future>] {
            base_pwm: self,
            channel,
            init_level: SIGNAL_LOW,
        }
    }
}

impl<I: PinId> $TYPE<I> {
    pub fn start(&mut self) {
        let count = self.tc.count8();
        count.ctrla().modify(|_, w| w.enable().set_bit());
        while count.syncbusy().read().enable().bit_is_set() {}
    }

    #[cfg(feature = "dma")]
    pub fn get_dma_ptr_from_tc(tc: crate::pac::$TC) -> PwmWaveformGeneratorPtr<u8> {
        PwmWaveformGeneratorPtr(tc.count8().ccbuf(1).as_ptr() as *mut _)
    }

    #[cfg(feature = "dma")]
    pub fn get_dma_ptr(&self) -> PwmWaveformGeneratorPtr<u8> {
        PwmWaveformGeneratorPtr(self.tc.count8().ccbuf(1).as_ptr() as *mut _)
    }

    pub fn get_period(&self) -> Hertz {
        let count = self.tc.count8();
        let divisor = count.ctrla().read().prescaler().bits();
        let top = count.cc(0).read().cc().bits();
        self.clock_freq / divisor as u32 / (top + 1) as u32
    }

    pub fn set_period(&mut self, period: Hertz) {
        let period = period.into();
        let params = TimerParams::new(period, self.clock_freq);
        let count = self.tc.count8();
        count.ctrla().modify(|_, w| w.enable().clear_bit());
        while count.syncbusy().read().enable().bit_is_set() {}
        count.ctrla().modify(|_, w| {
            match params.divider {
                1 => w.prescaler().div1(),
                2 => w.prescaler().div2(),
                4 => w.prescaler().div4(),
                8 => w.prescaler().div8(),
                16 => w.prescaler().div16(),
                64 => w.prescaler().div64(),
                256 => w.prescaler().div256(),
                1024 => w.prescaler().div1024(),
                _ => unreachable!(),
            }
        });
        count.ctrla().modify(|_, w| w.enable().set_bit());
        while count.syncbusy().read().enable().bit_is_set() {}
        count.cc(0).write(|w| unsafe { w.cc().bits(params.cycles as u8) });
        while count.syncbusy().read().cc0().bit_is_set() {}
    }
}
} // paste!

impl<I: PinId> $crate::ehal::pwm::ErrorType for $TYPE<I> {
    type Error = ::core::convert::Infallible;
}

impl<I: PinId> $crate::ehal::pwm::SetDutyCycle for $TYPE<I> {
    fn max_duty_cycle(&self) -> u16 {
        let count = self.tc.count8();
        let top = count.cc(0).read().cc().bits();
        top as u16
    }

    fn set_duty_cycle(&mut self, duty: u16) -> Result<(), Self::Error> {
        let count = self.tc.count8();
        unsafe { count.ccbuf(1).write(|w| w.ccbuf().bits(duty as u8)); }
        Ok(())
    }
}

impl<I: PinId> $crate::ehal_02::PwmPin for $TYPE<I> {
    type Duty = u16;

    fn disable(&mut self) {
        let count = self.tc.count8();
        count.ctrla().modify(|_, w| w.enable().clear_bit());
        while count.syncbusy().read().enable().bit_is_set() {}
    }

    fn enable(&mut self) {
        let count = self.tc.count8();
        count.ctrla().modify(|_, w| w.enable().set_bit());
        while count.syncbusy().read().enable().bit_is_set() {}
    }

    fn get_duty(&self) -> Self::Duty {
        let count = self.tc.count8();
        let duty: u8 = count.ccbuf(1).read().ccbuf().bits();
        duty as Self::Duty
    }

    fn get_max_duty(&self) -> Self::Duty {
        use $crate::ehal::pwm::SetDutyCycle;
        self.max_duty_cycle()
    }

    fn set_duty(&mut self, duty: Self::Duty) {
        use $crate::ehal::pwm::SetDutyCycle;
        let _ = self.set_duty_cycle(duty);
    }
}

)+}}

#[hal_cfg("tc0")]
pwm_wg! { PwmWg0: (Tc0, TC0Pinout, apbamask, tc0_, Tc0Ovf) }
#[hal_cfg("tc1")]
pwm_wg! { PwmWg1: (Tc1, TC1Pinout, apbamask, tc1_, Tc1Ovf) }
#[hal_cfg("tc2")]
pwm_wg! { PwmWg2: (Tc2, TC2Pinout, apbbmask, tc2_, Tc2Ovf) }
#[hal_cfg("tc3")]
pwm_wg! { PwmWg3: (Tc3, TC3Pinout, apbbmask, tc3_, Tc3Ovf) }
#[hal_cfg("tc4")]
pwm_wg! { PwmWg4: (Tc4, TC4Pinout, apbcmask, tc4_, Tc4Ovf) }
#[hal_cfg("tc5")]
pwm_wg! { PwmWg5: (Tc5, TC5Pinout, apbcmask, tc5_, Tc5Ovf) }
#[hal_cfg("tc6")]
pwm_wg! { PwmWg6: (Tc6, TC6Pinout, apbdmask, tc6_, Tc6Ovf) }
#[hal_cfg("tc7")]
pwm_wg! { PwmWg7: (Tc7, TC7Pinout, apbdmask, tc7_, Tc7Ovf) }
