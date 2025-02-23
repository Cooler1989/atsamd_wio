#![allow(non_snake_case)]

use atsamd_hal_macros::hal_cfg;

use crate::clock;
#[cfg(feature = "async")]
use crate::dmac::ReadyFuture;
use crate::dmac::{AnyChannel, Beat, Buffer, Error as DmacError, TriggerAction, TriggerSource};
use crate::gpio::*;
use crate::gpio::{AlternateE, AnyPin, Pin};
use crate::pac::Mclk;
use crate::time::Hertz;
use crate::timer_params::TimerParams;
use crate::typelevel::{Is, NoneT, Sealed};

use paste::paste;

const TIMER_CHANNEL: usize = 0;

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

//  impl<P, M, Z, D, R, T> Spi<Config<P, M, Z>, D, R, T>
//  where
//      P: ValidPads,
//      M: OpMode,
//      Z: Size,
//      Config<P, M, Z>: ValidConfig,
//      D: Capability,
//      Z::Word: Beat,
//  {
//      #[inline]
//      pub(in super::super) fn sercom_ptr(&self) -> SercomPtr<Z::Word> {
//          SercomPtr(self.config.regs.spi().data().as_ptr() as *mut _)
//      }
//  }

// Timer/Counter (TCx)
//

macro_rules! create_timer_capture {
    ($($TYPE:ident: ($TC:ident, $pinout:ident, $clock:ident, $apmask:ident, $apbits:ident, $wrapper:ident, $event:ident)),+) => {
        $(

use crate::pwm::$pinout;

pub struct $TYPE<I: PinId> {
    /// The frequency of the attached clock, not the period of the pwm.
    /// Used to calculate the period of the pwm.
    clock_freq: Hertz,
    tc: crate::pac::$TC,
    #[allow(dead_code)]
    pinout: $pinout<I>,
    //  _channel: Option<DmaCh>,
}

paste!{
pub struct [<$TYPE Future>]<I: PinId, DmaCh: AnyChannel<Status=ReadyFuture>>{
    base_pwm: $TYPE<I>,
    _channel: DmaCh
}

impl<I: PinId, DmaCh: AnyChannel<Status=ReadyFuture>> [<$TYPE Future>]<I, DmaCh> {

    fn start_capture(&mut self) {
        let count = self.base_pwm.tc.count32();
        count.cc(0).write(|w| unsafe { w.bits(0x00) });
        while count.syncbusy().read().cc0().bit_is_set() {}
        count.cc(1).write(|w| unsafe { w.bits(0x00) });
        while count.syncbusy().read().cc1().bit_is_set() {}

        count.ccbuf(0).write(|w| unsafe { w.bits(0x00) });
        count.ccbuf(1).write(|w| unsafe { w.bits(0x00) });

        count.ctrla().modify(|_, w| w.enable().set_bit());
        while count.syncbusy().read().enable().bit_is_set() {}
    }

    pub async fn start_timer_prepare_dma_transfer(&mut self) -> Result<(),DmacError> {

        self.start_capture();
        let count = self.base_pwm.tc.count32();

        count.cc(0).write(|w| unsafe { w.bits(0x00) });
        while count.syncbusy().read().cc0().bit_is_set() {}
        count.cc(1).write(|w| unsafe { w.bits(0) });
        while count.syncbusy().read().cc1().bit_is_set() {}
        count.ccbuf(0).write(|w| unsafe { w.bits(0x00) });
        count.ccbuf(1).write(|w| unsafe { w.bits(0) });

        let mut capture_memory: [u32; 128] = [0; 128];
        let pwm_dma_address = self.base_pwm.get_dma_ptr();
        let dma_future = self._channel.as_mut().transfer_future(
            pwm_dma_address,
            capture_memory.as_mut_slice(),
            TriggerSource::$event,
            TriggerAction::Burst,
        );
        //  Rest of the setup shall go into poll method: i.e. enabling interrupts and the counter
        //  of the timer.
        count.ctrla().modify(|_, w| w.enable().set_bit());
        while count.syncbusy().read().enable().bit_is_set() {}

        //  First poll the future starts the DMA transfer
        let value_to_return = dma_future.await;

        count.cc(1).write(|w| unsafe { w.bits(0) });
        count.ccbuf(1).write(|w| unsafe { w.bits(0) });
        count.cc(0).write(|w| unsafe { w.bits(0) });
        count.ccbuf(0).write(|w| unsafe { w.bits(0) });

        count.evctrl().write(|w| w.evact().stamp().mceo0().set_bit());

        // wait for the settings to be applied
        while count.syncbusy().read().cc0().bit_is_set() {}

        value_to_return
    }

    pub fn decompose(self) -> (DmaCh, crate::pac::$TC, $pinout<I>)
    {
        let $TYPE{clock_freq, tc, pinout} = self.base_pwm;
        (self._channel, tc, pinout)
    }
}

/*
///
/// PRESCALER = 6, in reference project written in C++
/// cc = 233
/// per = 233
///
/// As the timer is set to produce idle level signal, it can be started before
/// we start the DMA transfer. It will naturally pick and start from the
/// beginning of next cycle of the timer. Timer is configured to prodduce
/// constant period signal by setting PER register to a value that corresponds
/// to requested frequency of the manchester signal. Base of the working
/// principle is that the timer CCx register will be loaded with either 0x00
/// of 0xFF to produce either full cycle high or low signal.
*/
impl<I: PinId> $TYPE<I> {
    pub fn new_timer_capture(
        clock_freq: Hertz,
        freq: Hertz,
        tc: crate::pac::$TC,
        pinout: $pinout<I>,
        mclk: &mut Mclk,
    ) -> Self {
        let count = tc.count32();
        let tc_ccbuf_dma_data_register_address = count.cc(TIMER_CHANNEL).as_ptr() as *const ();
        //  let TimerCaptureWaveformSourcePtr()(pub(in super::super) *mut T);

        //  write(|w| w.ccbuf().bits(duty as u8));
        let params = TimerParams::new(freq.convert(), clock_freq);
        mclk.$apmask().modify(|_, w| w.$apbits().set_bit());
        count.ctrla().write(|w| w.swrst().set_bit());
        while count.ctrla().read().bits() & 1 != 0 {}
        count.ctrla().modify(|_, w| w.enable().clear_bit());
        while count.syncbusy().read().enable().bit_is_set() {}

        //  TODO:  resolve copen1 question:
        //  count.ctrla().modify(|_, w| w.copen1().set_bit());
        count.ctrla().modify(|_, w| w.mode().count32());
        count.ctrla().modify(|_, w| {
            w.prescaler().div256()
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
        count.ctrla().write(|w| w.capten0().set_bit());
        count.ctrla().write(|w| w.copen0().set_bit());

        count.cc(0).write(|w| unsafe { w.bits(0x00) });
        while count.syncbusy().read().cc0().bit_is_set() {}
        count.cc(1).write(|w| unsafe { w.bits(0x00) });
        while count.syncbusy().read().cc1().bit_is_set() {}

        Self {
            clock_freq: clock_freq,
            tc,
            pinout,
        }
    }

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

    pub fn start(&mut self) {
        //  Rest of the setup shall go into poll method: i.e. enabling interrupts and the counter
        //  of the timer.
        let count = self.tc.count32();
        count.ctrla().modify(|_, w| w.enable().set_bit());
        while count.syncbusy().read().enable().bit_is_set() {}
    }

    pub fn GetDmaPtr(tc: crate::pac::$TC) -> TimerCaptureWaveformSourcePtr<u32> {
        TimerCaptureWaveformSourcePtr(tc.count32().cc(TIMER_CHANNEL).as_ptr() as *mut _)
    }
    pub fn get_dma_ptr(&self) -> TimerCaptureWaveformSourcePtr<u32> {
        TimerCaptureWaveformSourcePtr(self.tc.count32().cc(TIMER_CHANNEL).as_ptr() as *mut _)
    }

    pub fn get_period(&self) -> Hertz {
        let count = self.tc.count32();
        let divisor = count.ctrla().read().prescaler().bits();
        let top = count.cc(0).read().cc().bits();
        self.clock_freq / divisor as u32 / (top + 1) as u32
    }

    pub fn set_period(&mut self, period: Hertz)
    {
        let period = period.into();
        let params = TimerParams::new(period, self.clock_freq);
        let count = self.tc.count32();
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
        count.cc(0).write(|w| unsafe { w.cc().bits(params.cycles as u32) });
        while count.syncbusy().read().cc0().bit_is_set() {}
    }
}
}

impl<I: PinId> $crate::ehal::pwm::ErrorType for$TYPE<I> {
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
