#![allow(non_snake_case)]

use atsamd_hal_macros::hal_cfg;

use crate::clock;
use crate::gpio::*;
use crate::gpio::{AlternateE, AnyPin, Pin};
use crate::dmac::{Beat, Buffer};
use crate::pac::Mclk;
use crate::time::Hertz;
use crate::timer_params::TimerParams;

#[derive(Clone)]
pub struct PwmWaveformGeneratorPtr<T: Beat>(pub(in super::super) *mut T);

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

macro_rules! pwm_wg {
    ($($TYPE:ident: ($TC:ident, $pinout:ident, $clock:ident, $apmask:ident, $apbits:ident, $wrapper:ident)),+) => {
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

impl<I: PinId> $TYPE<I> {
    pub fn new(
        clock: &clock::$clock,
        freq: Hertz,
        tc: crate::pac::$TC,
        pinout: $pinout<I>,
        mclk: &mut Mclk,
    ) -> Self {
        let count = tc.count8();
        let tc_ccbuf_dma_data_register_address = tc.count8().ccbuf(1).as_ptr() as *const ();
        //  let PwmWaveformGeneratorPtr()(pub(in super::super) *mut T);
        
        //  write(|w| w.ccbuf().bits(duty as u8)); 
        let params = TimerParams::new(freq.convert(), clock.freq());
        mclk.$apmask().modify(|_, w| w.$apbits().set_bit());
        count.ctrla().write(|w| w.swrst().set_bit());
        while count.ctrla().read().bits() & 1 != 0 {}
        count.ctrla().modify(|_, w| w.enable().clear_bit());
        while count.syncbusy().read().enable().bit_is_set() {}
        count.ctrla().modify(|_, w| w.copen1().set_bit());
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
        count.wave().write(|w| w.wavegen().mpwm());
        count.cc(0).write(|w| unsafe { w.cc().bits(params.cycles as u8) });
        while count.syncbusy().read().cc0().bit_is_set() {}
        count.cc(1).write(|w| unsafe { w.cc().bits(0) });
        while count.syncbusy().read().cc1().bit_is_set() {}

        //  Rest of the setup shall go into poll method: i.e. enabling interrupts and the counter
        //  of the timer. 
        count.ctrla().modify(|_, w| w.enable().set_bit());
        while count.syncbusy().read().enable().bit_is_set() {}

        Self {
            clock_freq: clock.freq(),
            tc,
            pinout,
        }
    }

    pub fn get_dma_ptr(&self) -> PwmWaveformGeneratorPtr<u8> {
        PwmWaveformGeneratorPtr(self.tc.count8().ccbuf(1).as_ptr() as *mut _)
    }

    pub fn get_period(&self) -> Hertz {
        let count = self.tc.count8();
        let divisor = count.ctrla().read().prescaler().bits();
        let top = count.cc(0).read().cc().bits();
        self.clock_freq / divisor as u32 / (top + 1) as u32
    }

    pub fn set_period(&mut self, period: Hertz)
    {
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

impl<I: PinId> $crate::ehal::pwm::ErrorType for$TYPE<I> {
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
        let _ignore_infaillible = self.set_duty_cycle(duty);
    }
}

)+}}

#[hal_cfg("tc0")]
pwm_wg! { PwmWg0: (Tc0, TC0Pinout, Tc0Tc1Clock, apbamask, tc0_, PwmWg0Wrapper) }
#[hal_cfg("tc1")]
pwm_wg! { PwmWg1: (Tc1, TC1Pinout, Tc0Tc1Clock, apbamask, tc1_, PwmWg1Wrapper) }
#[hal_cfg("tc2")]
pwm_wg! { PwmWg2: (Tc2, TC2Pinout, Tc2Tc3Clock, apbbmask, tc2_, PwmWg2Wrapper) }
#[hal_cfg("tc3")]
pwm_wg! { PwmWg3: (Tc3, TC3Pinout, Tc2Tc3Clock, apbbmask, tc3_, PwmWg3Wrapper) }
#[hal_cfg("tc4")]
pwm_wg! { PwmWg4: (Tc4, TC4Pinout, Tc4Tc5Clock, apbcmask, tc4_, PwmWg4Wrapper) }
#[hal_cfg("tc5")]
pwm_wg! { PwmWg5: (Tc5, TC5Pinout, Tc4Tc5Clock, apbcmask, tc5_, PwmWg5Wrapper) }
#[hal_cfg("tc6")]
pwm_wg! { PwmWg6: (Tc6, TC6Pinout, Tc6Tc7Clock, apbdmask, tc6_, PwmWg6Wrapper) }
#[hal_cfg("tc7")]
pwm_wg! { PwmWg7: (Tc7, TC7Pinout, Tc6Tc7Clock, apbdmask, tc7_, PwmWg7Wrapper) }
