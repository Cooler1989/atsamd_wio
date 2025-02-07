//  use crate::buttons::eic::Channel;
use atsamd_hal::clock::GenericClockController;
use atsamd_hal::eic::Channel;
use atsamd_hal::eic::ExtInt;
use atsamd_hal::eic::{Ch10, Ch11, Ch12, Ch3, Ch4, Ch5, Ch7, Eic, Sense};
//  use atsamd_hal::eic::pin::{
//      ExtInt10, ExtInt11, ExtInt12, ExtInt3, ExtInt4, ExtInt5, ExtInt7, ExternalInterrupt, Sense,
//  };
use atsamd_hal::pac::{interrupt, Eic as PacEic, Mclk};
use crate::hal::gpio::{PC27, PC28, PD08, PD09, PD10, PD12, PD20};
use crate::hal::gpio::Reset;

use cortex_m::peripheral::NVIC;

use super::pins::aliases::*;
use super::pins::*;

use atsamd_hal::eic;

/// pushbuttons and joystick
pub struct ButtonPins {
    /// button1 pin
    pub button1: Button1Reset,
    /// button2 pin
    pub button2: Button2Reset,
    /// button3 pin
    pub button3: Button3Reset,

    /// Joystick X
    pub switch_x: SwitchXReset,
    /// Joystick Y
    pub switch_y: SwitchYReset,
    /// Joystick Z
    pub switch_z: SwitchZReset,
    /// Joystick U
    pub switch_u: SwitchUReset,
    /// Joystick B
    pub switch_b: SwitchBReset,
}

impl ButtonPins {
    pub fn init(
        self,
        eic: PacEic,
        clocks: &mut GenericClockController,
        mclk: &mut Mclk,
    ) -> ButtonController {
        let gclk1 = clocks.gclk1();
        let eic_clock = clocks.eic(&gclk1).unwrap();
        //  let mut eic = eic::init_with_ulp32k(mclk, eic_clock, eic);

        //  eic.button_debounce_pins(&[
        //      self.button1.id(),
        //      self.button2.id(),
        //      self.button3.id(),
        //      self.switch_x.id(),
        //      self.switch_y.id(),
        //      self.switch_z.id(),
        //      self.switch_u.id(),
        //      self.switch_b.id(),
        //  ]);

        let eic_channels = Eic::new(mclk, eic_clock, eic).split();

        //  let button: Pin<_, PullUpInterrupt> = pins.d10.into();
        //  let mut extint = eic_channels.2.with_pin(button);

        //  extint.sense(Sense::Fall);
        //  extint.enable_interrupt();

        // Unfortunately, the pin assigned to B1 shares the same
        // ExtInt line as up on the joystick. As such, we don't
        // support B1.

        // let eic_clock = clocks.eic(&gclk0).unwrap();
        // // Initialize the EIC peripheral
        // let eic = Eic::new(&mut peripherals.pm, eic_clock, peripherals.eic);
        // // Split into channels
        // let eic_channels = eic.split();
        //
        // // Take the pin that we want to use
        // let button: Pin<_, PullUpInterrupt> = pins.d10.into();
        //
        // // Turn the EXTINT[2] channel into an ExtInt struct
        // let mut extint = eic_channels.2.with_pin(button);

        // let mut b1 = self.button1.into_floating_ei(port);
        //
        //  let mut b2 = eic_channels.11.with_pin(self.button2.into());
        let mut b2 = eic_channels.11.with_pin(self.button2.into());
        //  let mut b3 = eic_channels.12.with_pin(self.button3.into());
        let mut b3 = eic_channels.12.with_pin(self.button3.into());
        //  let mut x = eic_channels.3.with_pin(self.switch_x.into());
        let mut x = eic_channels.3.with_pin(self.switch_x.into());
        let mut y = eic_channels.4.with_pin(self.switch_y.into());
        let mut z = eic_channels.5.with_pin(self.switch_z.into());
        let mut u = eic_channels.10.with_pin(self.switch_u.into());
        let mut b = eic_channels.7.with_pin(self.switch_b.into());

        // b1.sense(&mut eic, Sense::Both);
        b2.sense(Sense::Both);
        b3.sense(Sense::Both);
        x.sense(Sense::Both);
        y.sense(Sense::Both);
        z.sense(Sense::Both);
        u.sense(Sense::Both);
        b.sense(Sense::Both);

        // b1.enable_interrupt(&mut eic);
        b2.enable_interrupt();
        b3.enable_interrupt();
        x.enable_interrupt();
        y.enable_interrupt();
        z.enable_interrupt();
        u.enable_interrupt();
        b.enable_interrupt();

        //  let b2_channel = E
        ButtonController {
            //  _eic: eic, //  .finalize()
            // b1,
            b2,
            b3,
            x,
            y,
            z,
            u,
            b,
        }
    }
}

#[derive(Debug, PartialEq, Eq)]
pub enum Button {
    TopLeft,
    TopMiddle,
    // TopRight,
    Down,
    Up,
    Left,
    Right,
    Click,
}

#[derive(Debug, PartialEq, Eq)]
pub struct ButtonEvent {
    pub button: Button,
    pub down: bool,
}

pub struct ButtonController {
    //  _eic: eic::Eic,
    //  _eic: PacEic,
    // b1: ExtInt10<Button1>,
    //  b2_channel: Channel<ID = 11>,
    b2: ExtInt<atsamd_hal::gpio::Pin<PC27, Reset>, Ch11>,
    b3: ExtInt<atsamd_hal::gpio::Pin<PC28, Reset>, Ch12>,
    x: ExtInt<atsamd_hal::gpio::Pin<PD08, Reset>, Ch3>,
    y: ExtInt<atsamd_hal::gpio::Pin<PD09, Reset>, Ch4>,
    z: ExtInt<atsamd_hal::gpio::Pin<PD10,Reset>, Ch5>,
    u: ExtInt<atsamd_hal::gpio::Pin<PD20,Reset>, Ch10>,
    b: ExtInt<atsamd_hal::gpio::Pin<PD12,Reset>, Ch7>,
}

macro_rules! isr {
    ($Handler:ident, $($Event:expr, $Button:ident),+) => {
        pub fn $Handler(&mut self) -> Option<ButtonEvent> {
            $(
                {
                    let b = &mut self.$Button;
                    if b.is_interrupt() {
                        b.clear_interrupt();
                        return Some(ButtonEvent {
                            button: $Event,
                            down: !b.state(),
                        })
                    }
                }
            )+

            None
        }
    };
}

impl ButtonController {
    pub fn enable(&self, nvic: &mut NVIC) {
        unsafe {
            nvic.set_priority(interrupt::EIC_EXTINT_10, 1);
            NVIC::unmask(interrupt::EIC_EXTINT_10);
            nvic.set_priority(interrupt::EIC_EXTINT_11, 1);
            NVIC::unmask(interrupt::EIC_EXTINT_11);
            nvic.set_priority(interrupt::EIC_EXTINT_12, 1);
            NVIC::unmask(interrupt::EIC_EXTINT_12);
            nvic.set_priority(interrupt::EIC_EXTINT_3, 1);
            NVIC::unmask(interrupt::EIC_EXTINT_3);
            nvic.set_priority(interrupt::EIC_EXTINT_4, 1);
            NVIC::unmask(interrupt::EIC_EXTINT_4);
            nvic.set_priority(interrupt::EIC_EXTINT_5, 1);
            NVIC::unmask(interrupt::EIC_EXTINT_5);
            nvic.set_priority(interrupt::EIC_EXTINT_7, 1);
            NVIC::unmask(interrupt::EIC_EXTINT_7);
        }
    }

    isr!(interrupt_extint3, Button::Down, x);
    isr!(interrupt_extint4, Button::Right, y);
    isr!(interrupt_extint5, Button::Click, z);
    isr!(interrupt_extint7, Button::Left, b);
    isr!(interrupt_extint10, Button::Up, u);
    // isr!(interrupt_extint10, Button::TopRight, b1);
    isr!(interrupt_extint11, Button::TopMiddle, b2);
    isr!(interrupt_extint12, Button::TopLeft, b3);
}

#[macro_export]
macro_rules! button_interrupt {
    ($controller:ident, unsafe fn $func_name:ident ($cs:ident: $cstype:ty, $event:ident: ButtonEvent ) $code:block) => {
        unsafe fn $func_name($cs: $cstype, $event: ButtonEvent) {
            $code
        }

        macro_rules! _button_interrupt_handler {
            ($Interrupt:ident, $Handler:ident) => {
                #[interrupt]
                fn $Interrupt() {
                    disable_interrupts(|cs| unsafe {
                        $controller.as_mut().map(|ctrlr| {
                            if let Some(event) = ctrlr.$Handler() {
                                $func_name(cs, event);
                            }
                        });
                    });
                }
            };
        }

        _button_interrupt_handler!(EIC_EXTINT_3, interrupt_extint3);
        _button_interrupt_handler!(EIC_EXTINT_4, interrupt_extint4);
        _button_interrupt_handler!(EIC_EXTINT_5, interrupt_extint5);
        _button_interrupt_handler!(EIC_EXTINT_7, interrupt_extint7);
        _button_interrupt_handler!(EIC_EXTINT_10, interrupt_extint10);
        _button_interrupt_handler!(EIC_EXTINT_11, interrupt_extint11);
        _button_interrupt_handler!(EIC_EXTINT_12, interrupt_extint12);
    };
}
