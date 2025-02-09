use atsamd_hal::clock::GenericClockController;
use atsamd_hal::ehal::delay::DelayNs;
//  use atsamd_hal::ehal::digital::v2::OutputPin;
use atsamd_hal::ehal::digital::OutputPin;
use atsamd_hal::ehal::spi::{
    ErrorKind, ErrorType, Operation as SpiOperation, Phase, Polarity, SpiDevice,
};
use atsamd_hal::pac::{Mclk, Sercom7 as PacSercom7};
use atsamd_hal::sercom::spi;
use atsamd_hal::sercom::{IoSet4, Sercom7};
use atsamd_hal::time::Hertz;
use atsamd_hal::typelevel::NoneT;
use display_interface::{WriteOnlyDataCommand, DisplayError, DataFormat};
use ili9341::{DisplaySize240x320, Ili9341, Orientation};
use atsamd_hal::dmac::DmaController;
use atsamd_hal::sercom::spi::Spi;
use atsamd_hal::gpio::Pin;

use super::pins::aliases::*;

/// ILI9341 LCD display pins (uses `SERCOM7`)
pub struct Display {
    /// LCD MISO pin
    pub miso: LcdMisoReset,

    /// LCD MOSI pin
    pub mosi: LcdMosiReset,

    /// LCD SCK pin
    pub sck: LcdSckReset,

    /// LCD chip select pin
    pub cs: LcdCsReset,

    /// LCD data/command pin
    pub dc: LcdDcReset,

    /// LCD reset pin
    pub reset: LcdResetReset,

    /// LCD backlight pin
    pub backlight: LcdBacklightReset,
}

pub type LcdPads = spi::Pads<Sercom7, IoSet4, NoneT, LcdMosi, LcdSck>;
pub type LcdSpi = spi::Spi<spi::Config<LcdPads>, spi::Tx>;

/// Type alias for the ILI9341 LCD display.
//  pub type LCD = Ili9341<SPIInterface<LcdSpi, LcdDc>, LcdReset>;
pub type LCD = Ili9341<SpiDeviceImpl, LcdReset>;
//  pub type SPI = Spi<Config<LcdPads>, spi::Tx>;

pub use ili9341::Scroller;

#[derive(Debug)]
enum TemporaryError {}

pub struct SpiDeviceImpl {
    spi: LcdSpi,
    cs: LcdCs, 
    dc: LcdDc,
}

impl ErrorType for SpiDeviceImpl {
    type Error = ErrorKind;
}

impl SpiDeviceImpl {
    fn new(
        spi: LcdSpi,
        cs: LcdCs, 
        dc: LcdDc) -> Self {
        Self {spi, cs, dc}
    }
}

struct ChipSelectGuard<'a> {
    cs: &'a mut LcdCs,
}

impl<'a> ChipSelectGuard<'a> {
    pub fn new(cs: &'a mut LcdCs) -> Self {
        cs.set_low().unwrap(); // Assert CS
        ChipSelectGuard { cs }
    }
}

impl<'a> Drop for ChipSelectGuard<'a> {
    fn drop(&mut self) {
        self.cs.set_high().unwrap(); // Deassert CS
    }
}

impl WriteOnlyDataCommand for SpiDeviceImpl {
    fn send_commands(&mut self, commands: DataFormat<'_>) -> Result<(), DisplayError> {
        ChipSelectGuard::new(&mut self.cs);
        self.dc.set_low().unwrap(); // Command mode
        Ok(())
    }
    fn send_data(&mut self, data: DataFormat<'_>) -> Result<(), DisplayError> {
        ChipSelectGuard::new(&mut self.cs);
        self.dc.set_high().unwrap(); // Data mode
        Ok(())
    }
}

//  impl SpiDevice for SpiDeviceImpl {
//      //  fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
//      fn transaction(&mut self, operations: &mut [SpiOperation<'_, u8>]) -> Result<(), Self::Error> {
//          Ok(())
//      }
//  }

impl Display {
    pub fn new(miso: LcdMisoReset, 
        mosi: LcdMosiReset, 
        sck: LcdSckReset, 
        cs: LcdCsReset, 
        dc: LcdDcReset, 
        reset: LcdResetReset, 
        backlight: LcdBacklightReset) -> Self {
        Self {
            miso,
            mosi,
            sck,
            cs,
            dc,
            reset,
            backlight,
        }
    }

    /// Initialize the display and its corresponding SPI bus peripheral. Return
    /// a tuple containing the configured display driver struct and backlight
    /// pin.
    pub fn init<D: DelayNs>(
        self,
        clocks: &mut GenericClockController,
        sercom7: PacSercom7,
        mclk: &mut Mclk,
        baud: Hertz,
        delay: &mut D,
    ) -> Result<(LCD, LcdBacklight), ()> {
        // Initialize the SPI peripherial on the configured pins, using SERCOM7.
        let gclk0 = clocks.gclk0();
        let clock = &clocks.sercom7_core(&gclk0).ok_or(())?;
        let pads = spi::Pads::default().data_out(self.mosi).sclk(self.sck);
        let spi = spi::Config::new(mclk, sercom7, pads, clock.freq())
            .spi_mode(spi::MODE_0)
            .baud(baud)
            .enable();

        // Configure the chip select, data/command, and reset pins as push-pull outputs.
        let cs = self.cs.into_push_pull_output();
        let dc = self.dc.into_push_pull_output();
        let reset = self.reset.into_push_pull_output();

        //  let dmac = peripherals.dmac;
        //  let mut dmac = DmaController::init(dmac, &mut peripherals.pm);
        //  let channels = dmac.split();
        //  let chan0 = channels.0.init(PriorityLevel::Lvl0);
        //  let chan1 = channels.1.init(PriorityLevel::Lvl0);

        //  let mut spi = bsp::spi_master(
        //      &mut clocks,
        //      100.kHz(),
        //      peripherals.sercom2,
        //      &mut peripherals.mclk,
        //      sclk,
        //      mosi,
        //      miso,
        //  )
        //  .with_dma_channels(chan0, chan1);

        // Create a SPIInterface over the peripheral, then create the ILI9341 driver
        // using said interface and set its default orientation.
        //  let interface = SPIInterface::new(spi, dc);
        let interface_spi = SpiDeviceImpl::new(spi, cs, dc);
        let ili9341 = Ili9341::new(
            interface_spi,
            reset,
            delay,
            Orientation::LandscapeFlipped,
            DisplaySize240x320,
        )
        .map_err(|_| ())?;

        // Configure the backlight pin as a push-pull output; unfortunately this pin
        // does not appear to support PWM.
        //   HIGH - backlight enabled
        //   LOW  - backlight disabled
        let mut backlight = self.backlight.into_push_pull_output();
        backlight.set_high().ok();

        // Return a result consisting of a Tuple containing the display driver and
        // backlight pin.
        Ok((ili9341, backlight))
    }
}
