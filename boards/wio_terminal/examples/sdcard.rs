#![no_std]
#![no_main]

/// Makes the wio_terminal read the SD card and print the filenames
/// of the first few entries.
use embedded_graphics as eg;
use embedded_sdmmc::SdCard;
use panic_halt as _;
use wio_terminal as wio;

use eg::mono_font::{ascii::FONT_9X15, MonoTextStyle};
use eg::pixelcolor::Rgb565;
use eg::prelude::*;
use eg::primitives::{PrimitiveStyleBuilder, Rectangle};
use eg::text::{Baseline, Text};

use wio::entry;
use wio::hal::clock::GenericClockController;
use wio::hal::delay::Delay;
use wio::pac::{CorePeripherals, Peripherals};
use wio::prelude::*;

use core::fmt::Write;
use heapless::String;

use embedded_sdmmc::{TimeSource, Timestamp, VolumeIdx, sdcard::Error as SdCardError};
use wio::SDCardController;

#[entry]
fn main() -> ! {
    let mut peripherals = Peripherals::take().unwrap();
    let core = CorePeripherals::take().unwrap();

    let mut clocks = GenericClockController::with_external_32kosc(
        peripherals.gclk,
        &mut peripherals.mclk,
        &mut peripherals.osc32kctrl,
        &mut peripherals.oscctrl,
        &mut peripherals.nvmctrl,
    );

    let mut delay = Delay::new(core.SYST, &mut clocks);

    let sets = wio::Pins::new(peripherals.port).split();

    // Initialize the ILI9341-based LCD display. Create a black backdrop the size of
    // the screen.
    let (mut display, _backlight) = sets
        .display
        .init(
            &mut clocks,
            peripherals.sercom7,
            &mut peripherals.mclk,
            58.MHz(),
            &mut delay,
        )
        .unwrap();

    let (mut sd_controller, _sd_present) = sets
        .sd_card
        .init(
            &mut clocks,
            peripherals.sercom6,
            &mut peripherals.mclk,
            delay,
            Clock,
        )
        .unwrap();
        //  TODO: handle error:
        //      Err(e) => {
        //          let mut data = String::<128>::new();
        //          writeln!(data, "Error!: {:?}", e).unwrap();
        //          Text::with_baseline(data.as_str(), Point::new(4, 2), style, Baseline::Top)
        //              .draw(&mut display)
        //              .ok()
        //              .unwrap();
        //      }

    let style = MonoTextStyle::new(&FONT_9X15, Rgb565::WHITE);

    loop {
        let mut data = String::<128>::new();
        match sd_controller.device().num_bytes() {
            Ok(size) => writeln!(data, "{}Mb", size / 1024 / 1024).unwrap(),
            Err(e) => writeln!(data, "Err: {:?}", e).unwrap(),
        }
        Text::with_baseline(data.as_str(), Point::new(4, 2), style, Baseline::Top)
            .draw(&mut display)
            .ok()
            .unwrap();
        if let Err(e) = print_contents(&mut sd_controller, &mut display) {
            let mut data = String::<128>::new();
            writeln!(data, "Err: {:?}", e).unwrap();
            Text::with_baseline(data.as_str(), Point::new(4, 20), style, Baseline::Top)
                .draw(&mut display)
                .ok()
                .unwrap();
        }

        //  match sd_controller {
        //      Ok(_) => {
        //          // Now that we have initialized, we can run the SPI bus at
        //          // a reasonable speed.
        //          sd_controller.set_baud(20.MHz());

        //          let mut data = String::<128>::new();
        //          write!(data, "OK! ").unwrap();
        //          match sd_controller.device().card_size_bytes() {
        //              Ok(size) => writeln!(data, "{}Mb", size / 1024 / 1024).unwrap(),
        //              Err(e) => writeln!(data, "Err: {:?}", e).unwrap(),
        //          }
        //          Text::with_baseline(data.as_str(), Point::new(4, 2), style, Baseline::Top)
        //              .draw(&mut display)
        //              .ok()
        //              .unwrap();

        //          if let Err(e) = print_contents(&mut sd_controller, &mut display) {
        //              let mut data = String::<128>::new();
        //              writeln!(data, "Err: {:?}", e).unwrap();
        //              Text::with_baseline(data.as_str(), Point::new(4, 20), style, Baseline::Top)
        //                  .draw(&mut display)
        //                  .ok()
        //                  .unwrap();
        //          }
        //      }
        //      Err(e) => {
        //          let mut data = String::<128>::new();
        //          writeln!(data, "Error!: {:?}", e).unwrap();
        //          Text::with_baseline(data.as_str(), Point::new(4, 2), style, Baseline::Top)
        //              .draw(&mut display)
        //              .ok()
        //              .unwrap();
        //      }
        //  }

        //  TODO: implement Mono or other way to provide delays
        //  delay.delay_ms(2500_u16);

        Rectangle::with_corners(Point::new(0, 0), Point::new(320, 240))
            .into_styled(
                PrimitiveStyleBuilder::new()
                    .fill_color(Rgb565::BLACK)
                    .build(),
            )
            .draw(&mut display)
            .ok()
            .unwrap();
    }
}

fn print_contents(
    sd_controller: &mut SDCardController<Clock>,
    lcd: &mut wio::LCD,
) -> Result<(), embedded_sdmmc::Error<SdCardError>> {
    let style = MonoTextStyle::new(&FONT_9X15, Rgb565::WHITE);

    let mut volume = sd_controller.open_volume(VolumeIdx(0)).expect("Failed to open volume");
    let mut dir = volume.open_root_dir().expect("Failed to open root dir");

    let mut count = 0;
    let out = dir.iterate_dir(|ent| {
        let mut data = String::<128>::new();
        writeln!(data, "{} - {:?}", ent.name, ent.attributes).unwrap();
        Text::with_baseline(
            data.as_str(),
            Point::new(4, 20 + count * 16),
            style,
            Baseline::Top,
        )
        .draw(lcd)
        .ok()
        .unwrap();
        count += 1;
    });
    let _ = dir.close();
    out
}

struct Clock;

impl TimeSource for Clock {
    fn get_timestamp(&self) -> Timestamp {
        Timestamp {
            year_since_1970: 0,
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}
