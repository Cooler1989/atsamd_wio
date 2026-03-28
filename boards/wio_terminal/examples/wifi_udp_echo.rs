//! WiFi UDP Echo Server example for the Wio Terminal.
//!
//! Connects to a WiFi access point via DHCP, binds a UDP socket on
//! port 4000, and echoes every received datagram back to its sender.
//!
//! # Testing
//!
//! From a PC on the same network:
//! ```sh
//! echo "hello wio" | nc -u <wio-ip> 4000
//! ```
//!
//! The Wio Terminal display shows firmware version, MAC, assigned IP,
//! and the last received message with its sender address.
//!
//! # Architecture
//!
//! See `doc/wifi_udp_architecture.md` for block and sequence diagrams.

#![no_std]
#![no_main]
#![allow(static_mut_refs)]

use embedded_graphics as eg;
use panic_halt as _;
use wio_terminal as wio;

use wio::hal::clock::GenericClockController;
use wio::hal::delay::Delay;
use wio::pac::{CorePeripherals, Peripherals};
use wio::prelude::*;
use wio::wifi_prelude::*;
use wio::wifi_rpcs as rpc;
use wio::wifi_types::Security;
use wio::{entry, wifi_singleton};

use core::fmt::Write;
use cortex_m::interrupt::free as disable_interrupts;
use eg::mono_font::{ascii::FONT_6X12, MonoTextStyle};
use eg::pixelcolor::Rgb565;
use eg::prelude::*;
use eg::primitives::{PrimitiveStyleBuilder, Rectangle};
use eg::text::{Baseline, Text};

use heapless::String;

// ── Configuration ───────────────────────────────────────────────────

const WIFI_SSID: &str = "NETWORK_NAME";
const WIFI_PASS: &str = "PASSWORD_HERE";
const UDP_PORT: u16 = 4000;

// ── Entry point ─────────────────────────────────────────────────────

#[entry]
fn main() -> ! {
    let mut peripherals = Peripherals::take().unwrap();
    let mut core = CorePeripherals::take().unwrap();

    let mut clocks = GenericClockController::with_external_32kosc(
        peripherals.gclk,
        &mut peripherals.mclk,
        &mut peripherals.osc32kctrl,
        &mut peripherals.oscctrl,
        &mut peripherals.nvmctrl,
    );
    let mut delay = Delay::new(core.SYST, &mut clocks);
    let sets = wio::Pins::new(peripherals.port).split();

    // ── Display init ────────────────────────────────────────────────
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
    clear(&mut display);
    let mut textbuf = String::<256>::new();

    let mut user_led = sets.user_led.into_push_pull_output();
    user_led.set_low().unwrap();

    // ── WiFi init ───────────────────────────────────────────────────
    let nvic = &mut core.NVIC;
    disable_interrupts(|cs| unsafe {
        wifi_init(
            cs,
            sets.wifi,
            peripherals.sercom0,
            &mut clocks,
            &mut peripherals.mclk,
            &mut delay,
        );
        if let Some(wifi) = WIFI.as_mut() {
            wifi.enable(cs, nvic);
        }
    });

    // ── Display firmware version ────────────────────────────────────
    let version = unsafe {
        WIFI.as_mut()
            .map(|wifi| wifi.blocking_rpc(rpc::GetVersion {}).unwrap())
            .unwrap()
    };
    writeln!(textbuf, "fw: {}", version).unwrap();
    draw_text(
        &mut display,
        textbuf.as_str(),
        Point::new(320 - (3 + version.len() * 6) as i32, 3),
    );
    textbuf.truncate(0);

    // ── Display MAC address ─────────────────────────────────────────
    let mac = unsafe {
        WIFI.as_mut()
            .map(|wifi| wifi.blocking_rpc(rpc::GetMacAddress {}).unwrap())
            .unwrap()
    };
    writeln!(textbuf, "mac: {}", mac).unwrap();
    draw_text(&mut display, textbuf.as_str(), Point::new(3, 3));
    textbuf.truncate(0);

    // ── Connect to AP ───────────────────────────────────────────────
    draw_text(&mut display, "Connecting...", Point::new(3, 30));

    let ip_info = unsafe {
        WIFI.as_mut()
            .map(|wifi| {
                wifi.connect_to_ap(
                    &mut delay,
                    WIFI_SSID,
                    WIFI_PASS,
                    Security::WPA2_SECURITY | Security::AES_ENABLED,
                )
                .unwrap()
            })
            .unwrap()
    };
    user_led.set_high().ok();

    // ── Display IP info ─────────────────────────────────────────────
    draw_text_with_clear(&mut display, "                    ", 30, Point::new(3, 30));
    writeln!(textbuf, "ip: {}", ip_info.ip).unwrap();
    draw_text(&mut display, textbuf.as_str(), Point::new(3, 30));
    textbuf.truncate(0);

    writeln!(textbuf, "gw: {}", ip_info.gateway).unwrap();
    draw_text(&mut display, textbuf.as_str(), Point::new(3, 42));
    textbuf.truncate(0);

    // ── Bind UDP socket ─────────────────────────────────────────────
    let socket_fd = unsafe {
        WIFI.as_mut()
            .map(|wifi| wifi.udp_bind(UDP_PORT).unwrap())
            .unwrap()
    };

    writeln!(textbuf, "UDP echo on :{}", UDP_PORT).unwrap();
    draw_text(&mut display, textbuf.as_str(), Point::new(3, 60));
    textbuf.truncate(0);

    draw_text(&mut display, "Waiting for packets...", Point::new(3, 80));

    // ── Packet counter ──────────────────────────────────────────────
    let mut pkt_count: u32 = 0;

    // ── Main loop: poll for UDP packets ─────────────────────────────
    loop {
        let result = unsafe {
            WIFI.as_mut()
                .map(|wifi| wifi.udp_recvfrom(socket_fd))
                .unwrap()
        };

        match result {
            Ok(Some(recv)) => {
                pkt_count += 1;
                user_led.toggle().ok();

                // Display sender and packet info
                draw_text_with_clear(
                    &mut display,
                    "                              ",
                    50,
                    Point::new(3, 80),
                );
                writeln!(
                    textbuf,
                    "#{} from {}.{}.{}.{}:{}",
                    pkt_count,
                    recv.remote_ip[0],
                    recv.remote_ip[1],
                    recv.remote_ip[2],
                    recv.remote_ip[3],
                    recv.remote_port,
                )
                .unwrap();
                draw_text(&mut display, textbuf.as_str(), Point::new(3, 80));
                textbuf.truncate(0);

                // Display received data (first 40 chars)
                draw_text_with_clear(
                    &mut display,
                    "                                        ",
                    45,
                    Point::new(3, 95),
                );
                let show_len = core::cmp::min(recv.data.len(), 40);
                if let Ok(s) = core::str::from_utf8(&recv.data[..show_len]) {
                    draw_text(&mut display, s, Point::new(3, 95));
                } else {
                    writeln!(textbuf, "[{} bytes binary]", recv.data.len()).unwrap();
                    draw_text(&mut display, textbuf.as_str(), Point::new(3, 95));
                    textbuf.truncate(0);
                }

                // Echo the data back to the sender
                let _ = unsafe {
                    WIFI.as_mut().map(|wifi| {
                        wifi.udp_sendto(
                            socket_fd,
                            &recv.data,
                            recv.remote_ip,
                            recv.remote_port,
                        )
                    })
                };
            }
            Ok(None) => {
                // No data available — yield briefly
            }
            Err(_) => {
                // Transient error — continue polling
            }
        }

        delay.delay_ms(5u8);
    }
}

// ── WiFi singleton ──────────────────────────────────────────────────

wifi_singleton!(WIFI);

// ── Display helpers ─────────────────────────────────────────────────

fn clear(display: &mut wio::LCD) {
    display.clear(Rgb565::BLACK).ok().unwrap();
}

fn draw_text<'a, T: Into<&'a str>>(display: &mut wio::LCD, text: T, pos: Point) {
    Text::with_baseline(
        text.into(),
        pos,
        MonoTextStyle::new(&FONT_6X12, Rgb565::WHITE),
        Baseline::Top,
    )
    .draw(display)
    .ok()
    .unwrap();
}

fn draw_text_with_clear<'a, T: Into<&'a str>>(
    display: &mut wio::LCD,
    _clear_text: T,
    num_chars: i32,
    pos: Point,
) {
    let style = PrimitiveStyleBuilder::new()
        .fill_color(Rgb565::BLACK)
        .build();
    Rectangle::with_corners(pos, Point::new(pos.x + (6 * num_chars), pos.y + 12))
        .into_styled(style)
        .draw(display)
        .ok()
        .unwrap();
}
