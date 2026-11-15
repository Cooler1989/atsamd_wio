//! Interrupt-driven UART transport: one interrupt per byte in each direction,
//! with `bbqueue` buffers between the interrupt handlers and the RPC code.

use atsamd_hal::{pac::interrupt, prelude::*, sercom::uart};
use bbqueue::{BBBuffer, Consumer, Producer};
use cortex_m::peripheral::NVIC;
use nb;

use super::{TransportError, WifiUart};

pub struct Transport {
    uart: WifiUart,

    rx_buff_isr: Producer<'static, 512>,
    rx_buff_input: Consumer<'static, 512>,
    tx_buff_isr: Consumer<'static, 128>,
    tx_buff_input: Producer<'static, 128>,
}

impl Transport {
    pub fn new(
        uart: WifiUart,
        rx_buff: &'static BBBuffer<512>,
        tx_buff: &'static BBBuffer<128>,
    ) -> Self {
        let (rx_buff_isr, rx_buff_input) = rx_buff.try_split().unwrap();
        let (tx_buff_input, tx_buff_isr) = tx_buff.try_split().unwrap();

        Transport {
            uart,
            rx_buff_isr,
            rx_buff_input,
            tx_buff_isr,
            tx_buff_input,
        }
    }

    /// Turns on the UART interrupts.
    pub fn enable(&mut self, nvic: &mut NVIC) {
        unsafe {
            nvic.set_priority(interrupt::SERCOM0_0, 1);
            NVIC::unmask(interrupt::SERCOM0_0);
            nvic.set_priority(interrupt::SERCOM0_2, 1);
            NVIC::unmask(interrupt::SERCOM0_2);
        }

        self.uart.enable_interrupts(uart::Flags::RXC);
    }

    /// Called from ISR: Handles the signal that the UART has recieved
    /// a byte that needs to be read.
    pub fn handle_rx(&mut self) {
        #[cfg(feature = "wifi-stats")]
        let _stats = super::stats::IsrGuard::enter();
        match self.uart.read() {
            Ok(b) => {
                if let Ok(mut wgr) = self.rx_buff_isr.grant_exact(1) {
                    wgr[0] = b;
                    wgr.commit(1);
                } else {
                    panic!("overrun");
                }
            }
            Err(nb::Error::Other(_e)) => {
                panic!("unrecoverable read error");
            }
            // Skip WouldBlock
            Err(nb::Error::WouldBlock) => (),
        };
    }

    /// Called from ISR: Handles the signal that the outgoing UART buffer
    /// has room for the next byte.
    pub fn handle_data_empty(&mut self) {
        #[cfg(feature = "wifi-stats")]
        let _stats = super::stats::IsrGuard::enter();
        if let Ok(rgr) = self.tx_buff_isr.read() {
            let buf = rgr.buf();
            self.uart.write(buf[0]).ok();
            rgr.release(1);
        } else {
            self.uart.disable_interrupts(uart::Flags::DRE);
        }
    }

    /// Queues `frame` for transmission.
    pub fn write(&mut self, frame: &[u8]) -> Result<(), TransportError> {
        for b in frame {
            if let Ok(mut wgr) = self.tx_buff_input.grant_exact(1) {
                wgr[0] = *b;
                wgr.commit(1);
            }
            self.uart.enable_interrupts(uart::Flags::DRE);
        }
        Ok(())
    }

    /// Nothing to do: the interrupt transport has no stale-data hazard.
    pub fn discard_pending(&mut self) {}

    /// Blocks until `buffer` has been filled from the receive queue.
    pub fn read_exact(&mut self, mut buffer: &mut [u8]) -> Result<(), TransportError> {
        while buffer.len() > 0 {
            let r = match self.rx_buff_input.read() {
                Ok(r) => r,
                Err(_) => {
                    continue;
                }
            };
            let b = r.buf();
            let copy_amt = if b.len() < buffer.len() {
                b.len()
            } else {
                buffer.len()
            };

            for (i, b) in b[..copy_amt].iter().enumerate() {
                buffer[i] = *b;
            }
            buffer = &mut buffer[copy_amt..];

            r.release(copy_amt);
        }
        Ok(())
    }
}
