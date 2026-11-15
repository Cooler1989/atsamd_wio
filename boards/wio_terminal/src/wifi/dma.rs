//! DMA-driven UART transport: the receiver runs forever in a circular DMA
//! transfer into a static ring, and the transmitter is a blocking DMA write.
//! No UART interrupts are used.

use atsamd_hal::dmac::{
    BufferPair, Busy, Ch0, Ch1, ChId, Channel, Ready, Transfer, TriggerAction,
};
use atsamd_hal::embedded_io::Write;
use atsamd_hal::pac;
use atsamd_hal::sercom::uart::{Config, RxDuplex, TxDuplex, Uart};
use atsamd_hal::sercom::{Sercom, Sercom0};
use atsamd_hal::typelevel::NoneT;
use cortex_m::peripheral::NVIC;

use super::{TransportError, WifiUart, WifiUartPads};

pub const RX_RING_LEN: usize = 2048;
pub type WifiRxChannel = Channel<Ch0, Ready>;
pub type WifiTxChannel = Channel<Ch1, Ready>;
type RxHalf = Uart<Config<WifiUartPads>, RxDuplex>;
type TxHalf = Uart<Config<WifiUartPads>, TxDuplex, NoneT, WifiTxChannel>;
type RxTransfer =
    Transfer<Channel<Ch0, Busy>, BufferPair<RxHalf, &'static mut [u8; RX_RING_LEN]>>;
const RX_CHANNEL: usize = <Ch0 as ChId>::USIZE;

pub struct Transport {
    _rx: RxTransfer,
    tx: TxHalf,
    ring: *const u8,
    read: usize,
}

// The raw pointer refers to a `'static` buffer that is only written by DMA.
unsafe impl Send for Transport {}

/// Index in the ring where the DMA will write the next byte.
///
/// The DMAC only writes a channel's descriptor back to memory when another
/// channel takes over, so the write-back BTCNT can be stale while this channel
/// was the last one to run. `ACTIVE` holds the live count for the most recent
/// channel even after `ABUSY` clears, so it is trusted whenever its ID is the
/// RX channel. The two registers are not read atomically, so retry until
/// `ACTIVE` is unchanged around the read.
fn write_index() -> usize {
    let dmac = unsafe { pac::Dmac::steal() };
    let remaining = loop {
        let before = dmac.active().read();
        let remaining = if before.id().bits() as usize == RX_CHANNEL {
            before.btcnt().bits()
        } else {
            let writeback = dmac.wrbaddr().read().bits() as *const u8;
            unsafe { core::ptr::read_volatile(writeback.add(RX_CHANNEL * 16 + 2) as *const u16) }
        };
        if dmac.active().read().bits() == before.bits() {
            break remaining as usize;
        }
    };
    // Before the first byte the write-back entry is still zero.
    if remaining == 0 || remaining > RX_RING_LEN {
        0
    } else {
        RX_RING_LEN - remaining
    }
}

/// Returns whether the UART receiver reported an overflow or framing error,
/// and clears the flags.
fn take_rx_error() -> bool {
    let usart = unsafe { pac::Sercom0::steal() };
    let status = usart.usart_int().status().read();
    let failed = status.bufovf().bit_is_set() || status.ferr().bit_is_set();
    if failed {
        usart
            .usart_int()
            .status()
            .write(|w| w.bufovf().set_bit().ferr().set_bit());
    }
    failed
}

impl Transport {
    pub fn new(
        uart: WifiUart,
        rx_channel: WifiRxChannel,
        tx_channel: WifiTxChannel,
        ring: &'static mut [u8; RX_RING_LEN],
    ) -> Self {
        let (rx, tx) = uart.split();
        let tx: TxHalf = tx.with_tx_channel(tx_channel);
        let ring_ptr: *const u8 = ring.as_ptr();
        let rx: RxTransfer = Transfer::new(rx_channel, rx, ring, true)
            .unwrap()
            .begin(<Sercom0 as Sercom>::DMA_RX_TRIGGER, TriggerAction::Burst);

        Transport {
            _rx: rx,
            tx,
            ring: ring_ptr,
            read: 0,
        }
    }

    /// Nothing to enable: the transport uses no interrupts.
    pub fn enable(&mut self, _nvic: &mut NVIC) {}

    pub fn write(&mut self, frame: &[u8]) -> Result<(), TransportError> {
        self.tx.write_all(frame).map_err(|_| TransportError::Rx)
    }

    /// Blocks until `buffer` has been filled from the ring. Fails if the
    /// UART reports an overflow or framing error while waiting.
    pub fn read_exact(&mut self, buffer: &mut [u8]) -> Result<(), TransportError> {
        for out in buffer.iter_mut() {
            while write_index() == self.read {
                if take_rx_error() {
                    return Err(TransportError::Rx);
                }
            }
            *out = unsafe { core::ptr::read_volatile(self.ring.add(self.read)) };
            self.read = (self.read + 1) % RX_RING_LEN;
        }
        Ok(())
    }

    /// Drops everything received so far, to resynchronise after an error.
    pub fn discard_pending(&mut self) {
        self.read = write_index();
    }
}
