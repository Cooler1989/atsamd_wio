//! DMA byte transport for the async eRPC client ([`crate::wifi_async`]).
//!
//! Same receive ring as the blocking DMA transport (`dma.rs`): a circular
//! transfer runs for the life of the program and copies every received byte
//! into a static ring. Two things differ, because this transport is meant to
//! share the DMA controller with `async` users (whose interrupt handler
//! stops any channel with a pending *enabled* interrupt):
//!
//! * **No DMA interrupt.** Ring wraps are counted by polling the RX channel's
//!   transfer-complete flag from [`WifiIo::try_read`] instead of a `DMAC_n`
//!   handler. The channel's interrupts stay disabled, so the HAL's async
//!   DMAC handler never touches it.
//! * **Non-blocking transmit.** [`WifiIo::try_write`] copies up to
//!   [`TX_CHUNK_LEN`] bytes into a static buffer, starts a one-shot DMA
//!   transfer and returns; the next call returns 0 until that transfer has
//!   finished (polled through the channel's enable bit).
//!
//! The channels are any two the application owns (generic `RxId`/`TxId`).
//! See `docs/wifi_dma.md` for the full description.

use atsamd_hal::dmac::{Buffer, BufferPair, Busy, ChId, Channel, Ready, Transfer, TriggerAction};
use atsamd_hal::ehal::digital::OutputPin;
use atsamd_hal::pac;
use atsamd_hal::sercom::uart::{Config, RxDuplex, TxDuplex, Uart};
use atsamd_hal::sercom::{Sercom, Sercom0};
use core::sync::atomic::Ordering;

use super::dma::{write_index, RX_RING_LEN};
use super::{WifiPwr, WifiUart, WifiUartPads, ASYNC_FAULT, LAST_UART_ERROR};
use crate::wifi_async::{ByteTransport, TransportError};

/// Largest chunk one [`WifiIo::try_write`] call sends. eRPC requests are at
/// most 256 bytes of message plus a separate 4-byte frame header.
pub const TX_CHUNK_LEN: usize = 256;

/// Static storage for the async DMA transport: the receive ring and the
/// transmit staging buffer. Place one in a `static` and pass it to
/// [`Wifi::init_async`](super::Wifi::init_async) once.
pub struct WifiDmaBuffers {
    rx: [u8; RX_RING_LEN],
    tx: [u8; TX_CHUNK_LEN],
}

impl WifiDmaBuffers {
    pub const fn new() -> Self {
        Self {
            rx: [0; RX_RING_LEN],
            tx: [0; TX_CHUNK_LEN],
        }
    }
}

impl Default for WifiDmaBuffers {
    fn default() -> Self {
        Self::new()
    }
}

type RxHalf = Uart<Config<WifiUartPads>, RxDuplex>;
type TxHalf = Uart<Config<WifiUartPads>, TxDuplex>;
type RxTransfer<Id> =
    Transfer<Channel<Id, Busy>, BufferPair<RxHalf, &'static mut [u8; RX_RING_LEN]>>;
type TxTransfer<Id> = Transfer<Channel<Id, Busy>, BufferPair<TxChunk, TxHalf>>;

/// The first `len` bytes of the static TX buffer, as a DMA source.
///
/// A raw pointer instead of a slice, so the full-size buffer can be reused
/// for every chunk without re-borrowing it from the `Transfer`.
struct TxChunk {
    ptr: *mut u8,
    len: usize,
}

// SAFETY: `ptr` points into the `'static` TX buffer, which only this
// transport accesses, and `len <= TX_CHUNK_LEN`. Following the `Buffer`
// contract, an incrementing buffer reports its end address.
unsafe impl Buffer for TxChunk {
    type Beat = u8;

    fn dma_ptr(&mut self) -> *mut u8 {
        if self.incrementing() {
            unsafe { self.ptr.add(self.len) }
        } else {
            self.ptr
        }
    }

    fn incrementing(&self) -> bool {
        self.len > 1
    }

    fn buffer_len(&self) -> usize {
        self.len
    }
}

enum TxState<Id: ChId> {
    Idle(Channel<Id, Ready>, TxHalf),
    Busy(TxTransfer<Id>),
    /// Only while moving between the two states above.
    Moving,
}

/// Task-owned DMA transport for [`crate::wifi_async::Client`].
pub struct WifiIo<RxId: ChId, TxId: ChId> {
    _rx: RxTransfer<RxId>,
    ring: *const u8,
    tx: TxState<TxId>,
    tx_buf: *mut u8,
    /// Ring wraps seen by this transport (transfer-complete flags taken).
    wraps: u32,
    /// Bytes consumed, in the same counting domain as `written_total`.
    read_total: u32,
    /// The last operation was a read: the next write starts a new request.
    reading: bool,
}

// The raw pointers refer to `'static` buffers used only by this transport
// and the DMA.
unsafe impl<RxId: ChId, TxId: ChId> Send for WifiIo<RxId, TxId> {}

/// Radio power and fault control for the async DMA transport (the
/// counterpart of the interrupt transport's `WifiInterrupts`; there are no
/// interrupt handlers to call).
pub struct WifiControl {
    pwr: WifiPwr,
}

pub(super) fn new<RxId: ChId, TxId: ChId>(
    pwr: WifiPwr,
    uart: WifiUart,
    rx_channel: Channel<RxId, Ready>,
    tx_channel: Channel<TxId, Ready>,
    buffers: &'static mut WifiDmaBuffers,
) -> (WifiIo<RxId, TxId>, WifiControl) {
    ASYNC_FAULT.store(0, Ordering::Relaxed);
    let (rx, tx) = uart.split();
    let ring: &'static mut [u8; RX_RING_LEN] = &mut buffers.rx;
    let ring_ptr = ring.as_ptr();
    let tx_buf = buffers.tx.as_mut_ptr();
    // The channel comes out of `init` with its interrupts disabled, and they
    // stay disabled (see the module documentation).
    let rx = Transfer::new(rx_channel, rx, ring, true)
        .unwrap()
        .begin(<Sercom0 as Sercom>::DMA_RX_TRIGGER, TriggerAction::Burst);
    (
        WifiIo {
            _rx: rx,
            ring: ring_ptr,
            tx: TxState::Idle(tx_channel, tx),
            tx_buf,
            wraps: 0,
            read_total: 0,
            reading: true,
        },
        WifiControl { pwr },
    )
}

impl<RxId: ChId, TxId: ChId> WifiIo<RxId, TxId> {
    /// Bytes written by the DMA since start, modulo 2^32, counted in this
    /// transport's domain.
    ///
    /// A wrap is counted when the RX channel's transfer-complete flag is
    /// found set; the flag is cleared before the write position is read. A
    /// wrap landing between the flag and the position reads one ring length
    /// behind for one poll and is counted on the next one; `try_read` treats
    /// a negative difference as "nothing new". Two wraps between polls count
    /// as one: data is then lost unnoticed by this check, and the eRPC frame
    /// CRC catches it.
    fn written_total(&mut self) -> u32 {
        let dmac = unsafe { pac::Dmac::steal() };
        let flags = dmac.channel(RxId::USIZE).chintflag();
        if flags.read().tcmpl().bit_is_set() {
            flags.write(|w| w.tcmpl().set_bit());
            self.wraps = self.wraps.wrapping_add(1);
        }
        self.wraps
            .wrapping_mul(RX_RING_LEN as u32)
            .wrapping_add(write_index(RxId::USIZE) as u32)
    }

    /// Drops everything received so far.
    fn resync(&mut self) {
        self.read_total = self.written_total();
    }

    /// Latches a UART overflow, framing or parity error as a transport fault.
    fn check_uart(&mut self) {
        let usart = unsafe { pac::Sercom0::steal() };
        let status = usart.usart_int().status().read();
        let kind = if status.bufovf().bit_is_set() {
            1
        } else if status.ferr().bit_is_set() {
            2
        } else if status.perr().bit_is_set() {
            3
        } else {
            return;
        };
        usart
            .usart_int()
            .status()
            .write(|w| w.bufovf().set_bit().ferr().set_bit().perr().set_bit());
        LAST_UART_ERROR.store(kind, Ordering::Relaxed);
        ASYNC_FAULT.store(2, Ordering::Relaxed);
    }

    /// Returns whether the transmitter is free, collecting a finished
    /// transfer.
    fn tx_idle(&mut self) -> bool {
        match &mut self.tx {
            TxState::Idle(..) => true,
            TxState::Busy(transfer) => {
                if !transfer.complete() {
                    return false;
                }
                let TxState::Busy(transfer) = core::mem::replace(&mut self.tx, TxState::Moving)
                else {
                    unreachable!()
                };
                let (channel, _chunk, uart) = transfer.wait();
                self.tx = TxState::Idle(channel, uart);
                true
            }
            TxState::Moving => false,
        }
    }
}

impl<RxId: ChId, TxId: ChId> ByteTransport for WifiIo<RxId, TxId> {
    fn try_write(&mut self, bytes: &[u8]) -> usize {
        if bytes.is_empty() || ASYNC_FAULT.load(Ordering::Relaxed) != 0 {
            return 0;
        }
        if self.reading {
            // First write of a request: nothing received before it can be
            // the reply, so drop it (stale replies, callbacks, line noise).
            // This also re-bases the wrap count after an idle period in
            // which nobody polled the ring.
            self.resync();
            self.reading = false;
        }
        if !self.tx_idle() {
            return 0;
        }
        let TxState::Idle(channel, uart) = core::mem::replace(&mut self.tx, TxState::Moving)
        else {
            unreachable!()
        };
        let count = bytes.len().min(TX_CHUNK_LEN);
        // SAFETY: the TX channel is idle, so the DMA is not reading the
        // buffer, and `count <= TX_CHUNK_LEN`.
        unsafe { core::ptr::copy_nonoverlapping(bytes.as_ptr(), self.tx_buf, count) };
        let chunk = TxChunk {
            ptr: self.tx_buf,
            len: count,
        };
        // SAFETY: the UART side is a single non-incrementing beat and the
        // chunk is at most 256 beats, so the lengths are valid; both buffers
        // are 'static. Same construction as the HAL's `send_with_dma`, but
        // without enabling the channel interrupt.
        let transfer = unsafe { Transfer::new_unchecked(channel, chunk, uart, false) }
            .begin(<Sercom0 as Sercom>::DMA_TX_TRIGGER, TriggerAction::Burst);
        self.tx = TxState::Busy(transfer);
        count
    }

    fn try_read(&mut self, out: &mut [u8]) -> usize {
        self.reading = true;
        if out.is_empty() || ASYNC_FAULT.load(Ordering::Relaxed) != 0 {
            return 0;
        }
        self.check_uart();
        let unread = self.written_total().wrapping_sub(self.read_total) as i32;
        if unread > RX_RING_LEN as i32 {
            ASYNC_FAULT.store(1, Ordering::Relaxed);
            return 0;
        }
        if unread <= 0 {
            return 0;
        }
        let count = (unread as usize).min(out.len());
        for (i, byte) in out[..count].iter_mut().enumerate() {
            let index = self.read_total.wrapping_add(i as u32) as usize % RX_RING_LEN;
            *byte = unsafe { core::ptr::read_volatile(self.ring.add(index)) };
        }
        // If the DMA lapped the copied bytes while they were copied, they
        // are not the ones received in order.
        if self.written_total().wrapping_sub(self.read_total) as i32 > RX_RING_LEN as i32 {
            ASYNC_FAULT.store(1, Ordering::Relaxed);
            return 0;
        }
        self.read_total = self.read_total.wrapping_add(count as u32);
        count
    }

    fn fault(&self) -> Option<TransportError> {
        match ASYNC_FAULT.load(Ordering::Relaxed) {
            0 => None,
            1 => Some(TransportError::RxOverflow),
            _ => Some(TransportError::Uart),
        }
    }

    fn drain(&mut self) {
        self.resync();
    }
}

impl WifiControl {
    /// Controls the radio power/reset pin.
    pub fn set_power(
        &mut self,
        enabled: bool,
    ) -> Result<(), <WifiPwr as atsamd_hal::ehal::digital::ErrorType>::Error> {
        if enabled {
            OutputPin::set_high(&mut self.pwr)
        } else {
            OutputPin::set_low(&mut self.pwr)
        }
    }

    /// Clears a latched transport fault after a hardware radio reset. Call
    /// `Client::recover` afterwards, which drops what the ring holds.
    pub fn reset_fault(&mut self) {
        let usart = unsafe { pac::Sercom0::steal() };
        usart
            .usart_int()
            .status()
            .write(|w| w.bufovf().set_bit().ferr().set_bit().perr().set_bit());
        ASYNC_FAULT.store(0, Ordering::Relaxed);
    }
}
