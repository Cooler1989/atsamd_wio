use atsamd_hal::{
    clock::GenericClockController,
    delay::Delay,
    ehal::digital::OutputPin,
    pac::Mclk,
    prelude::*,
    sercom::{uart, Sercom0},
};
#[cfg(not(feature = "wifi-dma"))]
use atsamd_hal::pac::interrupt;
#[cfg(not(feature = "wifi-dma"))]
use bbqueue::{BBBuffer, Consumer, Producer};
#[cfg(not(feature = "wifi-dma"))]
use core::sync::atomic::{AtomicU8, Ordering};

use cortex_m::interrupt::CriticalSection;
use cortex_m::peripheral::NVIC;

#[cfg(not(feature = "wifi-dma"))]
mod irq;
#[cfg(not(feature = "wifi-dma"))]
use irq::Transport;
#[cfg(feature = "wifi-dma")]
mod dma;
#[cfg(feature = "wifi-dma")]
use dma::Transport;
#[cfg(feature = "wifi-dma")]
pub use dma::{_handle_dma_rx_wrap, WifiRxChannel, WifiTxChannel, RX_RING_LEN};

pub use erpc::rpcs;
use seeed_erpc as erpc;

use super::pins::aliases::*;

use crate::WIFI_UART_BAUD;

mod connect;
mod diagnostics;
pub use connect::{WifiConnectError, WifiConnectStage, DHCP_ALREADY_STOPPED};

/// The set of pins which are connected to the RTL8720 in some way
pub struct WifiPins {
    pub pwr: WifiPwrReset,
    pub rxd: WifiRxdReset,
    pub txd: WifiTxdReset,
    pub mosi: WifiTxReset,
    pub clk: WifiClkReset,
    pub miso: WifiRxReset,
    pub cs: WifiCsReset,
    pub ready: WifiReadyReset,
    pub dir: WifiDirReset,
}

/// eRPC-based protocol to the RTL8720 chip
pub struct Wifi {
    _pwr: WifiPwr,
    transport: Transport,
    sequence: u32,
}

/// Errors reported by a wifi transport.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransportError {
    /// The UART reported an overflow or framing error, or received bytes had
    /// to be dropped because the receive buffer was full.
    Rx,
    /// The request could not be sent.
    Tx,
    /// Nothing arrived within the read budget (see [`REPLY_WAIT_BUDGET`] and
    /// [`INTER_BYTE_BUDGET`]).
    Timeout,
}

// Read timeouts, counted in polling iterations because the transport owns no
// timer. One iteration is roughly 50 CPU cycles (estimate, not calibrated),
// so at 120 MHz:

/// Wait for the first byte of a reply: about 10 s. Long enough for the
/// slowest RPC (`WifiConnect` blocks on the RTL8720 until it has associated),
/// short enough that a silent RTL8720 cannot hang the caller forever.
pub const REPLY_WAIT_BUDGET: u32 = 24_000_000;

/// Wait between bytes once a reply has started: about 10 ms. The RTL8720
/// sends a frame back to back (16 us per byte), so a longer gap means bytes
/// were lost and the frame will never complete.
pub const INTER_BYTE_BUDGET: u32 = 24_000;

/// UART pads for the labelled RX & TX pins
pub type WifiUartPads = uart::Pads<Sercom0, WifiRx, WifiTx>;

/// UART device for the labelled RX & TX pins
pub type WifiUart = uart::Uart<uart::Config<WifiUartPads>, uart::Duplex>;

// The async split (`Wifi::into_async`, `WifiIo`, `WifiInterrupts`) drives the
// UART from the SERCOM0 interrupts, so it is only available on the interrupt
// transport, not with `wifi-dma`.

// The board has one radio/UART. Latch ISR faults until its next initialization.
#[cfg(not(feature = "wifi-dma"))]
static ASYNC_FAULT: AtomicU8 = AtomicU8::new(0);
/// Kind of the last UART error seen by the interrupt handlers (kept across
/// `reset_fault`, for diagnostics): 0 none, 1 RX overflow, 2 frame, 3 parity,
/// 4 other RX error, 5 TX error.
#[cfg(not(feature = "wifi-dma"))]
static LAST_UART_ERROR: AtomicU8 = AtomicU8::new(0);

/// Text for the last UART error the radio interrupt handlers latched.
#[cfg(not(feature = "wifi-dma"))]
pub fn last_uart_error() -> &'static str {
    match LAST_UART_ERROR.load(Ordering::Relaxed) {
        0 => "UART",
        1 => "UART overflow",
        2 => "UART frame",
        3 => "UART parity",
        4 => "UART rx",
        _ => "UART tx",
    }
}

/// Task-owned queues; never aliases the interrupt-owned UART.
#[cfg(not(feature = "wifi-dma"))]
pub struct WifiIo {
    rx: Consumer<'static, 512>,
    tx: Producer<'static, 128>,
}

/// Store behind an interrupt-safe mutex and call the handlers from SERCOM0.
#[cfg(not(feature = "wifi-dma"))]
pub struct WifiInterrupts {
    pwr: WifiPwr,
    uart: WifiUart,
    rx: Producer<'static, 512>,
    tx: Consumer<'static, 128>,
}

#[cfg(not(feature = "wifi-dma"))]
impl crate::wifi_async::ByteTransport for WifiIo {
    fn try_write(&mut self, bytes: &[u8]) -> usize {
        let count = bytes.len().min(64);
        if count == 0 {
            return 0;
        }
        let Ok(mut grant) = self.tx.grant_exact(count) else {
            return 0;
        };
        grant.copy_from_slice(&bytes[..count]);
        grant.commit(count);
        NVIC::pend(interrupt::SERCOM0_0);
        count
    }

    fn try_read(&mut self, bytes: &mut [u8]) -> usize {
        let Ok(grant) = self.rx.read() else {
            return 0;
        };
        let count = bytes.len().min(grant.len());
        bytes[..count].copy_from_slice(&grant[..count]);
        grant.release(count);
        count
    }

    fn fault(&self) -> Option<crate::wifi_async::TransportError> {
        match ASYNC_FAULT.load(Ordering::Relaxed) {
            0 => None,
            1 => Some(crate::wifi_async::TransportError::RxOverflow),
            _ => Some(crate::wifi_async::TransportError::Uart),
        }
    }
}

#[cfg(not(feature = "wifi-dma"))]
impl WifiInterrupts {
    /// Controls the radio power/reset pin while the UART remains owned by the
    /// interrupt handler.
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

    /// Clears a latched UART/queue fault after a hardware radio reset.
    pub fn reset_fault(&mut self) {
        while let Ok(grant) = self.tx.read() {
            let count = grant.len();
            grant.release(count);
        }
        self.uart.flush_rx_buffer();
        ASYNC_FAULT.store(0, Ordering::Relaxed);
        self.uart
            .enable_interrupts(uart::Flags::RXC | uart::Flags::DRE);
    }

    pub fn enable(&mut self, _cs: &CriticalSection, nvic: &mut NVIC) {
        unsafe {
            nvic.set_priority(interrupt::SERCOM0_0, 1);
            nvic.set_priority(interrupt::SERCOM0_2, 1);
            NVIC::unmask(interrupt::SERCOM0_0);
            NVIC::unmask(interrupt::SERCOM0_2);
        }
        self.uart.enable_interrupts(uart::Flags::RXC);
    }

    /// Drains every byte the receiver holds under one queue grant. At 3 Mbaud
    /// a byte arrives every 3.3 µs; one interrupt (and grant) per byte left
    /// so little slack that any other short interrupt or critical section
    /// overran the SERCOM receiver during bulk transfers.
    pub fn on_rx(&mut self) {
        let Ok(mut grant) = self.rx.grant_max_remaining(32) else {
            // Queue full: a waiting byte would be lost.
            match self.uart.read() {
                Ok(_) => {
                    ASYNC_FAULT.store(1, Ordering::Relaxed);
                    self.uart.disable_interrupts(uart::Flags::RXC);
                }
                Err(nb::Error::WouldBlock) => {}
                Err(nb::Error::Other(error)) => self.rx_error(error),
            }
            return;
        };
        let mut count = 0;
        while count < grant.len() {
            match self.uart.read() {
                Ok(byte) => {
                    grant[count] = byte;
                    count += 1;
                }
                Err(nb::Error::WouldBlock) => break,
                Err(nb::Error::Other(error)) => {
                    grant.commit(count);
                    self.rx_error(error);
                    return;
                }
            }
        }
        // Bytes beyond the grant keep RXC set and re-enter the handler.
        grant.commit(count);
    }

    fn rx_error(&mut self, error: uart::Error) {
        let kind = match error {
            uart::Error::Overflow => 1,
            uart::Error::FrameError => 2,
            uart::Error::ParityError => 3,
            _ => 4,
        };
        LAST_UART_ERROR.store(kind, Ordering::Relaxed);
        ASYNC_FAULT.store(2, Ordering::Relaxed);
        self.uart.disable_interrupts(uart::Flags::RXC);
    }

    pub fn on_tx(&mut self) {
        if ASYNC_FAULT.load(Ordering::Relaxed) != 0 {
            self.uart.disable_interrupts(uart::Flags::DRE);
            return;
        }
        if let Ok(grant) = self.tx.read() {
            match self.uart.write(grant[0]) {
                Ok(()) => grant.release(1),
                Err(nb::Error::WouldBlock) => grant.release(0),
                Err(nb::Error::Other(_)) => {
                    grant.release(0);
                    LAST_UART_ERROR.store(5, Ordering::Relaxed);
                    ASYNC_FAULT.store(2, Ordering::Relaxed);
                    self.uart.disable_interrupts(uart::Flags::DRE);
                    return;
                }
            }
            self.uart.enable_interrupts(uart::Flags::DRE);
        } else {
            self.uart.disable_interrupts(uart::Flags::DRE);
        }
    }
}

impl connect::Transport for Wifi {
    fn call<R: erpc::RPC>(&mut self, rpc: R) -> Result<R::ReturnValue, erpc::Err<R::Error>> {
        self.blocking_rpc(rpc)
    }
}

impl connect::DelayMs for Delay {
    fn wait_ms(&mut self, ms: u8) {
        self.delay_ms(ms);
    }
}

impl Wifi {
    /// Consume an initialized, not-yet-enabled radio to separate task and ISR
    /// ownership. Existing blocking APIs and singleton macros are unchanged.
    #[cfg(not(feature = "wifi-dma"))]
    pub fn into_async(self) -> (crate::wifi_async::Client<WifiIo>, WifiInterrupts) {
        ASYNC_FAULT.store(0, Ordering::Relaxed);
        let parts = self.transport.into_parts();
        (
            crate::wifi_async::Client::new(WifiIo {
                rx: parts.rx_buff_input,
                tx: parts.tx_buff_input,
            }),
            WifiInterrupts {
                pwr: self._pwr,
                uart: parts.uart,
                rx: parts.rx_buff_isr,
                tx: parts.tx_buff_isr,
            },
        )
    }

    /// Builds the UART and power-cycles the RTL8720.
    fn start_uart(
        pins: WifiPins,
        sercom0: Sercom0,
        clocks: &mut GenericClockController,
        mclk: &mut Mclk,
        delay: &mut Delay,
    ) -> (WifiPwr, WifiUart) {
        let gclk0 = clocks.gclk0();

        let pads = uart::Pads::default().rx(pins.miso).tx(pins.mosi);
        let uart = uart::Config::new(
            mclk,
            sercom0,
            pads,
            clocks.sercom0_core(&gclk0).unwrap().freq(),
        )
        .baud(
            WIFI_UART_BAUD.Hz(),
            uart::BaudMode::Fractional(uart::Oversampling::Bits16),
        );

        delay.delay_ms(10u8);

        // Reset the RTL8720 MCU.
        let mut pwr: WifiPwr = pins.pwr.into();
        OutputPin::set_low(&mut pwr).ok();
        delay.delay_ms(100u8);
        OutputPin::set_high(&mut pwr).ok();
        delay.delay_ms(200u8);

        (pwr, uart.enable())
    }

    #[cfg(not(feature = "wifi-dma"))]
    pub fn init(
        pins: WifiPins,
        sercom0: Sercom0,
        clocks: &mut GenericClockController,
        mclk: &mut Mclk,
        delay: &mut Delay,
        rx_buff: &'static BBBuffer<512>,
        tx_buff: &'static BBBuffer<128>,
    ) -> Wifi {
        let (pwr, uart) = Self::start_uart(pins, sercom0, clocks, mclk, delay);
        Wifi {
            _pwr: pwr,
            transport: Transport::new(uart, rx_buff, tx_buff),
            sequence: 0,
        }
    }

    #[cfg(feature = "wifi-dma")]
    #[allow(clippy::too_many_arguments)]
    pub fn init(
        pins: WifiPins,
        sercom0: Sercom0,
        clocks: &mut GenericClockController,
        mclk: &mut Mclk,
        delay: &mut Delay,
        rx_channel: WifiRxChannel,
        tx_channel: WifiTxChannel,
        rx_ring: &'static mut [u8; RX_RING_LEN],
    ) -> Wifi {
        let (pwr, uart) = Self::start_uart(pins, sercom0, clocks, mclk, delay);
        Wifi {
            _pwr: pwr,
            transport: Transport::new(uart, rx_channel, tx_channel, rx_ring),
            sequence: 0,
        }
    }

    /// Turns on internal interrupts. Call this after you have finished
    /// initializing the rest of your peripherals but before you start
    /// issuing RPCs against the wifi chip.
    pub fn enable(&mut self, _cs: &CriticalSection, nvic: &mut NVIC) {
        self.transport.enable(nvic);
    }

    /// Connects to an access point and requests an IP address via DHCP.
    ///
    /// Firmware failures are returned as `erpc::Err::RPCErr(())`. Use
    /// [`Self::connect_to_ap_detailed`] to retain the operation and status.
    pub fn connect_to_ap<S: TryInto<heapless::String<64>>, P: TryInto<heapless::String<64>>>(
        &mut self,
        delay: &mut Delay,
        ssid: S,
        pw: P,
        security: erpc::Security,
    ) -> Result<erpc::IPInfo, erpc::Err<()>> {
        self.connect_to_ap_detailed(delay, ssid, pw, security)
            .map_err(WifiConnectError::into_legacy)
    }

    /// Connects to an access point and requests an IP address via DHCP.
    ///
    /// Returns an error identifying the failed operation or firmware status.
    /// The firmware's DHCP request blocks until a lease is assigned or it
    /// fails. Like [`Self::blocking_rpc`], this method has no host-side timeout.
    pub fn connect_to_ap_detailed<
        S: TryInto<heapless::String<64>>,
        P: TryInto<heapless::String<64>>,
    >(
        &mut self,
        delay: &mut Delay,
        ssid: S,
        pw: P,
        security: erpc::Security,
    ) -> Result<erpc::IPInfo, WifiConnectError> {
        connect::connect(self, delay, ssid, pw, security)
    }

    /// Returns the RTL8720's last connection-error flag.
    ///
    /// Read this immediately after an association failure; the firmware flag
    /// is diagnostic and may not identify the exact cause. Like
    /// [`Self::blocking_rpc`], this method has no host-side timeout.
    pub fn last_connection_error(&mut self) -> Result<i32, erpc::Err<()>> {
        self.blocking_rpc(diagnostics::GetLastError)
    }

    /// Create a non-blocking UDP socket bound to the given port.
    ///
    /// Sequences: `socket()` → `setsockopt(SO_REUSEADDR)` →
    /// `bind(INADDR_ANY, port)` → `fcntl(O_NONBLOCK)`.
    ///
    /// Returns the socket file descriptor on success.
    pub fn udp_bind(&mut self, port: u16) -> Result<i32, erpc::Err<()>> {
        let fd = self
            .blocking_rpc(rpcs::LwipSocket {
                domain: erpc::AF_INET,
                socket_type: erpc::SOCK_DGRAM,
                protocol: 0,
            })
            .map_err(|_| erpc::Err::RPCErr(()))?;
        if fd < 0 {
            return Err(erpc::Err::RPCErr(()));
        }

        let yes: i32 = 1;
        self.blocking_rpc(rpcs::LwipSetsockopt {
            socket: fd,
            level: erpc::SOL_SOCKET,
            optname: erpc::SO_REUSEADDR,
            optval: &yes.to_le_bytes(),
        })
        .map_err(|_| erpc::Err::RPCErr(()))?;

        let addr = erpc::encode_sockaddr_in([0, 0, 0, 0], port);
        let ret = self
            .blocking_rpc(rpcs::LwipBind {
                socket: fd,
                name: addr,
            })
            .map_err(|_| erpc::Err::RPCErr(()))?;
        if ret < 0 {
            let _ = self.blocking_rpc(rpcs::LwipClose { socket: fd });
            return Err(erpc::Err::RPCErr(()));
        }

        self.blocking_rpc(rpcs::LwipFcntl {
            socket: fd,
            cmd: erpc::F_SETFL,
            val: erpc::O_NONBLOCK,
        })
        .map_err(|_| erpc::Err::RPCErr(()))?;

        Ok(fd)
    }

    /// Attempt to receive a UDP datagram (non-blocking).
    ///
    /// Returns `Ok(Some(result))` with sender info and data if a
    /// packet was available, or `Ok(None)` if no data pending.
    pub fn udp_recvfrom(
        &mut self,
        socket: i32,
    ) -> Result<Option<erpc::RecvResult>, erpc::Err<()>> {
        match self.blocking_rpc(rpcs::LwipRecvfrom {
            socket,
            len: 256,
            flags: erpc::MSG_DONTWAIT,
            timeout_ms: 0,
        }) {
            Ok(result) => Ok(Some(result)),
            // EWOULDBLOCK = no data available → not an error
            Err(erpc::Err::RPCErr(_)) => Ok(None),
            Err(e) => Err(match e {
                erpc::Err::Parsing(p) => erpc::Err::Parsing(p),
                erpc::Err::CRCMismatch => erpc::Err::CRCMismatch,
                erpc::Err::TXErr => erpc::Err::TXErr,
                erpc::Err::NotOurs => erpc::Err::NotOurs,
                erpc::Err::ResponseOverrun => erpc::Err::ResponseOverrun,
                _ => erpc::Err::Unknown,
            }),
        }
    }

    /// Send a UDP datagram to a specific destination.
    ///
    /// Returns the number of bytes sent on success.
    pub fn udp_sendto(
        &mut self,
        socket: i32,
        data: &[u8],
        ip: [u8; 4],
        port: u16,
    ) -> Result<i32, erpc::Err<()>> {
        let dest = erpc::encode_sockaddr_in(ip, port);
        self.blocking_rpc(rpcs::LwipSendto {
            socket,
            data,
            flags: 0,
            to: dest,
        })
        .map_err(|_| erpc::Err::RPCErr(()))
    }

    /// Close a socket.
    pub fn udp_close(&mut self, socket: i32) -> Result<(), erpc::Err<()>> {
        self.blocking_rpc(rpcs::LwipClose { socket })
            .map_err(|_| erpc::Err::RPCErr(()))?;
        Ok(())
    }

    // ── TCP methods ─────────────────────────────────────────────────

    /// Create a TCP listening socket bound to the given port.
    ///
    /// The listener socket is left **blocking** so that `tcp_accept`
    /// will block inside `blocking_rpc` until a client connects.
    /// Use `tcp_set_nonblocking` on the *accepted* client socket
    /// for non-blocking recv polling.
    ///
    /// Sequence: `socket(SOCK_STREAM)` → `setsockopt(SO_REUSEADDR)` →
    /// `bind(INADDR_ANY, port)` → `listen(backlog)`.
    ///
    /// Returns the listener socket file descriptor on success.
    pub fn tcp_listen(&mut self, port: u16, backlog: i32) -> Result<i32, erpc::Err<()>> {
        let fd = self
            .blocking_rpc(rpcs::LwipSocket {
                domain: erpc::AF_INET,
                socket_type: erpc::SOCK_STREAM,
                protocol: 0,
            })
            .map_err(|_| erpc::Err::RPCErr(()))?;
        if fd < 0 {
            return Err(erpc::Err::RPCErr(()));
        }

        let yes: i32 = 1;
        self.blocking_rpc(rpcs::LwipSetsockopt {
            socket: fd,
            level: erpc::SOL_SOCKET,
            optname: erpc::SO_REUSEADDR,
            optval: &yes.to_le_bytes(),
        })
        .map_err(|_| erpc::Err::RPCErr(()))?;

        let addr = erpc::encode_sockaddr_in([0, 0, 0, 0], port);
        let ret = self
            .blocking_rpc(rpcs::LwipBind {
                socket: fd,
                name: addr,
            })
            .map_err(|_| erpc::Err::RPCErr(()))?;
        if ret < 0 {
            let _ = self.blocking_rpc(rpcs::LwipClose { socket: fd });
            return Err(erpc::Err::RPCErr(()));
        }

        let ret = self
            .blocking_rpc(rpcs::LwipListen {
                socket: fd,
                backlog,
            })
            .map_err(|_| erpc::Err::RPCErr(()))?;
        if ret < 0 {
            let _ = self.blocking_rpc(rpcs::LwipClose { socket: fd });
            return Err(erpc::Err::RPCErr(()));
        }

        Ok(fd)
    }

    /// Accept an incoming TCP connection (blocking).
    ///
    /// Blocks inside `blocking_rpc` until a client connects.
    /// Returns the new client fd and remote address on success.
    pub fn tcp_accept(
        &mut self,
        socket: i32,
    ) -> Result<erpc::AcceptResult, erpc::Err<()>> {
        self.blocking_rpc(rpcs::LwipAccept {
            socket,
        })
        .map_err(|_| erpc::Err::RPCErr(()))
    }

    /// Set a socket to non-blocking mode.
    ///
    /// Typically called on a client socket returned by `tcp_accept`
    /// so that `tcp_recv` can poll without blocking.
    pub fn tcp_set_nonblocking(&mut self, socket: i32) -> Result<(), erpc::Err<()>> {
        self.blocking_rpc(rpcs::LwipFcntl {
            socket,
            cmd: erpc::F_SETFL,
            val: erpc::O_NONBLOCK,
        })
        .map_err(|_| erpc::Err::RPCErr(()))?;
        Ok(())
    }

    /// Create a non-blocking TCP client socket connected to the given address.
    ///
    /// Sequence: `socket(SOCK_STREAM)` → `connect(addr)` → `fcntl(O_NONBLOCK)`.
    ///
    /// Returns the socket file descriptor on success.
    pub fn tcp_connect(&mut self, ip: [u8; 4], port: u16) -> Result<i32, erpc::Err<()>> {
        let fd = self
            .blocking_rpc(rpcs::LwipSocket {
                domain: erpc::AF_INET,
                socket_type: erpc::SOCK_STREAM,
                protocol: 0,
            })
            .map_err(|_| erpc::Err::RPCErr(()))?;
        if fd < 0 {
            return Err(erpc::Err::RPCErr(()));
        }

        let addr = erpc::encode_sockaddr_in(ip, port);
        let ret = self
            .blocking_rpc(rpcs::LwipConnect {
                socket: fd,
                name: addr,
            })
            .map_err(|_| erpc::Err::RPCErr(()))?;
        if ret < 0 {
            let _ = self.blocking_rpc(rpcs::LwipClose { socket: fd });
            return Err(erpc::Err::RPCErr(()));
        }

        self.blocking_rpc(rpcs::LwipFcntl {
            socket: fd,
            cmd: erpc::F_SETFL,
            val: erpc::O_NONBLOCK,
        })
        .map_err(|_| erpc::Err::RPCErr(()))?;

        Ok(fd)
    }

    /// Receive data from a connected TCP socket (non-blocking).
    ///
    /// Returns `Ok(Some(result))` with data if available,
    /// `Ok(None)` if no data pending (EWOULDBLOCK).
    pub fn tcp_recv(
        &mut self,
        socket: i32,
    ) -> Result<Option<erpc::TcpRecvResult>, erpc::Err<()>> {
        match self.blocking_rpc(rpcs::LwipRecv {
            socket,
            len: 256,
            flags: erpc::MSG_DONTWAIT,
            timeout_ms: 0,
        }) {
            Ok(result) => Ok(Some(result)),
            Err(erpc::Err::RPCErr(_)) => Ok(None),
            Err(e) => Err(match e {
                erpc::Err::Parsing(p) => erpc::Err::Parsing(p),
                erpc::Err::CRCMismatch => erpc::Err::CRCMismatch,
                erpc::Err::TXErr => erpc::Err::TXErr,
                erpc::Err::NotOurs => erpc::Err::NotOurs,
                erpc::Err::ResponseOverrun => erpc::Err::ResponseOverrun,
                _ => erpc::Err::Unknown,
            }),
        }
    }

    /// Send data on a connected TCP socket.
    ///
    /// Returns the number of bytes sent on success.
    pub fn tcp_send(&mut self, socket: i32, data: &[u8]) -> Result<i32, erpc::Err<()>> {
        self.blocking_rpc(rpcs::LwipSend {
            socket,
            data,
            flags: 0,
        })
        .map_err(|_| erpc::Err::RPCErr(()))
    }

    /// Close a TCP socket.
    pub fn tcp_close(&mut self, socket: i32) -> Result<(), erpc::Err<()>> {
        self.blocking_rpc(rpcs::LwipClose { socket })
            .map_err(|_| erpc::Err::RPCErr(()))?;
        Ok(())
    }

    /// Called from ISR: Handles the signal that the UART has recieved
    /// a byte that needs to be read.
    #[cfg(not(feature = "wifi-dma"))]
    pub fn _handle_rx(&mut self) {
        self.transport.handle_rx();
    }

    /// Called from ISR: Handles the signal that the outgoing UART buffer
    /// has room for the next byte.
    #[cfg(not(feature = "wifi-dma"))]
    pub fn _handle_data_empty(&mut self) {
        self.transport.handle_data_empty();
    }

    /// Issues an RPC, blocking till a response is recieved.
    pub fn blocking_rpc<'a, RPC: erpc::RPC>(
        &mut self,
        mut rpc: RPC,
    ) -> Result<RPC::ReturnValue, erpc::Err<RPC::Error>> {
        // Transmit the request.
        let mut tx_buff = heapless::Vec::new();
        tx_buff
            .extend_from_slice(&rpc.header(self.next_seq()).as_bytes())
            .map_err(|_| erpc::Err::TXErr)?;
        rpc.args(&mut tx_buff);
        self.write_frame(&tx_buff).map_err(|_| erpc::Err::TXErr)?;

        loop {
            let result = self.recieve_rpc_response(&mut rpc);
            if let Err(erpc::Err::NotOurs) = result {
                continue;
            };
            break result;
        }
    }

    fn recieve_rpc_response<'a, RPC: erpc::RPC>(
        &mut self,
        rpc: &mut RPC,
    ) -> Result<RPC::ReturnValue, erpc::Err<RPC::Error>> {
        // Read the frame header
        let fh = match self.recieve_frame_header(rpc) {
            Ok(fh) => fh,
            Err(e) => {
                self.transport.discard_pending();
                return Err(e);
            }
        };

        // Read the payload, check CRC, hand off to underlying trait to decode
        let mut buffer = [0u8; 2048];
        let sz = fh.msg_length as usize;
        if sz > buffer.len() {
            // A corrupt or oversized header; don't index out of range.
            self.transport.discard_pending();
            return Err(erpc::Err::ResponseOverrun);
        }
        if !self.read_ok(&mut buffer[..sz], INTER_BYTE_BUDGET) {
            self.transport.discard_pending();
            return Err(erpc::Err::Unknown);
        }

        let crc = fh.check_crc(&buffer[..sz]);
        if crc.is_err() {
            self.transport.discard_pending();
        }
        crc?;
        rpc.parse(&buffer[..sz])
    }

    fn recieve_frame_header<'a, RPC: erpc::RPC>(
        &mut self,
        _rpc: &mut RPC,
    ) -> Result<erpc::FrameHeader, erpc::Err<RPC::Error>> {
        let mut buffer = [0u8; 4];
        if !self.read_ok(&mut buffer, REPLY_WAIT_BUDGET) {
            self.transport.discard_pending();
            return Err(erpc::Err::Unknown);
        }

        match erpc::FrameHeader::parse(&buffer[..]) {
            Err(e) => Err(erpc::Err::Parsing(e)),
            Ok(fh) => Ok(fh.1),
        }
    }

    fn write_frame(&mut self, msg: &heapless::Vec<u8, 256>) -> Result<(), TransportError> {
        let header = erpc::FrameHeader::new_from_msg(msg);
        self.transport.write(&header.as_bytes())?;
        self.transport.write(msg)
    }

    /// Non-generic so the error path is not duplicated per RPC type.
    fn read_ok(&mut self, buffer: &mut [u8], first_byte_budget: u32) -> bool {
        self.transport.read_exact(buffer, first_byte_budget).is_ok()
    }

    fn next_seq(&mut self) -> u32 {
        self.sequence += 1;
        self.sequence
    }
}

/// Transport instrumentation, used by the `wifi_bench` example to compare the
/// interrupt-driven and DMA-driven UART backends.
///
/// Requires the DWT cycle counter to be running (see `wifi_bench`).
#[cfg(feature = "wifi-stats")]
pub mod stats {
    use core::sync::atomic::{AtomicU32, Ordering::Relaxed};
    use cortex_m::peripheral::DWT;

    static ISR_COUNT: AtomicU32 = AtomicU32::new(0);
    static ISR_CYCLES: AtomicU32 = AtomicU32::new(0);

    /// Counters accumulated since the last [`reset`].
    #[derive(Clone, Copy, Debug)]
    pub struct Snapshot {
        /// Number of wifi transport interrupt handler invocations.
        pub isr_count: u32,
        /// CPU cycles spent inside the handler bodies. Excludes exception
        /// entry/exit, which costs roughly another 25-30 cycles per invocation.
        pub isr_cycles: u32,
    }

    pub fn reset() {
        ISR_COUNT.store(0, Relaxed);
        ISR_CYCLES.store(0, Relaxed);
    }

    pub fn snapshot() -> Snapshot {
        Snapshot {
            isr_count: ISR_COUNT.load(Relaxed),
            isr_cycles: ISR_CYCLES.load(Relaxed),
        }
    }

    /// Accounts for one interrupt handler invocation; create it on entry to
    /// the handler and let it drop on exit.
    pub struct IsrGuard(u32);

    impl IsrGuard {
        #[inline(always)]
        pub fn enter() -> Self {
            Self(DWT::cycle_count())
        }
    }

    impl Drop for IsrGuard {
        #[inline(always)]
        fn drop(&mut self) {
            ISR_CYCLES.fetch_add(DWT::cycle_count().wrapping_sub(self.0), Relaxed);
            ISR_COUNT.fetch_add(1, Relaxed);
        }
    }
}

/// Imports necessary for using `wifi_singleton`.
pub mod wifi_prelude {
    pub use crate::wifi::*;
    pub use atsamd_hal::pac::Sercom0;
    pub use atsamd_hal::pac::{interrupt, Mclk};
    pub use bbqueue::{BBBuffer, Producer};

    pub use cortex_m::interrupt::CriticalSection;
}

/// Declares static globals for the wifi controller, and wires up interrupts.
#[cfg(not(feature = "wifi-dma"))]
#[macro_export]
macro_rules! wifi_singleton {
    ($global_name:ident) => {
        static mut $global_name: Option<Wifi> = None;
        static WIFI_RX: BBBuffer<512> = BBBuffer::new();
        static WIFI_TX: BBBuffer<128> = BBBuffer::new();

        /// Initializes the wifi controller from within an interrupt-free context.
        unsafe fn wifi_init(
            _cs: &CriticalSection,
            pins: WifiPins,
            sercom0: Sercom0,
            clocks: &mut GenericClockController,
            mclk: &mut Mclk,
            delay: &mut Delay,
        ) {
            unsafe {
                $global_name = Some(Wifi::init(
                    pins, sercom0, clocks, mclk, delay, &WIFI_RX, &WIFI_TX,
                ));
            }
        }

        #[interrupt]
        fn SERCOM0_0() {
            // Data Register Empty interrupt.
            unsafe {
                $global_name.as_mut().map(|wifi| {
                    wifi._handle_data_empty();
                });
            }
        }

        #[interrupt]
        fn SERCOM0_2() {
            // Recieve Complete interrupt.
            unsafe {
                $global_name.as_mut().map(|wifi| {
                    wifi._handle_rx();
                });
            }
        }
    };
}

/// Declares static globals for the wifi controller (DMA transport). Defines the
/// `DMAC_0` handler, which counts receive-ring wraps; no UART interrupts.
#[cfg(feature = "wifi-dma")]
#[macro_export]
macro_rules! wifi_singleton {
    ($global_name:ident) => {
        static mut $global_name: Option<Wifi> = None;
        static mut WIFI_RX_RING: [u8; RX_RING_LEN] = [0; RX_RING_LEN];

        /// Initializes the wifi controller from within an interrupt-free context.
        #[allow(clippy::too_many_arguments)]
        unsafe fn wifi_init(
            _cs: &CriticalSection,
            pins: WifiPins,
            sercom0: Sercom0,
            clocks: &mut GenericClockController,
            mclk: &mut Mclk,
            delay: &mut Delay,
            rx_channel: WifiRxChannel,
            tx_channel: WifiTxChannel,
        ) {
            unsafe {
                $global_name = Some(Wifi::init(
                    pins,
                    sercom0,
                    clocks,
                    mclk,
                    delay,
                    rx_channel,
                    tx_channel,
                    &mut *core::ptr::addr_of_mut!(WIFI_RX_RING),
                ));
            }
        }

        #[interrupt]
        fn DMAC_0() {
            // Receive ring wrapped (DMA channel 0 transfer complete).
            _handle_dma_rx_wrap();
        }
    };
}
