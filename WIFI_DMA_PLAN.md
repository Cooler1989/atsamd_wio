# WiFi over DMA for the Wio Terminal BSP — plan

Scope: the eRPC-over-UART link between the SAMD51 and the RTL8720 in
`atsamd/boards/wio_terminal` (`src/wifi.rs`), using `seeed-erpc` for the protocol.

Status of this document: design and estimates. Nothing here has been run on
hardware — the J-Link on this machine was held by another process, so the
benchmark has only been compiled. Numbers are marked **measured** (from builds
on this machine) or **estimated**.

---

## 1. Verdict

| Question | Answer |
|---|---|
| Is DMA worth doing? | **Yes, for robustness — not for speed.** |
| Faster command exchange? | No. Expect no measurable change in RPC round-trip time. |
| What does improve? | RX can no longer lose bytes when the CPU is busy; the two per-byte interrupt handlers disappear. |
| Effort | About 1.5–2 working days including hardware bring-up; roughly 300 new lines in the BSP. |
| Resource cost | Flash about +50 bytes, static RAM about +1.9 kB (**measured** on a spike). Not significant. |
| Feature-gated? | **Yes**, as `wifi-dma`, off by default — because it changes the init API and claims two DMA channels, not because of size. |
| Changes to `seeed-erpc-rs` | None required. |
| Changes to `atsamd-hal` | None required; the published 0.21.3 the BSP already uses is enough. |

### Why not speed

Round-trip time is wire time plus the RTL8720's turnaround. The UART runs at
614 400 baud (61 440 bytes/s, 16.3 µs per byte) whichever way bytes are moved
off it. A `GetVersion` exchange is a 12-byte request and a reply of a few tens
of bytes; the interrupt driver spends about 1 µs of CPU per byte (**estimated**
from 80 and 63 instructions in the RX and TX handlers plus exception
entry/exit, at 120 MHz), so roughly 50 µs per RPC against a round trip of
milliseconds. During a sustained reply the handlers cost about 6 % CPU, and
`blocking_rpc` busy-waits for the reply anyway, so nothing else was going to
use that time.

### Why robustness

The interrupt driver has a hard real-time deadline on every received byte and
panics when it is missed:

- The SERCOM holds about two received bytes before a third overruns, so the RX
  handler must run within roughly 33–49 µs of a byte arriving (**estimated**
  from the datasheet's two-level receive buffer). With the older RTL8720
  firmware at 1 843 200 baud (`wifi-fw-before-212`) the window is a third of
  that.
- On overrun `_handle_rx` hits `panic!("unrecoverable read error")`. If the
  512-byte queue fills it hits `panic!("overrun")`.
- The wifi interrupts cannot preempt anything. `Wifi::enable` sets priority
  `1`, but the SAMD51 implements only the top three priority bits
  (`NVIC_PRIO_BITS = 3`), so `1` is stored as `0`. The `clock` example sets the
  USB interrupts to `1` as well. Any other handler, or any critical section,
  longer than the window above takes the board down mid-RPC.

With DMA the hardware moves each byte into RAM with no CPU involvement. The
deadline becomes "drain the ring before it wraps": 2048 bytes is 33 ms of
continuous traffic, and the largest reply today is about 1 kB.

The same change is the prerequisite for anything heavier later — socket data
RPCs carrying 1.4 kB payloads, or an async API — where 61 000 interrupts per
second would start to matter.

### Interrupts or DMA for the command path

Evaluated per direction:

| Direction | Choice | Reason |
|---|---|---|
| RX | Circular DMA | This is where the benefit is. |
| TX | Blocking DMA write | No performance benefit for 12–68 byte commands. Chosen because it is *less* code than the queue + DRE interrupt it replaces: one `write_all` call. |
| Errors | Polled status register | The blocking receive loop polls anyway, so an error interrupt adds nothing. |

The DMA backend therefore uses **no interrupts at all**. If a second DMA
channel is ever too expensive, TX can fall back to polled byte writes
(`nb::block!(uart.write(b))`) with no other change.

---

## 2. Current design

`atsamd/boards/wio_terminal/src/wifi.rs`, feature `wifi`:

```
blocking_rpc ── tx() ──> BBBuffer<128> ──> SERCOM0_0 (DRE) ISR ──> UART TX   1 interrupt per byte
             <─ recieve_bytes() <── BBBuffer<512> <── SERCOM0_2 (RXC) ISR <── UART RX   1 interrupt per byte
```

- `Wifi` owns the `Uart` (SERCOM0, IoSet2) and both halves of two `bbqueue`
  buffers. `wifi_singleton!` declares the statics and the two handlers.
- `blocking_rpc` builds the frame (4-byte frame header, 8-byte RPC header,
  args ≤ 64 bytes), queues it, then spins on the RX queue: 4 header bytes,
  then `msg_length` bytes into a 2048-byte stack buffer, CRC check, parse.
  Replies that are not ours (`Err::NotOurs`, i.e. callbacks) are skipped.
- Existing weaknesses that the work below touches: the two panics above; a
  corrupt `msg_length` larger than 2048 panics on a slice index; a missing
  reply blocks forever.

---

## 3. Target design

```
blocking_rpc ── write_all(frame) ──> DMA ch1 (one-shot) ──> UART TX     blocks ~1 ms for 68 bytes
             <─ read_exact() <── static [u8; 2048] ring <── DMA ch0 (circular) <── UART RX
                     │
                     └─ polls: DMA write position, SERCOM STATUS error bits
```

### RX: circular DMA into a ring

- The UART is split with `uart.split()`. The RX half and a
  `&'static mut [u8; 2048]` go into `dmac::Transfer::new(ch, rx, buf, true)`
  (`true` = circular: the descriptor links to itself), started with
  `.begin(Sercom0::DMA_RX_TRIGGER, TriggerAction::Burst)`. It runs forever.
- The consumer keeps a read index. The write index is `N - remaining`, where
  `remaining` is the channel's live beat counter:
  - if `DMAC.ACTIVE` says this channel is the active one (`ABUSY` set and `ID`
    matches), take `ACTIVE.BTCNT`;
  - otherwise read `BTCNT` from the channel's write-back descriptor, at
    `DMAC.WRBADDR + 16 * channel + 2`.

  This is the method Zephyr's SAM0 DMA driver uses for the same purpose (from
  memory — see risk R1). The HAL keeps the write-back array private, so the BSP
  reads it through the PAC; no HAL change is needed.
- Bytes are read from the ring with `read_volatile` through a raw pointer taken
  before the buffer is handed to the transfer.
- Errors: each poll checks `SERCOM0.USART.STATUS` for `BUFOVF`/`FERR`, clears
  them, and returns an error instead of panicking. On any framing, CRC or
  length error the reader resynchronises by discarding everything pending
  (`read = write`).

### TX: blocking DMA write

- The TX half gets a channel with `tx.with_tx_channel(ch)`; the HAL then
  implements `embedded_io::Write` as "start a DMA transfer of this slice and
  spin until done".
- The frame header and message are assembled into one buffer and sent with a
  single `write_all`.

### API

Unchanged for RPC callers: `blocking_rpc`, `connect_to_ap`, `wifi_singleton!`.

Changed under `wifi-dma`:

- `wifi_init` (from the macro) and `Wifi::init` take two initialised DMA
  channels instead of the two `BBBuffer`s. The application creates the
  `DmaController`, splits it, and keeps the other channels.
- `Wifi::enable` stays for source compatibility and does nothing.
- No `SERCOM0_0` / `SERCOM0_2` handlers are defined.

### Types (verified to compile against atsamd-hal 0.21.3 with the `dma` feature)

```rust
use atsamd_hal::dmac::{self, Busy, BufferPair, Ch0, Ch1, ChId, Channel, Ready, Transfer, TriggerAction};
use atsamd_hal::sercom::uart::{self, Config, RxDuplex, TxDuplex, Uart};
use atsamd_hal::sercom::{Sercom, Sercom0};
use atsamd_hal::typelevel::NoneT;        // NOT uart::NoneT, that one is private
use atsamd_hal::embedded_io::Write;

pub const RX_RING_LEN: usize = 2048;
pub type WifiRxChannel = Channel<Ch0, Ready>;
pub type WifiTxChannel = Channel<Ch1, Ready>;
type RxHalf = Uart<Config<WifiUartPads>, RxDuplex>;
type TxHalf = Uart<Config<WifiUartPads>, TxDuplex, NoneT, WifiTxChannel>;
type RxTransfer = Transfer<Channel<Ch0, Busy>, BufferPair<RxHalf, &'static mut [u8; RX_RING_LEN]>>;
const RX_CHANNEL: usize = <Ch0 as ChId>::USIZE;
```

Start-up:

```rust
let (rx, tx) = uart.split();                       // uart is the enabled Duplex Uart
let tx: TxHalf = tx.with_tx_channel(tx_channel);
let ring_ptr: *const u8 = ring.as_ptr();           // ring: &'static mut [u8; RX_RING_LEN]
let rx: RxTransfer = Transfer::new(rx_channel, rx, ring, true)
    .unwrap()
    .begin(<Sercom0 as Sercom>::DMA_RX_TRIGGER, TriggerAction::Burst);
```

Write position:

```rust
fn write_index() -> usize {
    let dmac = unsafe { pac::Dmac::steal() };
    let active = dmac.active().read();
    let remaining = if active.abusy().bit_is_set() && active.id().bits() as usize == RX_CHANNEL {
        active.btcnt().bits()
    } else {
        let writeback = dmac.wrbaddr().read().bits() as *const u8;
        unsafe { core::ptr::read_volatile(writeback.add(RX_CHANNEL * 16 + 2) as *const u16) }
    } as usize;
    // Before the first byte the write-back entry is still zero.
    if remaining == 0 || remaining > RX_RING_LEN { 0 } else { RX_RING_LEN - remaining }
}
```

RX error check:

```rust
fn take_rx_error() -> bool {
    let usart = unsafe { pac::Sercom0::steal() };
    let status = usart.usart_int().status().read();
    let failed = status.bufovf().bit_is_set() || status.ferr().bit_is_set();
    if failed {
        usart.usart_int().status().write(|w| w.bufovf().set_bit().ferr().set_bit());
    }
    failed
}
```

---

## 4. Resource impact

**Measured**, release profile (`opt-level = "s"`, LTO), minimal program doing
one `GetVersion` RPC with each transport:

| | Interrupt (today) | DMA spike | Delta |
|---|---|---|---|
| Flash (`text`) | 3276 B | 3348 B | +72 B |
| Static RAM (`bss`) | 708 B | 2576 B | +1868 B |
| of which RX buffer | 532 B (`BBBuffer<512>`) | 2048 B (ring) | |
| of which TX buffer | 148 B (`BBBuffer<128>`) | 0 | |
| of which DMAC descriptors | 0 | 512 B (2 × 16 channels × 16 B) | |

(The first, simpler version of the spike was +52 B; +72 B includes the error
polling and named types shown above.)

- The device has 512 kB flash and 192 kB RAM, so this is under 1 % of RAM and
  negligible flash. A 1024-byte ring would cut the RAM delta to about 850 B and
  is still larger than the biggest reply; 2048 is recommended for margin.
- Enabling `atsamd-hal/dma` alone costs nothing (**measured**: the interrupt
  build is byte-identical with the HAL feature on).
- Hardware claimed: DMA channels 0 and 1. Interrupt vectors `SERCOM0_0` and
  `SERCOM0_2` are freed.

Conclusion: size is not a reason to gate it. It is gated anyway because the
init signature changes, an application may already own the DMAC, and the
default path should stay on the code that has been run on hardware until the
benchmark confirms the new one.

---

## 5. Benchmark

Added in this change set; builds today against the interrupt driver.

- `atsamd/boards/wio_terminal/examples/wifi_bench.rs`
- `wifi-stats` feature: counts transport interrupt handler invocations and the
  CPU cycles spent in them (`wio_terminal::wifi_stats`). Off by default; the
  handlers are untouched without it.

```
cd atsamd/boards/wio_terminal
cargo build --release --example wifi_bench --features wifi-stats            # interrupt
cargo build --release --example wifi_bench --features wifi-stats,wifi-dma   # DMA, after phase 2
```

Results appear on the LCD. The first line shows the backend (`irq`/`dma`),
firmware version and baud rate.

| Test | Workload | What to compare |
|---|---|---|
| 1 | `GetVersion` × 200 | min/avg/max µs — expect equal. `isr/rpc` — expect about 50 → 0. |
| 2 | `ScanGetAP<U16>` × 20 after one scan (≈ 1 kB reply) | same; `isr cpu` expect about 5 % → 0 %. |
| 3 | Both workloads while a 1 kHz timer interrupt spins for 0, 10, 20, 30, 50, 100, 200, 400 µs | Each step prints `N..` then `ok` or `E<errors>`. Interrupt driver: expected to freeze (panic) somewhere around 30–50 µs, leaving a label with no result. DMA: expected `ok` on every step. |

Test 3 is the one that shows the difference. Record the full screen for both
backends in the PR description.

---

## 6. Risks

| # | Risk | Likelihood | Impact | Mitigation |
|---|---|---|---|---|
| R1 | The write position cannot be read reliably: the write-back descriptor may not be refreshed after every byte on SAMD51, or `ACTIVE`/write-back may be read inconsistently. Recalled from Zephyr's driver, not verified here. | Medium | Blocks the RX design | Phase 2 starts with a gate test that proves or disproves it before anything else is built. Fallbacks, in order: (a) read twice until two consecutive values agree; (b) briefly suspend the channel (`CHCTRLB.CMD = SUSPEND`), which forces a write-back, read, resume; (c) hybrid — receive the 4-byte frame header by RXC interrupt, then start a one-shot DMA for exactly `msg_length` bytes. |
| R2 | Not tested on hardware. All timing and tolerance figures are estimates. | Certain | Estimates may be off | Phase 0 records the interrupt baseline before any driver change. |
| R3 | Ring overwritten if nobody drains it — e.g. the RTL8720 sends more than 2 kB of unsolicited callbacks while the application is idle. | Low | One RPC fails with a CRC/parse error, then resync | Strictly better than today's panic. Optional hardening: count wraps with the channel's transfer-complete interrupt. |
| R4 | A reply that never arrives, or a corrupt length, blocks or panics. Pre-existing, but more visible once RX errors are returned instead of panicking. | Medium | Hang | Phase 4: length bound check (`Err::ResponseOverrun` already exists) and a timeout. |
| R5 | Reading the ring through a raw pointer while the `Transfer` holds `&'static mut` to it is outside what Rust's aliasing rules strictly allow. | Low | Theoretical miscompile | Only ever use raw pointers and volatile reads, never form a reference. The HAL's own DMA design has the same property. |
| R6 | BSP is pinned to crates.io `atsamd-hal` 0.21.x; the in-tree HAL is 0.23.4 with a changed PAC API. | Low | Rework when the BSP is bumped | Only two PAC accessors are used (`Dmac::active/wrbaddr`, `Sercom0::usart_int().status()`); isolate them in two small functions. |
| R7 | API break for users who enable `wifi-dma`. | Certain, by design | Low | Opt-in feature; examples show the new init. |
| R8 | The stall test in `wifi_bench` halts on the interrupt backend's panic, so later steps do not run. | Certain | Cosmetic | Stall test runs last; label is printed before each step. |

---

## 7. Effort

| Phase | Work | Estimate |
|---|---|---|
| 0 | Baseline measurement on hardware | 0.5 h |
| 1 | Refactor: isolate the transport, no behaviour change | 2–3 h |
| 2 | DMA transport behind `wifi-dma` | 4–6 h, most of it bring-up on hardware |
| 3 | Examples, benchmark run, docs, changelog | 1–2 h |
| 4 | Hardening (optional) | 2–3 h |
| | **Total** | **≈ 1.5–2 days**, about 300 lines |

If R1 forces fallback (c), add about half a day.

---

## 8. Implementation phases

Written to be executed one phase at a time. Each phase ends in a state that
builds and can be committed. Do not start a phase before the previous one's
checks pass.

Conventions for every phase:

- Work in `atsamd/boards/wio_terminal`. Do not edit `atsamd/hal` or
  `seeed-erpc-rs`.
- Build commands are run from `atsamd/boards/wio_terminal`.
- The standard build check is both of:
  ```
  cargo build --release --example wifi_connect --example wifi_scan
  cargo build --release --example wifi_bench --features wifi-stats
  ```
  Building the BSP *without* the `wifi` feature already fails before this work
  (`src/pins.rs` imports `super::wifi` unconditionally). That is out of scope;
  do not use a no-wifi build as a check.
- Steps marked **[hardware]** need a Wio Terminal and a person to read the
  LCD. If no hardware is available, stop and report; do not skip them.
- `src/lib.rs` has `#![allow(warnings)]`, so the compiler will not flag unused
  code in the BSP. Check for leftovers by hand.

### Phase 0 — Baseline **[hardware]**

1. Flash `wifi_bench` built with `--features wifi-stats` (interrupt backend).
2. Record every line on the LCD, including which stall step it freezes on.
3. Save the result in `atsamd/boards/wio_terminal/examples/README.md` under
   the `wifi_bench` entry, as a small table headed "interrupt backend".

Done when: the three sets of numbers are recorded.

### Phase 1 — Isolate the transport (no behaviour change)

Goal: `Wifi` no longer touches the UART or the queues directly, so a second
transport can be dropped in.

1. Turn `src/wifi.rs` into a directory module: move it to `src/wifi/mod.rs`.
2. Create `src/wifi/irq.rs` containing a `pub struct Transport` that owns what
   `Wifi` owns today for I/O: `uart: WifiUart` and the four `bbqueue` halves.
   Move these into it, keeping the bodies unchanged:

   | New `Transport` method | Comes from |
   |---|---|
   | `new(uart, rx_buff: &'static BBBuffer<512>, tx_buff: &'static BBBuffer<128>) -> Self` | the `try_split` calls and `uart` field from `Wifi::init` |
   | `enable(&mut self, nvic: &mut NVIC)` | body of `Wifi::enable` |
   | `handle_rx(&mut self)` | `Wifi::_handle_rx` (keep the `wifi-stats` guard line) |
   | `handle_data_empty(&mut self)` | `Wifi::_handle_data_empty` (keep the guard line) |
   | `write(&mut self, frame: &[u8]) -> Result<(), TransportError>` | `Wifi::tx`, taking a slice instead of an iterator |
   | `read_exact(&mut self, buffer: &mut [u8]) -> Result<(), TransportError>` | `Wifi::recieve_bytes`; always returns `Ok` here |

   Define `pub enum TransportError { Rx }` in `mod.rs`.
3. In `mod.rs`: `Wifi` becomes `{ _pwr, transport: Transport, sequence }`.
   - `Wifi::init` builds the UART and power-cycles the RTL8720 exactly as now,
     then calls `Transport::new`.
   - `_handle_rx` and `_handle_data_empty` stay as public one-line forwards
     (the macro calls them).
   - `write_frame` builds one `heapless::Vec<u8, 68>` (frame header + message)
     and calls `transport.write`.
   - `recieve_frame_header` / `recieve_rpc_response` call
     `transport.read_exact(..)` and map `TransportError` to `erpc::Err::Unknown`.
   - The `stats` module stays in `mod.rs`.
4. `lib.rs` needs no change (`mod wifi;` resolves to the directory).

Checks:
- Standard build check passes.
- `arm-none-eabi-size` of `wifi_connect` is within ±200 bytes of 14324/708
  (`text`/`bss`).
- **[hardware]** `wifi_bench` numbers match phase 0 within noise.

**Status: done, hardware check pending the LCD readout.**

Findings:

- **Size.** The plan's refactor as written costs +1236 bytes of `text`
  (14324 -> 15560), outside the ±200 limit. The baseline was reproduced
  exactly (14324/708) by rebuilding the original `wifi.rs`, so the growth is
  from the refactor, not the environment. Two causes:
  1. Building one `Vec<u8, 68>` for header + message (~400 bytes). Fixed by
     calling `transport.write` twice (header, then message).
  2. `read_exact(..).map_err(|_| erpc::Err::Unknown)?` inside the generic
     `recieve_*` functions (~840 bytes): the error path is duplicated for
     every RPC type. Fixed by a small non-generic `Wifi::read_ok` helper that
     returns a `bool`; the generic code only does
     `if !ok { return Err(erpc::Err::Unknown) }`. Do not mark it
     `#[inline(never)]`: that measured larger (14580 vs 14488).
  Final: `wifi_connect` 14488 text / 708 bss (+164 bytes).
- Phase 2 should reuse that pattern: keep error handling out of the generic
  RPC functions.
- `write_frame` therefore does not build a single `Vec<u8, 68>`; deviation
  from step 3 of this phase, intentional.
- `src/wifi.rs` -> `src/wifi/mod.rs` + `src/wifi/irq.rs`; `Transport` and
  `TransportError` as specified. No unused imports left in the wifi module
  (checked with warnings temporarily enabled).
- Builds: standard check passes, `wifi_bench --features wifi-stats` passes,
  and `--features wifi-stats,bootloader` passes.

### Phase 2 — DMA transport behind `wifi-dma`

**Step 2.0 — gate test for risk R1 [hardware].** Before writing the transport,
add a temporary example `examples/wifi_dma_probe.rs` that sets up the UART and
DMA exactly as in section 3 ("Start-up"), sends one `GetVersion` frame, then
for 100 ms repeatedly prints `write_index()` on the LCD and finally the first
bytes of the ring in hex. Build it with `--features atsamd-hal/dma`.

- Pass: `write_index()` goes from 0 to the reply length and stays there, and
  the ring starts with a plausible 4-byte frame header (length matches).
- Fail: stop, report what was observed, and apply the fallbacks from R1 in
  order. Do not continue with the steps below until one of them passes.

Delete the probe example when done.

**Step 2.1 — feature.** In `Cargo.toml`:

```toml
# move the wifi UART from per-byte interrupts to DMA (uses DMA channels 0 and 1)
wifi-dma = ["wifi", "atsamd-hal/dma"]
```

**Step 2.2 — transport.** Create `src/wifi/dma.rs` with a `pub struct
Transport { _rx: RxTransfer, tx: TxHalf, ring: *const u8, read: usize }` using
the types and the three code blocks in section 3.

| Method | Behaviour |
|---|---|
| `new(uart: WifiUart, rx_channel: WifiRxChannel, tx_channel: WifiTxChannel, ring: &'static mut [u8; RX_RING_LEN]) -> Self` | "Start-up" block; `read = 0`. |
| `enable(&mut self, _nvic: &mut NVIC)` | Empty. |
| `write(&mut self, frame: &[u8]) -> Result<(), TransportError>` | `self.tx.write_all(frame)`; map the error to `TransportError::Rx` (rename the variant to something neutral if preferred). |
| `read_exact(&mut self, buffer: &mut [u8]) -> Result<(), TransportError>` | For each output byte: spin while `write_index() == self.read`, checking `take_rx_error()` in the loop and returning `Err` if set; then `read_volatile(self.ring.add(self.read))` and advance `read` modulo `RX_RING_LEN`. |
| `discard_pending(&mut self)` | `self.read = write_index()`. |

`Transport` contains a raw pointer, so add `unsafe impl Send for Transport {}`
with a comment that the pointer refers to a `'static` buffer.

**Step 2.3 — select the transport.** In `mod.rs`:

```rust
#[cfg(not(feature = "wifi-dma"))] mod irq;
#[cfg(not(feature = "wifi-dma"))] use irq::Transport;
#[cfg(feature = "wifi-dma")] mod dma;
#[cfg(feature = "wifi-dma")] use dma::Transport;
#[cfg(feature = "wifi-dma")] pub use dma::{WifiRxChannel, WifiTxChannel, RX_RING_LEN};
```

- Give `Wifi::init` two cfg variants that differ only in their last two
  parameters (`rx_buff`/`tx_buff` vs. `rx_channel`/`tx_channel`/`rx_ring`).
- Add an empty `discard_pending` to the interrupt transport. In
  `recieve_rpc_response`, call `self.transport.discard_pending()` before
  returning a frame-header parse error, a CRC error, or a transport error.
- Make `_handle_rx` / `_handle_data_empty` exist only without `wifi-dma`.

**Step 2.4 — macro.** Provide two `wifi_singleton!` definitions, each under
the matching `#[cfg]` *on the `macro_rules!` item* (not inside its body — a
`cfg` inside the body would be evaluated in the user's crate). The DMA one:

- declares `static mut $global_name: Option<Wifi>` and
  `static mut WIFI_RX_RING: [u8; RX_RING_LEN] = [0; RX_RING_LEN];`
- `wifi_init` takes two extra arguments `rx_channel: WifiRxChannel,
  tx_channel: WifiTxChannel` after `delay`, and passes
  `&mut *core::ptr::addr_of_mut!(WIFI_RX_RING)` as the ring;
- defines no interrupt handlers.

Add `WifiRxChannel`, `WifiTxChannel`, `RX_RING_LEN` to `wifi_prelude` under
the feature.

**Step 2.5 — examples.** In `wifi_bench.rs`, `wifi_connect.rs` and
`wifi_scan.rs`, wrap the init block so that with `wifi-dma` it first does:

```rust
let mut dmac = wio::hal::dmac::DmaController::init(peripherals.dmac, &mut peripherals.pm);
let channels = dmac.split();
let rx_channel = channels.0.init(wio::hal::dmac::PriorityLevel::Lvl3);
let tx_channel = channels.1.init(wio::hal::dmac::PriorityLevel::Lvl0);
```

and passes both to `wifi_init`. Use `#[cfg(feature = "wifi-dma")]` /
`#[cfg(not(...))]` on two small blocks; keep everything else shared. Remove the
`#![allow(unexpected_cfgs)]` line and its comment from `wifi_bench.rs`.

Checks:
- Standard build check passes (interrupt backend unchanged).
- The same example builds pass with `wifi-dma` added to the features.
- `arm-none-eabi-size`: interrupt `wifi_connect` unchanged from phase 1; DMA
  `wifi_connect` `bss` is about 1.9 kB larger.
- **[hardware]** `wifi_connect` with `wifi-dma` obtains an IP address.
- **[hardware]** `wifi_scan` with `wifi-dma` lists access points repeatedly
  for at least five minutes.

### Phase 3 — Measure and document **[hardware]**

1. Run `wifi_bench` with `--features wifi-stats,wifi-dma`; record the screen
   next to the phase 0 table in `examples/README.md`.
2. Acceptance:
   - tests 1 and 2: average round trip within ±5 % of the interrupt backend;
     `isr/rpc` is 0;
   - test 3: `ok` on every step up to 400 µs.
   If test 3 fails, report the failing step and stop; do not tune the test.
3. Document in `boards/wio_terminal/README.md` (wifi section): what
   `wifi-dma` does, that it needs DMA channels 0 and 1, and the changed
   `wifi_init` call.
4. Add a `CHANGELOG.md` entry.
5. Add the `wifi-dma` build of the wifi examples to whatever the workspace CI
   uses for this board (see `atsamd/crates.json` and `build-all.py`).

### Phase 4 — Hardening (optional, separate PR)

Applies to both transports.

1. In `recieve_rpc_response`, if `msg_length > 2048` call `discard_pending()`
   and return `erpc::Err::ResponseOverrun` instead of indexing out of range.
2. Replace the two panics in the interrupt transport's `handle_rx` with a
   sticky error flag that `read_exact` returns as `TransportError`.
3. Timeout: give `read_exact` a spin budget derived from the CPU frequency
   (a few seconds) and return a transport error when it runs out, so a silent
   RTL8720 cannot hang the caller. Mapping this to a dedicated error needs a
   new `seeed_erpc::Err` variant; until that exists use `Err::Unknown`.
4. Ring overrun detection for DMA: enable the RX channel's transfer-complete
   interrupt, count wraps in a `DMAC_0` handler, and have `read_exact` compare
   the count against its own position.

Checks: standard build check for both feature sets; **[hardware]** `wifi_bench`
results unchanged on both backends.

---

## 9. What exists now

Changed in `atsamd/boards/wio_terminal` (uncommitted):

| File | Change |
|---|---|
| `examples/wifi_bench.rs` | New benchmark (section 5). |
| `src/wifi/mod.rs`, `src/wifi/irq.rs` | Phase 1 split (was `src/wifi.rs`). `stats` module and one guard line in each interrupt handler, all under `wifi-stats`. |
| `src/lib.rs` | Re-export `wifi_stats` under the feature. |
| `Cargo.toml` | `bootloader` feature (see below); `wifi-stats` feature; `wifi_bench` example entry. |
| `examples/README.md` | Entry for `wifi_bench`. |

Dev setup changes made in Phase 0:

- `.cargo/config.toml` runner is now `probe-rs run --chip ATSAMD51P19A --protocol swd` (J-Link). The old gdb runner is a comment.
- The board's UF2 bootloader (first 16K) was found erased, so firmware is linked at 0x0 by default. `memory-nobootloader.x` is that layout; `memory-bootloader.x` is the old 16K-offset one, selected by `--features bootloader` in `build.rs`. Production builds must use `bootloader`, and the bootloader has to be restored first.
- Phase 0 baseline is in `examples/README.md`.

Verified here: `wifi_bench`, `wifi_connect` and `wifi_scan` build in release;
a throwaway DMA spike using exactly the types and code blocks of section 3
builds against `atsamd-hal` 0.21.3 with `dma`. Not verified: anything on a
device.
