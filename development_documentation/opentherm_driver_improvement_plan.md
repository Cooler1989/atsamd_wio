# OpenTherm driver improvement plan (atsamd_wio)

Goal: make the two OpenTherm HAL drivers robust and clean without changing how the bus
behaves on the wire.

- RX: `hal/src/peripherals/timer_capture_waveform/d5x.rs` (module `timer_capture_waveform`)
- TX: `hal/src/peripherals/pwm_waveform/d5x.rs` (module `pwm_wg`)
- Only consumer: `../rust_ot_wio/src/platform/ot_hw/` (`driver.rs`, `driver_impl.rs`,
  `device_set.rs`) plus the examples in `../rust_ot_wio/examples/`.

The plan is a list of small steps. Do **one step per session and one commit per step**.
Steps are ordered; do not skip ahead unless a step says it is independent.

## Rules for every step

1. Read `AGENTS.md` in this repo and in `../rust_ot_wio` first.
2. Touch only the files named in the step. Both repos have unrelated uncommitted work
   (Wi-Fi files here, network/MQTT files in `rust_ot_wio`): never stage, revert or
   reformat those. Stage files by name, never `git add -A`.
3. Do not run `rustfmt` on whole files before step 5.5; it would bury the real change.
4. Refer to code by symbol name. Line numbers move after every step.
5. Register behaviour comes from the datasheet in
   `../opentherm_doc_generic/samd5x_datasheet/` (`ch48-tc-timer-counter-*.md`, DMAC and
   EIC chapters). Do not guess; quote the section you relied on in the commit message.
6. Do not change these without an explicit step saying so. They come from the TX glitch
   investigation (`../rust_ot_wio/doc/pwm_glitch_root_cause_analysis.md`,
   `thermostat_tx_glitch_analysis.md`):
   - `DMA_IDLE_HEAD`, `DMA_IDLE_TAIL` and the head/tail layout of the DMA pattern
   - "arm DMA first, then start the timer" order in `send_waveform`
   - `COUNT` preloaded with `PER` in `PwmWaveformGenerator::new`
   - the `after_timer_start` callback (the consumer switches the pin to the TC function there)
   - DMA destination `CCBUF1` (not `CC1`)
7. Keep the ownership-transfer API of the consumer (`self` in, `(Self, Result)` out).
8. If a step turns out to need a change outside its listed files, or the "Done when"
   check fails for a reason you cannot explain, stop and report instead of improvising.

## Check commands

Run from `../rust_ot_wio`. This set is called **CHECK** below.

```bash
cargo build --bin boiler_controller
cargo build --bin ot_proxy
cargo build --example hardware_loopback
cargo build --example hardware_loopback_event_test
cargo build --example thermostat_tester
cargo test --manifest-path tests/host/Cargo.toml --target x86_64-unknown-linux-gnu \
  --features network-mock,nvs,network-driver-tests
```

Nothing here runs on hardware. Steps marked **[HW]** change timing or start-up behaviour
and are only finished once the user has run the loopback examples on the board (phase 6).
Say so in the step report; do not claim hardware behaviour was verified.

## Findings this plan is based on

Capture (RX) driver:

| # | Problem | Where |
|---|---|---|
| R1 | `intflag().modify(...)` is used to clear `MC0` / `MC1`. INTFLAG is write-one-to-clear, so `modify` writes back every flag that is currently set and clears it too. An `MC1` timeout that arrives while `MC0` is being cleared is lost. | `TimerCaptureInterruptHandler::on_interrupt` |
| R2 | The timeout flag is checked before the waker is registered. An interrupt between the two is a lost wake-up and the capture can stay pending. | `TimerCaptureDmaWrapper::poll` |
| R3 | A DMA error (`Poll::Ready(Err(_))`) is not handled: the code falls through and returns `Pending`. The next poll re-polls a finished `async fn`, which panics. The outer `Err` branch also replaces any error with `TransferError`. | `TimerCaptureDmaWrapper::poll`, `start_timer_execute_dma_transfer` |
| R4 | No `Drop` on the wrapper future. If the capture future is dropped, DMA stops but the timer keeps running and the TC NVIC interrupt stays enabled. | `TimerCaptureDmaWrapper` |
| R5 | `COUNT` is read right after `CMD = READSYNC` without waiting for synchronisation (3 ISRs and `counter_value`). Needs checking against the datasheet. | ISRs, `counter_value` |
| R6 | Tick-to-time conversion is the magic constant `x * 4173 * 2 / 1000` ns (about 119.8 MHz). `clock_freq` is stored but not used for it, while the timeouts *are* computed from `clock_freq`. | `get_public_data` |
| R7 | First-edge window (20 s) and inactivity gap (5 ms) are compile-time constants. The consumer's `timeout_till_active_capture` argument is ignored. | `CAPTURE_ACTIVE_WINDOW_MS`, `CAPTURE_INACTIVITY_MS` |
| R8 | 32-bit mode pairs TCn with TCn+1, but the code enables the `tc3` and `tc5` bus clocks for every instance, and the macro also instantiates odd TCs, which cannot be the master of a 32-bit pair. | `new_timer_capture_inner`, `enable_mclk_clocks`, macro invocations |
| R9 | End of captured data is found by scanning for a zero in the buffer. The counter value at termination is only appended when the buffer is completely full, so in the normal case the last interval is an edge-to-edge gap although the consumer labels it `RisingToStop`. | `ExtendWithFirstZeroAndTerminationValueIterator` |
| R10 | Two copy-pasted EIC handlers with hard-coded EXTINT0→TC2 and EXTINT8→TC4. | `TimerCaptureEventInputInterruptHandler`, `...Handler8` |
| R11 | Dead and placeholder code: `TimerTraitToDo`, `InterruptHandlerAsdf`, `TimerCounterTimeoutInterruptHandler`, `TimerCounterInterrupt`, `TimerSpecificInterruptAndRegisters`, `TimerFailed`, `is_mc1_interrupt_active`, `start_capture_timer`, `GetDmaPtr`, `ehal::pwm::ErrorType` impl, unused `_freq`, large commented-out blocks, `#![allow(non_snake_case)]`, d11/d21 `RegBlock` aliases in a d5x-only file. | whole file |
| R12 | The module is not feature-gated but uses `ReadyFuture` (async only) and DMA. | top of file |

Waveform (TX) driver:

| # | Problem | Where |
|---|---|---|
| T1 | A pattern longer than the buffer is silently truncated; the idle tail can be cut off by `.min(N)`. | `send_waveform` |
| T2 | If the `send_waveform` future is dropped mid-frame, DMA stops but `CCBUF1` keeps the last frame value, so the output may stay at the non-idle level. | `send_waveform` |
| T3 | Period is the constant `TIMER_PERIOD = 233` with `/256`; `clock_freq` and `freq` are ignored. The module docs say "60 MHz, ~1 kHz", but the half-bit rate must be 2 kHz, which matches a 120 MHz TC clock. Docs and code disagree. | `new`, module docs |
| T4 | `get_period`, `set_period`, `SetDutyCycle`, `PwmPin` were copied from the plain PWM driver: they treat `CC0` as TOP (TOP is `PER` here) and use the prescaler enum bits as a divisor. They are wrong for this driver. | bottom of macro |
| T5 | `COUNT` / `PER` writes in `new` do not wait for `SYNCBUSY`. Needs checking against the datasheet. | `new` |
| T6 | Unused `_ccx_value` parameter, `after_timer_start.take().unwrap()`, leftover TODOs. | `send_waveform` |

---

## Phase 0 — Baseline

### 0.1 Record what builds today
- Run CHECK. Do not fix anything.
- Report which commands pass. Commands that already fail are excluded from CHECK for the
  rest of the plan; list them in the report so later steps know.

## Phase 1 — Safety net (no behaviour change)

Pure logic is moved into files without HAL dependencies so it can be tested on the host,
the same way `tests/host/lib.rs` already includes `wifi/asynchronous.rs` by `#[path]`.

### 1.1 Extract capture conversion into `convert.rs`
- New file `hal/src/peripherals/timer_capture_waveform/convert.rs`, `core` only (no
  `crate::` imports).
- Move `CounterValueAtTermination` and `ExtendWithFirstZeroAndTerminationValueIterator`
  into it unchanged. Add one function that holds the duplicated body of the two match arms
  in `get_public_data`:
  `pub fn extend_with_intervals<C: Extend<Duration>>(container: &mut C, raw: &[u32], end: CounterValueAtTermination, max_items: usize)`.
  Keep the `4173 * 2 / 1000` ratio exactly as is for now.
- In `d5x.rs` add `#[path = "convert.rs"] mod convert;` and call the function from both
  arms. If the path does not resolve, try `mod convert;` and report which form worked.
- Done when: CHECK passes; `git diff` shows moved code only.

### 1.2 Host tests for `convert.rs`
- Files: `../rust_ot_wio/tests/host/lib.rs` (add a `#[path]` include behind a new feature
  `ot-driver-tests`), `../rust_ot_wio/tests/host/Cargo.toml` (the feature), one new test
  file next to the existing host tests.
- Tests pin **current** behaviour, including the odd parts: empty buffer; partly filled
  buffer (zero sentinel ends the data, termination value not emitted); completely full
  buffer (termination value emitted); a 32-bit counter wrap between two samples;
  `max_items` truncation.
- Done when: the new tests pass and CHECK passes. Do not "fix" behaviour in this step.

### 1.3 Extract TX pattern building into `pattern.rs` and test it
- New file `hal/src/peripherals/pwm_waveform/pattern.rs`, `core` only. Move
  `SIGNAL_HIGH`, `SIGNAL_LOW`, `DMA_IDLE_HEAD`, `DMA_IDLE_TAIL` and the buffer-filling
  loop of `send_waveform` into
  `pub fn build_pattern(buf: &mut [u8], bits: impl Iterator<Item = bool>, invert: bool, idle: u8) -> PatternLayout`
  where `PatternLayout { frame_len, total_len, head }`. Same truncation behaviour as now.
- `send_waveform` calls it; nothing else in that function changes.
- Add host tests (same mechanism as 1.2): head is 4 idle items, tail is 2 idle items,
  `invert` flips levels, over-long input is truncated.
- Done when: CHECK passes and tests pass.

## Phase 2 — Capture correctness

Each step is a few lines. All are in `timer_capture_waveform/d5x.rs` unless noted.

### 2.1 Clear interrupt flags with `write`, not `modify` (R1)
- In `TimerCaptureInterruptHandler::on_interrupt` read `intflag` once into a local, then
  clear with `intflag().write(|w| w.mc0().set_bit())` and
  `intflag().write(|w| w.mc1().set_bit())`. Decide `mc0` / `mc1` handling from the local.
- Done when: no `intflag().modify` remains in the file; CHECK passes.

### 2.2 Register the waker before checking the timeout flag (R2)
- In `TimerCaptureDmaWrapper::poll`, call `WAKERS[idx].register(cx.waker())` before
  loading `MC1_INTERRUPT_FIRED`. Remove the two stale TODO comments about it.
- Done when: CHECK passes.

### 2.3 Handle DMA errors (R3)
- In `poll`, `match` on the DMA poll result. `Ready(Ok(()))` as today.
  `Ready(Err(e))`: stop the timer, disable the interrupt, return
  `Ready(Err(TimerCaptureFailure::DmaFailed(e)))`. `Pending`: continue.
- In `start_timer_execute_dma_transfer` return the wrapper's error unchanged instead of
  constructing `DmacError::TransferError`.
- Done when: the future can never be polled again after the DMA future returned `Ready`;
  CHECK passes.

### 2.4 Stop the timer when the capture future is dropped (R4)
- Add a `Drop` impl (use `#[pinned_drop]` from `pin_project`) to
  `TimerCaptureDmaWrapper`: if `timer_started`, call `stop()` and `disable_interrupt()`.
  Both are idempotent, so the normal completion paths may keep their explicit calls or
  rely on the drop; pick one and make it consistent.
- Done when: CHECK passes. Add a doc comment on the struct saying dropping it is safe.

### 2.5 Synchronised COUNT read (R5) **[HW]**
- Read ch48 on reading `COUNT` (`CTRLBSET.CMD = READSYNC`, `SYNCBUSY`). Write one private
  helper, e.g. `fn read_count_sync(count: &Count32) -> u32`, that does exactly what the
  datasheet requires, and use it in the three ISRs and in `counter_value`.
- Done when: the READSYNC sequence exists in one place only; commit message quotes the
  datasheet section; CHECK passes.

### 2.6 Derive tick duration from `clock_freq` (R6) **[HW]**
- First find the value the consumer passes as `periph_clock_freq`
  (`../rust_ot_wio/src/platform/ot_hw/driver_impl.rs` and its callers) and report it.
- Change `extend_with_intervals` to take the tick frequency in Hz and compute
  `ns = ticks * 1_000_000_000 / hz` in `u64`. Pass `clock_freq` from the driver.
- Update the host tests: at the real clock frequency a 1 ms gap must convert to
  1 ms ± 0.5 %. If the real frequency is not about 119.8 MHz, stop and report: the old
  constant and the real clock then disagree and the user must decide.
- Done when: no `4173` in the tree; tests and CHECK pass.

### 2.7 Correct clock enabling for the 32-bit pair (R8)
- Add the partner TC's mask register and bit to the macro arguments (TC0→TC1, TC2→TC3,
  TC4→TC5, TC6→TC7) and enable exactly those two clocks. Remove the hard-coded `tc3_` /
  `tc5_` lines from both places; make `new_timer_capture_inner` call
  `enable_mclk_clocks`.
- Remove the macro invocations for TC1, TC3, TC5, TC7 after confirming with `grep` that
  nothing in `../rust_ot_wio` or `boards/wio_terminal` uses `TimerCapture1/3/5/7`.
- Done when: CHECK passes.

## Phase 3 — Capture timeouts and end-of-frame

### 3.1 Runtime timeouts in the HAL (R7)
- Add `pub struct CaptureTimeouts { pub first_edge: Duration, pub inactivity: Duration }`
  and take it as a parameter of `start_timer_execute_dma_transfer`. Inside, before the
  timer starts: convert both to ticks with `u64` maths, clamp to `u32::MAX`, write the
  first-edge value to `CC1` (wait for `SYNCBUSY.CC1`) and store the inactivity value in
  `CAPTURE_INACTIVITY_TICKS`.
- Keep the old constants as `CaptureTimeouts::DEFAULT` (20 s / 5 ms).
- Consumer change in the same commit: `driver_impl.rs` passes `CaptureTimeouts::DEFAULT`.
  No behaviour change yet.
- Done when: CHECK passes.

### 3.2 Use the caller's timeouts in the consumer **[HW]**
- File: `../rust_ot_wio/src/platform/ot_hw/driver_impl.rs`, `start_capture`.
- Pass `timeout_till_active_capture` as `first_edge` and `timeout_inactive_capture` as
  `inactivity`. First `grep` every caller of `start_capture` in `../embassyot`,
  `../boiler_controller_ot_lib` and `rust_ot_wio` and list the values they pass; if any
  caller passes zero or a placeholder, keep the default for that field and report it.
- Do not edit the `TODO(capture-timeout)` notes or the README section yet; that is 6.2.
- Done when: CHECK passes; the report lists the timeout values now in effect per binary.

### 3.3 One implementation for the EIC event-input handlers (R10)
- Generate both handlers from one small macro taking (type name, EXTINT number, EIC
  interrupt type, TC field). Keep the public names
  `TimerCaptureEventInputInterruptHandler` and `TimerCaptureEventInputInterruptHandler8`
  so `../rust_ot_wio/src/platform/macros.rs` and the examples keep compiling.
- Use the helper from 2.5 for the counter read.
- Done when: CHECK passes; the handler body exists once.

### 3.4 Decide the end-of-frame semantics (R9) — needs a user decision
- Do not implement. Write a short note in the step report with the two options:
  (a) keep today's behaviour and fix the docs in `driver_impl.rs` that call the last item
  a real `RisingToStop`; (b) always append the termination interval and adjust the
  consumer's slicing. Include what the host tests from 1.2 show. The user picks.

## Phase 4 — Waveform (TX) robustness

### 4.1 Report an over-long pattern instead of truncating (T1)
- `build_pattern` returns `Result<PatternLayout, PatternTooLong>`; it fails when
  head + frame + tail does not fit. Add
  `pub enum WaveformError { PatternTooLong, Dma(DmacError) }` and make `send_waveform`
  return it. Nothing is transmitted when the pattern does not fit.
- Consumer: `driver_impl.rs` already maps any error to `TriggerError::GenericError`;
  adjust the types only. Check with `grep` that the buffer size `N` used by every binary
  and example is at least 4 + 68 + 2 half-bits (34 bits per frame, 2 half-bits each).
- Update the host tests from 1.3.
- Done when: CHECK passes.

### 4.2 Return to idle when `send_waveform` is cancelled (T2) **[HW]**
- Add a small guard struct created right before the DMA future is first polled. On drop,
  unless disarmed after normal completion, it writes `init_level` to `CCBUF1` (the same
  buffered write `set_idle_level` uses while the timer runs). Do not stop the timer and
  do not write `CC1` directly.
- Done when: CHECK passes; the normal path is byte-for-byte the same register sequence.

### 4.3 Remove the wrong copied PWM methods (T4)
- `grep` for uses of `get_period`, `set_period`, `set_duty_cycle`, `PwmPin`,
  `get_dma_ptr_from_tc` on `PwmWg*` types in `../rust_ot_wio` and `boards/wio_terminal`.
  Delete every one that is unused, with its trait impl. Report any that are used.
- Done when: CHECK passes.

### 4.4 Compute the period from the clock (T3) **[HW]**
- Add a `core`-only function in `pattern.rs`:
  `pub fn half_bit_timer_params(clock_hz: u32) -> Option<(Prescaler, u8 /*PER*/)>` that
  picks the smallest prescaler for which `PER` fits in 8 bits for a 2 kHz update rate.
  Host test: for the clock found in 2.6 it must return `/256` and `233`.
- Use it in `PwmWaveformGenerator::new`; if it returns `None`, keep `/256` and `233`.
- Fix the module docs: 2 kHz item rate (500 µs half-bit), the real clock frequency.
- Done when: test and CHECK pass, and the values programmed for the real clock are
  unchanged (`/256`, `233`).

### 4.5 SYNCBUSY handling in `new` (T5) **[HW]**
- Check ch48 for which of `COUNT`, `PER`, `PERBUF`, `CCx` are write-synchronised in 8-bit
  mode and add the missing `SYNCBUSY` waits. Use `SYNCBUSY.SWRST` for the reset wait.
- Done when: CHECK passes; commit message quotes the datasheet section.

### 4.6 Small API cleanup (T6)
- Remove the unused `_ccx_value` parameter of `send_waveform` and the unused `_freq`
  parameter of `PwmWaveformGenerator::new` (trait, impl, consumer, examples). Replace
  `after_timer_start.take().unwrap()` with `if let Some(f) = ...take() { f() }`. Remove
  TODOs that earlier steps resolved.
- Done when: CHECK passes.

## Phase 5 — Cleanup (no behaviour change)

### 5.1 Remove dead code from the capture driver (R11, part 1)
- Delete, after a `grep` in both repos shows no use: `TimerTraitToDo`,
  `InterruptHandlerAsdf`, `TimerCounterTimeoutInterruptHandler`, `TimerCounterInterrupt`,
  `TimerSpecificInterruptAndRegisters`, `TimerCaptureFailure::TimerFailed`,
  `is_mc1_interrupt_active`, `start_capture_timer`, `GetDmaPtr`, the
  `ehal::pwm::ErrorType` impl, the `TIMER_TIMEOUT_CHANNEL` constant if unused, the d11 and
  d21 `RegBlock` aliases, the `$clock` and `$wrapper` macro arguments if unused, and all
  commented-out code blocks.
- Then remove `#![allow(non_snake_case)]` and fix what the compiler reports.
- Done when: CHECK passes with no new warnings from these two modules.

### 5.2 Naming and structure in the capture driver (R11, part 2)
- Rename the field `base_pwm` to `capture` and `_channel` to `channel`; drop leading
  underscores from fields that are used. Replace `extract_number` (parsing the TC name
  string) with an explicit index argument in the macro. Remove the unused `_freq`
  parameter from `new_timer_capture*` (trait, impl, consumer, examples).
- Done when: CHECK passes.

### 5.3 One result type instead of two parallel enums
- Replace `TimerCaptureResultAvailable::{DmaPollReady, TimerTimeout}` (and the raw twin)
  with `pub struct CaptureResult<C> { pub data: C, pub ended_by: CaptureEnd }` and
  `pub enum CaptureEnd { BufferFull, Timeout }`. Update the single use in
  `driver_impl.rs` (`timer_timed_out` becomes `ended_by == CaptureEnd::Timeout`).
- Done when: CHECK passes.

### 5.4 Feature gating (R12)
- Gate the whole `timer_capture_waveform` module and the DMA parts it needs on
  `all(feature = "dma", feature = "async")`, the way `pwm_wg` gates its async API.
- Done when: CHECK passes, and `cargo check` of `hal/` with the Wio Terminal chip feature
  but without `async` gets past these two modules (report other, unrelated errors as is).

### 5.5 Documentation and formatting
- Write module docs for the capture driver in the style of the `pwm_wg` module docs:
  what TC mode is used, the two capture sources (TC pin, EIC/EVSYS event input), how the
  two timeouts work with `CC1`, cancellation safety, a short usage example.
- Run `rustfmt` on the two driver directories only.
- Done when: CHECK passes; diff contains docs and formatting only.

## Phase 6 — Hardware validation and wrap-up (user at the bench)

### 6.1 Loopback and real-device runs
- User flashes `hardware_loopback`, `hardware_loopback_event_test`, then
  `boiler_controller` against the simulator or boiler. Things to look at: frames decode
  in both directions; no TX start glitch on the scope; with the bus unplugged the capture
  returns a timeout after the configured first-edge time and recovers on reconnect.

### 6.2 Update the notes
- Only after 6.1 passes: update the "capture timeout not implemented" sentences in
  `AGENTS.md` (both repos), the `TODO(capture-timeout)` comments and the "Deferred:
  capture-timeout recovery" section of `../rust_ot_wio/README.md` to describe what is now
  true. State only what 6.1 actually showed.

## Not in this plan (needs design work, not a small step)

- Reading the real number of captured edges from the DMA channel (remaining beat count)
  instead of the zero sentinel. Needs a new accessor in `hal/src/dmac`.
- One half-duplex OpenTherm driver that owns the TC for both directions, instead of the
  consumer decomposing and re-creating (with a software reset) on every direction change.
- Trying the TC period/pulse-width capture mode (PPW/PWP) mentioned in the old TODO.
