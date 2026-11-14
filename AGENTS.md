# AGENTS.md

Guidance for AI coding agents (Claude Code, GitHub Copilot, etc.) when working in this repository.

## What this is

A personal **fork of `atsamd-rs/atsamd`** (the Rust HAL, PACs and board support crates for
Microchip SAM D/E MCUs). Only two parts matter for this project:

- `hal/` — `atsamd-hal`, with added peripherals used for OpenTherm.
- `boards/wio_terminal/` — the Wio Terminal (ATSAMD51P19A) BSP, with Wi-Fi/UDP/SD changes.

It exists to serve `../rust_ot_wio`, the Rust OpenTherm boiler-controller firmware for the
Wio Terminal (part of a multi-repo project in `~/projects/rusty_boiler_workspace/`; goal:
OpenTherm heating control with Home Assistant). Everything else in the tree is upstream
code — leave other boards, PACs and unrelated HAL modules alone unless asked.

## How it is consumed — important

`../rust_ot_wio/Cargo.toml` declares `wio_terminal` and `atsamd-hal` as git dependencies on
`Cooler1989/atsamd_wio`, branch `udp_sockets_and_sdmmc_v23`, and then **`[patch]`es both
to this local checkout**. So:

- Edits here take effect in the firmware build immediately, committed or not.
- The firmware therefore depends on local, possibly unpushed commits (8 were ahead of
  `origin` on 2026-11-14). Don't switch branch, reset or stash here without checking what
  `rust_ot_wio` currently needs.
- `../rust_ot_wio/tests/host/lib.rs` compiles
  `boards/wio_terminal/src/wifi/asynchronous.rs` directly on the host (feature
  `network-driver-tests`). Keep that file free of HAL-only dependencies that would break a
  host build, and don't move it without updating that test crate.

Validate changes from the firmware side:

```bash
cd ../rust_ot_wio
cargo build --bin boiler_controller
cargo build --bin ot_proxy
cargo test --manifest-path tests/host/Cargo.toml --target x86_64-unknown-linux-gnu \
  --features network-mock,nvs,network-driver-tests network_
```

BSP examples can be built in place (`cd boards/wio_terminal && cargo build --example <name>`),
but that crate's runner is an old cgdb/OpenOCD setup and its `use_opentherm` default
feature pulls `opentherm_boiler_controller_lib` from an old Bitbucket feature branch over
SSH, so it may not resolve. Prefer exercising code through `rust_ot_wio`.

## Where the project-specific code is

| Path | What |
|---|---|
| `hal/src/peripherals/timer_capture_waveform/d5x.rs` | TC input capture + DMA: records OpenTherm edge timings (RX). Supports inverted capture edge (`set_capture_edge_inverted`, `DRVCTRL.INVEN0`). The capture future owns timer, pin and DMA channel; a bounded capture timeout is **not** implemented yet (`TODO(capture-timeout)` in `rust_ot_wio`) |
| `hal/src/peripherals/pwm_waveform/d5x.rs` | DMA-driven waveform output used for Manchester TX. Recent commits on this branch (idle values prepended to the DMA array to avoid a short first transfer, direct pin control before driving the level) come from the TX-glitch investigation documented in `../rust_ot_wio/doc/*glitch*.md`; read those before changing TX start-up |
| `boards/wio_terminal/src/wifi.rs`, `src/wifi/connect.rs`, `src/wifi/diagnostics.rs` | blocking eRPC Wi-Fi client for the RTL8720D coprocessor, connect layer with error diagnostics, UDP/TCP sockets |
| `boards/wio_terminal/src/wifi/asynchronous.rs` | async split-ownership Wi-Fi transport used by `rust_ot_wio`'s `NetworkManager` |
| `boards/wio_terminal/src/storage.rs` | SD/MMC support |
| `boards/wio_terminal/examples/async_opentherm.rs` | the original OpenTherm experiment on this BSP (superseded by `rust_ot_wio`) |
| `boards/wio_terminal/doc/wifi_udp_architecture.md` | design notes for the Wi-Fi/UDP stack |
| `development_documentation/` | local development notes |

The branch also differs from the fork's `master` in ADC, RTC, SERCOM SPI/DMA and clock
code. Those look like upstream `atsamd-hal` 0.23 changes rather than project work, but
that has not been checked commit by commit.

## Remotes

- `origin` / `priv_push` — the fork on GitHub (HTTPS and SSH).
- `upstream_github` — `atsamd-rs/atsamd`.

Working branch: `udp_sockets_and_sdmmc_v23`. Upstream conventions (formatting via
`rustfmt.sh` / `.rustfmt.toml`, release tooling) still apply to anything that might be
sent upstream.

## References

- `../opentherm_doc_generic/samd5x_datasheet/` — the SAM D5x datasheet as Markdown with a
  register index (`INDEX.md`, `REGISTER_INDEX.md`); use it for TC, DMAC, EIC, EVSYS and
  NVMCTRL questions instead of guessing register behaviour.
- `../opentherm_doc_generic/opentherm_design/` — the hardware-assisted two-bus proxy design
  these peripherals implement.
- The production C++ firmware (`~/projects/wio-viessmann-open-therm`,
  `lib/opentherm_samd51`) drives the same timer + DMA hardware and is known to work with
  the real boiler; compare against it when timing behaviour is in doubt.
