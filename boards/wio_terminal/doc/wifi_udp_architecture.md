# WiFi UDP Architecture — Wio Terminal (Rust)

This document describes the architecture of the WiFi UDP communication
stack on the Wio Terminal, focusing on how the Rust application on the
SAMD51 host MCU communicates with the RTL8720DN WiFi co-processor to
send and receive UDP datagrams.

## System Block Diagram

The Wio Terminal has a **dual-MCU architecture**: the main application
runs on an Atmel SAMD51 (ARM Cortex-M4F), while WiFi is handled by a
Realtek RTL8720DN co-processor connected via a UART-based eRPC link.

```mermaid
block-beta
  columns 3

  block:samd51["SAMD51 (Host MCU — Cortex-M4F)"]:2
    columns 1
    app["Rust Application\n(wifi_udp_echo.rs)"]
    wifi_api["wio_terminal::Wifi API\nudp_bind() · udp_recvfrom() · udp_sendto()"]
    erpc_client["seeed-erpc Crate\nRPC trait · codec · CRC16"]
    uart_drv["SERCOM0 UART Driver\nISR ring-buffers (512B RX / 128B TX)"]
  end

  block:rtl8720["RTL8720DN (WiFi Co-processor)"]:1
    columns 1
    erpc_server["eRPC Server\n(firmware v2.1.3+)"]
    lwip["LwIP Stack\nBSD Socket API"]
    wifi_radio["802.11 b/g/n Radio\n2.4 GHz + 5 GHz"]
  end

  uart_drv -- "UART @ 614400 baud" --> erpc_server

  style samd51 fill:#1a1a2e,color:#e0e0e0
  style rtl8720 fill:#16213e,color:#e0e0e0
```

### Data Flow Summary

| Direction | Path |
|-----------|------|
| **TX (send)** | App → `Wifi::udp_sendto()` → `LwipSendto` RPC → eRPC frame → UART TX → RTL8720 eRPC server → lwIP `sendto()` → WiFi radio → air |
| **RX (receive)** | Air → WiFi radio → lwIP buffer → RTL8720 eRPC server → UART RX → eRPC frame decode → `LwipRecvfrom` RPC → `Wifi::udp_recvfrom()` → App |

---

## eRPC Protocol Layer

All communication between SAMD51 and RTL8720DN uses the **eRPC
(Embedded RPC) protocol** over UART:

```mermaid
block-beta
  columns 1

  block:frame["eRPC Frame (on wire)"]
    columns 4
    fh["Frame Header\n(4 bytes)\nmsg_length + CRC16"]
    rh["RPC Header\n(8 bytes)\nservice + request + type + seq"]
    args["Arguments\n(variable)\nbinary_t / scalars"]
    pad[" "]
  end

  style frame fill:#0f3460,color:#e0e0e0
```

| Field | Size | Description |
|-------|------|-------------|
| Frame Header | 4 B | `u16 msg_length` + `u16 CRC16` of the payload |
| RPC Header | 8 B | Codec version (1), Service ID, Request ID, MsgType, Sequence |
| Arguments | variable | RPC-specific, little-endian encoded |

### eRPC Services Used

| Service | ID | Purpose |
|---------|----|---------|
| System | 1 | `GetVersion` — firmware version query |
| Wifi | 14 | `WifiOn`, `WifiConnect`, `ScanStart`, `GetMacAddress`, … |
| TCPIP | 15 | `AdapterInit`, `DHCPClientStart/Stop`, `GetIPInfo` |
| **LwIP** | **16** | **BSD socket API: `socket`, `bind`, `sendto`, `recvfrom`, `close`, `fcntl`, `setsockopt`** |

---

## Sequence Diagram: Startup & Socket Setup

This diagram shows the full initialization sequence from power-on to
the UDP echo server being ready to receive packets.

```mermaid
sequenceDiagram
    participant App as Rust App<br/>(SAMD51)
    participant Wifi as Wifi struct<br/>(wio_terminal)
    participant RTL as RTL8720DN<br/>(eRPC Server)
    participant Net as WiFi Network<br/>(AP + DHCP)

    Note over App,RTL: Phase 1 — Hardware Init
    App->>Wifi: Wifi::init() — reset RTL8720, configure UART
    Wifi-->>RTL: UART @ 614400 baud established
    App->>Wifi: wifi.enable() — unmask SERCOM0 IRQs

    Note over App,RTL: Phase 2 — Firmware Check
    App->>Wifi: blocking_rpc(GetVersion)
    Wifi->>RTL: [Service=1, Fn=1] GetVersion
    RTL-->>Wifi: "2.1.3"
    App->>Wifi: blocking_rpc(GetMacAddress)
    Wifi->>RTL: [Service=14, Fn=8] GetMacAddress
    RTL-->>Wifi: "AA:BB:CC:DD:EE:FF"

    Note over App,Net: Phase 3 — WiFi Association (connect_to_ap)
    App->>Wifi: connect_to_ap(ssid, pass, WPA2_AES)
    Wifi->>RTL: [S=15] AdapterInit
    RTL-->>Wifi: OK
    Wifi->>RTL: [S=15] DHCPClientStop
    RTL-->>Wifi: OK
    Wifi->>RTL: [S=14] WifiOff
    RTL-->>Wifi: OK
    Wifi->>RTL: [S=14] WifiOn(Station)
    RTL-->>Wifi: OK
    Wifi->>RTL: [S=14] WifiConnect(ssid, pass, WPA2)
    RTL->>Net: 802.11 Association + WPA2 handshake
    Net-->>RTL: Associated
    RTL-->>Wifi: OK
    Wifi->>RTL: [S=15] DHCPClientStart
    RTL->>Net: DHCP Discover → Offer → Request → Ack
    Net-->>RTL: IP=192.168.1.x
    RTL-->>Wifi: OK
    Wifi->>RTL: [S=15] GetIPInfo
    RTL-->>Wifi: {ip, netmask, gateway}
    Wifi-->>App: IPInfo

    Note over App,RTL: Phase 4 — UDP Socket Setup (udp_bind)
    App->>Wifi: udp_bind(4000)
    Wifi->>RTL: [S=16, Fn=18] socket(AF_INET, SOCK_DGRAM, 0)
    RTL-->>Wifi: fd=3
    Wifi->>RTL: [S=16, Fn=7] setsockopt(fd, SO_REUSEADDR, 1)
    RTL-->>Wifi: OK
    Wifi->>RTL: [S=16, Fn=2] bind(fd, 0.0.0.0:4000)
    RTL-->>Wifi: OK
    Wifi->>RTL: [S=16, Fn=23] fcntl(fd, F_SETFL, O_NONBLOCK)
    RTL-->>Wifi: OK
    Wifi-->>App: fd=3

    Note over App: ✓ UDP echo server ready on :4000
```

---

## Sequence Diagram: UDP Echo Round-Trip

This diagram shows a single UDP echo exchange — a remote PC sends a
datagram to the Wio Terminal and receives the same data back.

```mermaid
sequenceDiagram
    participant PC as Remote PC<br/>(192.168.1.10)
    participant Net as WiFi Network
    participant RTL as RTL8720DN<br/>(LwIP stack)
    participant Wifi as Wifi struct
    participant App as Rust App

    Note over PC,App: Echo request
    PC->>Net: UDP 192.168.1.10:54321 → 192.168.1.100:4000<br/>payload: "hello wio"
    Net->>RTL: 802.11 frame → lwIP reassembly
    RTL->>RTL: lwIP buffers datagram

    Note over App,RTL: App polls for data
    App->>Wifi: udp_recvfrom(fd)
    Wifi->>RTL: [S=16, Fn=14] recvfrom(fd, 256, MSG_DONTWAIT)
    RTL-->>Wifi: data="hello wio", from=192.168.1.10:54321
    Wifi-->>App: RecvResult { data, remote_ip, remote_port }

    Note over App: Display sender + data on LCD

    Note over App,RTL: Echo response
    App->>Wifi: udp_sendto(fd, "hello wio", 192.168.1.10, 54321)
    Wifi->>RTL: [S=16, Fn=17] sendto(fd, "hello wio", 192.168.1.10:54321)
    RTL->>Net: 802.11 frame
    Net->>PC: UDP 192.168.1.100:4000 → 192.168.1.10:54321<br/>payload: "hello wio"

    Note over PC: ✓ Echo received
```

---

## Software Layer Map

```
┌─────────────────────────────────────────────────────────────┐
│                    wifi_udp_echo.rs                          │  Application
│         configure, connect, bind, poll loop                  │
├─────────────────────────────────────────────────────────────┤
│              wio_terminal::Wifi                              │  Board Support
│    connect_to_ap()  udp_bind()  udp_recvfrom()              │
│    udp_sendto()     udp_close()  blocking_rpc()             │
├─────────────────────────────────────────────────────────────┤
│              seeed-erpc (v0.2.0)                             │  eRPC Codec
│    RPC trait  ·  codec::Header  ·  FrameHeader              │
│    ────────────────────────────────────────────              │
│    wifi_rpcs:   GetVersion, WifiConnect, ScanStart, …       │
│    tcpip_rpcs:  AdapterInit, DHCPClientStart, GetIPInfo     │
│    lwip_rpcs:   LwipSocket, LwipBind, LwipRecvfrom,        │
│                 LwipSendto, LwipClose, LwipFcntl, …        │
├─────────────────────────────────────────────────────────────┤
│              SERCOM0 UART (614400 baud)                      │  Transport
│    ISR-driven ring buffers  ·  512B RX  ·  128B TX          │
╞═════════════════════════════════════════════════════════════╡
│              RTL8720DN Firmware (v2.1.3)                      │  Co-processor
│    eRPC server  ·  lwIP  ·  802.11 b/g/n radio             │
└─────────────────────────────────────────────────────────────┘
```

---

## C++ Reference (wio-viessmann-open-therm)

The Rust implementation mirrors the C++ WiFi UDP gateway from the
`wio-viessmann-open-therm` project:

| C++ Component | Rust Equivalent |
|---------------|-----------------|
| `WiFiUDP::begin(port)` | `Wifi::udp_bind(port)` |
| `WiFiUDP::parsePacket()` + `read()` | `Wifi::udp_recvfrom(fd)` |
| `WiFiUDP::beginPacket()` + `write()` + `endPacket()` | `Wifi::udp_sendto(fd, data, ip, port)` |
| `WiFiUDP::stop()` | `Wifi::udp_close(fd)` |
| `WiFi.begin(ssid, pass)` | `Wifi::connect_to_ap(ssid, pass, security)` |

The key difference is that the C++ Arduino library wraps the BSD
socket calls behind a `WiFiUDP` class that manages buffers internally,
while the Rust API exposes individual socket operations directly,
matching the underlying eRPC service 16 (LwIP) calls.
