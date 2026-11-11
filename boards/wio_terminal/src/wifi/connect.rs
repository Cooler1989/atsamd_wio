use core::fmt;

use heapless::String;
use seeed_erpc::{self as erpc, rpcs};

pub const DHCP_ALREADY_STOPPED: i32 = 0x5005; // ESP_ERR_TCPIP_ADAPTER_DHCP_ALREADY_STOPPED

/// A firmware operation performed while connecting to an access point.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WifiConnectStage {
    AdapterInit,
    DhcpStop,
    WifiOff,
    WifiOn,
    Association,
    DhcpStart,
}

impl fmt::Display for WifiConnectStage {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::AdapterInit => "AdapterInit",
            Self::DhcpStop => "DHCPClientStop",
            Self::WifiOff => "WifiOff",
            Self::WifiOn => "WifiOn",
            Self::Association => "WifiConnect",
            Self::DhcpStart => "DHCPClientStart",
        })
    }
}

/// A connection failure, including the firmware status when available.
#[derive(Debug, Clone, PartialEq)]
pub enum WifiConnectError {
    /// The SSID could not be converted to a string of at most 64 bytes.
    InvalidSsid,
    /// The password could not be converted to a string of at most 64 bytes.
    InvalidPassword,
    /// An eRPC failed before a firmware status was returned.
    Rpc {
        stage: WifiConnectStage,
        source: erpc::Err<()>,
    },
    /// The firmware returned an unsuccessful status.
    FirmwareStatus {
        stage: WifiConnectStage,
        status: i32,
    },
    /// Reading the assigned IP configuration failed.
    IpInfo(erpc::Err<i32>),
    /// DHCP reported success, but the returned address was 0.0.0.0.
    UnspecifiedIp,
}

impl fmt::Display for WifiConnectError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidSsid => f.write_str("SSID conversion failed"),
            Self::InvalidPassword => f.write_str("password conversion failed"),
            Self::Rpc { stage, source } => write!(f, "{} RPC: {:?}", stage, source),
            Self::FirmwareStatus { stage, status } => {
                write!(f, "{} status: {}", stage, status)
            }
            Self::IpInfo(source) => write!(f, "GetIPInfo RPC: {:?}", source),
            Self::UnspecifiedIp => f.write_str("DHCP returned 0.0.0.0"),
        }
    }
}

impl WifiConnectError {
    pub(super) fn into_legacy(self) -> erpc::Err<()> {
        match self {
            Self::InvalidSsid | Self::InvalidPassword => erpc::Err::Unknown,
            Self::Rpc { source, .. } => source,
            Self::FirmwareStatus { .. } | Self::UnspecifiedIp => erpc::Err::RPCErr(()),
            Self::IpInfo(source) => match source {
                erpc::Err::Parsing(error) => erpc::Err::Parsing(error),
                erpc::Err::CRCMismatch => erpc::Err::CRCMismatch,
                erpc::Err::TXErr => erpc::Err::TXErr,
                erpc::Err::NotOurs => erpc::Err::NotOurs,
                erpc::Err::RPCErr(_) => erpc::Err::RPCErr(()),
                erpc::Err::ResponseOverrun => erpc::Err::ResponseOverrun,
                erpc::Err::Unknown => erpc::Err::Unknown,
            },
        }
    }
}

pub(super) trait Transport {
    fn call<R: erpc::RPC>(&mut self, rpc: R) -> Result<R::ReturnValue, erpc::Err<R::Error>>;
}

pub(super) trait DelayMs {
    fn wait_ms(&mut self, ms: u8);
}

fn call<T: Transport, R: erpc::RPC<Error = ()>>(
    transport: &mut T,
    stage: WifiConnectStage,
    rpc: R,
) -> Result<R::ReturnValue, WifiConnectError> {
    transport
        .call(rpc)
        .map_err(|source| WifiConnectError::Rpc { stage, source })
}

fn check_status<T: Transport, R: erpc::RPC<ReturnValue = i32, Error = ()>>(
    transport: &mut T,
    stage: WifiConnectStage,
    rpc: R,
    accepted: &[i32],
) -> Result<(), WifiConnectError> {
    let status = call(transport, stage, rpc)?;
    if accepted.contains(&status) {
        Ok(())
    } else {
        Err(WifiConnectError::FirmwareStatus { stage, status })
    }
}

pub(super) fn connect<T, D, S, P>(
    transport: &mut T,
    delay: &mut D,
    ssid: S,
    password: P,
    security: erpc::Security,
) -> Result<erpc::IPInfo, WifiConnectError>
where
    T: Transport,
    D: DelayMs,
    S: TryInto<String<64>>,
    P: TryInto<String<64>>,
{
    let ssid = ssid.try_into().map_err(|_| WifiConnectError::InvalidSsid)?;
    let password = password
        .try_into()
        .map_err(|_| WifiConnectError::InvalidPassword)?;

    call(
        transport,
        WifiConnectStage::AdapterInit,
        rpcs::AdapterInit {},
    )?;
    check_status(
        transport,
        WifiConnectStage::DhcpStop,
        rpcs::DHCPClientStop {
            interface: erpc::L3Interface::Station,
        },
        &[0, DHCP_ALREADY_STOPPED],
    )?;
    check_status(transport, WifiConnectStage::WifiOff, rpcs::WifiOff {}, &[0])?;
    delay.wait_ms(35);
    // The RTL8720 returns 1 if the WLAN is already running.
    check_status(
        transport,
        WifiConnectStage::WifiOn,
        rpcs::WifiOn {
            mode: erpc::WifiMode::Station,
        },
        &[0, 1],
    )?;
    check_status(
        transport,
        WifiConnectStage::Association,
        rpcs::WifiConnect {
            ssid,
            password,
            security,
            semaphore: 0,
        },
        &[0],
    )?;
    // Seeed's firmware returns 0 only after DHCP_ADDRESS_ASSIGNED.
    check_status(
        transport,
        WifiConnectStage::DhcpStart,
        rpcs::DHCPClientStart {
            interface: erpc::L3Interface::Station,
        },
        &[0],
    )?;

    delay.wait_ms(25);
    let info = transport
        .call(rpcs::GetIPInfo {
            interface: erpc::L3Interface::Station,
        })
        .map_err(WifiConnectError::IpInfo)?;
    if info.ip.is_unspecified() {
        return Err(WifiConnectError::UnspecifiedIp);
    }
    Ok(info)
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use std::{cell::RefCell, rc::Rc, vec::Vec};

    type Request = (u8, u8);
    const ADAPTER_INIT: Request = (15, 1);
    const DHCP_STOP: Request = (15, 14);
    const WIFI_OFF: Request = (14, 28);
    const WIFI_ON: Request = (14, 27);
    const WIFI_CONNECT: Request = (14, 1);
    const DHCP_START: Request = (15, 13);
    const IP_INFO: Request = (15, 7);

    #[derive(Debug, Clone, PartialEq, Eq)]
    enum Event {
        Rpc(Request),
        Delay(u8),
    }

    struct FakeTransport {
        events: Rc<RefCell<Vec<Event>>>,
        arguments: Vec<(Request, Vec<u8>)>,
        status: Option<(Request, i32)>,
        transport_error: Option<Request>,
        ip_error: i32,
        ip: [u8; 4],
    }

    struct FakeDelay(Rc<RefCell<Vec<Event>>>);

    fn fakes() -> (FakeTransport, FakeDelay) {
        let events = Rc::new(RefCell::new(Vec::new()));
        (
            FakeTransport {
                events: events.clone(),
                arguments: Vec::new(),
                status: None,
                transport_error: None,
                ip_error: 0,
                ip: [192, 168, 1, 80],
            },
            FakeDelay(events),
        )
    }

    impl Transport for FakeTransport {
        fn call<R: erpc::RPC>(
            &mut self,
            mut rpc: R,
        ) -> Result<R::ReturnValue, erpc::Err<R::Error>> {
            let mut header = rpc.header(0).as_bytes();
            let request = (header[2], header[1]);
            self.events.borrow_mut().push(Event::Rpc(request));
            let mut arguments = heapless::Vec::new();
            rpc.args(&mut arguments);
            self.arguments
                .push((request, Vec::from(arguments.as_slice())));
            if self.transport_error == Some(request) {
                return Err(erpc::Err::TXErr);
            }

            header[0] = 2;
            let mut response = Vec::from(header);
            if request == IP_INFO {
                response.extend_from_slice(&12u32.to_le_bytes());
                response.extend_from_slice(&self.ip);
                response.extend_from_slice(&[255, 255, 255, 0]);
                response.extend_from_slice(&[192, 168, 1, 1]);
                response.extend_from_slice(&self.ip_error.to_le_bytes());
            } else if request != ADAPTER_INIT {
                let status = self
                    .status
                    .filter(|(stage, _)| *stage == request)
                    .map_or(0, |(_, status)| status);
                response.extend_from_slice(&status.to_le_bytes());
            }
            rpc.parse(&response)
        }
    }

    impl DelayMs for FakeDelay {
        fn wait_ms(&mut self, ms: u8) {
            self.0.borrow_mut().push(Event::Delay(ms));
        }
    }

    fn attempt(
        transport: &mut FakeTransport,
        delay: &mut FakeDelay,
    ) -> Result<erpc::IPInfo, WifiConnectError> {
        connect(
            transport,
            delay,
            "test network",
            "test password",
            erpc::Security::WPA2_AES_PSK,
        )
    }

    #[test]
    fn successful_lease_including_dot_80_preserves_operation_order() {
        let (mut transport, mut delay) = fakes();
        let info = attempt(&mut transport, &mut delay).unwrap();
        assert_eq!(info.ip.octets(), [192, 168, 1, 80]);
        assert_eq!(
            &*transport.events.borrow(),
            &[
                Event::Rpc(ADAPTER_INIT),
                Event::Rpc(DHCP_STOP),
                Event::Rpc(WIFI_OFF),
                Event::Delay(35),
                Event::Rpc(WIFI_ON),
                Event::Rpc(WIFI_CONNECT),
                Event::Rpc(DHCP_START),
                Event::Delay(25),
                Event::Rpc(IP_INFO),
            ]
        );
        assert_eq!(
            transport
                .arguments
                .iter()
                .find(|(request, _)| *request == WIFI_ON)
                .unwrap()
                .1,
            1u32.to_le_bytes()
        );
        for request in [DHCP_STOP, DHCP_START, IP_INFO] {
            assert_eq!(
                transport
                    .arguments
                    .iter()
                    .find(|(stage, _)| *stage == request)
                    .unwrap()
                    .1,
                0u32.to_le_bytes()
            );
        }

        let ssid = b"test network";
        let password = b"test password";
        let mut expected = Vec::new();
        expected.extend_from_slice(&(ssid.len() as u32).to_le_bytes());
        expected.extend_from_slice(ssid);
        expected.push(0);
        expected.extend_from_slice(&(password.len() as u32).to_le_bytes());
        expected.extend_from_slice(password);
        expected.extend_from_slice(&erpc::Security::WPA2_AES_PSK.bits().to_le_bytes());
        expected.extend_from_slice(&(-1i32).to_le_bytes());
        expected.extend_from_slice(&0u32.to_le_bytes());
        assert_eq!(
            transport
                .arguments
                .iter()
                .find(|(request, _)| *request == WIFI_CONNECT)
                .unwrap()
                .1,
            expected
        );
    }

    #[test]
    fn already_running_wifi_and_already_stopped_dhcp_are_accepted() {
        for (request, status) in [(WIFI_ON, 1), (DHCP_STOP, DHCP_ALREADY_STOPPED)] {
            let (mut transport, mut delay) = fakes();
            transport.status = Some((request, status));
            assert!(attempt(&mut transport, &mut delay).is_ok());
        }
    }

    #[test]
    fn failed_firmware_statuses_stop_before_the_next_operation() {
        for (request, stage, status) in [
            (DHCP_STOP, WifiConnectStage::DhcpStop, -13),
            (DHCP_STOP, WifiConnectStage::DhcpStop, 0x5004),
            (WIFI_OFF, WifiConnectStage::WifiOff, -13),
            (WIFI_ON, WifiConnectStage::WifiOn, -13),
            (WIFI_ON, WifiConnectStage::WifiOn, 2),
            (WIFI_CONNECT, WifiConnectStage::Association, 1),
            (DHCP_START, WifiConnectStage::DhcpStart, 1),
        ] {
            let (mut transport, mut delay) = fakes();
            transport.status = Some((request, status));
            assert_eq!(
                attempt(&mut transport, &mut delay).unwrap_err(),
                WifiConnectError::FirmwareStatus { stage, status }
            );
            assert_eq!(transport.events.borrow().last(), Some(&Event::Rpc(request)));
        }
    }

    #[test]
    fn failed_association_and_dhcp_never_return_the_fallback_address() {
        for (request, stage) in [
            (WIFI_CONNECT, WifiConnectStage::Association),
            (DHCP_START, WifiConnectStage::DhcpStart),
        ] {
            let (mut transport, mut delay) = fakes();
            transport.status = Some((request, -1));
            assert_eq!(
                attempt(&mut transport, &mut delay).unwrap_err(),
                WifiConnectError::FirmwareStatus { stage, status: -1 }
            );
            assert!(!transport.events.borrow().contains(&Event::Rpc(IP_INFO)));
        }
    }

    #[test]
    fn transport_errors_retain_the_failing_stage() {
        for (request, stage) in [
            (ADAPTER_INIT, WifiConnectStage::AdapterInit),
            (DHCP_STOP, WifiConnectStage::DhcpStop),
            (WIFI_OFF, WifiConnectStage::WifiOff),
            (WIFI_ON, WifiConnectStage::WifiOn),
            (WIFI_CONNECT, WifiConnectStage::Association),
            (DHCP_START, WifiConnectStage::DhcpStart),
        ] {
            let (mut transport, mut delay) = fakes();
            transport.transport_error = Some(request);
            assert_eq!(
                attempt(&mut transport, &mut delay).unwrap_err(),
                WifiConnectError::Rpc {
                    stage,
                    source: erpc::Err::TXErr,
                }
            );
            assert_eq!(transport.events.borrow().last(), Some(&Event::Rpc(request)));
        }

        let (mut transport, mut delay) = fakes();
        transport.transport_error = Some(IP_INFO);
        assert_eq!(
            attempt(&mut transport, &mut delay).unwrap_err(),
            WifiConnectError::IpInfo(erpc::Err::TXErr)
        );
    }

    #[test]
    fn ip_info_errors_and_unspecified_addresses_are_rejected() {
        let (mut transport, mut delay) = fakes();
        transport.ip_error = 123;
        assert_eq!(
            attempt(&mut transport, &mut delay).unwrap_err(),
            WifiConnectError::IpInfo(erpc::Err::RPCErr(123))
        );

        let (mut transport, mut delay) = fakes();
        transport.ip = [0; 4];
        assert_eq!(
            attempt(&mut transport, &mut delay).unwrap_err(),
            WifiConnectError::UnspecifiedIp
        );
    }

    #[test]
    fn invalid_credentials_are_rejected_before_touching_the_radio() {
        let too_long = "x".repeat(65);
        let (mut transport, mut delay) = fakes();
        assert_eq!(
            connect(
                &mut transport,
                &mut delay,
                too_long.as_str(),
                "password",
                erpc::Security::WPA2_AES_PSK,
            )
            .unwrap_err(),
            WifiConnectError::InvalidSsid
        );
        assert_eq!(
            connect(
                &mut transport,
                &mut delay,
                "network",
                too_long.as_str(),
                erpc::Security::WPA2_AES_PSK,
            )
            .unwrap_err(),
            WifiConnectError::InvalidPassword
        );
        assert!(transport.events.borrow().is_empty());
    }

    #[test]
    fn legacy_errors_preserve_transport_failures() {
        assert_eq!(
            WifiConnectError::Rpc {
                stage: WifiConnectStage::Association,
                source: erpc::Err::CRCMismatch,
            }
            .into_legacy(),
            erpc::Err::CRCMismatch
        );
        assert_eq!(
            WifiConnectError::IpInfo(erpc::Err::TXErr).into_legacy(),
            erpc::Err::TXErr
        );
        assert_eq!(
            WifiConnectError::FirmwareStatus {
                stage: WifiConnectStage::DhcpStart,
                status: -1,
            }
            .into_legacy(),
            erpc::Err::RPCErr(())
        );
    }

    #[test]
    fn error_messages_identify_the_operation_and_status() {
        assert_eq!(
            std::format!(
                "{}",
                WifiConnectError::FirmwareStatus {
                    stage: WifiConnectStage::Association,
                    status: -1,
                }
            ),
            "WifiConnect status: -1"
        );
        assert_eq!(
            std::format!(
                "{}",
                WifiConnectError::FirmwareStatus {
                    stage: WifiConnectStage::DhcpStart,
                    status: -1,
                }
            ),
            "DHCPClientStart status: -1"
        );
        assert_eq!(
            std::format!("{}", WifiConnectError::IpInfo(erpc::Err::RPCErr(123))),
            "GetIPInfo RPC: RPCErr(123)"
        );
        assert_eq!(
            std::format!(
                "{}",
                WifiConnectError::Rpc {
                    stage: WifiConnectStage::WifiOn,
                    source: erpc::Err::CRCMismatch,
                }
            ),
            "WifiOn RPC: CRCMismatch"
        );
    }
}
