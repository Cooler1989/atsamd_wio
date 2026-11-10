use seeed_erpc::{self as erpc, rpcs, RPC};

const WIFI_GET_LAST_ERROR: u8 = 49;
const REPLY_MESSAGE: u8 = 2;

pub(super) struct GetLastError;

impl RPC for GetLastError {
    type ReturnValue = i32;
    type Error = ();

    fn header(&self, sequence: u32) -> erpc::Header {
        let mut header = rpcs::WifiOff {}.header(sequence);
        header.request = WIFI_GET_LAST_ERROR;
        header
    }

    fn parse(&mut self, data: &[u8]) -> Result<i32, erpc::Err<()>> {
        let (data, header) = erpc::Header::parse::<_, ()>(data)?;
        let expected = self.header(header.sequence);
        if header.msg_type != REPLY_MESSAGE.into()
            || header.service != expected.service
            || header.request != expected.request
        {
            return Err(erpc::Err::NotOurs);
        }

        let [a, b, c, d, ..] = data else {
            return Err(erpc::Err::RPCErr(()));
        };
        Ok(i32::from_le_bytes([*a, *b, *c, *d]))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn response(value: i32) -> [u8; 12] {
        let mut header = GetLastError.header(0x01020304).as_bytes();
        header[0] = REPLY_MESSAGE;
        let mut response = [0; 12];
        response[..8].copy_from_slice(&header);
        response[8..].copy_from_slice(&value.to_le_bytes());
        response
    }

    #[test]
    fn request_uses_the_firmware_wifi_service_and_last_error_id() {
        assert_eq!(
            GetLastError.header(0x01020304).as_bytes(),
            [0, 49, 14, 1, 4, 3, 2, 1]
        );
    }

    #[test]
    fn parses_known_and_unknown_last_error_values() {
        for value in [0, 1, 2, 3, 4, 5, 6, i32::MIN, i32::MAX] {
            assert_eq!(GetLastError.parse(&response(value)), Ok(value));
        }
    }

    #[test]
    fn rejects_other_messages_and_incomplete_results() {
        for index in [0, 1, 2] {
            let mut other = response(3);
            other[index] = other[index].wrapping_add(1);
            assert_eq!(GetLastError.parse(&other), Err(erpc::Err::NotOurs));
        }
        assert_eq!(
            GetLastError.parse(&response(3)[..11]),
            Err(erpc::Err::RPCErr(()))
        );
        assert!(matches!(
            GetLastError.parse(&response(3)[..7]),
            Err(erpc::Err::Parsing(_))
        ));
    }
}
