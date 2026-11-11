//! Bounded, cooperative eRPC transport. One owner issues requests; interrupt
//! code owns the UART. A cancelled or failed request poisons the transport:
//! reset the radio and construct a new client before issuing another request.

use core::future::Future;
use heapless::Vec;
use seeed_erpc as erpc;
pub use seeed_erpc::RPC;

pub trait ByteTransport {
    fn try_write(&mut self, data: &[u8]) -> usize;
    fn try_read(&mut self, data: &mut [u8]) -> usize;
    fn fault(&self) -> Option<TransportError> {
        None
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransportError {
    RxOverflow,
    Uart,
}

#[derive(Debug, Clone, PartialEq)]
pub enum Error<E> {
    Timeout,
    Unavailable,
    FrameTooLarge,
    Transport(TransportError),
    Rpc(erpc::Err<E>),
}

pub struct Client<T> {
    transport: T,
    sequence: u32,
    ready: bool,
}

impl<T: ByteTransport> Client<T> {
    pub fn new(transport: T) -> Self {
        Self {
            transport,
            sequence: 0,
            ready: true,
        }
    }

    /// `wait` must yield and return false once the overall request deadline
    /// expires. It is called even while bytes are flowing, bounding callbacks
    /// and malformed streams as well as an entirely silent radio.
    pub async fn rpc<R, W, F>(
        &mut self,
        mut rpc: R,
        mut wait: W,
    ) -> Result<R::ReturnValue, Error<R::Error>>
    where
        R: RPC,
        W: FnMut() -> F,
        F: Future<Output = bool>,
    {
        if !self.ready {
            return Err(Error::Unavailable);
        }
        self.ready = false;
        self.check_transport()?;
        self.sequence = self.sequence.wrapping_add(1);
        let mut message = Vec::<u8, 256>::new();
        message
            .extend_from_slice(&rpc.header(self.sequence).as_bytes())
            .map_err(|_| Error::Rpc(erpc::Err::TXErr))?;
        rpc.args(&mut message);
        let header = erpc::FrameHeader::new_from_msg(&message).as_bytes();
        for mut bytes in [&header[..], message.as_slice()] {
            while !bytes.is_empty() {
                if !wait().await {
                    return Err(Error::Timeout);
                }
                self.check_transport()?;
                let count = self.transport.try_write(bytes);
                bytes = &bytes[count..];
            }
        }
        loop {
            let mut header = [0; 4];
            self.read_exact(&mut header, &mut wait).await?;
            let (_, header) = erpc::FrameHeader::parse(&header[..])
                .map_err(|error| Error::Rpc(erpc::Err::Parsing(error)))?;
            let mut payload = [0; 2048];
            let len = header.msg_length as usize;
            if len > payload.len() {
                return Err(Error::FrameTooLarge);
            }
            self.read_exact(&mut payload[..len], &mut wait).await?;
            header.check_crc(&payload[..len]).map_err(Error::Rpc)?;
            let (_, reply) = erpc::Header::parse(&payload[..len])
                .map_err(|error| Error::Rpc(erpc::Err::Parsing(error)))?;
            if reply.sequence != self.sequence {
                continue;
            }
            match rpc.parse(&payload[..len]) {
                Err(erpc::Err::NotOurs) => continue,
                result => {
                    self.ready = matches!(result, Ok(_) | Err(erpc::Err::RPCErr(_)));
                    return result.map_err(Error::Rpc);
                }
            }
        }
    }

    async fn read_exact<E, W, F>(
        &mut self,
        mut bytes: &mut [u8],
        wait: &mut W,
    ) -> Result<(), Error<E>>
    where
        W: FnMut() -> F,
        F: Future<Output = bool>,
    {
        while !bytes.is_empty() {
            if !wait().await {
                return Err(Error::Timeout);
            }
            self.check_transport()?;
            let count = self.transport.try_read(bytes);
            bytes = &mut bytes[count..];
        }
        Ok(())
    }

    fn check_transport<E>(&self) -> Result<(), Error<E>> {
        match self.transport.fault() {
            Some(error) => Err(Error::Transport(error)),
            None => Ok(()),
        }
    }
}
