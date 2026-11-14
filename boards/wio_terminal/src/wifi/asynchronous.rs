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
    /// Discards every byte currently readable. Used by [`Client::recover`]
    /// to drop the tail of an abandoned reply.
    fn drain(&mut self) {
        let mut scratch = [0; 32];
        while self.try_read(&mut scratch) != 0 {}
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

/// Largest reply frame the client accepts.
const REPLY_MAX: usize = 2048;

pub struct Client<T> {
    transport: T,
    sequence: u32,
    ready: bool,
    // Kept in the client, not in the `rpc()` future, so that every caller's
    // future does not embed its own copy (about 2.3 KiB).
    message: Vec<u8, 256>,
    payload: [u8; REPLY_MAX],
}

impl<T: ByteTransport> Client<T> {
    pub fn new(transport: T) -> Self {
        Self {
            transport,
            sequence: 0,
            ready: true,
            message: Vec::new(),
            payload: [0; REPLY_MAX],
        }
    }

    /// Makes a poisoned client usable again.
    ///
    /// Discards unread bytes and re-arms the client. Call it only after the
    /// radio has been reset and no request is in flight, otherwise a late
    /// reply to the abandoned request may be consumed by the next one.
    /// A latched transport fault (`ByteTransport::fault`) is not cleared here:
    /// it belongs to the transport owner, and `rpc` keeps reporting it.
    pub fn recover(&mut self) {
        self.transport.drain();
        self.ready = true;
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
        self.message.clear();
        self.message
            .extend_from_slice(&rpc.header(self.sequence).as_bytes())
            .map_err(|_| Error::Rpc(erpc::Err::TXErr))?;
        rpc.args(&mut self.message);
        let header = erpc::FrameHeader::new_from_msg(&self.message).as_bytes();
        for mut bytes in [&header[..], self.message.as_slice()] {
            while !bytes.is_empty() {
                if !wait().await {
                    return Err(Error::Timeout);
                }
                if let Some(error) = self.transport.fault() {
                    return Err(Error::Transport(error));
                }
                let count = self.transport.try_write(bytes);
                bytes = &bytes[count..];
            }
        }
        loop {
            let mut header = [0; 4];
            read_exact(&mut self.transport, &mut header, &mut wait).await?;
            let (_, header) = erpc::FrameHeader::parse(&header[..])
                .map_err(|error| Error::Rpc(erpc::Err::Parsing(error)))?;
            let len = header.msg_length as usize;
            if len > self.payload.len() {
                return Err(Error::FrameTooLarge);
            }
            read_exact(&mut self.transport, &mut self.payload[..len], &mut wait).await?;
            let payload = &self.payload[..len];
            header.check_crc(payload).map_err(Error::Rpc)?;
            let (_, reply) = erpc::Header::parse(payload)
                .map_err(|error| Error::Rpc(erpc::Err::Parsing(error)))?;
            if reply.sequence != self.sequence {
                continue;
            }
            match rpc.parse(payload) {
                Err(erpc::Err::NotOurs) => continue,
                result => {
                    self.ready = matches!(result, Ok(_) | Err(erpc::Err::RPCErr(_)));
                    return result.map_err(Error::Rpc);
                }
            }
        }
    }

    fn check_transport<E>(&self) -> Result<(), Error<E>> {
        match self.transport.fault() {
            Some(error) => Err(Error::Transport(error)),
            None => Ok(()),
        }
    }
}

async fn read_exact<T, E, W, F>(
    transport: &mut T,
    mut bytes: &mut [u8],
    wait: &mut W,
) -> Result<(), Error<E>>
where
    T: ByteTransport,
    W: FnMut() -> F,
    F: Future<Output = bool>,
{
    while !bytes.is_empty() {
        if !wait().await {
            return Err(Error::Timeout);
        }
        if let Some(error) = transport.fault() {
            return Err(Error::Transport(error));
        }
        let count = transport.try_read(bytes);
        bytes = &mut bytes[count..];
    }
    Ok(())
}
