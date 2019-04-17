use core::ops::Deref;
use core::marker::PhantomData;

use nrf52832_hal::target_constants::EASY_DMA_SIZE;

use heapless::{ArrayLength, Vec};
use postcard::to_vec_cobs;
use serde::{Serialize};

use crate::LogOnLine;
use crate::private::*;
use crate::Logger;
use crate::Receiver;
use crate::BinMessage;

/// The Sender trait represents options for sending from the
/// nRF52 to another device. This is commonly used for logging.
///
/// It is a sealed marker trait, and can not be implemented outside
/// of this crate
pub trait Sender: Default + Sealed {}

/// The NullSender does not allow you to send anything. This
/// is useful when you only receive on the given serial port
pub struct NullSender;

impl Sender for NullSender {}

/// The RealSender is used when actually sending data via the
/// serial port. RealSender has two generic parameters:
///
/// `T`: This is the serializable type that can be sent over
/// the serial port. It will be serialized using `postcard`,
/// and will be Cobs encoded for framing.
///
/// `BUFSZ`: This is the size of the buffer used to serialize
/// the message to before sending. It must be large enough to
/// hold a serialized + cobs encoded message of type `T`.
///
/// This sender currently blocks during transmission
pub struct RealSender<T, BUFSZ>
where
    BUFSZ: ArrayLength<u8>,
{
    _t: PhantomData<T>,
    _b: PhantomData<BUFSZ>,
}

// Fulfill the marker trait
impl<T, BUFSZ> Sender for RealSender<T, BUFSZ>
where
    T: Serialize,
    BUFSZ: ArrayLength<u8>
{ }

// We need a generic way to make a new sender, Default
// is a good enough way
impl Default for NullSender
{
    fn default() -> Self {
        NullSender
    }
}

// Same for the RealSender
impl<T, BUFSZ> Default for RealSender<T, BUFSZ>
where
    T: Serialize,
    BUFSZ: ArrayLength<u8>
{
    fn default() -> Self {
        RealSender {
            _t: PhantomData,
            _b: PhantomData,
        }
    }
}

// These methods are only available when using a real serial port
impl<RECV, T, BUFSZ> Logger<RealSender<T, BUFSZ>, RECV>
where
    T: Serialize,
    BUFSZ: ArrayLength<u8>,
    RECV: Receiver + Default,
{
    /// Send a log level &str message
    pub fn log(&mut self, data: &str) -> Result<(), ()> {
        self.send(&LogOnLine::Log(data))
    }

    /// Send a warn level &str message
    pub fn warn(&mut self, data: &str) -> Result<(), ()> {
        self.send(&LogOnLine::Warn(data))
    }

    /// Send an error level &str message
    pub fn error(&mut self, data: &str) -> Result<(), ()> {
        self.send(&LogOnLine::Error(data))
    }

    /// Send a byte slice message
    pub fn raw_bin(&mut self, description: &str, data: &[u8]) -> Result<(), ()> {
        self.send(&LogOnLine::BinaryRaw(BinMessage { description, data }))
    }

    /// Send a log level &str message
    pub fn data(&mut self, data: T) -> Result<(), ()> {
        self.send(&LogOnLine::ProtocolMessage(data))
    }

    fn send(&mut self, msg: &LogOnLine<T>) -> Result<(), ()> {
        let vec: Vec<u8, BUFSZ> = to_vec_cobs(msg).map_err(|_| ())?;

        // Remove once nrf52832_hal reaches 0.8.0
        for c in vec.deref().chunks(EASY_DMA_SIZE) {
            self.uart.write(c).map_err(|_| ())?;
        }

        Ok(())
    }
}
