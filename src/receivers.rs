//! These items are used to obtain data from an external device to the nRF52, such as an incoming command.

#[cfg(feature = "unstable")]
use heapless::{spsc::Queue, ArrayLength, Vec};

#[cfg(feature = "unstable")]
#[derive(Eq, PartialEq, Debug)]
pub(crate) enum PingPongMode {
    Idle,
    AActive,
    BActive,
}

/// The Receiver trait represents options for receiving as the
/// nRF52 from another device. This is commonly used for commands.
///
/// It is a sealed marker trait, and can not be implemented outside
/// of this crate
pub trait Receiver: Default + crate::private::Sealed {}

/// The RealReceiver is used when actually sending data via the
/// serial port. RealReceiver has three generic parameters:
///
/// `T`: This is the deserializable type that can be received over
/// the serial port. It will be deserialized using `postcard`,
/// and will be Cobs decoded for framing.
///
/// `BUFSZ`: This is the size of the buffer used to store a single
/// message before deserializing. It must be large enough to
/// hold a serialized + cobs encoded message of type `T`.
///
/// `MSGCT`: This is the depth of a FIFO queue of type T. When
/// decoded, messages will be enqueued here for reception
///
/// This sender currently blocks during transmission
#[cfg(feature = "unstable")]
pub struct RealReceiver<T, BUFSZ, MSGCT>
where
    BUFSZ: ArrayLength<u8>,
    MSGCT: ArrayLength<T>,
{
    pub(crate) inc_q: Vec<u8, BUFSZ>,
    pub(crate) msg_q: Queue<T, MSGCT>,
    pub(crate) ppm: PingPongMode,
}

/// This Receiver does not expose any methods, and does not actually
/// receive any data from the serial port. Useful when only logging
pub struct NullReceiver;

// Fulfill the marker trait
impl Receiver for NullReceiver {}

#[cfg(feature = "unstable")]
impl<T, BUFSZ, MSGCT> Receiver for RealReceiver<T, BUFSZ, MSGCT>
where
    BUFSZ: ArrayLength<u8>,
    MSGCT: ArrayLength<T>,
{
}

#[cfg(feature = "unstable")]
impl<T, BUFSZ, MSGCT> Default for RealReceiver<T, BUFSZ, MSGCT>
where
    BUFSZ: ArrayLength<u8>,
    MSGCT: ArrayLength<T>,
{
    fn default() -> Self {
        Self {
            inc_q: Vec::new(),
            msg_q: Queue::new(),
            ppm: PingPongMode::Idle,
        }
    }
}

impl Default for NullReceiver {
    fn default() -> Self {
        NullReceiver
    }
}
