//! `nrf52-bin-logger`
//!
//! This is a handy way to change the nRF52 UART from a byte-stream oriented
//! interface to a "Rust Struct" focused interface. Users can decide if they
//! want to send, receive, or do both over the serial port.
//!
//! Messages are serialized and deserialized using `postcard` + `serde`, and
//! all messages are COBS encoded for framing. This can be used to quickly set
//! up a uni- or bi-directional communications protocol over a serial port
//! for the nRF52.
//!
//! ```rust
//! use serde::{Serialize, Deserialize};
//! use heapless::consts::*;
//! use nrf52_bin_logger::{Logger, senders::RealSender, receivers::RealReceiver};
//!
//! #[derive(Serialize, Deserialize)]
//! enum MyProtocol {
//!     MsgA(u32),
//!     MsgB(f32),
//!     MsgC(bool),
//! }
//!
//! type ModemLogger = Logger<
//!     // `MyProtocol` outgoing messages, 16 bytes used as a serialization buffer
//!     RealSender<MyProtocol, U16>,
//!     // `MyProtocol` incoming messages, 16 bytes used as a deserialization buffer,
//!     // a max of 8 `MyProtocol` messages can be enqueued
//!     RealReceiver<MyProtocol, U16, U8>,
//! >;
//! ```
//!
//! Don't need a sender or a receiver? Just replace the type with a NullSender/NullReceiver.
//! No code will be generated for this half of the interface.
//!
//! ```rust
//! use serde::{Serialize, Deserialize};
//! use heapless::consts::*;
//! use nrf52_bin_logger::{Logger, senders::RealSender, receivers::NullReceiver};
//!
//! #[derive(Serialize, Deserialize)]
//! enum MyProtocol {
//!     MsgA(u32),
//!     MsgB(f32),
//!     MsgC(bool),
//! }
//!
//! type ModemLogger = Logger<
//!     // `MyProtocol` outgoing messages, 16 bytes used as a serialization buffer
//!     RealSender<MyProtocol, U16>,
//!     // Nothing will be received
//!     NullReceiver,
//! >;
//! ```

#![cfg_attr(not(test), no_std)]

use nrf52832_hal::{nrf52832_pac::UARTE0, uarte::Uarte};

#[cfg(feature = "unstable")]
mod unstable {
    pub(crate) use core::cell::UnsafeCell;
    pub(crate) use core::sync::atomic::{compiler_fence, Ordering::SeqCst};

    pub(crate) use crate::nrf52_ports::start_read;
    pub(crate) use crate::receivers::{PingPongMode, RealReceiver};

    pub(crate) use bare_metal::Mutex;
    pub(crate) use cortex_m::interrupt;
    pub(crate) use heapless::ArrayLength;
    pub(crate) use postcard::from_bytes_cobs;
    pub(crate) use serde::de::DeserializeOwned;
}

#[cfg(feature = "unstable")]
use crate::unstable::*;

#[cfg(feature = "unstable")]
static A_SIDE: Mutex<UnsafeCell<[u8; 255]>> = Mutex::new(UnsafeCell::new([0u8; 255]));
#[cfg(feature = "unstable")]
static B_SIDE: Mutex<UnsafeCell<[u8; 255]>> = Mutex::new(UnsafeCell::new([0u8; 255]));

use serde::{Deserialize, Serialize};

#[cfg(feature = "unstable")]
mod nrf52_ports;

pub mod receivers;
pub mod senders;

use receivers::Receiver;
use senders::Sender;

mod private {
    use heapless::ArrayLength;

    pub trait Sealed {}

    impl Sealed for crate::receivers::NullReceiver {}
    impl Sealed for crate::senders::NullSender {}

    #[cfg(feature = "unstable")]
    impl<T, U, V> Sealed for crate::receivers::RealReceiver<T, U, V>
    where
        U: ArrayLength<u8>,
        V: ArrayLength<T>,
    {
    }
    impl<T, U> Sealed for crate::senders::RealSender<T, U> where U: ArrayLength<u8> {}
}

/// This is the primary type sent via the Sender
#[derive(Serialize, Deserialize, Debug, Eq, PartialEq)]
pub enum LogOnLine<'a, T> {
    /// A Log Level Message
    Log(&'a str),

    /// A Warning Level Message
    Warn(&'a str),

    /// An Error Level Message
    Error(&'a str),

    /// A binary payload, with a human readable description
    BinaryRaw(BinMessage<'a>),

    /// A type chosen by the user as a general communication protocol
    ProtocolMessage(T),
}

/// A binary payload with UTF-8 Description
#[derive(Serialize, Deserialize, Debug, Eq, PartialEq)]
pub struct BinMessage<'a> {
    description: &'a str,
    data: &'a [u8],
}

/// A binary logging interface, using UARTE0.
///target
/// In the future other serial interfaces might be supported
pub struct Logger<SEND, RECV>
where
    SEND: Sender,
    RECV: Receiver,
{
    uart: Uarte<UARTE0>,
    _send: SEND,

    #[allow(dead_code)]
    recv: RECV,
    pub good_msgs: usize,
    pub good_bytes: usize,
    pub dropped_bytes: usize,
    pub dropped_msgs: usize,
    pub bad_cobs: usize,
    pub full_buf: usize,
    pub full_msg: usize,
    pub ttl_got: usize,
}

impl<SEND, RECV> Logger<SEND, RECV>
where
    SEND: Sender + Default,
    RECV: Receiver + Default,
{
    pub fn new(mut uart: Uarte<UARTE0>) -> Self {
        // Send termination character
        // NOTE: this only ever returns an error if the buffer passed in is >= DMA_SIZE.
        // Since we always use a fixed buffer of 1, this can never fail
        uart.write(&[0x00]).unwrap();

        Self {
            uart,
            _send: SEND::default(),
            recv: RECV::default(),
            dropped_msgs: 0,
            dropped_bytes: 0,
            bad_cobs: 0,
            full_buf: 0,
            full_msg: 0,
            good_msgs: 0,
            good_bytes: 0,
            ttl_got: 0,
        }
    }
}

#[cfg(feature = "unstable")]
impl<SEND, T, BUFSZ, MSGCT> Logger<SEND, RealReceiver<T, BUFSZ, MSGCT>>
where
    T: DeserializeOwned,
    BUFSZ: ArrayLength<u8>,
    MSGCT: ArrayLength<T>,
    SEND: Sender + Default,
{
    /// Start the receive process using the internal double-buffers
    pub fn start_receive(&mut self) -> Result<(), ()> {
        if self.recv.ppm != PingPongMode::Idle {
            return Err(());
        }

        self.recv.ppm = PingPongMode::AActive;

        let periph = unsafe { &*UARTE0::ptr() };
        periph.events_rxdrdy.write(|w| unsafe { w.bits(0) });

        interrupt::free(|cs| unsafe {
            // NOTE: this only ever returns an error if the buffer passed in is >= DMA_SIZE.
            // Since we always use a fixed buffer of 255, this can never fail
            start_read(&mut *A_SIDE.borrow(cs).get(), false).unwrap();
        });

        Ok(())
    }

    /// Obtain any pending bytes in the active buffer. This also causes
    /// the internal double buffers to flip. When this function is successful,
    /// a sub-slice of `output` is returned, containing the read bytes.
    ///
    /// Bytes are obtained as they are received on the line, so decoding
    /// and deserialization must be performed manually.
    ///
    /// NOTE: Either this function or `service_receive()` must be polled
    /// periodically, or there will be data loss! In general, the calculation
    /// for "how often do I need to poll this" is `1 / (BAUDRATE / LINE_BITS_PER_DATA_BYTE / 255)`.
    ///
    /// For example, at 230400 Baud, and 8N1 settings (8 data bits per 10 line bits), it is necessary
    /// to poll this function once per ~11ms.
    pub fn get_pending_manual<'a, 'b>(
        &'b mut self,
        output: &'a mut [u8],
    ) -> Result<&'a mut [u8], ()> {
        let periph = unsafe { &*UARTE0::ptr() };

        if periph.events_rxdrdy.read().bits() == 0 {
            return Ok(&mut []);
        }
        periph.events_rxdrdy.write(|w| unsafe { w.bits(0) });

        let (old, new, new_ppm) = match self.recv.ppm {
            PingPongMode::AActive => (&A_SIDE, &B_SIDE, PingPongMode::BActive),
            PingPongMode::BActive => (&B_SIDE, &A_SIDE, PingPongMode::AActive),
            _ => return Err(()),
        };

        self.recv.ppm = new_ppm;

        periph.tasks_stoprx.write(|w| unsafe { w.bits(1) });

        while periph.events_endrx.read().bits() != 1 {}
        periph.events_endrx.write(|w| unsafe { w.bits(0) });

        while periph.events_rxto.read().bits() != 1 {}
        periph.events_rxto.write(|w| unsafe { w.bits(0) });
        compiler_fence(SeqCst);

        let used = periph.rxd.amount.read().bits() as usize;
        self.ttl_got += used;
        if used == 255 {
            panic!("FULL!");
        }

        interrupt::free(|cs| unsafe {
            // NOTE: this only ever returns an error if the buffer passed in is >= DMA_SIZE.
            // Since we always use a fixed buffer of 255, this can never fail
            start_read(&mut *new.borrow(cs).get(), true).unwrap();
            if used > 0 {
                (&mut output[..used]).copy_from_slice(&mut (*old.borrow(cs).get())[..used]);
            }
        });

        compiler_fence(SeqCst);

        if used > 0 {
            Ok(&mut output[..used])
        } else {
            Ok(&mut [])
        }
    }

    /// This is an automatic handler, that will clear any pending
    /// bytes, and attempt to parse any pending messages. If successful,
    /// the return value is the number messages ready to be cleared with
    /// `get_msg()`.
    ///
    /// NOTE: Either this function or `get_pending_manual()` must be polled
    /// periodically, or there will be data loss! In general, the calculation
    /// for "how often do I need to poll this" is `1 / (BAUDRATE / LINE_BITS_PER_DATA_BYTE / 255)`.
    ///
    /// For example, at 230400 Baud, and 8N1 settings (8 data bits per 10 line bits), it is necessary
    /// to poll this function once per ~11ms.
    ///
    /// Care must also be taken to ensure that MSGCT has a deep enough queue.
    pub fn service_receive(&mut self) -> Result<usize, ()> {
        let mut buf = [0u8; 255];
        let mut less_buf = self.get_pending_manual(&mut buf)?;

        // If we've received any data...
        if less_buf.len() > 0 {
            // See if this new data contains a `0`, the cobs framing byte
            while let Some(idx) = less_buf.iter().position(|&n| n == 0u8) {
                // Split the new buffer, including the framing byte in the
                // broken off chunk...
                let (frm, lat) = less_buf.split_at_mut(idx + 1);

                // If we can't push, just drain the buffer
                if self.recv.inc_q.extend_from_slice(frm).is_ok() {
                    // Silently discard any poorly serialized or encoded messages
                    // Keep trying to decode further messages to prevent data loss
                    let mut flag = false;

                    if let Ok(msg) = from_bytes_cobs(&mut *self.recv.inc_q) {
                        if self.recv.msg_q.enqueue(msg).is_ok() {
                            flag = true;
                            self.good_msgs += 1;
                            self.good_bytes += self.recv.inc_q.len();
                        } else {
                            self.full_msg += 1;
                        }
                    } else {
                        self.bad_cobs += 1;
                    }

                    if !flag {
                        self.dropped_bytes += self.recv.inc_q.len();
                        self.dropped_msgs += 1;
                    }
                } else {
                    self.full_buf += 1;
                    self.dropped_bytes += self.recv.inc_q.len();
                    self.dropped_bytes += frm.len();
                }

                self.recv.inc_q.clear();

                less_buf = lat;
            }

            // No room? No message.
            if less_buf.len() > 0 {
                if self.recv.inc_q.extend_from_slice(less_buf).is_err() {
                    self.full_buf += 1;
                    self.dropped_bytes += self.recv.inc_q.len();
                    self.recv.inc_q.clear();
                }
            }
        }

        Ok(self.recv.msg_q.len())
    }

    /// Pop a single message off of the decoded/deserialized queue
    pub fn get_msg(&mut self) -> Option<T> {
        self.recv.msg_q.dequeue()
    }

    pub fn get_stats(&self) -> (usize, usize) {
        (self.dropped_bytes, self.dropped_msgs)
    }
}
