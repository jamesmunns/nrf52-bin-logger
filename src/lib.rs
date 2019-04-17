//! `nrf52-bin-logger`
//!
//! This is a handy way to change the nRF52 UART from a byte-stream oriented
//! interface to a "Rust Struct" focused interface. Users can decide if they
//! want to send, receive, or do both over the serial port.
//!
//! Messages are serialized and deserialized using `postcard` + `serde`, and
//! all messages are COBS encoded for framing. This can be used to quickly set
//! up a communications protocol over a serial port for the nRF52.

#![no_std]

use nrf52832_hal::{nrf52832_pac::UARTE0, uarte::Uarte};

use bare_metal::Mutex;
use core::cell::UnsafeCell;
use core::sync::atomic::{compiler_fence, Ordering::SeqCst};
use cortex_m::interrupt;
use heapless::ArrayLength;
use postcard::{from_bytes_cobs};
use serde::{de::DeserializeOwned, Deserialize, Serialize};

static A_SIDE: Mutex<UnsafeCell<[u8; 255]>> = Mutex::new(UnsafeCell::new([0u8; 255]));
static B_SIDE: Mutex<UnsafeCell<[u8; 255]>> = Mutex::new(UnsafeCell::new([0u8; 255]));

mod nrf52_ports;
pub mod senders;
pub mod receivers;

use nrf52_ports::{
    start_read,
    finalize_read,
};
use senders::{
    Sender,
};
use receivers::{
    PingPongMode,
    RealReceiver,
    Receiver,
};

mod private {
    use heapless::ArrayLength;

    pub trait Sealed {}

    impl Sealed for crate::receivers::NullReceiver {}
    impl Sealed for crate::senders::NullSender {}
    impl<T, U, V> Sealed for crate::receivers::RealReceiver<T, U, V>
    where
        U: ArrayLength<u8>,
        V: ArrayLength<T>,
    {}
    impl<T, U> Sealed for crate::senders::RealSender<T, U>
    where
        U: ArrayLength<u8>,
    {}
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
///
/// In the future other serial interfaces might be supported
pub struct Logger<SEND, RECV>
where
    SEND: Sender,
    RECV: Receiver,
{
    uart: Uarte<UARTE0>,
    _send: SEND,
    recv: RECV,
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
        }
    }

}

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

        interrupt::free(|cs| unsafe {
            // NOTE: this only ever returns an error if the buffer passed in is >= DMA_SIZE.
            // Since we always use a fixed buffer of 255, this can never fail
            start_read(&mut *A_SIDE.borrow(cs).get()).unwrap();
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
        let (old, new, new_ppm) = match self.recv.ppm {
            PingPongMode::AActive => (&A_SIDE, &B_SIDE, PingPongMode::BActive),
            PingPongMode::BActive => (&B_SIDE, &A_SIDE, PingPongMode::AActive),
            _ => return Err(()),
        };

        self.recv.ppm = new_ppm;

        let periph = unsafe { &*UARTE0::ptr() };
        periph.tasks_stoprx.write(|w| unsafe { w.bits(1) });

        while periph.events_endrx.read().bits() != 1 {}

        compiler_fence(SeqCst);

        let used = periph.rxd.amount.read().bits() as usize;
        finalize_read();

        interrupt::free(|cs| unsafe {
            // NOTE: this only ever returns an error if the buffer passed in is >= DMA_SIZE.
            // Since we always use a fixed buffer of 255, this can never fail
            start_read(&mut *new.borrow(cs).get()).unwrap();
            (&mut output[..used]).copy_from_slice(&mut (*old.borrow(cs).get())[..used]);
        });

        Ok(&mut output[..used])
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
                    if let Ok(msg) = from_bytes_cobs(&mut *self.recv.inc_q) {
                        self.recv.msg_q
                            .enqueue(msg)
                            .map_err(|_| ())?;
                    }
                }

                self.recv.inc_q.clear();

                less_buf = lat;
            }

            // No room? No message.
            if self.recv.inc_q.extend_from_slice(less_buf).is_err() {
                self.recv.inc_q.clear();
            }
        }

        Ok(self.recv.msg_q.len())
    }

    /// Pop a single message off of the decoded/deserialized queue
    pub fn get_msg(&mut self) -> Option<T> {
        self.recv.msg_q.dequeue()
    }
}
