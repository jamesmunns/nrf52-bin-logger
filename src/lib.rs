#![no_std]


use nrf52832_hal::{nrf52832_pac::UARTE0, uarte::Uarte};

use bare_metal::Mutex;
use core::cell::UnsafeCell;
use core::sync::atomic::{compiler_fence, Ordering::SeqCst};
use cortex_m::interrupt;
use heapless::{spsc::Queue, ArrayLength, Vec};
use postcard::{from_bytes_cobs};
use serde::{de::DeserializeOwned, Deserialize, Serialize};

static A_SIDE: Mutex<UnsafeCell<[u8; 255]>> = Mutex::new(UnsafeCell::new([0u8; 255]));
static B_SIDE: Mutex<UnsafeCell<[u8; 255]>> = Mutex::new(UnsafeCell::new([0u8; 255]));

mod nrf52_ports;
mod senders;

use nrf52_ports::{
    start_read,
    finalize_read,
};
use senders::*;

mod private {
    use heapless::ArrayLength;

    pub trait Sealed {}

    impl Sealed for crate::NullReceiver {}
    impl Sealed for crate::senders::NullSender {}
    impl<T, U, V> Sealed for crate::RealReceiver<T, U, V>
    where
        U: ArrayLength<u8>,
        V: ArrayLength<T>,
    {}
    impl<T, U> Sealed for crate::senders::RealSender<T, U>
    where
        U: ArrayLength<u8>,
    {}
}
use crate::private::Sealed;

#[derive(Eq, PartialEq, Debug)]
enum PingPongMode {
    Idle,
    AActive,
    BActive,
}

#[derive(Serialize, Deserialize, Debug, Eq, PartialEq)]
pub enum LogOnLine<'a, T> {
    Log(&'a str),
    Warn(&'a str),
    Error(&'a str),
    BinaryRaw(BinMessage<'a>),
    ProtocolMessage(T),
}

#[derive(Serialize, Deserialize, Debug, Eq, PartialEq)]
pub struct BinMessage<'a> {
    description: &'a str,
    data: &'a [u8],
}


pub trait Receiver: Default + Sealed {}

pub struct RealReceiver<T, BUFSZ, MSGCT>
where
    BUFSZ: ArrayLength<u8>,
    MSGCT: ArrayLength<T>,
{
    inc_q: Vec<u8, BUFSZ>,
    msg_q: Queue<T, MSGCT>,
    ppm: PingPongMode,
}

pub struct NullReceiver;

impl Receiver for NullReceiver {}


impl<T, BUFSZ, MSGCT> Receiver for RealReceiver<T, BUFSZ, MSGCT>
where
    BUFSZ: ArrayLength<u8>,
    MSGCT: ArrayLength<T>,
{

}

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

impl Default for NullReceiver
{
    fn default() -> Self {
        NullReceiver
    }
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
    pub fn start_receive(&mut self) -> Result<(), ()> {
        if self.recv.ppm != PingPongMode::Idle {
            return Err(());
        }

        self.recv.ppm = PingPongMode::AActive;

        interrupt::free(|cs| unsafe {
            start_read(&mut *A_SIDE.borrow(cs).get()).unwrap();
        });

        Ok(())
    }

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
            start_read(&mut *new.borrow(cs).get()).unwrap();
            (&mut output[..used]).copy_from_slice(&mut (*old.borrow(cs).get())[..used]);
        });

        Ok(&mut output[..used])
    }


    pub fn service_receive(&mut self) -> Result<usize, ()> {
        let mut buf = [0u8; 255];
        let mut less_buf = self.get_pending_manual(&mut buf)?;

        if less_buf.len() > 0 {
            while let Some(idx) = less_buf.iter().position(|&n| n == 0u8) {
                let (frm, lat) = less_buf.split_at_mut(idx + 1);

                self.recv.inc_q.extend_from_slice(frm).unwrap();
                self.recv.msg_q
                    .enqueue(from_bytes_cobs(&mut *self.recv.inc_q).map_err(|_| ())?)
                    .map_err(|_| ())?;
                self.recv.inc_q.clear();

                less_buf = lat;
            }

            self.recv.inc_q.extend_from_slice(less_buf).unwrap();
        }

        Ok(self.recv.msg_q.len())
    }

    pub fn get_msg(&mut self) -> Option<T> {
        self.recv.msg_q.dequeue()
    }
}
