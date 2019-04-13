#![no_std]

use serde::{Serialize, Deserialize};
use core::marker::PhantomData;
use core::ops::{Deref};

use nrf52832_hal::{
    uarte::Uarte,
    target_constants::EASY_DMA_SIZE,
    nrf52832_pac::{
        UARTE0,
    },
};

use postcard::to_vec_cobs;
use heapless::{
    ArrayLength,
    Vec,
};

#[derive(Serialize, Deserialize, Debug, Eq, PartialEq)]
pub enum LogOnLine<'a, T>
where
    T: Serialize,
{
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

/// A binary logging interface, using UARTE0.
///
/// In the future other serial interfaces might be supported
pub struct Logger<BUFSZ, T>
where
    BUFSZ: ArrayLength<u8>,
    T: Serialize,
{
    uart: Uarte<UARTE0>,
    _scratch_sz: PhantomData<BUFSZ>,
    _bin_data: PhantomData<T>
}

impl<BUFSZ, T> Logger<BUFSZ, T>
where
    BUFSZ: ArrayLength<u8>,
    T: Serialize,
 {
    pub fn new(mut uart: Uarte<UARTE0>) -> Self {
        // Send termination character
        uart.write(&[0x00]).unwrap();

        Self {
            uart,
            _scratch_sz: PhantomData,
            _bin_data: PhantomData,
        }
    }

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
        self.send(&LogOnLine::BinaryRaw(BinMessage {
            description,
            data,
        }))
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
