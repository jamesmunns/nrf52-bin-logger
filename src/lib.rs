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
pub enum LogOnLine<'a, T> {
    Log(&'a str),
    Warn(&'a str),
    Error(&'a str),
    BinaryRaw(&'a [u8]),
    ProtocolMessage(T),
}

pub struct Logger<BUFSZ, T>
where
    BUFSZ: ArrayLength<u8>,
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

    pub fn log(&mut self, data: &str) -> Result<(), ()> {
        self.send(&LogOnLine::Log(data))
    }

    pub fn warn(&mut self, data: &str) -> Result<(), ()> {
        self.send(&LogOnLine::Warn(data))
    }

    pub fn error(&mut self, data: &str) -> Result<(), ()> {
        self.send(&LogOnLine::Error(data))
    }

    pub fn raw_bin(&mut self, data: &[u8]) -> Result<(), ()> {
        self.send(&LogOnLine::BinaryRaw(data))
    }

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
