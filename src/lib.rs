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
    pub uart: Uarte<UARTE0>,
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

use bare_metal::Mutex;
use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicUsize, Ordering::{self, SeqCst}, compiler_fence};
use cortex_m::interrupt;

static A_SIDE: Mutex<UnsafeCell<[u8; 255]>> = Mutex::new(UnsafeCell::new([0u8; 255]));
static B_SIDE: Mutex<UnsafeCell<[u8; 255]>> = Mutex::new(UnsafeCell::new([0u8; 255]));
static SELECTOR: AtomicUsize = AtomicUsize::new(IDLE);

const IDLE:         usize = 0b00;
const A_ACTIVE:     usize = 0b01;
const B_ACTIVE:     usize = 0b10;
const MASK_ACTIVE:  usize = 0b11;


pub trait HackRxUart {
    fn start_receive(&mut self);
    fn get_pending<'a, 'b>(&'b mut self, output: &'a mut [u8]) -> Result<&'a mut [u8], ()>;
}

impl HackRxUart for Uarte<UARTE0> {
    fn start_receive(&mut self) {
        interrupt::free(|cs| {

            let val = SELECTOR.compare_and_swap(IDLE, A_ACTIVE, Ordering::SeqCst);
            assert_eq!(val, IDLE);

            unsafe {
                start_read(&mut *A_SIDE.borrow(cs).get()).unwrap();
            }
        });
    }

    fn get_pending<'a, 'b>(&'b mut self, output: &'a mut [u8]) -> Result<&'a mut [u8], ()> {
        let mut used = 0;

        interrupt::free(|cs| {

            let val = SELECTOR.fetch_xor(MASK_ACTIVE, SeqCst);

            let (old, new) = match val {
                A_ACTIVE => (&A_SIDE, &B_SIDE),
                B_ACTIVE => (&B_SIDE, &A_SIDE),
                _ => panic!(),
            };

            let periph = unsafe{ &*UARTE0::ptr() };
            periph.tasks_stoprx.write(|w| unsafe { w.bits(1) });

            while periph.events_endrx.read().bits() != 1 {}

            compiler_fence(SeqCst);

            used = periph.rxd.amount.read().bits() as usize;
            finalize_read();

            unsafe {
                start_read(&mut *new.borrow(cs).get()).unwrap();
                (&mut output[..used]).copy_from_slice(&mut (*old.borrow(cs).get())[..used]);
            }
        });

        Ok(&mut output[..used])
    }
}


/// Start a UARTE read transaction by setting the control
/// values and triggering a read task
fn start_read(rx_buffer: &mut [u8]) -> Result<(), nrf52832_hal::uarte::Error> {
    let periph = unsafe{ &*UARTE0::ptr() };

    // This is overly restrictive. See (similar SPIM issue):
    // https://github.com/nrf-rs/nrf52/issues/17
    if rx_buffer.len() > u8::max_value() as usize {
        return Err(nrf52832_hal::uarte::Error::TxBufferTooLong);
    }

    // Conservative compiler fence to prevent optimizations that do not
    // take in to account actions by DMA. The fence has been placed here,
    // before any DMA action has started
    compiler_fence(SeqCst);

    // Set up the DMA read
    periph.rxd.ptr.write(|w|
        // We're giving the register a pointer to the stack. Since we're
        // waiting for the UARTE transaction to end before this stack pointer
        // becomes invalid, there's nothing wrong here.
        //
        // The PTR field is a full 32 bits wide and accepts the full range
        // of values.
        unsafe { w.ptr().bits(rx_buffer.as_ptr() as u32) }
    );
    periph.rxd.maxcnt.write(|w|
        // We're giving it the length of the buffer, so no danger of
        // accessing invalid memory. We have verified that the length of the
        // buffer fits in an `u8`, so the cast to `u8` is also fine.
        //
        // The MAXCNT field is at least 8 bits wide and accepts the full
        // range of values.
        unsafe { w.maxcnt().bits(rx_buffer.len() as _) });

    // Start UARTE Receive transaction
    periph.tasks_startrx.write(|w|
        // `1` is a valid value to write to task registers.
        unsafe { w.bits(1) });

    Ok(())
}

/// Finalize a UARTE read transaction by clearing the event
fn finalize_read() {
    let periph = unsafe{ &*UARTE0::ptr() };

    // Reset the event, otherwise it will always read `1` from now on.
    periph.events_endrx.write(|w| w);

    // Conservative compiler fence to prevent optimizations that do not
    // take in to account actions by DMA. The fence has been placed here,
    // after all possible DMA actions have completed
    compiler_fence(SeqCst);
}
