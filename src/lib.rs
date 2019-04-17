#![no_std]

use core::ops::Deref;

use nrf52832_hal::{
    uarte::Uarte,
    target_constants::EASY_DMA_SIZE,
    nrf52832_pac::{
        UARTE0,
    },
};

use bare_metal::Mutex;
use core::cell::UnsafeCell;
use core::sync::atomic::{Ordering::SeqCst, compiler_fence};
use cortex_m::interrupt;
use heapless::{
    ArrayLength,
    Vec,
    spsc::Queue,
};
use postcard::{to_vec_cobs, from_bytes_cobs};
use serde::{Serialize, de::DeserializeOwned, Deserialize};


static A_SIDE: Mutex<UnsafeCell<[u8; 255]>> = Mutex::new(UnsafeCell::new([0u8; 255]));
static B_SIDE: Mutex<UnsafeCell<[u8; 255]>> = Mutex::new(UnsafeCell::new([0u8; 255]));

#[derive(Eq, PartialEq, Debug)]
enum PingPongMode {
    Idle,
    AActive,
    BActive
}

#[derive(Serialize, Deserialize, Debug, Eq, PartialEq)]
pub enum LogOnLine<'a, T>
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
pub struct Logger<BUFSZ, T, MSGS>
where
    BUFSZ: ArrayLength<u8>,
    MSGS: ArrayLength<T>,
{
    uart: Uarte<UARTE0>,
    inc_q: Vec<u8, BUFSZ>,
    msg_q: Queue<T, MSGS>,
    ppm: PingPongMode,
}

impl<BUFSZ, T, MSGS> Logger<BUFSZ, T, MSGS>
where
    BUFSZ: ArrayLength<u8>,
    MSGS: ArrayLength<T>,
    T: Serialize,
 {
    pub fn new(mut uart: Uarte<UARTE0>) -> Self {
        // Send termination character
        uart.write(&[0x00]).unwrap();

        Self {
            uart,
            inc_q: Vec::new(),
            msg_q: Queue::new(),
            ppm: PingPongMode::Idle,
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

    pub fn start_receive(&mut self) -> Result<(), ()> {
        if self.ppm != PingPongMode::Idle {
            return Err(());
        }

        self.ppm = PingPongMode::AActive;

        interrupt::free(|cs| {
            unsafe {
                start_read(&mut *A_SIDE.borrow(cs).get()).unwrap();
            }
        });

        Ok(())
    }

    pub fn get_pending_manual<'a, 'b>(&'b mut self, output: &'a mut [u8]) -> Result<&'a mut [u8], ()> {
        let (old, new, new_ppm) = match self.ppm {
            PingPongMode::AActive => (&A_SIDE, &B_SIDE, PingPongMode::BActive),
            PingPongMode::BActive => (&B_SIDE, &A_SIDE, PingPongMode::AActive),
            _ => return Err(())
        };

        self.ppm = new_ppm;

        let periph = unsafe{ &*UARTE0::ptr() };
        periph.tasks_stoprx.write(|w| unsafe { w.bits(1) });

        while periph.events_endrx.read().bits() != 1 {}

        compiler_fence(SeqCst);

        let used = periph.rxd.amount.read().bits() as usize;
        finalize_read();

        interrupt::free(|cs| {
            unsafe {
                start_read(&mut *new.borrow(cs).get()).unwrap();
                (&mut output[..used]).copy_from_slice(&mut (*old.borrow(cs).get())[..used]);
            }
        });

        Ok(&mut output[..used])
    }
}

impl<BUFSZ, T, MSGS> Logger<BUFSZ, T, MSGS>
where
    BUFSZ: ArrayLength<u8>,
    MSGS: ArrayLength<T>,
    T: Serialize + DeserializeOwned,
{
    pub fn service_receive(&mut self) -> Result<usize, ()> {
        let mut buf = [0u8; 255];
        let mut less_buf = self.get_pending_manual(&mut buf)?;

        if less_buf.len() > 0 {
            while let Some(idx) = less_buf.iter().position(|&n| n == 0u8) {
                let (frm, lat) = less_buf.split_at_mut(idx + 1);

                self.inc_q.extend_from_slice(frm).unwrap();
                self.msg_q.enqueue(
                    from_bytes_cobs(&mut *self.inc_q)
                        .map_err(|_| ())?)
                    .map_err(|_| ())?;
                self.inc_q.clear();

                less_buf = lat;
            }

            self.inc_q.extend_from_slice(less_buf).unwrap();
        }

        Ok(self.msg_q.len())
    }

    pub fn get_msg(&mut self) -> Option<T> {
        self.msg_q.dequeue()
    }
}

////////////////////////////////////////////////////////////////////////
// These functions are taken from nrf52-hal. This is because
// they are private functions (rightly so), but we need this
// behavior to enable non-blocking reading to the ping pong
// buffers


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
