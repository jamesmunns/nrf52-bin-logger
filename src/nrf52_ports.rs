////////////////////////////////////////////////////////////////////////
// These functions are taken from nrf52-hal. This is because
// they are private functions (rightly so), but we need this
// behavior to enable non-blocking reading to the ping pong
// buffers

use core::sync::atomic::{compiler_fence, Ordering::SeqCst};
use nrf52832_hal::nrf52832_pac::UARTE0;

/// Start a UARTE read transaction by setting the control
/// values and triggering a read task
pub(crate) fn start_read(
    rx_buffer: &mut [u8],
    flush: bool,
) -> Result<(), nrf52832_hal::uarte::Error> {
    let periph = unsafe { &*UARTE0::ptr() };

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
        unsafe { w.ptr().bits(rx_buffer.as_ptr() as u32) });
    periph.rxd.maxcnt.write(|w|
        // We're giving it the length of the buffer, so no danger of
        // accessing invalid memory. We have verified that the length of the
        // buffer fits in an `u8`, so the cast to `u8` is also fine.
        //
        // The MAXCNT field is at least 8 bits wide and accepts the full
        // range of values.
        unsafe { w.maxcnt().bits(rx_buffer.len() as _) });

    if flush {
        periph.tasks_flushrx.write(|w| unsafe { w.bits(1) });
        while periph.events_endrx.read().bits() != 1 {}
        periph.events_endrx.write(|w| unsafe { w.bits(0) });
    }

    // Start UARTE Receive transaction
    periph.tasks_startrx.write(|w|
        // `1` is a valid value to write to task registers.
        unsafe { w.bits(1) });

    Ok(())
}
