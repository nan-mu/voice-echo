use core::{
    future::{poll_fn, Future},
    pin::pin,
    task::Poll,
};

use alloc::boxed::Box;
use esp_hal::analog::adc::{Adc, AdcPin};

fn read_async<'d, ADCI, PIN, CS>(
    adc: Adc<'d, ADCI>,
    pin: &mut AdcPin<PIN, ADCI, CS>,
) -> impl Future<Output = u16>
where
    ADCI: esp_hal::analog::adc::RegisterAccess,
{
    let adc = pin!(adc);
    let pin = pin!(pin);
    let ans = poll_fn(move |ctx| {
        let adc = adc.get_mut();
        let mut pin = pin.as_mut();
        match adc.read_oneshot(&mut pin) {
            Ok(v) => Poll::Ready(v),
            Err(_) => Poll::Pending,
        }
    });
    ans
}
