use core::cell::RefCell;
use critical_section::Mutex;
use esp_hal::{
    analog::adc::{Adc, AdcCalLine},
    gpio::GpioPin,
    peripherals::{self, ADC1},
    prelude::*,
    timer::{systimer::SystemTimer, ErasedTimer, PeriodicTimer},
};
use esp_println::println;

use crate::CsMutex;
pub static TIMER1: CsMutex<PeriodicTimer<ErasedTimer>> = Mutex::new(RefCell::new(None));
pub static ADC1: CsMutex<Adc<ADC1>> = Mutex::new(RefCell::new(None));
pub static ADC_PIN: CsMutex<
    esp_hal::analog::adc::AdcPin<GpioPin<2>, peripherals::ADC1, AdcCalLine<peripherals::ADC1>>,
> = Mutex::new(RefCell::new(None));

#[handler]
pub fn adc_to_rfft() {
    println!("now: {}", SystemTimer::now());
    critical_section::with(|cs| {
        let mut timer1 = TIMER1.borrow(cs).borrow_mut();
        let timer1 = timer1.as_mut().unwrap();
        timer1.clear_interrupt();
    })
}
