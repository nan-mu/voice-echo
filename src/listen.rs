use crate::{CsMutex, GAIN_INDEX};
use core::cell::RefCell;
use critical_section::Mutex;
use esp_hal::{
    analog::adc::{Adc, AdcCalLine},
    gpio::GpioPin,
    peripherals::{self, ADC1},
    prelude::*,
    timer::{ErasedTimer, PeriodicTimer},
};
use log::debug;
use nb::block;

pub static TIMER1: CsMutex<PeriodicTimer<ErasedTimer>> = Mutex::new(RefCell::new(None));
pub static ADC1: CsMutex<Adc<ADC1>> = Mutex::new(RefCell::new(None));
pub static ADC_PIN: CsMutex<
    esp_hal::analog::adc::AdcPin<GpioPin<2>, peripherals::ADC1, AdcCalLine<peripherals::ADC1>>,
> = Mutex::new(RefCell::new(None));

static RFFT_ARRAY_A: CsMutex<[f32; crate::SAMPLE_LEN]> =
    Mutex::new(RefCell::new(Some([0.0; crate::SAMPLE_LEN])));
static RFFT_ARRAY_B: CsMutex<[f32; crate::SAMPLE_LEN]> =
    Mutex::new(RefCell::new(Some([0.0; crate::SAMPLE_LEN])));
static RFFT_INDEX: CsMutex<u16> = Mutex::new(RefCell::new(Some(0)));
static AB_FLAG: CsMutex<bool> = Mutex::new(RefCell::new(Some(false)));

#[handler]
pub fn adc_to_rfft() {
    // 固定时间间隔中读取电平
    critical_section::with(|cs| {
        TIMER1 //清除中断标志
            .borrow(cs)
            .borrow_mut()
            .as_mut()
            .unwrap()
            .clear_interrupt();

        let mut adc1 = ADC1.borrow(cs).borrow_mut();
        let adc1 = adc1.as_mut().unwrap();
        let mut adc_pin = ADC_PIN.borrow(cs).borrow_mut();
        let adc_pin = adc_pin.as_mut().unwrap();
        let pin_mv = block!(adc1.read_oneshot(adc_pin)).unwrap();
        let mut rfft_index = RFFT_INDEX.borrow(cs).borrow_mut();
        let rfft_index = rfft_index.as_mut().unwrap();
        // 这里是怕一个每次时钟中断中间算不完傅里叶，所以用两个数组交替。要是两个数组还不够就应该remake
        // 那么按道理来说这个互斥锁是可以不用加的，当懒得改了。
        let rfft_array = match *rfft_index {
            // 这里左闭右开区间不知道为什么不能用
            0..=1023 => &RFFT_ARRAY_A,
            1024 => {
                *AB_FLAG.borrow(cs).borrow_mut().as_mut().unwrap() = true;
                &RFFT_ARRAY_B
            }
            1025..=2047 => &RFFT_ARRAY_B,
            _ => {
                *rfft_index = 0;
                *AB_FLAG.borrow(cs).borrow_mut().as_mut().unwrap() = false;
                &RFFT_ARRAY_A
            }
        };
        let mut rfft_array = rfft_array.borrow(cs).borrow_mut();
        let rfft_array = rfft_array.as_mut().unwrap();

        rfft_array[rfft_index.clone() as usize] = pin_mv as f32;
    });
}

pub async fn rfft() -> f32 {
    let mut gain = 0.0;
    critical_section::with(|cs| {
        let rfft_array = match *AB_FLAG.borrow(cs).borrow_mut().as_mut().unwrap() {
            true => &RFFT_ARRAY_A,
            false => &RFFT_ARRAY_B,
        };
        let mut rfft_array = rfft_array.borrow(cs).borrow_mut();
        let rfft_array = rfft_array.as_mut().unwrap();
        let spectrum = microfft::real::rfft_1024(rfft_array);
        debug!("{:?}", spectrum);
        gain = spectrum[GAIN_INDEX].l1_norm();
    });
    gain
}
