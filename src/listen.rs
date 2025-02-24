use crate::{CsMutex, GAIN_INDEX};
use core::cell::{Cell, RefCell};
use critical_section::Mutex;
use esp_hal::{
    analog::adc::{Adc, AdcCalLine},
    gpio::GpioPin,
    peripherals::{self, ADC1},
    prelude::*,
    timer::{ErasedTimer, PeriodicTimer},
};
// use log::debug;
use nb::block;

pub static TIMER1: CsMutex<PeriodicTimer<ErasedTimer>> = Mutex::new(RefCell::new(None));
pub static ADC1: CsMutex<Adc<ADC1>> = Mutex::new(RefCell::new(None));
pub static ADC_PIN: CsMutex<
    esp_hal::analog::adc::AdcPin<GpioPin<2>, peripherals::ADC1, AdcCalLine<peripherals::ADC1>>,
> = Mutex::new(RefCell::new(None));

const ARRAY_REPEAT_VALUE: Cell<u16> = Cell::new(0);
static RFFT_ARRAY_A: Mutex<[Cell<u16>; crate::SAMPLE_LEN]> =
    Mutex::new([ARRAY_REPEAT_VALUE; crate::SAMPLE_LEN]);
static RFFT_ARRAY_B: Mutex<[Cell<u16>; crate::SAMPLE_LEN]> =
    Mutex::new([ARRAY_REPEAT_VALUE; crate::SAMPLE_LEN]);

static RFFT_INDEX: Mutex<Cell<usize>> = Mutex::new(Cell::new(0));
/// 为了交替使用两个数组，true表示A可以使用，false表示B可以使用
static AB_FLAG: Mutex<Cell<bool>> = Mutex::new(Cell::new(false));

/// 固定时间间隔中读取电平
#[handler]
pub fn adc_to_rfft() {
    // 清除中断标志
    critical_section::with(|cs| {
        TIMER1
            .borrow(cs)
            .borrow_mut()
            .as_mut()
            .unwrap()
            .clear_interrupt();
    });

    let mut pin_mv = 0;
    // 读取电平
    critical_section::with(|cs| {
        let mut adc1 = ADC1.borrow(cs).borrow_mut();
        let adc1 = adc1.as_mut().unwrap();
        let mut adc_pin = ADC_PIN.borrow(cs).borrow_mut();
        let adc_pin = adc_pin.as_mut().unwrap();
        pin_mv = block!(adc1.read_oneshot(adc_pin)).unwrap();
    });

    let mut rfft_index = 0;
    // 获得数组的索引
    critical_section::with(|cs| {
        rfft_index = RFFT_INDEX.borrow(cs).get();
    });

    let static_arrary = match rfft_index {
        0..=1023 => &RFFT_ARRAY_A,
        1024 => {
            critical_section::with(|cs| {
                AB_FLAG.borrow(cs).set(true);
            });
            &RFFT_ARRAY_B
        }
        1025..=2047 => &RFFT_ARRAY_B,
        _ => {
            rfft_index = 0;
            critical_section::with(|cs| {
                AB_FLAG.borrow(cs).set(false);
            });
            &RFFT_ARRAY_A
        }
    };

    critical_section::with(|cs| {
        // 这里是怕一个每次时钟中断中间算不完傅里叶，所以用两个数组交替。要是两个数组还不够就应该remake
        // 那么按道理来说这个互斥锁是可以不用加的，当懒得改了。
        // debug!("rfft_index: {}", rfft_index);
        static_arrary.borrow(cs)[match rfft_index >= 1024 {
            true => rfft_index - 1024,
            false => rfft_index,
        }]
        .set(pin_mv);
        RFFT_INDEX.borrow(cs).set(rfft_index + 1);
    });
}

pub async fn rfft() -> f32 {
    let mut rfft_array = [0.0; crate::SAMPLE_LEN];

    // 关键区域会禁用中断，需要其中不能有高cpu负载的任务
    critical_section::with(|cs| {
        rfft_array = match AB_FLAG.borrow(cs).get() {
            true => &RFFT_ARRAY_A,
            false => &RFFT_ARRAY_B,
        }
        .borrow(cs)
        .clone()
        .map(|val| val.get() as f32);
    });
    // debug!("{:?}", rfft_array);
    let spectrum = microfft::real::rfft_1024(&mut rfft_array);
    spectrum[GAIN_INDEX].l1_norm()
}
