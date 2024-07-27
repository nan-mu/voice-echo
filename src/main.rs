#![no_std]
#![no_main]

use alloc::vec::Vec;
use esp_backtrace as _;
use esp_hal::{
    analog::adc::{Adc, AdcConfig, Attenuation},
    clock::ClockControl,
    delay::Delay,
    gpio::Io,
    peripherals::Peripherals,
    prelude::*,
    system::SystemControl,
};
use esp_println::println;
use microfft::real;

extern crate alloc;
use core::mem::MaybeUninit;

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

fn init_heap() {
    const HEAP_SIZE: usize = 32 * 1024;
    static mut HEAP: MaybeUninit<[u8; HEAP_SIZE]> = MaybeUninit::uninit();

    unsafe {
        ALLOCATOR.init(HEAP.as_mut_ptr() as *mut u8, HEAP_SIZE);
    }
}

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    let analog_pin = io.pins.gpio2;

    // Create ADC instances
    // You can try any of the following calibration methods by uncommenting
    // them. Note that only AdcCalLine returns readings in mV; the other two
    // return raw readings in some unspecified scale.
    //
    //type AdcCal = ();
    type AdcCal = esp_hal::analog::adc::AdcCalBasic<esp_hal::peripherals::ADC1>;
    // type AdcCal = esp_hal::analog::adc::AdcCalLine<ADC1>;
    // type AdcCal = esp_hal::analog::adc::AdcCalCurve<ADC1>;
    init_heap();

    let mut adc1_config = AdcConfig::new();
    let mut adc1_pin =
        adc1_config.enable_pin_with_cal::<_, AdcCal>(analog_pin, Attenuation::Attenuation11dB);
    let mut adc1 = Adc::new(peripherals.ADC1, adc1_config);

    let delay = Delay::new(&clocks);

    let mut sample: [f32; 1024] = [0.0; 1024];
    let mut sample_count = 0;
    loop {
        delay.delay_micros(4);
        let pin_mv = nb::block!(adc1.read_oneshot(&mut adc1_pin)).unwrap();
        println!("{:?}", pin_mv);
        // sample_count += 1;
        // sample[sample_count] = (pin_mv as f32) / 1000.0;
        // if sample_count == 1023 {
        //     let spectrum = real::rfft_1024(&mut sample);
        //     let amplitudes: Vec<_> = spectrum.iter().map(|c| (c.norm_sqr() as f32)).collect();
        //     println!("{}", amplitudes[66]);
        //     sample_count = 0;
        // }
    }
}
