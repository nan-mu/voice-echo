#![no_std]
#![no_main]

use core::{cell::RefCell, mem::MaybeUninit};
use critical_section::Mutex;
use embassy_executor::Spawner;
use esp_backtrace as _;
use esp_hal::{
    analog::adc::{Adc, AdcConfig, Attenuation},
    clock::ClockControl,
    delay::Delay,
    gpio::Io,
    interrupt::{self, Priority},
    ledc::{
        channel,
        timer::{self, config, LSClockSource},
        LSGlobalClkSource, Ledc, LowSpeed,
    },
    peripherals::{self, Interrupt, Peripherals},
    prelude::*,
    system::SystemControl,
    timer::{timg::TimerGroup, ErasedTimer, OneShotTimer, PeriodicTimer},
};
use fugit::Rate;
use log::{debug, LevelFilter};

type CsMutex<T> = Mutex<RefCell<Option<T>>>;

mod listen;
#[macro_use]
mod macros;

extern crate alloc;

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

fn init_heap() {
    const HEAP_SIZE: usize = 32 * 1024;
    static mut HEAP: MaybeUninit<[u8; HEAP_SIZE]> = MaybeUninit::uninit();

    unsafe {
        ALLOCATOR.init(HEAP.as_mut_ptr() as *mut u8, HEAP_SIZE);
    }
}

const TARGET_FREQ: u32 = 5000;
const SAMPLE_FREQ: u32 = TARGET_FREQ * 4;
const SAMPLE_LEN: usize = 1024;
const GAIN_INDEX: usize = TARGET_FREQ as usize * SAMPLE_LEN / SAMPLE_FREQ as usize;

#[main]
async fn main(_spawner: Spawner) {
    // 初始化系统寄存器
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    // "初始化日志和延时"
    esp_println::logger::init_logger(LevelFilter::Debug);
    let delay = Delay::new(&clocks);

    debug!("初始化动态分配堆");
    init_heap();

    debug!("初始化异步时钟，使用timer group 0");
    let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks, None);
    let timer0: ErasedTimer = timg0.timer0.into();
    let timers = [OneShotTimer::new(timer0)];
    let timers = mk_static!([OneShotTimer<ErasedTimer>; 1], timers);
    esp_hal_embassy::init(&clocks, timers);

    debug!("初始化引脚");
    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    let adc_pin = io.pins.gpio2; //模拟引脚，输入PCM麦克风信号
    let pwm_pin = io.pins.gpio0; // 数字引脚，输出PWM控制蜂鸣器

    debug!("初始化ADC");
    // 这里是三种校准模式，分别为基础校准、线性校准和曲线校准。精度依次提高，速度依次下降。
    // type AdcCal = esp_hal::analog::adc::AdcCalBasic<peripherals::ADC1>;
    type AdcCal = esp_hal::analog::adc::AdcCalLine<peripherals::ADC1>;
    // type AdcCal = esp_hal::analog::adc::AdcCalCurve<peripherals::ADC1>;
    let mut adc1_config = AdcConfig::new();
    let adc_pin =
        adc1_config.enable_pin_with_cal::<_, AdcCal>(adc_pin, Attenuation::Attenuation11dB);
    let adc1 = Adc::new(peripherals.ADC1, adc1_config);

    debug!("初始化PWM");
    let mut pwm_controller = Ledc::new(peripherals.LEDC, &clocks); // pwm控制器，提供pwm（ledc）的寄存器访问接口
    pwm_controller.set_global_slow_clock(LSGlobalClkSource::APBClk); // 链接全局慢速时钟
    let mut pwm_timer = pwm_controller.get_timer::<LowSpeed>(timer::Number::Timer1);
    pwm_timer
        .configure(config::Config {
            duty: config::Duty::Duty5Bit,
            clock_source: LSClockSource::APBClk,
            frequency: TARGET_FREQ.Hz(),
        })
        .unwrap();
    let mut pwm_channel = pwm_controller.get_channel(channel::Number::Channel0, pwm_pin);
    pwm_channel
        .configure(channel::config::Config {
            timer: &pwm_timer,
            duty_pct: 10,
            pin_config: channel::config::PinConfig::PushPull,
        })
        .unwrap();

    debug!("初始化中断时钟，使用timer group 1");
    let timg1 = TimerGroup::new(peripherals.TIMG1, &clocks, None);
    let timer0: ErasedTimer = timg1.timer0.into();
    let mut timer = PeriodicTimer::new(timer0);
    let sample_freq = Rate::<u32, 1, 1>::Hz(SAMPLE_FREQ);
    let sample_cycle = sample_freq.into_duration::<1, 1_000_000>().into();
    critical_section::with(|cs| {
        debug!("发射时钟中断，周期：{}", sample_cycle);
        timer.start(sample_cycle).unwrap();
        timer.set_interrupt_handler(listen::adc_to_rfft);
        timer.enable_interrupt(true);
        listen::TIMER1.borrow(cs).replace(Some(timer));
        listen::ADC1.borrow(cs).replace(Some(adc1));
        listen::ADC_PIN.borrow(cs).replace(Some(adc_pin));
    });
    interrupt::enable(Interrupt::TG1_T0_LEVEL, Priority::Priority1).unwrap();

    loop {
        // Wait for 4 seconds
        delay.delay(4.secs());
        let gain = listen::rfft().await;
        debug!("Gain: {}", gain);
    }
}
