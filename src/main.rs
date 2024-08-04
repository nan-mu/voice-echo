#![no_std]
#![no_main]

use core::{cell::RefCell, mem::MaybeUninit};
use critical_section::Mutex;
use embassy_executor::Spawner;
// use embassy_sync::zerocopy_channel::Channel;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::{
    analog::adc::{Adc, AdcConfig, Attenuation},
    clock::ClockControl,
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
use fugit::{MicrosDurationU32, MillisDurationU32, Rate};
use listen::INTERRUPT_TIME;
use log::{debug, LevelFilter};

type CsMutex<T> = Mutex<RefCell<Option<T>>>;

mod listen;
mod speak;
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

/// 目标声音频率
const TARGET_FREQ: Rate<u32, 1, 1> = Rate::<u32, 1, 1>::Hz(5000);
/// 采样系数，表示采样频率是目标频率的多少倍
const SAMPLE_COEFF: u32 = 2;
/// 采样样本长度
const SAMPLE_LEN: usize = 128;
/// 通信周期内增益计算次数
const GAIN_COUNT: u32 = 24;

#[main]
async fn main(spawner: Spawner) {
    // 采样频率
    let sample_freq = TARGET_FREQ * SAMPLE_COEFF;
    // 增益所在区间索引
    let gain_index = (TARGET_FREQ.to_Hz() * SAMPLE_LEN as u32 / sample_freq.to_Hz()) as usize;
    // 增益计算周期
    let gain_cycle = sample_freq.into_duration() as MicrosDurationU32 * SAMPLE_LEN as u32;
    // 通信周期
    let comm_cycle = gain_cycle * GAIN_COUNT;
    // 增益区间
    let target_range = (
        (sample_freq.to_Hz() as f32 / SAMPLE_LEN as f32) * gain_index as f32,
        (sample_freq.to_Hz() as f32 / SAMPLE_LEN as f32) * (gain_index + 1) as f32,
    );

    // 初始化系统寄存器
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    // "初始化日志并打印功能参数："
    esp_println::logger::init_logger(LevelFilter::Debug);
    debug!("日志初始化完毕");
    debug!("功能相关参数：");
    debug!("目标频率：{}", TARGET_FREQ);
    debug!("采样系数：{}", SAMPLE_COEFF);
    debug!("采样频率：{}", sample_freq);
    debug!("采样样本长度：{}", SAMPLE_LEN);
    debug!("增益区间：({:.2}, {:.2})", target_range.0, target_range.1);
    debug!("增益所在区间索引：{}", gain_index);
    debug!("通信周期内增益计算次数：{}", GAIN_COUNT);
    debug!("通信周期：{}", comm_cycle.convert() as MillisDurationU32);
    debug!(
        "增益计算周期：{}",
        gain_cycle.convert() as MillisDurationU32
    );
    debug!("-------------------------------");
    debug!("初始化动态分配堆");
    init_heap();

    debug!("初始化异步时钟，使用timer group 0");
    let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks, None);
    let timer0: ErasedTimer = timg0.timer0.into();
    let timers = [OneShotTimer::new(timer0)];
    let timers = mk_static!([OneShotTimer<ErasedTimer>; 1], timers);
    esp_hal_embassy::init(&clocks, timers);

    debug!("初始化异步通信通道");
    // let gain_buffer = mk_static!([[f32; 16]; 1], [0.0; 16]);
    // let gain_channel = Channel::new(gain_buffer);

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

    debug!("初始化中断时钟，使用timer group 1");
    let timg1 = TimerGroup::new(peripherals.TIMG1, &clocks, None);
    let timer0: ErasedTimer = timg1.timer0.into();
    let mut timer = PeriodicTimer::new(timer0);
    let sample_cycle: fugit::Duration<u64, 1, 1000000> =
        sample_freq.into_duration::<1, 1_000_000>().into();
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

    debug!("初始化PWM");
    let clocks = mk_static_ref!(clocks);
    let mut pwm_controller = Ledc::new(peripherals.LEDC, clocks); // pwm控制器，提供pwm（ledc）的寄存器访问接口
    pwm_controller.set_global_slow_clock(LSGlobalClkSource::APBClk); // 链接全局慢速时钟
    let pwm_controller = mk_static_ref!(pwm_controller);
    let mut pwm_timer = pwm_controller.get_timer::<LowSpeed>(timer::Number::Timer1);
    pwm_timer
        .configure(config::Config {
            duty: config::Duty::Duty5Bit,
            clock_source: LSClockSource::APBClk,
            frequency: TARGET_FREQ,
        })
        .unwrap();
    let pwm_timer = mk_static_ref!(pwm_timer);

    let mut pwm_channel = pwm_controller.get_channel(channel::Number::Channel0, pwm_pin);
    pwm_channel
        .configure(channel::config::Config {
            timer: pwm_timer,
            duty_pct: 10,
            pin_config: channel::config::PinConfig::PushPull,
        })
        .unwrap();
    pwm_channel.set_duty(0).unwrap();

    let pwm = mk_static_ref!(pwm_channel);
    spawner
        .spawn(speak::speak(comm_cycle.to_micros() as u64, pwm))
        .ok();

    loop {
        Timer::after(Duration::from_micros(
            gain_cycle.to_micros() as u64 - INTERRUPT_TIME * SAMPLE_LEN as u64,
        ))
        .await;
        let gain = listen::rfft(gain_index).await;
        debug!("Gain: {}", gain);
    }
}
