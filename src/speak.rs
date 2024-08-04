use embassy_time::{Duration, Timer};
use esp_hal::{
    gpio::GpioPin,
    ledc::{channel::Channel, LowSpeed},
    prelude::_esp_hal_ledc_channel_ChannelIFace,
};

#[embassy_executor::task]
pub async fn speak(comm_cycle: u64, pwm: &'static Channel<'static, LowSpeed, GpioPin<0>>) {
    loop {
        pwm.set_duty(50).unwrap();
        Timer::after(Duration::from_micros(comm_cycle / 2)).await;
        pwm.set_duty(0).unwrap();
        Timer::after(Duration::from_micros(comm_cycle / 2)).await;
    }
}
