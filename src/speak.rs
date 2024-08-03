use embassy_time::{Duration, Timer};
use esp_hal::{
    gpio::GpioPin,
    ledc::{channel::Channel, LowSpeed},
    prelude::_esp_hal_ledc_channel_ChannelIFace,
};

use crate::COMM_FREQ;

#[embassy_executor::task]
pub async fn speak(pwm: &'static Channel<'static, LowSpeed, GpioPin<0>>) {
    loop {
        pwm.set_duty(50).unwrap();
        Timer::after(Duration::from_hz((COMM_FREQ * 2) as u64)).await;
        pwm.set_duty(0).unwrap();
        Timer::after(Duration::from_hz((COMM_FREQ * 2) as u64)).await;
    }
}
