// CYW43 file for the modular project.
/*!
 * -----------------------------------------------------------------------------
 *  Project     : CYW43 file for the modular project.
 *  File        : cyw43.rs
 *  Created by  : Everton Oriente
 *  Date        : 2025-07-22
 *  * -----------------------------------------------------------------------------
 *  Description :
 *      The module is responsible about the interaction with the CYW43 module, about the MQTT protocol, the messages with the values of .
 *
 *  Target MCU  : Raspberry Pi Pico W (RP2040 and CYW43)
 *  Framework   : Embassy, no_std
 *
 */

use cyw43_pio::PioSpi;
use defmt::*; // For logging via RTT
use embassy_rp::gpio::Output;
use embassy_rp::peripherals::{DMA_CH0, PIO0};
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _}; // RTT logging and panic handler

/// Background task for running the CYW43 Wi-Fi driver.
/// This must be running at all times for the chip to function.
#[embassy_executor::task]
pub async fn cyw43_task(runner: cyw43::Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>) -> ! {
    info!("Starting runner cyw43");
    runner.run().await
}

/// LED blink task — toggles CYW43 GPIO 0 (onboard LED) on and off with the given delay.
#[embassy_executor::task]
pub async fn led_blink_task(mut control: cyw43::Control<'static>, delay: Duration) {
    loop {
        info!("led on!");
        control.gpio_set(0, true).await; // Turn LED on
        Timer::after(delay).await;

        info!("led off!");
        control.gpio_set(0, false).await; // Turn LED off
        Timer::after(delay).await;
    }
}
