// Led file for the modular project.
/*!
 * -----------------------------------------------------------------------------
 *  Project     : Led file for the modular project.
 *  File        : led.rs
 *  Created by  : Everton Oriente
 *  Date        : 2025-07-22
 *  * -----------------------------------------------------------------------------
 *  Description :
 *      The module is responsible about to control the LED in the GP16.
 *
 *  Target MCU  : Raspberry Pi Pico W (RP2040 and CYW43)
 *  Framework   : Embassy, no_std
 *
 */

use defmt::*; // For logging via RTT
use embassy_rp::gpio::Output;
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _}; // RTT logging and panic handler

// LED blink task - to the gpio in the board
#[embassy_executor::task]
pub async fn toogle_led(mut led: Output<'static>) {
    loop {
        info!("LED ON");
        led.set_high();
        Timer::after_millis(2500).await;

        info!("LED OFF");
        led.set_low();
        Timer::after_millis(2500).await;
    }
}
