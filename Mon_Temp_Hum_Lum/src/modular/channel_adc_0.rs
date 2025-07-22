// Watch file for the modular project.
/*!
 * -----------------------------------------------------------------------------
 *  Project     : Channel/Watch file for the modular project.
 *  File        : adc.rs
 *  Created by  : Everton Oriente
 *  Date        : 2025-07-22
 *  * -----------------------------------------------------------------------------
 *  Description :
 *      The module is responsible about to test about the information through channel/watch, where i can generate
 *      one information and exchange with all task the same value, like temperature humidity and lumonisity.
 *      here we can test if the information is correct.
 *      Receive information.
 *
 *  Target MCU  : Raspberry Pi Pico W (RP2040 and CYW43)
 *  Framework   : Embassy, no_std
 *
 */

use defmt::info;
use embassy_time::Timer;

use crate::modular::adc::get_receiver_adc0;

#[embassy_executor::task]
pub async fn process_adc_channel_0() {
    loop {
        let mut rx = get_receiver_adc0().unwrap();
        let adc0 = rx.get().await;
        info!("LUZ CHEGOU COM: {}", adc0);
        Timer::after_secs(4).await;
    }
}
