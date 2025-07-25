// Watch file for the modular project.
/*!
 * -----------------------------------------------------------------------------
 *  Project     : Channel/Watch file for the modular project.
 *  File        : channel_temp.rs
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

use crate::modular::adc::get_receiver_adctemp;

#[embassy_executor::task]
pub async fn process_adc_channel_temp() {
    loop {
        let mut rx = get_receiver_adctemp().unwrap();
        let adctemp = rx.get().await;
        info!("TEMP CHEGOU COM: {}", adctemp);
        Timer::after_secs(4).await;
    }
}
