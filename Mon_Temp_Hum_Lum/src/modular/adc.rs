// ADC file for the modular project.
/*!
 * -----------------------------------------------------------------------------
 *  Project     : ADC file for the modular project.
 *  File        : adc.rs
 *  Created by  : Everton Oriente
 *  Date        : 2025-07-22
 *  * -----------------------------------------------------------------------------
 *  Description :
 *      The module is responsible about to acquire and send the information regarding the temperature,
 *      humidity and luminosity from the sensors.
 *
 *  Target MCU  : Raspberry Pi Pico W (RP2040 and CYW43)
 *  Framework   : Embassy, no_std
 *
 */

use defmt::*; // For logging via RTT
use embassy_rp::adc::{Adc, Async, Channel};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::watch::{DynReceiver, Watch};
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _}; // RTT logging and panic handler

const ADC0_CONSUMERS: usize = 3;
static ADC0_CHANNEL: Watch<ThreadModeRawMutex, u16, ADC0_CONSUMERS> = Watch::new();

pub fn get_receiver_adc0() -> Option<DynReceiver<'static, u16>> {
    ADC0_CHANNEL.dyn_receiver()
}
/*
const ADC1_CONSUMERS: usize = 1;
static ADC1_CHANNEL: Watch<ThreadModeRawMutex, u16, ADC1_CONSUMERS> = Watch::new();

pub fn get_receiver_adc1() -> Option<DynReceiver<'static, u16>> {
    ADC1_CHANNEL.dyn_receiver()
}
*/

/*
const ADC2_CONSUMERS: usize = 1;
static ADC2_CHANNEL: Watch<ThreadModeRawMutex, u16, ADC2_CONSUMERS> = Watch::new();

pub fn get_receiver_adc2() -> Option<DynReceiver<'static, u16>> {
    ADC2_CHANNEL.dyn_receiver()
}
*/

const ADCTEMP_CONSUMERS: usize = 2;
static ADCTEMP_CHANNEL: Watch<ThreadModeRawMutex, u16, ADCTEMP_CONSUMERS> = Watch::new();

pub fn get_receiver_adctemp() -> Option<DynReceiver<'static, u16>> {
    ADCTEMP_CHANNEL.dyn_receiver()
}

// This task is used to read all the ADC channels regarding the RPI Pico W, where ADC0-ADC2 can be used to measure anything from 0 to 3.3V.
// ADC3 is used to measure the temperature die of the RP2040 or RP2350.
#[embassy_executor::task]
pub async fn read_adc_channels(
    adc_mutex: &'static Mutex<ThreadModeRawMutex, Adc<'static, Async>>,
    mut chan_0: Channel<'static>, // Luminosity
    //mut chan_1: Channel<'static>,
    //mut chan_2: Channel<'static>,
    mut chan_temp: Channel<'static>, // Temperature of the DIE
) {
    loop {
        info!("Reading luminosity...");
        let tx_adc0 = ADC0_CHANNEL.sender();
        // let tx_adc1 = ADC1_CHANNEL.sender();
        // let tx_adc2 = ADC2_CHANNEL.sender();
        let tx_adctemp = ADCTEMP_CHANNEL.sender();

        let result_adc_0 = {
            let mut adc_channel_0 = adc_mutex.lock().await;
            adc_channel_0.read(&mut chan_0).await
        };

        match result_adc_0 {
            Ok(value) => {
                info!("Luminosity: {}", value);
                // Send the value to the channel
                tx_adc0.send(value);
            }
            Err(e) => error!("ADC read error: {}", e),
        }
        /*
        info!("Reading ADC_1...");
        let result_adc_1 = {
            let adc_refcell_adc_channel_1 = adc_mutex.lock().await;
            let mut adc_channel_1 = adc_refcell_adc_channel_1.borrow_mut();
            adc_channel_1.read(&mut chan_1).await
        };

        match result_adc_1 {
            Ok(value) => {
                info!("Luminosity: {}", value);
                // Send the value to the channel
                tx_adc1.send(value);
            }
            Err(e) => error!("ADC read error: {}", e),
        }
        */
        /*
        info!("Reading ADC_2...");
        let result_adc_2 = {
            let adc_refcell_adc_channel_2 = adc_mutex.lock().await;
            let mut adc_channel_2 = adc_refcell_adc_channel_2.borrow_mut();
            adc_channel_2.read(&mut chan_2).await
        };

        match result_adc_2 {
            Ok(value) => {
                info!("Luminosity: {}", value);
                // Send the value to the channel
                tx_adc2.send(value);
            }
            Err(e) => error!("ADC read error: {}", e),
        }
        */
        // The temperature should never rises up to 75°C, because this is the limit of the chip.
        info!("Reading temperature of the die...");
        let result_adc_temp = {
            let mut adc_channel_temp = adc_mutex.lock().await;
            adc_channel_temp.read(&mut chan_temp).await
        };

        match result_adc_temp {
            Ok(raw) => {
                let voltage = raw as f32 * 3.3 / 4096.0;
                let temp = 27.0 - (voltage - 0.706) / 0.001721;
                info!("Temp Die: {} °C (raw: {})", temp, raw);
                tx_adctemp.send(raw);
            }
            Err(e) => error!("Temp read error: {}", e),
        }

        Timer::after_millis(300_000).await; // Wait for 5 minutes before reading again
    }
}
