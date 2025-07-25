// DHT file for the modular project.
/*!
 * -----------------------------------------------------------------------------
 *  Project     : DHT file for the modular project.
 *  File        : dht.rs
 *  Created by  : Everton Oriente
 *  Date        : 2025-07-22
 *  * -----------------------------------------------------------------------------
 *  Description :
 *      The module is responsible about to acquire and send the environment temperature and
 *      humidity.
 *
 *  Target MCU  : Raspberry Pi Pico W (RP2040 and CYW43)
 *  Framework   : Embassy, no_std
 *
 */

use embassy_time::{Duration, Timer};
use embassy_rp::gpio::{Flex};
use embassy_dht_sensor::{DHTSensor};
use {defmt::info, defmt_rtt as _, panic_probe as _};


#[embassy_executor::task]
pub async fn dht_task(pin: Flex<'static>) {
info!("Initializing DHT11");
let mut dht_sensor = DHTSensor::new(pin);


    loop {
        info!("Initializing reading of DHT");
        match dht_sensor.read() {
            Ok(data) => {
                info!("Temperature: {:?}, Humidity: {}", data.temperature, data.humidity);
            },
            Err(e) => {
                info!("Error reading from DHT sensor: {}", defmt::Debug2Format(&e));
            }
        }
        Timer::after(Duration::from_secs(5)).await;
    }
}