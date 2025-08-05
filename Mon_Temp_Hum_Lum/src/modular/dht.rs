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

// Always compile in release mode, because the dht sensor maybe not work in debug mode

use defmt::info;
use embassy_dht_sensor::DHTSensor;
use embassy_rp::gpio::Flex;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::watch::{DynReceiver, Watch};
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

const DHT_CONSUMERS_TEMPERATURE: usize = 2;
static DHT_CHANNEL_TEMPERATURE: Watch<ThreadModeRawMutex, f32, DHT_CONSUMERS_TEMPERATURE> = Watch::new();

pub fn get_receiver_dht_temperature() -> Option<DynReceiver<'static, f32>> {
    DHT_CHANNEL_TEMPERATURE.dyn_receiver()
}

const DHT_CONSUMERS_HUMIDITY: usize = 2;
static DHT_CHANNEL_HUMIDITY: Watch<ThreadModeRawMutex, f32, DHT_CONSUMERS_HUMIDITY> = Watch::new();

pub fn get_receiver_dht_humidity() -> Option<DynReceiver<'static, f32>> {
    DHT_CHANNEL_HUMIDITY.dyn_receiver()
}
#[embassy_executor::task]
pub async fn dht_task(pin: Flex<'static>) {
    info!("Initializing DHT11");
    let mut dht_sensor = DHTSensor::new(pin);
    let tx_dht_temperature = DHT_CHANNEL_TEMPERATURE.sender();
    let tx_dht_humidity = DHT_CHANNEL_HUMIDITY.sender();

    loop {
        info!("Reading of DHT");

        match dht_sensor.read() {
            Ok(data) => {
                tx_dht_temperature.send(data.temperature);
                tx_dht_humidity.send(data.humidity);
                info!("Temperature: {:?}, Humidity: {}", data.temperature, data.humidity);
            }
            Err(e) => {
                info!("Error reading from DHT sensor: {}", defmt::Debug2Format(&e));
            }
        }
        Timer::after(Duration::from_secs(300)).await;
    }
}
