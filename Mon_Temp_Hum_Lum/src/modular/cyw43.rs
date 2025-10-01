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

//use core::str::from_utf8;
use embassy_net::tcp::TcpSocket;
use cyw43_pio::PioSpi;
use defmt::{info, warn};
// For logging via RTT
use embassy_rp::gpio::Output;
use embassy_rp::peripherals::{DMA_CH0, PIO0};
use embassy_time::{Duration};
use core::fmt::Write;
use heapless::String;
use rust_mqtt::client::client::MqttClient;
use rust_mqtt::client::client_config::{ClientConfig, MqttVersion};
use rust_mqtt::network::NetworkConnection;
use rust_mqtt::packet::v5::publish_packet::{PublishPacket, QualityOfService};
use rand_core::RngCore;

use {defmt_rtt as _, panic_probe as _}; // RTT logging and panic handler


use crate::modular::adc::{get_receiver_adc0};
use crate::modular::{get_receiver_dht_humidity, get_receiver_dht_temperature};

// MQTT Broker settings
const MQTT_BROKER: &str = "broker.emqx.io";  // EMQX broker endpoint
const MQTT_TOPIC: &str = "emqx/rp2040";     // MQTT topic
const MQTT_USERNAME: &str = "emqx";  // MQTT username for authentication
const MQTT_PASSWORD: &str = "public";  // MQTT password for authentication
const MQTT_PORT: u16 = 1883;  // MQTT port (TCP)


pub struct SimpleRng(u32);

    impl SimpleRng {
        // Initialize with a seed.
        pub fn new(seed: u32) -> Self {
            Self(seed)
        }
    }

    impl RngCore for SimpleRng {
        fn next_u32(&mut self) -> u32 {
            // An example LCG: values chosen arbitrarily.
            self.0 = self.0.wrapping_mul(1664525).wrapping_add(1013904223);
            self.0
        }

        fn next_u64(&mut self) -> u64 {
            // Combine two 32-bit outputs.
            let high = self.next_u32() as u64;
            let low = self.next_u32() as u64;
            (high << 32) | low
        }

        fn fill_bytes(&mut self, dest: &mut [u8]) {
            for chunk in dest.chunks_mut(4) {
                let rand = self.next_u32().to_le_bytes();
                let len = chunk.len();
                chunk.copy_from_slice(&rand[..len]);
            }
        }

    }



/// Background task for running the CYW43 Wi-Fi driver.
/// This must be running at all times for the chip to function.
#[embassy_executor::task]
pub async fn cyw43_task(runner: cyw43::Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>) -> ! {
    info!("Starting runner cyw43");
    runner.run().await
}
/// LED blink task — toggles CYW43 GPIO 0 (onboard LED) on and off with the given delay.
/* 
#[embassy_executor::task]
pub async fn led_blink_task(mut control: cyw43::Control<'static>, delay: Duration) {
    loop {
        info!("led on!");
        control.gpio_set(0, true).await; // Turn LED on
        Timer::after(delay).await;

        info!("led off!");
        control.gpio_set(0, false).await; // Turn LED off
        Timer::after_millis(299_000).await;
    }
}
*/

#[embassy_executor::task]
pub async fn net_task(mut runner: embassy_net::Runner<'static, cyw43::NetDriver<'static>>) -> ! {
    runner.run().await;
}

/* This task is responsible for the TCP client that sends the values of the sensors to a server */
// This is also used as tests to see if i can send the information via TCP to the computer
/*
#[embassy_executor::task]
pub async fn tcp_server_task(
    stack: embassy_net::Stack<'static>,
    mut control: cyw43::Control<'static>,)
{
    
    loop {
        control.gpio_set(0, true).await;
        info!("Starting TCP server task");
        let mut rx_buffer = [0; 4096];
        let mut tx_buffer = [0; 4096];
        //let mut buf = [0; 4096];

        // acquiring the value of the Die Temperature
        let mut rx_luminosity = get_receiver_adc0().unwrap();
        let mut rx_dht_temperature = get_receiver_dht_temperature().unwrap();
        let mut rx_dht_humidity = get_receiver_dht_humidity().unwrap();
        let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
        socket.set_timeout(Some(Duration::from_millis(300_000)));
        // Change this to your PC's IP and port
        let server_addr = embassy_net::IpEndpoint::new(
            embassy_net::IpAddress::v4(192,168,0,201), // <-- your PC IP
            1234, // port
        );

        info!("Connecting to {:?}", server_addr);
        
        if let Err(e) = socket.connect(server_addr).await {
            warn!("connect error: {:?}", e);
            Timer::after(Duration::from_millis(1_000)).await;
            continue;
        }

        info!("Connected, sending message with the values of the sensors");
        let adclum = rx_luminosity.get().await; // Get the value of the sensor
        let dht_temperature = rx_dht_temperature.get().await; // Get the value of dht temperature
        let dht_humidity = rx_dht_humidity.get().await; // Get the value of the dht humidity
        if let Err(e) = socket.write(b"Hello World!\n").await {
            warn!("write error: {:?}", e);
        }
        let mut msg = String::<128>::new();
        let _ = core::write!(&mut msg, "Luminosity: {} DHT Temperature: {} DHT Humidity: {}",
        adclum, 
        dht_temperature, 
        dht_humidity
    );
        if let Err(e) = socket.write(msg.as_bytes()).await {
            warn!("write error: {:?}", e);
        }
        control.gpio_set(0, false).await;
        info!("Closing socket");
        Timer::after(Duration::from_millis(1_000)).await;
    }
    
}
*/     

// --- MQTT Task using actual TcpSocket and updated ClientConfig ---
#[embassy_executor::task]
pub async fn mqtt_server_task(
    stack: embassy_net::Stack<'static>,
    mut control: cyw43::Control<'static>,
) {
    loop {
        control.gpio_set(0, true).await;
        defmt::info!("Starting MQTT server task");

        // Allocate TCP socket buffers.
        let mut rx_buffer = [0u8; 4096];
        let mut tx_buffer = [0u8; 4096];

        // Get sensor interfaces.
        let mut rx_luminosity = crate::modular::adc::get_receiver_adc0().unwrap();
        let mut rx_dht_temperature = crate::modular::get_receiver_dht_temperature().unwrap();
        let mut rx_dht_humidity = crate::modular::get_receiver_dht_humidity().unwrap();

        // Initialize the TCP socket.
        let mut socket =TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
        socket.set_timeout(Some(Duration::from_millis(300_000)));

        // Define the MQTT broker endpoint.
        let server_addr = embassy_net::IpEndpoint::new(
            embassy_net::IpAddress::v4(192, 168, 0, 201), // broker IP
            1883, // MQTT TCP port
        );

        if let Err(e) = socket.connect(server_addr).await {
            defmt::warn!("TCP connect error");
            embassy_time::Timer::after(embassy_time::Duration::from_millis(1_000)).await;
            continue;
        }
        defmt::info!("TCP connection established");

        // MQTT client's internal buffers.
        let mut mqtt_tx_buffer = [0u8; 512];
        let mut mqtt_rx_buffer = [0u8; 512];
        
        // Create a no_std-compatible RNG.
        let rng = SimpleRng::new(0xDEADBEEF);

        // Build the client configuration using the new API:
        // First, create the config using the MQTT version and RNG.
        let mut config = rust_mqtt::client::client_config::ClientConfig::new(
            rust_mqtt::client::client_config::MqttVersion::MQTTv5,
            rng
        );
        // Then assign additional fields.
        config.server = MQTT_BROKER;
        config.port = MQTT_PORT;
        config.client_id = "pico_client";
        config.username = Some(MQTT_USERNAME);
        config.password = Some(MQTT_PASSWORD);
        config.keep_alive = 60;

        // Create the MQTT client.
        let mut mqtt_client =
            MqttClient::<_, 128, _>::new(
                socket,
                &mut mqtt_tx_buffer,
                mqtt_tx_buffer.len(),
                &mut mqtt_rx_buffer,
                mqtt_rx_buffer.len(),
                config,
            );

        // Connect to the MQTT broker.
        if let Err(e) = mqtt_client.connect_to_broker().await {
            defmt::info!("MQTT connect error");
            embassy_time::Timer::after(embassy_time::Duration::from_millis(1_000)).await;
            continue;
        }
        defmt::info!("Connected to MQTT broker");

        // Read sensor values.
        let lum = rx_luminosity.get().await;
        let temp = rx_dht_temperature.get().await;
        let hum = rx_dht_humidity.get().await;

        // Prepare payload.
        let mut payload = heapless::String::<128>::new();
        let _ = core::write!(&mut payload, "Lum: {} Temp: {} Hum: {}", lum, temp, hum);

        // Publish sensor data to MQTT topic.
        if let Err(_e) = mqtt_client
            .send_message(
                MQTT_TOPIC, 
                payload.as_bytes(), 
                rust_mqtt::packet::v5::publish_packet::QualityOfService::QoS1, 
                false
            )
            .await
        {
            defmt::warn!("MQTT send message error");
        } else {
            defmt::info!("Published sensor data via MQTT");
        }

        // Disconnect from the broker.
        if let Err(_e) = mqtt_client.disconnect().await {
            defmt::warn!("MQTT disconnect error");
        }

        control.gpio_set(0, false).await;
        embassy_time::Timer::after(embassy_time::Duration::from_millis(1_000)).await;
    }
}