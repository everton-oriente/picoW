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
use embassy_time::{Duration, Timer};
use core::fmt::Write;
use heapless::String;
use {defmt_rtt as _, panic_probe as _}; // RTT logging and panic handler


use crate::modular::adc::{get_receiver_adc0};
use crate::modular::{get_receiver_dht_humidity, get_receiver_dht_temperature};

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
     
