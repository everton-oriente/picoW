// OLED file for the modular project.
/*!
 * -----------------------------------------------------------------------------
 *  Project     : OLED file for the modular project.
 *  File        : oled.rs
 *  Created by  : Everton Oriente
 *  Date        : 2025-07-22
 *  * -----------------------------------------------------------------------------
 *  Description :
 *      The module is responsible about to acquire and send the information to the display OLED,
 *      regarding the values about the temperature, humidity and luminosity.
 *
 *  Target MCU  : Raspberry Pi Pico W (RP2040 and CYW43)
 *  Framework   : Embassy, no_std
 *
 */

// Crate regarding I2C Oled Display
use core::fmt::Write;

use embassy_rp::i2c::{Async, I2c};
// Crate regarding I2C Oled Display
use embassy_rp::peripherals::I2C0;
use embassy_time::Timer;
use embedded_graphics::geometry::Point;
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::mono_font::ascii::FONT_6X10;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Circle, PrimitiveStyle};
use embedded_graphics::text::Text;
use heapless::String;
use micromath::F32Ext;
use ssd1306::prelude::*;
use ssd1306::{I2CDisplayInterface, Ssd1306};

use crate::modular::adc::{get_receiver_adc0, get_receiver_adctemp};

#[embassy_executor::task]
pub async fn oled_task(i2c: I2c<'static, I2C0, Async>) {
    let interface = I2CDisplayInterface::new(i2c);
    let mut display =
        Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0).into_buffered_graphics_mode();

    if let Err(_) = display.init() {
        defmt::error!("Display init failed");
        core::panic!("Display init failed");
    }

    let header_style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);
    let temp_style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);

    // Header text
    let header_text = "Smart Vivarium";
    let header_x = (128 - header_text.len() as i32 * 6) / 2;
    let header_y = 12;

    // acquiring the value of the Die Temperature
    let mut rx_temp = get_receiver_adctemp().unwrap();
    let mut rx_luminosity = get_receiver_adc0().unwrap();

    loop {
        let mut buffer_temp: String<32> = String::new(); // Create a buffer to store the text

        let adctemp = rx_temp.get().await; // Get the value of the sensor
        let adclum = rx_luminosity.get().await; // Get the value of the sensor

        let voltage = adctemp as f32 * 3.3 / 4096.0;
        let temp = 27.0 - (voltage - 0.706) / 0.001721;
        let temp_trunk = (temp * 100.0).trunc() / 100.0;
        core::write!(buffer_temp, "Temp: {}  C", temp_trunk).unwrap();

        let buffer_temp_x = (128 - buffer_temp.len() as i32 * 6) / 2; // Calculate the x-coordinate for the text
        let buffer_temp_y = 32; // Set the y-coordinate for the text

        let mut buffer_lum: String<32> = String::new(); // Create a buffer to store the text

        //let lumens = adclum as f32;  // Convert the value of ADC into Lux
        //let lumens_trunk = (lumens * 100.0).trunc() / 100.0;

        core::write!(buffer_lum, "Luminosity: {} lux", adclum).unwrap();

        let buffer_lum_x = (128 - buffer_lum.len() as i32 * 6) / 2; // Calculate the x-coordinate for the text
        let buffer_lum_y = 44;

        // Clear the display
        if let Err(_) = display.clear(BinaryColor::Off) {
            defmt::error!("Clear failed");
        }
        // Display the text in the OLED Display
        // Display first line
        Text::new(header_text, Point::new(header_x, header_y), header_style)
            .draw(&mut display)
            .unwrap();
        // Display second line
        Text::new(&buffer_temp, Point::new(buffer_temp_x, buffer_temp_y), temp_style)
            .draw(&mut display)
            .unwrap();
        // Display third line
        Text::new(&buffer_lum, Point::new(buffer_lum_x, buffer_lum_y), temp_style)
            .draw(&mut display)
            .unwrap();

        // Draw the temperature indicator

        let x_circle = buffer_temp_x + (buffer_temp.len() as i32 * 6) - 12;
        let y_circle = buffer_temp_y - 6;
        let degree_pos = Point::new(x_circle, y_circle); // adjust these values as needed
        Circle::new(degree_pos, 4) // a small filled circle
            .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
            .draw(&mut display)
            .unwrap();

        if let Err(_) = display.flush() {
            defmt::error!("Flush failed");
        }

        Timer::after_secs(3).await; // Update the display every 3 seconds
    }
}
