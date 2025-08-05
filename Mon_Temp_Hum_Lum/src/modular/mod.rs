// Master file for the modular project.
/*!
 * -----------------------------------------------------------------------------
 *  Project     : Master file for the modular project.
 *  File        : mod.rs
 *  Created by  : Everton Oriente
 *  Date        : 2025-07-22
 *  * -----------------------------------------------------------------------------
 *  Description :
 *      The master module is responsible about to add or remove modules from the project.
 *
 *  Target MCU  : Raspberry Pi Pico W (RP2040 and CYW43)
 *  Framework   : Embassy, no_std
 *
 */

// Import required crates and modules

mod adc;
mod channel_adc_0;
mod channel_temp;
mod cyw43;
mod dht;
mod led;
mod oled;

pub(crate) use adc::*;
pub(crate) use channel_adc_0::*;
pub(crate) use channel_temp::*;
pub(crate) use cyw43::*;
pub(crate) use dht::*;
pub(crate) use led::*;
pub(crate) use oled::*;
