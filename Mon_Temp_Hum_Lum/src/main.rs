/*!
 * -----------------------------------------------------------------------------
 *  Project     : Monitor of Temperature, Humidity and Luminosity
 *  File        : main.rs
 *  Created by  : Everton Oriente
 *  Date        : 2025-06-18
 * -----------------------------------------------------------------------------
 *  Description :
 *      The monitor should acquire information regarding temperature, humidity and luminosity,
 *      and send this information to the SCADA system to hold and store the information regarding the sensors.
 *
 *  Target MCU  : Raspberry Pi Pico W (RP2040 and CYW43)
 *  Framework   : Embassy, no_std
 *
 */
#![no_std] // (needed for embedded targets)
#![no_main] // Disable normal main entry point; we use `#[embassy_executor::main]` instead

mod modular;

// Import required crates and modules
use core::cell::RefCell;
use cyw43_pio::{DEFAULT_CLOCK_DIVIDER, PioSpi};
use defmt::*; // For logging via RTT
use embassy_executor::Spawner;
use embassy_rp::adc::{Adc, Async, Channel, Config as AdcConfig, InterruptHandler as AdcIrq};
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Level, Output, Pull};
use embassy_rp::i2c::{Config as I2c_config, I2c, InterruptHandler};
use embassy_rp::peripherals::{I2C0, PIO0};
use embassy_rp::pio::{InterruptHandler as PioIrq, Pio};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::Duration;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _}; // RTT logging and panic handler


// Bind the interrupts handlers ,if it is needed more IRQ we just need to declare here on this struct like I2C
bind_interrupts!(struct Irqs {
    // Bind the interrupt handler to  PIO IRQ
    PIO0_IRQ_0 => PioIrq<PIO0>;

    // Bind the interrupt handler to  ADC IRQ
    ADC_IRQ_FIFO => AdcIrq;

    // Bind the interrupt handler to  I2C IRQ
    I2C0_IRQ => InterruptHandler<I2C0>;

});

/// Main async entry point
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Information thats the MCU has started the boot process
    info!("Initializing the system");
    // Initialize Embassy peripherals and clocks
    let p = embassy_rp::init(Default::default());
    // The peripherals has initialized
    info!("Peripherals has initialized with sucess");
    // Create an Output to the LED
    let blinky_led = Output::new(p.PIN_16, Level::Low); // Led external

    // Create an ADC peripheral
    let adc = Adc::new(p.ADC, Irqs, AdcConfig::default());
    // Create a mutex to protect the ADC
    static ADC: StaticCell<Mutex<ThreadModeRawMutex, RefCell<Adc<'static, Async>>>> = StaticCell::new();
    // Initialize the ADC
    let adc_mutex = ADC.init(Mutex::new(RefCell::new(adc)));
    // Create the ADC thats read the internal temperature of the DIE, like usage in the watchdog feed, if the temperature goes high we turn off the system
    let temp_adc = Channel::new_temp_sensor(p.ADC_TEMP_SENSOR);
    // Create the ADC 0 to read the luminosity of the system
    let lum_adc_0 = Channel::new_pin(p.PIN_26, Pull::Down);
    // Create the ADC 1 to read the ADC 1
    //let adc_1 = Channel::new_pin(p.PIN_27, Pull::Down);
    // Create the ADC 2 to read the ADC 2
    //let adc_2 = Channel::new_pin(p.PIN_28, Pull::Down);

    // Configure I2C
    let sda = p.PIN_20;
    let scl = p.PIN_21;

    let mut i2c_config = I2c_config::default();
    i2c_config.frequency = 100_000;
    let i2c = I2c::new_async(p.I2C0, scl, sda, Irqs, i2c_config);

    // Load firmware for CYW43 Wi-Fi chip
    let fw = include_bytes!("../cyw43-firmware/43439A0.bin");
    let clm = include_bytes!("../cyw43-firmware/43439A0_clm.bin");

    // Optional: For faster flashing during development, use fixed memory regions instead
    // let fw = unsafe { core::slice::from_raw_parts(0x10100000 as *const u8, 230321) };
    // let clm = unsafe { core::slice::from_raw_parts(0x10140000 as *const u8, 4752) };

    // Configure power and chip-select GPIOs for the CYW43 chip
    let pwr = Output::new(p.PIN_23, Level::Low); // Power control
    let cs = Output::new(p.PIN_25, Level::High); // SPI chip select (active low)

    // Initialize PIO peripheral with interrupts
    let mut pio = Pio::new(p.PIO0, Irqs);

    // Create a PIO-based SPI instance to talk to the CYW43 chip
    let spi = PioSpi::new(
        &mut pio.common,
        pio.sm0,
        DEFAULT_CLOCK_DIVIDER,
        pio.irq0,
        cs,
        p.PIN_24,  // SPI SCK
        p.PIN_29,  // SPI MOSI
        p.DMA_CH0, // DMA channel
    );

    // Initialize shared state for the CYW43 driver (required for internal coordination)
    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());

    // Create the CYW43 driver instance
    let (_net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw).await;

    // Spawn the background task to keep the CYW43 chip running
    unwrap!(spawner.spawn(modular::cyw43_task(runner)));

    // Initialize the CYW43 chip with the CLM (regulatory) data
    control.init(clm).await;

    // Enable power-saving mode (optional)
    control
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;

    // Configure delay for blinking LED
    let delay = Duration::from_millis(5000);

    // Spawn the LED blinking task with desired delay
    unwrap!(spawner.spawn(modular::led_blink_task(control, delay)));

    // Spawn the LED task
    spawner.spawn(modular::toogle_led(blinky_led)).unwrap();

    // Spawn the luminosity task to read the luminosity
    unwrap!(spawner.spawn(modular::read_adc_channels(adc_mutex, lum_adc_0, temp_adc))); // Here you should add in compliance how many adc are going to use

    // Spawn the process_adc_channel_0 task
    unwrap!(spawner.spawn(modular::process_adc_channel_0()));

    // Spawn the process_adc_channel_temp task
    unwrap!(spawner.spawn(modular::process_adc_channel_temp()));

    // Spawn the I2C Display task
    unwrap!(spawner.spawn(modular::oled_task(i2c)));
}
