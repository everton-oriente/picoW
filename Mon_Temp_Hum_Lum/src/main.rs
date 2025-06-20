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
#![no_std]  // (needed for embedded targets)
#![no_main] // Disable normal main entry point; we use `#[embassy_executor::main]` instead

// Import required crates and modules
use cyw43_pio::{PioSpi, DEFAULT_CLOCK_DIVIDER};
use defmt::*; // For logging via RTT
use embassy_executor::Spawner;
use embassy_rp::adc::{Adc, Async, Channel, Config as AdcConfig, InterruptHandler as AdcIrq};
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Level, Output, Pull};
use embassy_rp::peripherals::{DMA_CH0, PIO0};
use embassy_rp::pio::{InterruptHandler as PioIrq, Pio};
use embassy_time::{Duration, Timer};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _}; // RTT logging and panic handler

// Bind the interrupts handlers ,if it is needed more IRQ we just need to declare here on this struct like I2C
bind_interrupts!(struct Irqs {
    // Bind the interrupt handler to  PIO IRQ
    PIO0_IRQ_0 => PioIrq<PIO0>;

    // Bind the interrupt handler to  ADC IRQ
    ADC_IRQ_FIFO => AdcIrq;

});

// Bind the interrupt handler to  ADC IRQ
//bind_interrupts!(struct Irqs {
    
//});

/// Background task for running the CYW43 Wi-Fi driver.
/// This must be running at all times for the chip to function.
#[embassy_executor::task]
async fn cyw43_task(
    runner: cyw43::Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>
) -> ! {
    runner.run().await
}

/// LED blink task — toggles CYW43 GPIO 0 (onboard LED) on and off with the given delay.
#[embassy_executor::task]
async fn led_blink_task(mut control: cyw43::Control<'static>, delay: Duration) {
    loop {
        info!("led on!");
        control.gpio_set(0, true).await; // Turn LED on
        Timer::after(delay).await;

        info!("led off!");
        control.gpio_set(0, false).await; // Turn LED off
        Timer::after(delay).await;
    }
}

// LED blink task - to the gpio in the board 
#[embassy_executor::task]
async fn toogle_led(mut led: Output<'static>){
    loop{
        info!("LED ON");
        led.set_high();
        Timer::after_millis(2500).await;

        info!("LED OFF");
        led.set_low();
        Timer::after_millis(2500).await;
    }
}

#[embassy_executor::task]
async fn read_luminosity(mut adc_lum: Adc<'static, Async>, mut chan: Channel<'static>){// Select the ADC channel for luminosity, mode of the ADC and which channel to use (ADC0 to ADC3)
    loop{
        info!("Reading luminosity");
        match adc_lum.read(&mut chan).await{
            Ok(value) => {
                info!("Luminosity: {}", value);
            }
            Err(e) => {
                error!("Error reading luminosity: {}", e);
            }
        }
        Timer::after_millis(1000).await;
    }
}
/*
#[embassy_executor::task]
async fn read_temperature_die(mut adc_temp: Adc<'static, Async>, mut chan: Channel<'static>){// Select the ADC channel for temperature sensor inside die, mode of the ADC and which channel to use (ADC4)      
    loop{
        loop {
            // Read raw ADC value from internal temperature sensor (channel 4)
            match adc.read_internal_temp_sensor().await {
                Ok(raw_value) => {
                    // Convert ADC raw value (12-bit) to voltage
                    let v_ref = 3.3; // typical reference voltage for RP2040 ADC
                    let voltage = raw_value as f32 * v_ref / 4096.0;

                    // Convert voltage to temperature using RP2040 formula from datasheet
                    let temperature = 27.0 - (voltage - 0.706) / 0.001721;

                    info!("Raw: {}, Voltage: {} V, Temp: {} °C", raw_value, voltage, temperature);
                    }
                Err(e) => {
                    warn!("Failed to read internal temp sensor: {:?}", e);
                    }
            }

            Timer::after_millis(1000).await;
        }   
        
    }
}

*/
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
    // Create the ADC thats read the internal temperature of the DIE, like usage in the watchdog feed, if the temperature goes high we turn off the system
    //let temp_adc = Channel::new_temp_sensor(p.ADC_TEMP_SENSOR);
    // Create the ADC 0 to read the luminosity of the system
    let lum_adc_0 = Channel::new_pin(p.PIN_26, Pull::Down);

    // Load firmware for CYW43 Wi-Fi chip
    let fw = include_bytes!("../cyw43-firmware/43439A0.bin");
    let clm = include_bytes!("../cyw43-firmware/43439A0_clm.bin");

    // Optional: For faster flashing during development, use fixed memory regions instead
    // let fw = unsafe { core::slice::from_raw_parts(0x10100000 as *const u8, 230321) };
    // let clm = unsafe { core::slice::from_raw_parts(0x10140000 as *const u8, 4752) };

    // Configure power and chip-select GPIOs for the CYW43 chip
    let pwr = Output::new(p.PIN_23, Level::Low);    // Power control
    let cs  = Output::new(p.PIN_25, Level::High);   // SPI chip select (active low)

    // Initialize PIO peripheral with interrupts
    let mut pio = Pio::new(p.PIO0, Irqs);

    // Create a PIO-based SPI instance to talk to the CYW43 chip
    let spi = PioSpi::new(
        &mut pio.common,
        pio.sm0,
        DEFAULT_CLOCK_DIVIDER,
        pio.irq0,
        cs,
        p.PIN_24,    // SPI SCK
        p.PIN_29,    // SPI MOSI
        p.DMA_CH0,   // DMA channel
    );

    // Initialize shared state for the CYW43 driver (required for internal coordination)
    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());

    // Create the CYW43 driver instance
    let (_net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw).await;

    // Spawn the background task to keep the CYW43 chip running
    unwrap!(spawner.spawn(cyw43_task(runner)));

    // Initialize the CYW43 chip with the CLM (regulatory) data
    control.init(clm).await;

    // Enable power-saving mode (optional)
    control.set_power_management(cyw43::PowerManagementMode::PowerSave).await;

    // Configure delay for blinking LED
    let delay = Duration::from_millis(5000);

    // Spawn the LED blinking task with desired delay
    unwrap!(spawner.spawn(led_blink_task(control, delay)));

    // Spawn the LED task
    spawner.spawn(toogle_led(blinky_led)).unwrap();

    // Spawn the luminosity task to read the luminosity
    unwrap!(spawner.spawn(read_luminosity(adc, lum_adc_0)));

    // Spawn the temperature task to read the temperature
    //unwrap!(spawner.spawn(read_temperature_die(adc, temp_adc)));
}
