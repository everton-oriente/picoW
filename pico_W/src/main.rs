//! This example tests the RP Pico W on-board LED (connected to CYW43 wireless chip).
//! Note: Only works on Pico W, not base Pico (see blinky.rs for base board).

#![no_std]       // Disable Rust standard library (bare-metal environment)
#![no_main]      // Disable standard main interface

// Import necessary libraries and modules
use cyw43_pio::{PioSpi, DEFAULT_CLOCK_DIVIDER};
use defmt::*;    // Formatted logging
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{DMA_CH0, PIO0};
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_time::{Duration, Timer};
use static_cell::StaticCell;  // For static memory allocation
use {defmt_rtt as _, panic_probe as _};  // Logging and panic handlers

// Bind processor interrupts to our handler
bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

// Task definition for handling CYW43 wireless chip operations
#[embassy_executor::task]
async fn cyw43_task(
    runner: cyw43::Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>
) -> ! {
    runner.run().await  // Start processing CYW43 events (runs forever)
}

// New separate task for LED blinking
#[embassy_executor::task]
async fn blink_task(
    mut control: cyw43::Control<'static>,  // Exclusive access to LED control
    interval: Duration                     // Blink interval
) -> ! {
    info!("Blink task started");
    
    // Main blink loop - runs forever
    loop {
        info!("LED on!");
        control.gpio_set(0, true).await;   // Set CYW43 GPIO0 high (LED on)
        Timer::after(interval).await;      // Wait
        
        info!("LED off!");
        control.gpio_set(0, false).await;  // Set CYW43 GPIO0 low (LED off)
        Timer::after(interval).await;      // Wait
    }
}

// Main entry point - uses Embassy's async executor
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Initialize RP2040 peripherals
    let p = embassy_rp::init(Default::default());
    
    // Load CYW43 firmware and regulatory data (CLM) from files
    let fw = include_bytes!("../cyw43-firmware/43439A0.bin");
    let clm = include_bytes!("../cyw43-firmware/43439A0_clm.bin");

    /* Alternative development flashing method:
    let fw = unsafe { core::slice::from_raw_parts(0x10100000 as *const u8, 230321) };
    let clm = unsafe { core::slice::from_raw_parts(0x10140000 as *const u8, 4752) };
    */

    // Configure power control pin (active low)
    let pwr = Output::new(p.PIN_23, Level::Low);
    // Configure SPI chip select pin
    let cs = Output::new(p.PIN_25, Level::High);
    
    // Initialize PIO (Programmable I/O) block for SPI
    let mut pio = Pio::new(p.PIO0, Irqs);
    // Create PIO-based SPI interface for CYW43
    let spi = PioSpi::new(
        &mut pio.common,      // PIO shared state
        pio.sm0,              // Use state machine 0
        DEFAULT_CLOCK_DIVIDER,// Default SPI clock speed
        pio.irq0,             // Interrupt handler
        cs,                   // Chip select pin
        p.PIN_24,             // MOSI pin
        p.PIN_29,             // MISO pin
        p.DMA_CH0,            // DMA channel for transfers
    );

    // Allocate static memory for CYW43 driver state
    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());
    
    // Initialize CYW43 driver and split into components
    let (_net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw).await;
    
    // Spawn the CYW43 handler task on executor
    unwrap!(spawner.spawn(cyw43_task(runner)));

    // Initialize CYW43 firmware with regulatory data
    control.init(clm).await;
    
    // Configure power saving mode
    control
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;

    // Create blink interval (500ms)
    let blink_interval = Duration::from_millis(1000);
    
    // Spawn the blink task with exclusive access to control
    info!("Spawning blink task");
    unwrap!(spawner.spawn(blink_task(control, blink_interval)));
    
    // Main task can now sleep forever or handle other operations
    info!("Main task going to sleep");
    loop {
        Timer::after(Duration::from_secs(3600)).await; // Sleep for 1 hour intervals
    }
}