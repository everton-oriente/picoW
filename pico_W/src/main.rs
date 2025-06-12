//! This example tests the RP Pico W onboard LED via the CYW43 Wi-Fi chip.
//!
//! It does not work with the original RP Pico board (non-W version). See `blinky.rs` for that.

#![no_std] // Don't link the standard library (needed for embedded targets)
#![no_main] // Disable normal main entry point; we use `#[embassy_executor::main]` instead

// Import required crates and modules
use cyw43_pio::{PioSpi, DEFAULT_CLOCK_DIVIDER};
use defmt::*; // For logging via RTT
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{DMA_CH0, PIO0};
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_time::{Duration, Timer};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _}; // RTT logging and panic handler

// Bind the interrupt handler to PIO0 IRQ
bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

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

/// Main async entry point
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Initialize Embassy peripherals and clocks
    let p = embassy_rp::init(Default::default());

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
    let delay = Duration::from_millis(500);

    // Spawn the LED blinking task with desired delay
    unwrap!(spawner.spawn(led_blink_task(control, delay)));
}
