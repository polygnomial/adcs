//! The starter code slowly blinks the LED, sets up
//! USB logging, and creates a UART driver using pins
//! 14 and 15. The UART baud rate is [`UART_BAUD`].
//!
//! Despite targeting the Teensy 4.0, this starter code
//! also works on the Teensy 4.1.

#![no_std]
#![no_main]

use bsp::board;
use teensy4_bsp as bsp;
use teensy4_panic as _;
use nb;
use embedded_hal::serial::{Read};
use bsp::hal::timer::Blocking;

// use core::fmt::Write as _;

use bsp::hal as hal;
use bsp::ral as ral;
use hal::adc;

mod Rm3100;

#[global_allocator]
static ALLOCATOR: emballoc::Allocator<4096> = emballoc::Allocator::new();
extern crate alloc;

// use nmea0183::{Parser, ParseResult};

// let nmea = b"$GPGGA,145659.00,5956.695396,N,03022.454999,E,2,07,0.6,9.0,M,18.0,M,,*62\r\n$GPGGA,,,,,,,,,,,,,,*00\r\n";
// let mut parser = Parser::new();
// for b in &nmea[..] {
//     if let Some(result) = parser.parse_from_byte(*b) {
//         match result {
//             Ok(ParseResult::GGA(Some(gga))) => { }, // Got GGA sentence
//             Ok(ParseResult::GGA(None)) => { }, // Got GGA sentence without valid data, receiver ok but has no solution
//             Ok(_) => {}, // Some other sentences..
//             Err(e) => { } // Got parse error
//         }
//     }
// }

/// CHANGE ME to vary the baud rate.
const UART_BAUD: u32 = 9600;
/// Milliseconds to delay before toggling the LED
/// and writing text outputs.
const DELAY_MS: u32 = 500;

#[bsp::rt::entry]
fn main() -> ! {
    // These are peripheral instances. Let the board configure these for us.
    // This function can only be called once!
    let instances = board::instances();

    // This is all the pads on the board
    let pads = unsafe {hal::iomuxc::pads::Pads::new()};

    // Driver resources that are configured by the board. For more information,
    // see the `board` documentation.
    let board::Resources {
        // `pins` has objects that represent the physical pins. The object
        // for pin 13 is `p13`.
        pins,
        // This is a hardware timer. We'll use it for blocking delays.
        mut gpt1,
        // These are low-level USB resources. We'll pass these to a function
        // that sets up USB logging.
        usb,
        // This is the GPIO2 port. We need this to configure the LED as a
        // GPIO output.
        mut gpio2,
        // This resource is for the UART we're creating.
        lpuart4,
        ..
    } = board::t40(instances);

    // When this returns, you can use the `log` crate to write text
    // over USB. Use either `screen` (macOS, Linux) or PuTTY (Windows)
    // to visualize the messages from this example.
    bsp::LoggingFrontend::default_log().register_usb(usb);

    // This configures the LED as a GPIO output.
    let led = board::led(&mut gpio2, pins.p13);

    // Configures the GPT1 timer to run at GPT1_FREQUENCY. See the
    // constants below for more information.
    gpt1.disable();
    gpt1.set_divider(GPT1_DIVIDER);
    gpt1.set_clock_source(GPT1_CLOCK_SOURCE);

    // Convenience for blocking delays.
    let mut delay = Blocking::<_, GPT1_FREQUENCY>::from_gpt(gpt1);

    // Create the UART driver using pins 14 and 15.
    // Cast it to a embedded_hal trait object so we can
    // use it with the write! macro.
    let mut lpuart4: board::Lpuart4 = board::lpuart(lpuart4, pins.p8, pins.p7, UART_BAUD);
    // let lpuart4: &mut dyn embedded_hal::serial::Write<u8, Error = _> = &mut lpuart4;

    // Configure the ADC pins.

    let adc1 = unsafe {ral::adc::ADC1::instance() };
    let mut adc1 = adc::Adc::new(adc1, adc::ClockSelect::ADACK, adc::ClockDivision::Div2);
    let mut a0 = adc::AnalogInput::new(pads.gpio_ad_b1.p02);
    let mut a1 = adc::AnalogInput::new(pads.gpio_ad_b1.p03);
    let mut a2 = adc::AnalogInput::new(pads.gpio_ad_b1.p07);
    let mut a3 = adc::AnalogInput::new(pads.gpio_ad_b1.p06);

    let mut counter: u32 = 0;
    loop {
        led.toggle();
        log::info!("Hello from the USB logger! The count is {counter}");
        // write!(
        //     lpuart4,
        //     "Hello from the UART driver! The count is {counter}\r\n"
        // )
        // .ok();
        
        // let mut buffer = [0u8; 64];
        // lpuart4.read(&mut dma_channel, &mut buffer);
        loop {
            log::info!("looking for bytes");
            let byte = nb::block!(lpuart4.read()).unwrap();
            // if byte == b'\n' {
            //     break;
            // }
            log::info!("byte: {byte}");
        }
        // let byte = nb::block!(lpuart4.read()).unwrap();
        // nb::block!(lpuart4.write_byte(byte)).unwrap();

        delay.block_ms(DELAY_MS);
        counter = counter.wrapping_add(1);
        
        // read photodiodes
        let reading0: u16 = adc1.read_blocking(&mut a0);
        let reading1: u16 = adc1.read_blocking(&mut a1);
        let reading2: u16 = adc1.read_blocking(&mut a2);
        let reading3: u16 = adc1.read_blocking(&mut a3);

        log::info!("ADC readings: {reading0}, {reading1}, {reading2}, {reading3}");
    }
}

// We're responsible for configuring our timers.
// This example uses PERCLK_CLK as the GPT1 clock source,
// and it configures a 1 KHz GPT1 frequency by computing a
// GPT1 divider.
use bsp::hal::gpt::ClockSource;

/// The intended GPT1 frequency (Hz).
const GPT1_FREQUENCY: u32 = 1_000;
/// Given this clock source...
const GPT1_CLOCK_SOURCE: ClockSource = ClockSource::HighFrequencyReferenceClock;
/// ... the root clock is PERCLK_CLK. To configure a GPT1 frequency,
/// we need a divider of...
const GPT1_DIVIDER: u32 = board::PERCLK_FREQUENCY / GPT1_FREQUENCY;
