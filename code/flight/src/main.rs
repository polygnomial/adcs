// ! The starter code slowly blinks the LED, sets up
// ! USB logging, and creates a UART driver using pins
// ! 14 and 15. The UART baud rate is [`UART_BAUD`].
// !
// ! Despite targeting the Teensy 4.0, this starter code
// ! also works on the Teensy 4.1.

#![no_main]
#![no_std]


use teensy4_panic as _;
use teensy4_bsp as bsp;

use bsp::{
    board,
    hal::{timer::Blocking},
};
use embedded_hal::prelude::_embedded_hal_serial_Read;

use bsp::hal::iomuxc;

use tiny_nmea::NMEA;
use heapless::String;

use libgeomag::{GeodeticLocation, ModelExt, WMM, IGRF};

const I2C_PIN_CONFIG: iomuxc::Config = iomuxc::Config::zero()
    .set_open_drain(iomuxc::OpenDrain::Enabled)
    .set_slew_rate(iomuxc::SlewRate::Fast)
    .set_drive_strength(iomuxc::DriveStrength::R0_4)
    .set_speed(iomuxc::Speed::Fast)
    .set_pull_keeper(Some(iomuxc::PullKeeper::Pullup22k));

use teensy4_panic as _;

// use core::fmt::Write as _;

use bsp::hal as hal;
use bsp::ral as ral;
use hal::adc;

// mod Rm3100;
mod constants;
mod rm3100;
mod photodiode;
mod allocator;
mod time;
mod solarsystem_model;
mod linear_algebra;
mod attitude_determination;

use attitude_determination::triad;
use linear_algebra::vec3;
use libm::{sinf, cosf};

/// CHANGE ME to vary the baud rate.
const UART_BAUD: u32 = 9600;
/// Milliseconds to delay before toggling the LED
/// and writing text outputs.
const DELAY_MS: u32 = 50;

#[bsp::rt::entry]
fn main() -> ! {
    // These are peripheral instances. Let the board configure these for us.
    // This function can only be called once!
    let instances = board::instances();

    // This is all the pads on the board
    let pads = unsafe {hal::iomuxc::pads::Pads::new()};

    // Driver resources that are configured by the board. For more information,
    // see the `board` documentation.
    // TODO: refactor into Common and Specifics
    let board::Resources {
        // `pins` has objects that represent the physical pins. The object
        // for pin 13 is `p13`.
        mut pins,
        
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

        // I2C
        lpi2c1,
        ..
    } = board::t41(instances);

    // When this returns, you can use the `log` crate to write text
    // over USB. Use either `screen` (macOS, Linux) or PuTTY (Windows)
    // to visualize the messages from this example.
    bsp::LoggingFrontend::default_log().register_usb(usb);

    // configure pull up resistors on the I2C pins
    iomuxc::configure(&mut pins.p18, I2C_PIN_CONFIG);
    iomuxc::configure(&mut pins.p19, I2C_PIN_CONFIG);

    // configure I2C object
    let lpi2c1: board::Lpi2c1 =
        board::lpi2c(lpi2c1, pins.p19, pins.p18, board::Lpi2cClockSpeed::KHz100);

    let lpuart4: board::Lpuart4 =
        board::lpuart(lpuart4, pins.p8, pins.p7, UART_BAUD);

    // let mut rm3100 = RM3100Driver::new(lpi2c1, 0x20);
    let _status_buffer = [0u8; 1]; // Adjust the size of the buffer as needed

    // This configures the LED as a GPIO output.
    let led = board::led(&mut gpio2, pins.p13);

    // Configures the GPT1 timer to run at GPT1_FREQUENCY. See the
    // constants below for more information.
    gpt1.disable();
    gpt1.set_divider(GPT1_DIVIDER);
    gpt1.set_clock_source(GPT1_CLOCK_SOURCE);

    // Convenience for blocking delays.
    let mut delay = Blocking::<_, GPT1_FREQUENCY>::from_gpt(gpt1);

    // Configure the RM3100 magnetometer
    let mut sensor = rm3100::RM3100Driver::new(lpi2c1, 0x20);

    sensor.begin();
    delay.block_ms(250);
    sensor.set_cycle_counts_xyz_equal(200);
    
    // this is the only way I've been able to configure the cycle counts
    // let cc_arr = [0u8, 0u8, 0u8, 0u8, 0u8, 0u8, 200u8, 0u8, 200u8, 0u8, 200u8];
    // sensor.write(rm3100::Register::RM3100_REG_CCX, &cc_arr );
    delay.block_ms(250);
    let mut cc = rm3100::CycleCounts { x: 0, y: 0, z: 0 };
    sensor.get_cycle_counts(&mut cc);
    delay.block_ms(250);
    let mut status_buffer = [0u8; 0x36]; // Adjust the size of the buffer as needed
    sensor.read(rm3100::Register::RM3100_REG_POLL, &mut status_buffer);
    
    sensor.set_rate(100.0);
    delay.block_ms(250);
    sensor.read(rm3100::Register::RM3100_REG_POLL, &mut status_buffer);
    
    sensor.set_continuous_measurement_mode_enabled(true);
    delay.block_ms(250);
    sensor.read(rm3100::Register::RM3100_REG_POLL, &mut status_buffer);

    // Configure the ADC pins.
    let adc1 = unsafe {ral::adc::ADC1::instance() };
    let mut adc1 = adc::Adc::new(adc1, adc::ClockSelect::ADACK, adc::ClockDivision::Div2);
    let mut a0 = adc::AnalogInput::new(pads.gpio_ad_b1.p02);
    let mut a1 = adc::AnalogInput::new(pads.gpio_ad_b1.p03);
    let mut a2 = adc::AnalogInput::new(pads.gpio_ad_b1.p07);
    let mut a3 = adc::AnalogInput::new(pads.gpio_ad_b1.p06);

    let mut counter: u32 = 0;
    
    // test read magnetometer
    
    // log::info!("RM3100 sample: {}, {}, {}", sample.x, sample.y, sample.z);

    let mut nmea = NMEA::new();
    
    let mut string: String<84> = String::new();

    // Add characters to the string
    string.push_str("$GPGGA,161229.487,3723.2475,N,12158.3416,W,1,07,1.0,9.0,M,,,,0000*18\r\n");
    // string.push_str("$GNGLL,4315.68533,N,07955.20234,W,080023.000,A,A*5D\r\n");
    nmea.update(&string);
    // log::info!("longitude: {}", nmea.longitude.unwrap());
    // log::info!("latitude: {}", nmea.latitude.unwrap());
    // log::info!("altitude: {}", nmea.altitude.unwrap());
    // log::info!("time: {:?}", nmea.utc.unwrap());

    // test converting time to julian date
    let date = time::Date {
        day: 17,
        month: 1,
        year: 2020,
    };
    let time = time::Time {
        hour: 10,
        minute: 45,
        second: 0,
        millisecond: 0,
    };
    let jd = time::utc_to_jd(date, time).unwrap();
    // log::info!("julian date: {}", jd);
    // log::info!("sun position: {}, {}, {}", sun_pos_model.x, sun_pos_model.y, sun_pos_model.z);

    // Stanford
    let l = GeodeticLocation::new(37.4277, -122.1701, 0.028956);
    let days_decimal = 2020.0;

    // let wmm = WMM::new(days_decimal).unwrap();
    let igrf = IGRF::new(days_decimal).unwrap();

    // log::info!("created models");
    // let m = wmm.single(l);
    // log::info!("{:?}", m);

    loop {
        led.toggle();
        delay.block_ms(DELAY_MS);
        counter = counter.wrapping_add(1);
        
        // read photodiodes
        let reading0: u16 = adc1.read_blocking(&mut a0);
        let reading1: u16 = adc1.read_blocking(&mut a1);
        let reading2: u16 = adc1.read_blocking(&mut a2);
        let reading3: u16 = adc1.read_blocking(&mut a3);

        // read magnetometer
        let sample: rm3100::Sample = sensor.get_sample().unwrap();
        let mag_vec_measurement = vec3::new(sample.x as f32, sample.y as f32, sample.z as f32) * 1000.0; // to nT

        let θ: f32 = photodiode::photodiode_pair(reading2, reading0).unwrap() * photodiode::DEGREES_PER_RADIAN;
        let ψ: f32 = photodiode::photodiode_pair(reading3, reading1).unwrap() * photodiode::DEGREES_PER_RADIAN;
        let sun_vec_measurement = vec3::new(
            sinf(ψ)*sinf(θ), 
            cosf(ψ), 
            sinf(ψ)*cosf(θ)
        );

        // // read magnetometer
        let sample: rm3100::Sample = sensor.get_sample().unwrap();
        
        let m = igrf.single(l);
        let mag_vec_model = vec3::new(m.x as f32, m.y as f32, m.z as f32);

        let sun_pos_model: vec3 = solarsystem_model::sun_position(jd).unwrap();

        // test triad
        let res = triad(sun_pos_model, mag_vec_model, sun_vec_measurement, mag_vec_measurement);
        log::info!("Triad test {:?}", res);
        // log::info!("RM3100 sample: {}, {}, {}", sample.x, sample.y, sample.z);
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
