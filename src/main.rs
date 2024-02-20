#![no_std]
#![no_main]

use bsp::{entry, hal::gpio::PullUp};
use core::fmt::Write;
use core::panic::PanicInfo;
use defmt::info;
use defmt_rtt as _;

use mpu6050::*;
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    gpio::{FunctionI2C, Pin},
    i2c::I2C,
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

// Some traits we need
use bsp::hal::fugit::RateExtU32;

// Panic handler
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mpu_scl_pin: Pin<_, FunctionI2C, PullUp> = pins.gpio27.reconfigure();
    let mpu_sda_pin: Pin<_, FunctionI2C, PullUp> = pins.gpio26.reconfigure();

    let mpu_i2c = I2C::new_controller(
        pac.I2C1,
        mpu_sda_pin,
        mpu_scl_pin,
        400.kHz(),
        &mut pac.RESETS,
        clocks.system_clock.freq(),
    );

    let ssd1306_scl_pin: Pin<_, FunctionI2C, PullUp> = pins.gpio21.reconfigure();
    let ssd1306_sda_pin: Pin<_, FunctionI2C, PullUp> = pins.gpio20.reconfigure();

    let display_i2c = I2C::new_controller(
        pac.I2C0,
        ssd1306_sda_pin,
        ssd1306_scl_pin,
        400.kHz(),
        &mut pac.RESETS,
        clocks.system_clock.freq(),
    );

    let interface = I2CDisplayInterface::new(display_i2c);

    let mut mpu = Mpu6050::new(mpu_i2c);
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    mpu.init(&mut delay).unwrap();

    let mut display =
        Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0).into_terminal_mode();

    display.init().unwrap();

    loop {
        let _ = display.clear();

        let temp = mpu.get_temp().unwrap();
        defmt::info!("temp: {}°C", defmt::Debug2Format(&temp));
        let _ = display.write_str("temp:");
        let _ = writeln!(display, "{:.1}°C", temp);

        // get gyro data, scaled with sensitivity
        let gyro = mpu.get_gyro().unwrap();
        defmt::info!("gyro: {}", defmt::Debug2Format(&gyro));
        let _ = display.write_str("gyro:\n");
        let _ = writeln!(display, "{:.1}, {:.1}, {:.1}", gyro.x, gyro.y, gyro.z);

        // get accelerometer data, scaled with sensitivity
        let acc = mpu.get_acc_angles().unwrap();
        defmt::info!("r/p: {}", defmt::Debug2Format(&acc));
        let _ = display.write_str("acc:\n");
        let _ = writeln!(display, "{:.1}, {:.1}", acc.x, acc.y);

        delay.delay_ms(250_u32);
    }
}

// End of file
