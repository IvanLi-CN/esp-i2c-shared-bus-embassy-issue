#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embassy_time::{Duration, Timer};
use embedded_hal_async::i2c::I2c;
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    gpio::Io,
    i2c::I2C,
    peripherals::{Peripherals, I2C0},
    prelude::*,
    system::SystemControl,
    timer::{timg::TimerGroup, OneShotTimer},
    Async,
};
use static_cell::make_static;

#[main]
async fn main(spawner: Spawner) {
    esp_println::logger::init_logger_from_env();

    log::info!("starting");

    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::max(system.clock_control).freeze();

    let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks, None);
    let timer0 = OneShotTimer::new(timg0.timer0.into());
    let timers = [timer0];
    let timers = make_static!(timers);
    esp_hal_embassy::init(&clocks, timers);

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    // Init I2C driver
    let i2c = I2C::new_async(
        peripherals.I2C0,
        io.pins.gpio4,
        io.pins.gpio5,
        100u32.kHz(),
        &clocks,
    );

    let i2c_mutex = make_static!(Mutex::<CriticalSectionRawMutex, _>::new(i2c));

    let dev_a = I2cDevice::new(i2c_mutex);
    let dev_b = I2cDevice::new(i2c_mutex);

    let dev_a = make_static!(dev_a);
    let dev_b = make_static!(dev_b);

    spawner.spawn(run_i2c_a(dev_a)).ok();

    spawner.spawn(run_i2c_b(dev_b)).ok();

    loop {
        // log::info!("Hello world!");
        Timer::after(Duration::from_millis(5_000)).await;
    }
}

#[embassy_executor::task]
async fn run_i2c_a(
    dev: &'static mut I2cDevice<'static, CriticalSectionRawMutex, I2C<'static, I2C0, Async>>,
) {
    let addr = 0x4a;

    log::info!("run_i2c addr: 0x{:02x}", addr);
    loop {
        Timer::after(Duration::from_millis(1000)).await;

        match dev.write(addr, &[0x11]).await {
            Ok(_) => {
                log::info!("set addr: 0x{:02x}", addr);
            }
            Err(err) => {
                log::error!("failed to set addr: 0x{:02x}. {:?}", addr, err);
            }
        }
    }
}

#[embassy_executor::task]
async fn run_i2c_b(
    dev: &'static mut I2cDevice<'static, CriticalSectionRawMutex, I2C<'static, I2C0, Async>>,
) {
    let addr = 0x4b;

    log::info!("run_i2c addr: 0x{:02x}", addr);
    loop {
        Timer::after(Duration::from_millis(1000)).await;

        match dev.write(addr, &[0x11]).await {
            Ok(_) => {
                log::info!("set addr: 0x{:02x}", addr);
            }
            Err(err) => {
                log::error!("failed to set addr: 0x{:02x}. {:?}", addr, err);
            }
        }
    }
}
