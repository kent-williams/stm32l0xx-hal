#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

extern crate panic_semihosting;

use cortex_m_rt::entry;
use stm32l0xx_hal::{pac, prelude::*, rcc::Config};

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // Configure the clock.
    let mut rcc = dp.RCC.freeze(Config::hsi16());

    // Acquire the GPIOA peripheral. This also enables the clock for GPIOA in
    // the RCC register.
    let gpiob = dp.GPIOB.split(&mut rcc);

    let mut lp5562_en = gpiob.pb14.into_push_pull_output();

    // Get the delay provider.
    let mut delay = cp.SYST.delay(rcc.clocks);

    let scl = gpiob.pb8.into_open_drain_output();
    let sda = gpiob.pb9.into_open_drain_output();

    let mut i2c = dp.I2C1.i2c(sda, scl, 100.khz(), &mut rcc);

    let mut buffer: [u8; 2];// = [0u8; 2];

    const LP5562_ADDR: u8 = 0x30;

    const LP5562_REG_ENABLE: u8 = 0x00;
    const LP5562_RUN_ENG2: u8 = 0x08;
    const LP5562_REG_ENG_SEL: u8 = 0x70;

    // BRIGHTNESS Registers
    const LP5562_REG_R_PWM: u8 = 0x01;
    // const LP5562_REG_G_PWM: u8 = 0x03;
    // const LP5562_REG_B_PWM: u8 = 0x02;
    // const LP5562_REG_W_PWM: u8 = 0x0E;

    // Set EN High
    lp5562_en.set_high().unwrap();

    // 1 ms delay
    delay.delay_ms(100_u16);

    buffer = [LP5562_REG_ENABLE, 0b01000000];
    // chip_en = 1
    i2c.write(LP5562_ADDR, &mut buffer).unwrap();
    
    // 1 ms delay
    delay.delay_ms(10_u16);
    
    // enable internal clock 
    buffer = [LP5562_RUN_ENG2, 0b00000001];
    i2c.write(LP5562_ADDR, &mut buffer).unwrap();
    // configure all LED outputs to be ontrolled from i2c registers
    buffer = [LP5562_REG_ENG_SEL, 0b00000000];
    i2c.write(LP5562_ADDR, &mut buffer).unwrap();
    // B driver PWM 50% duty cycle
    buffer = [LP5562_REG_R_PWM, 0b11000000];
    i2c.write(LP5562_ADDR, &mut buffer).unwrap();

    loop {
    }
}
