#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

use embedded_hal::blocking::i2c::{Write};

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

    // let mut buffer: [u8; 2];// = [0u8; 2];

    // const LP5562_ADDR: u8 = 0x30;

    let led_driver = Lp5562::new(AddrSel::Addr00);

    // Set EN High
    lp5562_en.set_high().unwrap();

    // 1 ms delay
    delay.delay_ms(100_u16);

    // buffer = [Register::Enable as u8, 0b01000000];
    // chip_en = 1
    // i2c.write(LP5562_ADDR, &mut buffer).unwrap();

   let _s = led_driver.init_direct_pwm(&mut i2c);

    // 1 ms delay
    delay.delay_ms(10_u16);

    // led_driver.set_led_current(&mut i2c, LED::Blue, 0x00).unwrap();
    let _s = led_driver.set_led_pwm(&mut i2c, LED::Red, 0x0F);

    loop {
    }
}

pub struct Lp5562 {
    addr: u8,
}

pub enum AddrSel {
    Addr00 = 0x60,
    Addr01 = 0x62,
    Addr10 = 0x64,
    Addr11 = 0x66,
}

pub enum LED {
    Blue,
    Green,
    Red,
    White,
}

pub enum Error<I> {
    I2c(I),
}

// impl fmt::Debug for Error {
//     fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
//         write!(f, "({:?}, {:?})", self.longitude, self.latitude)
//     }
// }

impl Lp5562 {
    pub fn new(addrsel: AddrSel) -> Lp5562 {
        Lp5562 {
            addr: (addrsel as u8) >> 1, // Shift right by one for 7-bit Address
        }
    }

    pub fn write_addr<I>(
        &self,
        i2c: &mut I,
        addr: Register,
        value: u8,
    ) -> Result<(), Error<I::Error>>
    where
        I: Write,
    {
        i2c.write(self.addr, &[addr as u8, value])
            .map_err(|e| Error::I2c(e))
    }

    pub fn set_led_pwm<I>(
        &self,
        i2c: &mut I,
        led: LED,
        value: u8,
    ) -> Result<(), Error<I::Error>>
    where
        I: Write,
    {
        match led {
            LED::Blue => self.write_addr(i2c, Register::Bpwm, value)?,
            LED::Green => self.write_addr(i2c, Register::Gpwm, value)?,
            LED::Red => self.write_addr(i2c, Register::Rpwm, value)?,
            LED::White => self.write_addr(i2c, Register::Wpwm, value)?,
        }

        Ok(())
    }

    pub fn set_led_current<I>(
        &self,
        i2c: &mut I,
        led: LED,
        value: u8,
    ) -> Result<(), Error<I::Error>>
    where
        I: Write,
    {
        match led {
            LED::Blue => self.write_addr(i2c, Register::Bcurrent, value)?,
            LED::Green => self.write_addr(i2c, Register::Gcurrent, value)?,
            LED::Red => self.write_addr(i2c, Register::Rcurrent, value)?,
            LED::White => self.write_addr(i2c, Register::Wcurrent, value)?,
        }

        Ok(())
    }

    pub fn init_direct_pwm<I>(
        &self,
        i2c: &mut I,
    ) -> Result<(), Error<I::Error>>
    where
        I: Write,
    {
        // chip enable
        self.write_addr(i2c, Register::Enable, 0b01000000)?;
        // enable internal clock 
        self.write_addr(i2c, Register::Config, 0b00000001)?;
        // configure all LED outputs to be ontrolled from i2c registers
        self.write_addr(i2c, Register::LedMap, 0b00000000)?;

        // Set all LED PWM to zero
        self.write_addr(i2c, Register::Bpwm, 0x00)?;
        self.write_addr(i2c, Register::Gpwm, 0x00)?;
        self.write_addr(i2c, Register::Rpwm, 0x00)?;
        self.write_addr(i2c, Register::Wpwm, 0x00)?;

        Ok(())
    }
}

pub enum Register {
    Enable = 0x00,
    OpMode = 0x01,
    Bpwm = 0x02,
    Gpwm = 0x03,
    Rpwm = 0x04,
    Bcurrent = 0x05,
    Gcurrent = 0x06,
    Rcurrent = 0x07,
    Config = 0x08,
    Eng1Pc = 0x09,
    Eng2Pc = 0x0A,
    Eng3Pc = 0x0B,
    Status = 0x0C,
    Reset = 0x0D,
    Wpwm = 0x0E,
    Wcurrent = 0x0F,
    ProgMemEng1Base = 0x10,
    ProgMemEng2Base = 0x30,
    ProgMemEng3Base = 0x50,
    LedMap = 0x70,
}
