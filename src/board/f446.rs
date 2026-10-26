//! Nucleo-64 STM32F446RE, 180 MHz.

use defmt::info;
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::peripherals::{TIM2, TIM5};
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::qei::{Config as QeiConfig, Qei};
use embassy_stm32::usart::{self, Config as UartConfig, Uart, UartRx};
use embassy_stm32::{bind_interrupts, dma, exti, i2c, interrupt, peripherals};

use super::{Board, init_motors};

/// Encodeur gauche : TIM2 sur PB8 / PB9.
pub type QeiLeft = Qei<'static, TIM2>;
/// Encodeur droit : TIM5 sur PA0 / PA1.
pub type QeiRight = Qei<'static, TIM5>;

bind_interrupts!(
    struct Irqs {
        EXTI15_10 => exti::InterruptHandler<interrupt::typelevel::EXTI15_10>;
        USART2 => usart::InterruptHandler<peripherals::USART2>;
        UART5 => usart::InterruptHandler<peripherals::UART5>;
        DMA1_STREAM0 => dma::InterruptHandler<peripherals::DMA1_CH0>; // UART5 RX (SBUS)
        DMA1_STREAM2 => dma::InterruptHandler<peripherals::DMA1_CH2>; // I2C2 RX
        DMA1_STREAM5 => dma::InterruptHandler<peripherals::DMA1_CH5>; // USART2 RX
        DMA1_STREAM6 => dma::InterruptHandler<peripherals::DMA1_CH6>; // USART2 TX
        DMA1_STREAM7 => dma::InterruptHandler<peripherals::DMA1_CH7>; // I2C2 TX
        I2C2_ER => i2c::ErrorInterruptHandler<peripherals::I2C2>;
        I2C2_EV => i2c::EventInterruptHandler<peripherals::I2C2>;
    }
);

impl Board {
    pub fn new() -> Self {
        let mut config = embassy_stm32::Config::default();
        {
            use embassy_stm32::rcc::*;
            config.rcc.hse = Some(Hse {
                freq: Hertz(8_000_000),
                // Sur Nucleo, le HSE vient de la sortie MCO 8 MHz du ST-Link.
                mode: HseMode::Bypass,
            });
            config.rcc.pll_src = PllSource::HSE;
            config.rcc.pll = Some(Pll {
                prediv: PllPreDiv::DIV4,
                mul: PllMul::MUL180,
                divp: Some(PllPDiv::DIV2), // 8 MHz / 4 * 180 / 2 = 180 MHz
                divq: None,
                divr: None,
            });
            config.rcc.ahb_pre = AHBPrescaler::DIV1;
            config.rcc.apb1_pre = APBPrescaler::DIV4; // 45 MHz
            config.rcc.apb2_pre = APBPrescaler::DIV2; // 90 MHz (TIM1 à 180 MHz)
            config.rcc.sys = Sysclk::PLL1_P;
        }
        let p = embassy_stm32::init(config);
        info!("System clock: {:?} Hz", embassy_stm32::rcc::clocks(&p.RCC));

        let led = Output::new(p.PA5, Level::Low, Speed::Low);
        let switch = Input::new(p.PC13, Pull::Up);

        // === USB Serial (ST-Link VCP) ===
        let serial_usb = Uart::new(
            p.USART2,
            p.PA3,
            p.PA2,
            p.DMA1_CH6,
            p.DMA1_CH5,
            Irqs,
            UartConfig::default(),
        )
        .expect("USART2 unavailable");

        // === I2C pour l'IMU MPU9250 (I2C2), à réactiver ===
        // let i2c = I2c::new(p.I2C2, p.PB10, p.PC12, p.DMA1_CH7, p.DMA1_CH2, Irqs, I2cConfig::default());
        
        // === Encodeurs ===
        let qei_left = Qei::new(p.TIM2, p.PB8, p.PB9, QeiConfig::default());
        let qei_right = Qei::new(p.TIM5, p.PA0, p.PA1, QeiConfig::default());

        // === Moteurs ===
        let (motor_left, motor_right, max_duty) =
            init_motors(p.TIM1, p.PA8, p.PA9, p.PA10, p.PA11, p.PC4, p.PB3);

        // === Récepteur SBUS sur UART5 ===
        let sbus = UartRx::new(p.UART5, p.PD2, p.DMA1_CH0, Irqs, UartConfig::default())
            .expect("UART5 unavailable");

        Self {
            led,
            switch,
            serial_usb,
            qei_left,
            qei_right,
            motor_left,
            motor_right,
            max_duty,
            sbus,
            iwdg: p.IWDG,
        }
    }
}
