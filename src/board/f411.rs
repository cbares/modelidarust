//! Nucleo-64 STM32F411RE, 100 MHz.
//!
//! Différences de brochage avec la F446 :
//! - encodeur gauche sur TIM3 PB4 / PB5 (PB8 / PB9 ne sont pas des voies 1-2 de timer sur F411) ;
//! - SBUS sur USART6 RX (PC7), la F411 n'a pas d'UART5.

use defmt::info;
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::peripherals::{TIM3, TIM5};
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::qei::{Config as QeiConfig, Qei};
use embassy_stm32::usart::{self, Config as UartConfig, Uart, UartRx};
use embassy_stm32::{bind_interrupts, dma, peripherals};

use super::{Board, init_motors, sbus_config};

/// Encodeur gauche : TIM3 sur PB4 / PB5 (CN10-27 / CN10-29).
pub type QeiLeft = Qei<'static, TIM3>;
/// Encodeur droit : TIM5 sur PA0 / PA1.
pub type QeiRight = Qei<'static, TIM5>;

bind_interrupts!(
    struct Irqs {
        USART2 => usart::InterruptHandler<peripherals::USART2>;
        USART6 => usart::InterruptHandler<peripherals::USART6>;
        DMA1_STREAM5 => dma::InterruptHandler<peripherals::DMA1_CH5>; // USART2 RX
        DMA1_STREAM6 => dma::InterruptHandler<peripherals::DMA1_CH6>; // USART2 TX
        DMA2_STREAM1 => dma::InterruptHandler<peripherals::DMA2_CH1>; // USART6 RX (SBUS)
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
                mul: PllMul::MUL100,
                divp: Some(PllPDiv::DIV2), // 8 MHz / 4 * 100 / 2 = 100 MHz
                divq: None,
                divr: None,
            });
            config.rcc.ahb_pre = AHBPrescaler::DIV1;
            config.rcc.apb1_pre = APBPrescaler::DIV2; // 50 MHz (max)
            config.rcc.apb2_pre = APBPrescaler::DIV1; // 100 MHz (TIM1 à 100 MHz)
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

        // === Encodeurs ===
        let qei_left = Qei::new(p.TIM3, p.PB4, p.PB5, QeiConfig::default());
        let qei_right = Qei::new(p.TIM5, p.PA0, p.PA1, QeiConfig::default());

        // === Moteurs ===
        let (motor_left, motor_right, max_duty) =
            init_motors(p.TIM1, p.PA8, p.PA9, p.PA10, p.PA11, p.PC4, p.PB3);

        // === Récepteur SBUS sur USART6 ===
        let sbus = UartRx::new(p.USART6, p.PC7, p.DMA2_CH1, Irqs, sbus_config())
            .expect("USART6 unavailable");

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
