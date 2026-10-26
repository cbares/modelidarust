//! Nucleo-64 STM32L476RG (carte MB1136), 80 MHz.
//!
//! Différences de brochage avec la F446 :
//! - encodeur droit sur TIM3 PB4 / PB5 (PB8 / PB9 ne sont pas des voies 1-2 de timer sur L476) ;
//! - SBUS sur UART5 RX (PD2) comme sur la F446, mais l'USART de la L4 sait
//!   inverser le RX : pas besoin d'inverseur matériel.

use defmt::info;
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::peripherals::{TIM3, TIM5};
use embassy_stm32::timer::qei::{Config as QeiConfig, Qei};
use embassy_stm32::usart::{self, Config as UartConfig, Uart, UartRx};
use embassy_stm32::{bind_interrupts, dma, peripherals};

use super::{Board, init_motors, sbus_config};

/// Encodeur gauche : TIM5 sur PA0 / PA1.
pub type QeiLeft = Qei<'static, TIM5>;
/// Encodeur droit : TIM3 sur PB4 / PB5 (CN10-27 / CN10-29).
pub type QeiRight = Qei<'static, TIM3>;

bind_interrupts!(
    struct Irqs {
        USART2 => usart::InterruptHandler<peripherals::USART2>;
        UART5 => usart::InterruptHandler<peripherals::UART5>;
        DMA1_CHANNEL6 => dma::InterruptHandler<peripherals::DMA1_CH6>; // USART2 RX
        DMA1_CHANNEL7 => dma::InterruptHandler<peripherals::DMA1_CH7>; // USART2 TX
        DMA2_CHANNEL2 => dma::InterruptHandler<peripherals::DMA2_CH2>; // UART5 RX (SBUS)
    }
);

impl Board {
    pub fn new() -> Self {
        let mut config = embassy_stm32::Config::default();
        {
            use embassy_stm32::rcc::*;
            // HSI16 interne : ne dépend pas du câblage MCO du ST-Link.
            config.rcc.hsi = true;
            config.rcc.pll = Some(Pll {
                source: PllSource::HSI,
                prediv: PllPreDiv::DIV1,
                mul: PllMul::MUL10,
                divp: None,
                divq: None,
                divr: Some(PllRDiv::DIV2), // 16 MHz * 10 / 2 = 80 MHz
            });
            config.rcc.sys = Sysclk::PLL1_R;
            config.rcc.ahb_pre = AHBPrescaler::DIV1;
            config.rcc.apb1_pre = APBPrescaler::DIV1; // 80 MHz
            config.rcc.apb2_pre = APBPrescaler::DIV1; // 80 MHz (TIM1 à 80 MHz)
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
            p.DMA1_CH7,
            p.DMA1_CH6,
            Irqs,
            UartConfig::default(),
        )
        .expect("USART2 unavailable");

        // === Encodeurs ===
        let qei_left = Qei::new(p.TIM5, p.PA0, p.PA1, QeiConfig::default());
        let qei_right = Qei::new(p.TIM3, p.PB4, p.PB5, QeiConfig::default());

        // === Moteurs ===
        let (motor_left, motor_right, max_duty) =
            init_motors(p.TIM1, p.PA8, p.PA9, p.PA10, p.PA11, p.PC4, p.PB3);

        // === Récepteur SBUS sur UART5, inversion faite par l'USART ===
        let mut sbus_cfg = sbus_config();
        sbus_cfg.invert_rx = true;
        let sbus = UartRx::new(p.UART5, p.PD2, p.DMA2_CH2, Irqs, sbus_cfg)
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
