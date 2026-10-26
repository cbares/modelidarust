//! Configuration matérielle. Une seule carte est compilée, choisie par feature :
//! `stm32f446re` (par défaut) ou `stm32f411re`.
//!
//! Chaque module de carte fournit `QeiLeft`, `QeiRight` et `Board::new()`.

#[cfg(all(feature = "stm32f446re", feature = "stm32f411re"))]
compile_error!("Activer une seule carte : `stm32f446re` ou `stm32f411re`.");
#[cfg(not(any(feature = "stm32f446re", feature = "stm32f411re")))]
compile_error!("Aucune carte activée : utiliser la feature `stm32f446re` ou `stm32f411re`.");

#[cfg(feature = "stm32f446re")]
mod f446;
#[cfg(feature = "stm32f446re")]
pub use f446::{QeiLeft, QeiRight};

#[cfg(feature = "stm32f411re")]
mod f411;
#[cfg(feature = "stm32f411re")]
pub use f411::{QeiLeft, QeiRight};

use defmt::info;
use embassy_stm32::Peri;
use embassy_stm32::gpio::{Input, Level, Output, OutputType, Speed};
use embassy_stm32::mode::Async;
use embassy_stm32::peripherals::{IWDG, PA8, PA9, PA10, PA11, PB3, PC4, TIM1};
use embassy_stm32::time::khz;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm, SimplePwmChannel};
use embassy_stm32::usart::{Uart, UartRx};
use motor_driver_hal::{MotorDriverBuilder, MotorDriverWrapper};

pub type MotorDriverType = MotorDriverWrapper<
    Output<'static>,
    Output<'static>,
    SimplePwmChannel<'static, TIM1>,
    SimplePwmChannel<'static, TIM1>,
>;

pub struct Board {
    pub led: Output<'static>,
    pub switch: Input<'static>,
    pub serial_usb: Uart<'static, Async>,
    pub qei_left: QeiLeft,
    pub qei_right: QeiRight,
    pub motor_left: MotorDriverType,
    pub motor_right: MotorDriverType,
    /// Rapport cyclique maximal des PWM moteurs (dépend de l'horloge de TIM1).
    pub max_duty: u16,
    pub sbus: UartRx<'static, Async>,
    pub iwdg: Peri<'static, IWDG>,
}

impl Default for Board {
    fn default() -> Self {
        Self::new()
    }
}

/// Moteurs sur TIM1 (PA8..PA11, 20 kHz) avec enables PC4 / PB3 :
/// brochage identique sur F446 et F411.
#[allow(clippy::too_many_arguments)]
fn init_motors(
    tim: Peri<'static, TIM1>,
    pa8: Peri<'static, PA8>,
    pa9: Peri<'static, PA9>,
    pa10: Peri<'static, PA10>,
    pa11: Peri<'static, PA11>,
    pc4: Peri<'static, PC4>,
    pb3: Peri<'static, PB3>,
) -> (MotorDriverType, MotorDriverType, u16) {
    let pwm = SimplePwm::new(
        tim,
        Some(PwmPin::new(pa8, OutputType::PushPull)),
        Some(PwmPin::new(pa9, OutputType::PushPull)),
        Some(PwmPin::new(pa10, OutputType::PushPull)),
        Some(PwmPin::new(pa11, OutputType::PushPull)),
        khz(20),
        Default::default(),
    );

    let ch = pwm.split();
    let mut in1a = ch.ch1; // PA8
    let mut in2a = ch.ch2; // PA9
    let mut in1b = ch.ch3; // PA10
    let mut in2b = ch.ch4; // PA11
    in1a.enable();
    in2a.enable();
    in1b.enable();
    in2b.enable();

    let max_duty = u16::try_from(in1a.max_duty_cycle()).expect("PWM max duty > u16");
    info!("PWM max duty: {}, frequency: {}", max_duty, in1a.get_frequency());

    let en_l = Output::new(pc4, Level::Low, Speed::Low);
    let en_r = Output::new(pb3, Level::Low, Speed::Low);

    let left = MotorDriverBuilder::new()
        .with_single_enable(en_l)
        .with_dual_pwm(in1a, in2a)
        .with_max_duty(max_duty)
        .build_and_init()
        .expect("Left Driver Motor problem");
    // Voies B inversées : le moteur droit est monté en miroir.
    let right = MotorDriverBuilder::new()
        .with_single_enable(en_r)
        .with_dual_pwm(in2b, in1b)
        .with_max_duty(max_duty)
        .build_and_init()
        .expect("Right Driver Motor problem");

    (left, right, max_duty)
}
