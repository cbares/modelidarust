use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm, SimplePwmChannel};
use embassy_stm32::{bind_interrupts, dma, exti, i2c, interrupt, peripherals};
use embassy_stm32::gpio::{Input, Level, Output, OutputType, Pull, Speed};
use embassy_stm32::peripherals::{TIM2, TIM3, TIM4};
use embassy_stm32::usart::{self, Uart, UartRx, Config as UartConfig};
use embassy_stm32::mode::Async;
use embassy_stm32::i2c::{I2c, Config as I2cConfig, Master};
use embassy_stm32::timer::qei::{Qei, Config as QeiConfig};
use embassy_stm32::time::khz;
use motor_driver_hal::{MotorDriverBuilder, MotorDriverWrapper};


bind_interrupts!(
    struct Irqs {
        EXTI15_10 => exti::InterruptHandler<interrupt::typelevel::EXTI15_10>;
        USART1 => usart::InterruptHandler<peripherals::USART1>;
        USART2 => usart::InterruptHandler<peripherals::USART2>;
        DMA1_STREAM0 => dma::InterruptHandler<peripherals::DMA1_CH0>; // I2C1 RX
        DMA1_STREAM5 => dma::InterruptHandler<peripherals::DMA1_CH5>; // USART2 TX
        DMA1_STREAM6 => dma::InterruptHandler<peripherals::DMA1_CH6>; // USART2 RX
        DMA1_STREAM7 => dma::InterruptHandler<peripherals::DMA1_CH7>; // I2C1 TX
        DMA2_STREAM5 => dma::InterruptHandler<peripherals::DMA2_CH5>; // USART1 RX == SBUS
        I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
        I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    }
);

type MotorDriverType = MotorDriverWrapper<Output<'static>, Output<'static>, SimplePwmChannel<'static, TIM3>, SimplePwmChannel<'static, TIM3>>;

pub struct Board {
    pub led: Output<'static>,
    pub switch: Input<'static>,
    pub serial_usb: Uart<'static, Async>,
    pub i2c: I2c<'static, Async, Master>,
    pub encoder_l: Qei<'static, TIM2>,
    pub encoder_r: Qei<'static, TIM4>,
    pub motor_left: MotorDriverType,
    pub motor_right: MotorDriverType,
    // pub battery_pin: Input<'static>,
    pub sbus: UartRx<'static, Async>,
}

impl Default for Board {
    fn default() -> Self {
        Self::new()
    }
}


impl Board {
    pub fn new() -> Self {
        let p = embassy_stm32::init(Default::default());

        let led = Output::new(p.PA5, Level::Low, Speed::Low);
        let switch = Input::new(p.PC13, Pull::Up);

        let serial = Uart::new(
            p.USART2,
            p.PA3,
            p.PA2,
            p.DMA1_CH6,
            p.DMA1_CH5,
            Irqs,
            UartConfig::default()
        ).expect("USART2 unavailable");

        // === I2C for IMU: MPU6050 ===
        let i2c = I2c::new(
            p.I2C1,
            p.PB8,
            p.PB9,
            p.DMA1_CH7,
            p.DMA1_CH0,
            Irqs,
            I2cConfig::default(),
        );

         // === Encoders ===
        let encoder_l = Qei::new(
            p.TIM2,
            p.PA15,
            p.PB3,
            QeiConfig::default());

        let encoder_r = Qei::new(
            p.TIM4,
            p.PB6,
            p.PB7,
            QeiConfig::default());



        // let battery_pin = Input::new(p.PB0, Pull::Up);

        // === Motor driver TB6612FNG ===
        let pwm1 = PwmPin::new(p.PC6, OutputType::PushPull);
        let pwm2 = PwmPin::new(p.PC7, OutputType::PushPull);
        // let pwm3 = PwmPin::new(p.PC8, OutputType::PushPull);
        // let pwm4 = PwmPin::new(p.PC9, OutputType::PushPull);

        let simple_pwm = SimplePwm::new(//)::<AfioRemap<3>>(
            p.TIM3,
            Some(pwm1),  // Channel 1
            Some(pwm2),  // Channel 2
            None,  // Channel 3
            None,  // Channel 4
            khz(100),  // Frequency
            Default::default()
        );
        let pwm_chanels = simple_pwm.split();
        let pwm_ch1 = pwm_chanels.ch1;
        let pwm_ch2 = pwm_chanels.ch2;
        let en1_l = Output::new(p.PC11, Level::Low, Speed::Low);
        let en2_l = Output::new(p.PB4, Level::Low, Speed::Low);
        let en1_r = Output::new(p.PC12, Level::Low, Speed::Low);
        let en2_r = Output::new(p.PB5, Level::Low, Speed::Low);

        let motor_left = MotorDriverBuilder::new()
            .with_dual_enable(en1_l, en2_l)
            .with_single_pwm(pwm_ch1)
            .with_max_duty(10000)
            .build_and_init().expect("Left Driver Motor problem");
        let motor_right = MotorDriverBuilder::new()
            .with_dual_enable(en1_r, en2_r)
            .with_single_pwm(pwm_ch2)
            .with_max_duty(10000)
            .build_and_init().expect("Right Driver Motor problem");

        let sbus = UartRx::new(
            p.USART1, 
            p.PA10, 
             p.DMA2_CH5,
             Irqs,
            UartConfig::default()
        ).expect("USART1 unavailable");

        Self {
            led,
            switch,
            serial_usb: serial,
            i2c,
            encoder_l,
            encoder_r,
            motor_left,
            motor_right,
            // battery_pin,
            sbus,
        }
    }
}