use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm, SimplePwmChannel};
use embassy_stm32::{Peri, bind_interrupts, dma, exti, i2c, interrupt, peripherals};
use embassy_stm32::gpio::{Input, Level, Output, OutputType, Pull, Speed};
use embassy_stm32::peripherals::{TIM1, TIM2, TIM5, IWDG};
use embassy_stm32::usart::{self, Config as UartConfig, Uart, UartRx};
use embassy_stm32::mode::Async;
use embassy_stm32::i2c::{I2c, Config as I2cConfig, Master};
use embassy_stm32::timer::qei::{Qei, Config as QeiConfig};
use embassy_stm32::time::khz;
use motor_driver_hal::{MotorDriverBuilder, MotorDriverWrapper};


bind_interrupts!(
    struct Irqs {
        EXTI15_10 => exti::InterruptHandler<interrupt::typelevel::EXTI15_10>;
        USART2 => usart::InterruptHandler<peripherals::USART2>;
        UART5 => usart::InterruptHandler<peripherals::UART5>;
        DMA1_STREAM0 => dma::InterruptHandler<peripherals::DMA1_CH0>; // UART5 RX == SBUS
        DMA1_STREAM2 => dma::InterruptHandler<peripherals::DMA1_CH2>; // I2C2 RX
        DMA1_STREAM5 => dma::InterruptHandler<peripherals::DMA1_CH5>; // USART2 TX
        DMA1_STREAM6 => dma::InterruptHandler<peripherals::DMA1_CH6>; // USART2 RX
        DMA1_STREAM7 => dma::InterruptHandler<peripherals::DMA1_CH7>; // I2C2 TX
        DMA2_STREAM1 => dma::InterruptHandler<peripherals::DMA2_CH1>; 
        I2C2_ER => i2c::ErrorInterruptHandler<peripherals::I2C2>;
        I2C2_EV => i2c::EventInterruptHandler<peripherals::I2C2>;
    }
);

type MotorDriverType = MotorDriverWrapper<Output<'static>, Output<'static>, SimplePwmChannel<'static, TIM1>, SimplePwmChannel<'static, TIM1>>;

pub struct Board {
    pub led: Output<'static>,
    pub switch: Input<'static>,
    pub serial_usb: Uart<'static, Async>,
    pub i2c: I2c<'static, Async, Master>,
    pub qei_left: Qei<'static, TIM2>,
    pub qei_right: Qei<'static, TIM5>,
    pub motor_left: MotorDriverType,
    pub motor_right: MotorDriverType,
    // pub battery_pin: Input<'static>,
    pub sbus: UartRx<'static, Async>,
    pub iwdg: Peri<'static, IWDG>,
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

        // === I2C for IMU: MPU9250 ===
        let i2c = I2c::new(
            p.I2C2,
            p.PB10,
            p.PB3,
            p.DMA1_CH7,
            p.DMA1_CH2,
            Irqs,
            I2cConfig::default(),
        );

         // === Encoders ===
        let encoder_l = Qei::new(
            p.TIM2,
            p.PB8,
            p.PB9,
            QeiConfig::default());

        let encoder_r = Qei::new(
            p.TIM5,
            p.PA0,
            p.PA1,
            QeiConfig::default());



        // let battery_pin = Input::new(p.PB0, Pull::Up);

        // === Motor driver TB6612FNG ===
        let pwm1 = PwmPin::new(p.PA8, OutputType::PushPull);
        let pwm2 = PwmPin::new(p.PA9, OutputType::PushPull);
        let pwm3 = PwmPin::new(p.PA10, OutputType::PushPull);
        let pwm4 = PwmPin::new(p.PA11, OutputType::PushPull);

        let simple_pwm = SimplePwm::new(//)::<AfioRemap<3>>(
            p.TIM1,
            Some(pwm1),
            Some(pwm2), 
            Some(pwm3),
            Some(pwm4),
            khz(100),  // Frequency
            Default::default()
        );

        let pwm_chanels = simple_pwm.split();
        let pwm_ch1 = pwm_chanels.ch1;
        let pwm_ch2 = pwm_chanels.ch2;
        let pwm_ch3 = pwm_chanels.ch3;
        let pwm_ch4 = pwm_chanels.ch4;
        let en_l = Output::new(p.PC11, Level::Low, Speed::Low);
        let en_r = Output::new(p.PC12, Level::Low, Speed::Low);

        let motor_left = MotorDriverBuilder::new()
            .with_single_enable(en_l)
            .with_dual_pwm(pwm_ch1, pwm_ch2)
            .with_max_duty(10000)
            .build_and_init().expect("Left Driver Motor problem");
        let motor_right = MotorDriverBuilder::new()
            .with_single_enable(en_r)
            .with_dual_pwm(pwm_ch3, pwm_ch4)
            .with_max_duty(10000)
            .build_and_init().expect("Right Driver Motor problem");

        let sbus = UartRx::new(
            p.UART5, 
            p.PD2, 
            p.DMA1_CH0,
            Irqs,
            UartConfig::default()
        ).expect("UART5 unavailable");


        let iwdg = p.IWDG;

        Self {
            led,
            switch,
            serial_usb: serial,
            i2c,
            qei_left: encoder_l,
            qei_right: encoder_r,
            motor_left,
            motor_right,
            // battery_pin,
            sbus,
            iwdg,
        }
    }
}