use defmt::info;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use motor_driver_hal::MotorDriver;

use crate::{board::MotorDriverType, tasks::uart_usb::ZERO};

pub enum MotorCommand {
    SetSpeed(i16, i16), // (alpha_left, alpha_right)
    Stop,
    Brake,
}

pub static MOTOR_UPDATE:  Signal<CriticalSectionRawMutex, MotorCommand>  = Signal::new();


#[embassy_executor::task]
pub async fn motors_task(mut left: MotorDriverType, mut right: MotorDriverType) {

    left.enable().unwrap();
    right.enable().unwrap();
    let coef = 9000 / ZERO ;

    loop {
        match MOTOR_UPDATE.wait().await{
            MotorCommand::SetSpeed(alpha_left, alpha_right) => {
                left.enable().unwrap();
                right.enable().unwrap();
                left.set_speed((alpha_left as i32 * coef) as i16).unwrap();
                right.set_speed((alpha_right as i32 * coef) as i16).unwrap();
                info!("Motor signals: alpha_l={}, alpha_r={}", alpha_left, alpha_right);
                info!("Motor speeds updated: alpha_l={}, alpha_r={}", alpha_left as i32 * coef, alpha_right as i32 * coef);
            }
            MotorCommand::Stop => {
                left.stop().unwrap();
                left.disable().unwrap();
                right.stop().unwrap();
                right.disable().unwrap();
            }
            MotorCommand::Brake => {
                left.enable().unwrap();
                right.enable().unwrap();
                left.brake().unwrap();
                right.brake().unwrap();
            }
        }
        info!("Motor command processed, waiting for next command...");
    }
}
