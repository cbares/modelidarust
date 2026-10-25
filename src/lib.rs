#![no_std]
#![no_main]

pub mod board;
pub mod state;
pub mod tasks{
    pub mod odometry;
    pub mod uart_usb;
    pub mod watchdog;
    pub mod motors;
}
