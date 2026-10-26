#![no_std]
#![no_main]

use defmt::info;
use defmt_rtt as _;
use panic_probe as _;

use embassy_time::{Ticker, Duration};
use embassy_executor::Spawner;

use modelidar::board::Board;
use modelidar::tasks::uart_usb::uart_task;
use modelidar::tasks::odometry::odometry_task;
use modelidar::tasks::watchdog::watchdog_task;
use modelidar::tasks::motors::motors_task;


#[embassy_executor::main]
 async fn main(spawner: Spawner) {
    let mut board = Board::new();
    let mut led = board.led;
  
    info!("MCU started");
    board.serial_usb.write(b"MCU started\r\n").await.unwrap();
    
    // Spawn the watchdog task with a 10ms timeout
    let watchdog_timeout_us = 10_000;
    spawner.spawn(watchdog_task(board.iwdg, watchdog_timeout_us).unwrap());
    
    // Spawn the UART communication task
    spawner.spawn(uart_task(board.serial_usb).unwrap());
    
    // Spawn the odometry task
    let odom_interval = Duration::from_millis(100);
    spawner.spawn(odometry_task(odom_interval, board.qei_left, board.qei_right).unwrap());

    // Spawn the motors task
    spawner.spawn(motors_task(board.motor_left, board.motor_right, board.max_duty).unwrap());

    let main_loop_duration = Duration::from_secs(10);
    let mut ticker = Ticker::every(main_loop_duration);

    loop {
        led.toggle();
        info!("LED toggled");
        ticker.next().await;
    }
}
