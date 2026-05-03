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


#[embassy_executor::main]
 async fn main(spawner: Spawner) {
    let board = Board::new();

    let mut led = board.led;
    let mut serial = board.serial_usb;
    let qei_left = board.qei_left;
    let qei_right = board.qei_right;
    
    // Spawn the watchdog task with a 10ms timeout
    spawner.spawn(watchdog_task(board.iwdg, 10000).unwrap());
    
    info!("Starting blink example");
    serial.write(b"MCU started\r\n").await.unwrap();
    
    // Spawn the UART communication task
    spawner.spawn(uart_task(serial).unwrap());
    
    // Spawn the odometry task
    let odom_interval = Duration::from_millis(100);
    spawner.spawn(odometry_task(odom_interval, qei_left, qei_right).unwrap());


    let mut ticker = Ticker::every(Duration::from_millis(1000));

    loop {
        led.toggle();
        info!("LED toggled");
        ticker.next().await;
    }
}
