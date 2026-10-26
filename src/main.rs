#![no_std]
#![no_main]

use core::fmt::Write;

use defmt::{info, warn};
use defmt_rtt as _;
use panic_probe as _;

use embassy_time::{Ticker, Duration};
use embassy_executor::Spawner;

use modelidar::board::Board;
use modelidar::tasks::uart_usb::uart_task;
use modelidar::tasks::odometry::odometry_task;
use modelidar::tasks::watchdog::{ResetCause, reset_cause, watchdog_task};
use modelidar::tasks::motors::motors_task;
use modelidar::tasks::sbus::sbus_task;


#[embassy_executor::main]
 async fn main(spawner: Spawner) {
    let mut board = Board::new();
    let mut led = board.led;
  
    // Cause du redémarrage, aussi envoyée au PC (ligne hors protocole).
    let cause = reset_cause();
    if cause == ResetCause::Watchdog {
        warn!("MCU started after a watchdog reset: a task was stuck");
    } else {
        info!("MCU started (reset: {})", cause);
    }
    let mut msg: heapless::String<48> = heapless::String::new();
    let _ = write!(msg, "MCU started (reset: {})\r\n", cause.tag());
    board.serial_usb.write(msg.as_bytes()).await.unwrap();

    // Watchdog : redémarre la carte si une tâche surveillée se bloque
    spawner.spawn(watchdog_task(board.iwdg).unwrap());
    
    // Spawn the UART communication task
    spawner.spawn(uart_task(board.serial_usb).unwrap());
    
    // Spawn the odometry task
    spawner.spawn(odometry_task(board.qei_left, board.qei_right).unwrap());

    // Spawn the motors task
    spawner.spawn(motors_task(board.motor_left, board.motor_right, board.max_duty).unwrap());

    // Spawn the RC receiver task (SBUS), which can take over from USB
    spawner.spawn(sbus_task(board.sbus).unwrap());

    let main_loop_duration = Duration::from_secs(10);
    let mut ticker = Ticker::every(main_loop_duration);

    loop {
        led.toggle();
        info!("LED toggled");
        ticker.next().await;
    }
}
