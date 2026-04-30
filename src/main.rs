 #![no_std]
 #![no_main]
 

use defmt::info;
use defmt_rtt as _;
use panic_probe as _;

use modelidar::board::Board;
use embassy_time::{Ticker, Duration};
use embassy_executor::Spawner;


#[embassy_executor::main]
 async fn main(_spawner: Spawner) {
    let board = Board::new();
    let mut led = board.led;
    let mut serial = board.serial_usb;

    info!("Starting blink example");
    serial.write(b"MCU started\r\n").await.unwrap();

    let mut ticker = Ticker::every(Duration::from_millis(500));
    loop {
        led.toggle();
        info!("LED toggled");
        serial.write(b"LED changed !\r\n").await.unwrap();
        ticker.next().await;
    }

}