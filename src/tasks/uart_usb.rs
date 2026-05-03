use defmt::info;
use embassy_stm32::usart::{RingBufferedUartRx, Uart, UartTx};
use embassy_stm32::mode::Async;
use heapless::{String, format, Vec};
use static_cell::StaticCell;

use crate::state::{MOTOR_CMD, MotorCmd, ODOM_STATE};

const CMD_LENGTH: usize = 10; // "#AAAA,BBBB"
const STEPS: i32 = 1000;
const ZERO: i32 = STEPS / 2;
const _DELTA_STEP: i32 = 10;

static SBUS_RX_BUF: StaticCell<[u8; CMD_LENGTH * 2]> = StaticCell::new();


#[embassy_executor::task]
pub async fn uart_task(
    serial: Uart<'static, Async>,
) {
    let rx_buf = SBUS_RX_BUF.init([0; CMD_LENGTH * 2]);
    let (mut serial_tx, serial_rx) = serial.split(); 
    let mut serial_rx = serial_rx.into_ring_buffered( rx_buf);

    loop {
        // --- Etape 1 : attendre 1 octet (détection de commande) ---
        let mut header = [0u8; 1];
        if serial_rx.read(&mut header).await.is_err() {
            continue;
        }

        match header[0] {
            b'?' => {
                // Query: send current state
                info!("Received query command");
                transmit_state(&mut serial_tx).await;
                continue;
            },

            b'*' => {
                // Command: reset motor command to zero
                info!("Received reset command");
                let motor_cmd = MotorCmd { alpha_left: 0, alpha_right: 0 };
                MOTOR_CMD.lock(|s| s.set(motor_cmd));
                transmit_state(&mut serial_tx).await;
                info!("state transmitted: alpha_l={}, alpha_r={}", motor_cmd.alpha_left, motor_cmd.alpha_right);
            },

            b'#' => {
                // Command: Read next CMD_LENGTH octets for "alphaL,alphaR"
                info!("Received command header, waiting for parameters");
                let mut rest = [0u8; CMD_LENGTH];
                rest[0] = header[0];

                // Lire jusqu'au CMD_LENGTH octets
                match <RingBufferedUartRx<'static> as embedded_io_async::Read>::read_exact(&mut serial_rx, &mut rest[1..]).await {
                    Ok(_) => {
                        info!("Received command parameters: {:?}", rest);
                        // Parse "alphaL,alphaR" ex: "0312,0768"
                        if let Some((al, ar)) = parse_pattern(rest) {
                            info!("Updated state: alpha_l={}, alpha_r={}", al, ar);
                            let motor_cmd = MotorCmd {
                                alpha_left: al.clamp(0, STEPS) - ZERO,
                                alpha_right: ar.clamp(0, STEPS) - ZERO,
                            };
                            MOTOR_CMD.lock(|s| s.set(motor_cmd));
                            transmit_state(&mut serial_tx).await;
                            info!("state transmitted: alpha_l={}, alpha_r={}", motor_cmd.alpha_left, motor_cmd.alpha_right);
                        } else {
                            info!("Failed to parse command parameters");
                            /* parsing failed → ignore */ }
                    }
                    Err(_) => { 
                        info!("Failed to read command parameters (timeout or error)");
                        /* timeout ou erreur → ignore */ }
                }
            },
            
            _ => {
                // Ignore unrecognized commands
                continue;
            }
        }
    }
}


/// Parse "AAAA,BBBB" → Some((a, b))
fn parse_pattern(buf: [u8; CMD_LENGTH]) -> Option<(i32, i32)> {
    let p1 = String::<4>::from_utf8(
        Vec::<u8, 4>::from_slice(&buf[1..5]).unwrap()).ok()?;
    let p2 = String::<4>::from_utf8(
        Vec::<u8, 4>::from_slice(&buf[6..10]).unwrap()).ok()?;
    let a = p1.parse::<i32>().ok()?;
    let b = p2.parse::<i32>().ok()?;
    Some((a, b))
}


async fn transmit_state(serial: &mut UartTx<'static, Async>) {
    let state = ODOM_STATE.lock(|s| s.get());
    let odom_left = &state.odom_left;
    let odom_right = &state.odom_right;

    let motor = MOTOR_CMD.lock(|s| s.get());
    let alpha_l = motor.alpha_left + ZERO;
    let alpha_r = motor.alpha_right + ZERO;

    let buf: String<64> = format!("!{:04},{:04} L:{:+},{:+} R:{:+},{:+}\r\n", 
            alpha_l, alpha_r,
			odom_left.velocity, odom_left.position,
			odom_right.velocity, odom_right.position).expect("format macro failed");
    serial.write(buf.as_bytes()).await.unwrap();
}