use defmt::{debug, info};
use embassy_stm32::usart::{RingBufferedUartRx, Uart, UartTx};
use embassy_stm32::mode::Async;
use heapless::{String, format, Vec};
use static_cell::StaticCell;

use crate::state::{MOTOR_CMD, MotorCmd, ODOM_STATE};
use crate::tasks::motors::{MOTOR_UPDATE, MotorCommand};

const CMD_LENGTH: usize = 10; // "#AAAA,BBBB"
const STEPS: i32 = 1000;
pub const ZERO: i32 = STEPS / 2;
const _DELTA_STEP: i32 = 10;

static SBUS_RX_BUF: StaticCell<[u8; CMD_LENGTH * 2]> = StaticCell::new();


#[embassy_executor::task]
pub async fn uart_task(
    serial: Uart<'static, Async>,
) {
    let rx_buf = SBUS_RX_BUF.init([0; CMD_LENGTH * 2]);
    let (mut serial_tx, serial_rx) = serial.split(); 
    let mut serial_rx = serial_rx.into_ring_buffered( rx_buf);
    let mut motors = MotorCommand::SetSpeed(0, 0);

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
                transmit_state(&mut serial_tx, &motors).await;
                continue;
            },

            b'*' => {
                // Command: reset motor command to zero
                info!("Received reset command");
                // let motor_cmd = MotorCmd::new(0, 0, 500);
                let motor_cmd = MotorCommand::Brake;
                // MOTOR_CMD.lock(|s| s.set(motor_cmd));
                MOTOR_UPDATE.signal(motor_cmd);
                motors = MotorCommand::Brake;
                transmit_state(&mut serial_tx, &motors).await;
                debug!("state transmitted: Brake");
            },

            b'/' => {
                // Command: break motor command
                info!("Received break command");
                // let motor_cmd = MotorCmd::new(0, 0, 500);
                // MOTOR_CMD.lock(|s| s.set(motor_cmd));
                MOTOR_UPDATE.signal(MotorCommand::Stop);
                motors = MotorCommand::Stop;
                transmit_state(&mut serial_tx, &motors).await;
                debug!("state transmitted: Stop");
            },

            b'#' => {
                // Command: Read next CMD_LENGTH octets for "alphaL,alphaR"
                info!("Received command header, waiting for parameters");
                let mut rest = [0u8; CMD_LENGTH];
                rest[0] = header[0];

                // Lire jusqu'au CMD_LENGTH octets
                match <RingBufferedUartRx<'static> as embedded_io_async::Read>::read_exact(&mut serial_rx, &mut rest[1..]).await {
                    Ok(_) => {
                        debug!("Received command parameters: {:?}", rest);
                        // Parse "alphaL,alphaR" ex: "0312,0768"
                        if let Some((al, ar)) = parse_pattern(rest) {
                            info!("Updated state: alpha_l={}, alpha_r={}", al, ar);
                            let alpha_left: i16 = (al.clamp(0, STEPS) - ZERO) as i16;
                            let alpha_right: i16 = (ar.clamp(0, STEPS) - ZERO) as i16;
                            // let motor_cmd = MotorCmd::new(
                            //      al.clamp(0, STEPS) - ZERO,
                            //      ar.clamp(0, STEPS) - ZERO,
                            //      500);                      
                            // MOTOR_CMD.lock(|s| s.set(motor_cmd));
                            MOTOR_UPDATE.signal(MotorCommand::SetSpeed(alpha_left, alpha_right));
                            motors = MotorCommand::SetSpeed(alpha_left, alpha_right);
                            transmit_state(&mut serial_tx, &motors).await;
                            info!("state transmitted: alpha_l={}, alpha_r={}", alpha_left, alpha_right);
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


async fn transmit_state(serial: &mut UartTx<'static, Async>, motor_cmd: &MotorCommand) {
    let state = ODOM_STATE.lock(|s| s.get());
    let odom_left = &state.odom_left;
    let odom_right = &state.odom_right;

    // let motor = MOTOR_CMD.lock(|s| s.get());
    let (alpha_l, alpha_r) = match motor_cmd {
        MotorCommand::SetSpeed(alpha_left, alpha_right) => {
            (*alpha_left as i32 + ZERO, *alpha_right as i32 + ZERO)
        },
        MotorCommand::Brake | MotorCommand::Stop => {
            (ZERO, ZERO)
        },
    };

    let buf: String<64> = format!("!{:04},{:04} L:{:+},{:+} R:{:+},{:+}\r\n", 
            alpha_l, alpha_r,
			odom_left.velocity, odom_left.position,
			odom_right.velocity, odom_right.position).expect("format macro failed");
    serial.write(buf.as_bytes()).await.unwrap();
}