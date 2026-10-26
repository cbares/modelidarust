use defmt::{debug, info};
use embassy_stm32::usart::{RingBufferedUartRx, Uart, UartTx};
use embassy_stm32::mode::Async;
use heapless::{String, format, Vec};
use static_cell::StaticCell;

use crate::state::ODOM_STATE;
use crate::tasks::motors::{MOTOR_STATE, MotorCommand, Owner, USB_CMD};

const CMD_LENGTH: usize = 10; // "#AAAA,BBBB"
const STEPS: i32 = 1000;
pub const ZERO: i32 = STEPS / 2;

static USB_RX_BUF: StaticCell<[u8; 128]> = StaticCell::new();


#[embassy_executor::task]
pub async fn uart_task(
    serial: Uart<'static, Async>,
) {
    let rx_buf = USB_RX_BUF.init([0; 128]);
    let (mut serial_tx, serial_rx) = serial.split();
    let mut serial_rx = serial_rx.into_ring_buffered(rx_buf);

    loop {
        // --- Etape 1 : attendre 1 octet (détection de commande) ---
        let mut header = [0u8; 1];
        if serial_rx.read(&mut header).await.is_err() {
            continue;
        }

        // Commande à transmettre aux moteurs (None pour une simple requête).
        let cmd = match header[0] {
            b'?' => {
                info!("Received query command");
                None
            }

            b'*' => {
                info!("Received brake command");
                Some(MotorCommand::Brake)
            }

            b'/' => {
                info!("Received stop command");
                Some(MotorCommand::Stop)
            }

            b'#' => {
                // Lire les CMD_LENGTH - 1 octets restants : "AAAA,BBBB"
                let mut rest = [0u8; CMD_LENGTH];
                rest[0] = header[0];
                match <RingBufferedUartRx<'static> as embedded_io_async::Read>::read_exact(&mut serial_rx, &mut rest[1..]).await {
                    Ok(_) => match parse_pattern(rest) {
                        Some((al, ar)) => {
                            let alpha_left = (al.clamp(0, STEPS) - ZERO) as i16;
                            let alpha_right = (ar.clamp(0, STEPS) - ZERO) as i16;
                            info!("Received speed command: alpha_l={}, alpha_r={}", alpha_left, alpha_right);
                            Some(MotorCommand::SetSpeed(alpha_left, alpha_right))
                        }
                        None => {
                            info!("Failed to parse command parameters: {:?}", rest);
                            continue;
                        }
                    },
                    Err(_) => {
                        info!("Failed to read command parameters");
                        continue;
                    }
                }
            }

            // Ignore unrecognized commands
            _ => continue,
        };

        if let Some(cmd) = cmd {
            // Ignorée par la tâche moteurs si la radiocommande a la main.
            USB_CMD.signal(cmd);
        }
        transmit_state(&mut serial_tx, cmd).await;
    }
}


/// Parse "#AAAA,BBBB" → Some((a, b))
fn parse_pattern(buf: [u8; CMD_LENGTH]) -> Option<(i32, i32)> {
    if buf[5] != b',' {
        return None;
    }
    let p1 = String::<4>::from_utf8(
        Vec::<u8, 4>::from_slice(&buf[1..5]).unwrap()).ok()?;
    let p2 = String::<4>::from_utf8(
        Vec::<u8, 4>::from_slice(&buf[6..10]).unwrap()).ok()?;
    let a = p1.parse::<i32>().ok()?;
    let b = p2.parse::<i32>().ok()?;
    Some((a, b))
}


/// Envoie `!AAAA,BBBB L:vit,pos R:vit,pos C:<pilote>`.
/// La consigne affichée est celle réellement appliquée aux moteurs : si la radio
/// a la main, une commande USB n'y apparaît donc pas.
async fn transmit_state(serial: &mut UartTx<'static, Async>, sent: Option<MotorCommand>) {
    let state = ODOM_STATE.lock(|s| s.get());
    let odom_left = &state.odom_left;
    let odom_right = &state.odom_right;

    let (owner, applied) = MOTOR_STATE.lock(|s| s.get());
    // La tâche moteurs n'a peut-être pas encore traité la commande qu'on vient d'envoyer.
    let shown = match sent {
        Some(cmd) if owner == Owner::Usb => cmd,
        _ => applied,
    };
    let (alpha_l, alpha_r) = match shown {
        MotorCommand::SetSpeed(alpha_left, alpha_right) => {
            (alpha_left as i32 + ZERO, alpha_right as i32 + ZERO)
        },
        MotorCommand::Brake | MotorCommand::Stop => {
            (ZERO, ZERO)
        },
    };

    let buf: String<96> = format!("!{:04},{:04} L:{:+},{:+} R:{:+},{:+} C:{}\r\n",
            alpha_l, alpha_r,
            odom_left.velocity, odom_left.position,
            odom_right.velocity, odom_right.position,
            owner.tag()).expect("format macro failed");
    serial.write(buf.as_bytes()).await.unwrap();
    debug!("state transmitted");
}
