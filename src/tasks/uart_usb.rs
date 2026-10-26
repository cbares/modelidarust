use defmt::{debug, info, warn};
use embassy_stm32::usart::{RingBufferedUartRx, Uart, UartTx};
use embassy_stm32::mode::Async;
use embassy_time::{Duration, with_timeout};
use heapless::{String, format, Vec};
use static_cell::StaticCell;

use crate::control::Gains;
use crate::state::ODOM_STATE;
use crate::tasks::motors::{Fault, GAINS, MAX_SPEED, MOTOR_STATE, MotorCommand, Owner, USB_ALIVE, USB_CMD};

/// Longueur maximale des commandes à ligne (`V`, `K`), terminées par '\n' ou '\r'.
const LINE_MAX: usize = 48;
/// Délai maximal pour recevoir la suite d'une commande (`#`, `V`, `K`).
const LINE_TIMEOUT: Duration = Duration::from_millis(50);

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
                // Lire les CMD_LENGTH - 1 octets restants : "AAAA,BBBB". Une trame
                // incomplète est abandonnée au bout de LINE_TIMEOUT, au lieu de bloquer
                // la liaison en avalant les trames suivantes.
                let mut rest = [0u8; CMD_LENGTH];
                rest[0] = header[0];
                let read = <RingBufferedUartRx<'static> as embedded_io_async::Read>::read_exact(&mut serial_rx, &mut rest[1..]);
                match with_timeout(LINE_TIMEOUT, read).await {
                    Ok(Ok(_)) => match parse_pattern(rest) {
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
                    Ok(Err(_)) => {
                        warn!("Failed to read command parameters");
                        continue;
                    }
                    Err(_) => {
                        warn!("Incomplete speed command: dropped after {} ms", LINE_TIMEOUT.as_millis());
                        continue;
                    }
                }
            }

            // Consigne de vitesse : "V<gauche>,<droite>\n" en mrad/s, ex. "V3142,-1571".
            b'V' => {
                let mut line = [0u8; LINE_MAX];
                match read_line(&mut serial_rx, &mut line).await.and_then(parse_velocity) {
                    Some((l, r)) => {
                        debug!("Received velocity command: left={} rad/s, right={} rad/s", l, r);
                        Some(MotorCommand::SetVelocity(l, r))
                    }
                    None => {
                        warn!("Invalid velocity command");
                        continue;
                    }
                }
            }

            // Gains : "K\n" pour les lire, "K<kp>,<ki>,<kff>,<dmin>\n" pour les changer.
            b'K' => {
                let mut line = [0u8; LINE_MAX];
                let Some(text) = read_line(&mut serial_rx, &mut line).await else {
                    warn!("Invalid gains command");
                    continue;
                };
                if !text.is_empty() {
                    match parse_gains(text) {
                        Some(g) => {
                            GAINS.lock(|c| c.set(g));
                            info!("New gains: {}", g);
                        }
                        None => {
                            warn!("Invalid gains: {=str}", text);
                            continue;
                        }
                    }
                }
                transmit_gains(&mut serial_tx).await;
                None
            }

            // Ignore unrecognized commands
            _ => continue,
        };

        match cmd {
            // Ignorée par la tâche moteurs si la radiocommande a la main ou si une roue est bloquée.
            Some(cmd) => USB_CMD.signal(cmd),
            // Une requête seule prouve aussi que le PC est là (voir USB_TIMEOUT).
            None => USB_ALIVE.signal(()),
        }
        transmit_state(&mut serial_tx, cmd).await;
    }
}


/// Parse "#AAAA,BBBB" (chiffres uniquement) → Some((a, b))
fn parse_pattern(buf: [u8; CMD_LENGTH]) -> Option<(i32, i32)> {
    let digits = |s: &[u8]| s.iter().all(u8::is_ascii_digit);
    if buf[5] != b',' || !digits(&buf[1..5]) || !digits(&buf[6..10]) {
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


/// Lit la suite d'une commande jusqu'à '\n' ou '\r'. `None` si trop longue,
/// interrompue (plus de LINE_TIMEOUT entre deux octets) ou invalide.
async fn read_line<'a>(
    serial_rx: &mut RingBufferedUartRx<'static>,
    buf: &'a mut [u8; LINE_MAX],
) -> Option<&'a str> {
    let mut n = 0;
    loop {
        let mut byte = [0u8; 1];
        match with_timeout(LINE_TIMEOUT, serial_rx.read(&mut byte)).await {
            Ok(Ok(_)) => {}
            _ => return None,
        }
        if byte[0] == b'\n' || byte[0] == b'\r' {
            break;
        }
        if n == LINE_MAX {
            return None;
        }
        buf[n] = byte[0];
        n += 1;
    }
    core::str::from_utf8(&buf[..n]).ok()
}

/// "3142,-1571" (mrad/s) → (3.142, -1.571) rad/s, bornés à ±MAX_SPEED.
fn parse_velocity(text: &str) -> Option<(f32, f32)> {
    let (l, r) = text.split_once(',')?;
    let to_rad = |s: &str| s.trim().parse::<i32>().ok().map(|v| (v as f32 / 1000.0).clamp(-MAX_SPEED, MAX_SPEED));
    Some((to_rad(l)?, to_rad(r)?))
}

/// "kp,ki,kff,dmin" → gains, refusés s'ils sont négatifs ou absurdes.
fn parse_gains(text: &str) -> Option<Gains> {
    let mut values = [0.0f32; 4];
    let mut fields = text.split(',');
    for v in values.iter_mut() {
        *v = fields.next()?.trim().parse().ok()?;
        if !v.is_finite() || *v < 0.0 || *v > 100.0 {
            return None;
        }
    }
    if fields.next().is_some() || values[3] > 0.9 {
        return None;
    }
    Some(Gains { kp: values[0], ki: values[1], kff: values[2], dmin: values[3] })
}

/// Envoie `K:kp,ki,kff,dmin`.
async fn transmit_gains(serial: &mut UartTx<'static, Async>) {
    let g = GAINS.lock(|c| c.get());
    let buf: String<80> = format!("K:{},{},{},{}\r\n", g.kp, g.ki, g.kff, g.dmin).expect("format macro failed");
    serial.write(buf.as_bytes()).await.unwrap();
}

/// Envoie `!AAAA,BBBB L:vit,pos R:vit,pos C:<pilote> F:<défaut>`.
/// La consigne affichée est celle réellement appliquée aux moteurs : si la radio
/// a la main, une commande USB n'y apparaît donc pas. En mode vitesse, c'est le
/// rapport cyclique choisi par la régulation.
async fn transmit_state(serial: &mut UartTx<'static, Async>, sent: Option<MotorCommand>) {
    let state = ODOM_STATE.lock(|s| s.get());
    let odom_left = &state.odom_left;
    let odom_right = &state.odom_right;

    let status = MOTOR_STATE.lock(|s| s.get());
    // La tâche moteurs n'a peut-être pas encore traité la commande qu'on vient d'envoyer.
    let shown = match sent {
        Some(cmd) if status.owner == Owner::Usb && status.fault != Fault::Stall => cmd,
        _ => status.applied,
    };
    let (al, ar) = match shown {
        MotorCommand::SetSpeed(l, r) => (l, r),
        MotorCommand::SetVelocity(..) => status.output,
        MotorCommand::Brake | MotorCommand::Stop => (0, 0),
    };
    let (alpha_l, alpha_r) = (al as i32 + ZERO, ar as i32 + ZERO);

    let buf: String<112> = format!("!{:04},{:04} L:{:+},{:+} R:{:+},{:+} C:{} F:{}\r\n",
            alpha_l, alpha_r,
            odom_left.velocity, odom_left.position,
            odom_right.velocity, odom_right.position,
            status.owner.tag(), status.fault.tag()).expect("format macro failed");
    serial.write(buf.as_bytes()).await.unwrap();
    debug!("state transmitted");
}
