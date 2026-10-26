//! Récepteur de radiocommande SBUS (100 000 bauds, 8E2, signal inversé).
//!
//! Le STM32F4 ne sait pas inverser le RX : voir `board::sbus_config`.
//!
//! Chaque trame valide est envoyée à la tâche moteurs, qui décide qui pilote
//! (voir `motors::arbitrate`).

use defmt::{Format, debug, info, warn};
use embassy_stm32::mode::Async;
use embassy_stm32::usart::UartRx;
use embassy_time::{Duration, Instant, with_timeout};
use sbus_rs::{SBUS_FRAME_LENGTH, SBUS_HEADER, SbusPacket};
use static_cell::StaticCell;

use crate::tasks::motors::{MotorCommand, RC_EVENT, RcEvent};
use crate::tasks::uart_usb::ZERO;
use crate::tasks::watchdog::{self, Task};

// --- Affectation des voies (index 0 = CH1), à adapter à l'émetteur ---
/// CH1 : manche droit horizontal (direction).
const CH_STEER: usize = 0;
/// CH2 : manche droit vertical (avance / recul).
const CH_THROTTLE: usize = 1;
/// CH5 : interrupteur de prise de main, actif en position haute.
const CH_OVERRIDE: usize = 4;

// --- Plage des voies SBUS (valeurs FrSky / OpenTX) ---
const SBUS_MIN: i32 = 172;
const SBUS_MID: i32 = 992;
const SBUS_MAX: i32 = 1811;
/// Zone morte autour du neutre, en unités de consigne (sur ±ZERO).
const DEADBAND: i32 = 20;

/// Sans trame valide pendant ce délai, la liaison est considérée perdue.
const LINK_TIMEOUT: Duration = Duration::from_millis(100);

/// Position d'un interrupteur 2 ou 3 positions.
#[derive(Clone, Copy, PartialEq, Format)]
pub enum TriState {
    Low,
    Mid,
    High,
}

impl TriState {
    pub fn from_channel(value: u16) -> Self {
        match value {
            0..=599 => TriState::Low,
            600..=1399 => TriState::Mid,
            _ => TriState::High,
        }
    }
}

/// Reconstitue les trames SBUS à partir du flux d'octets.
///
/// Un 0x0F peut apparaître dans les données et faire décoder une fausse trame
/// (c'est le cas du `StreamingParser` de sbus-rs). Une trame candidate n'est donc
/// acceptée qu'une fois la synchro confirmée par l'en-tête de la trame suivante.
pub struct FrameSync {
    buf: [u8; SBUS_FRAME_LENGTH],
    len: usize,
    /// Synchro confirmée : chaque trame valide est émise directement.
    locked: bool,
    /// Trame candidate complète, en attente de l'octet suivant pour confirmer.
    confirming: bool,
}

impl Default for FrameSync {
    fn default() -> Self {
        Self::new()
    }
}

impl FrameSync {
    pub const fn new() -> Self {
        Self { buf: [0; SBUS_FRAME_LENGTH], len: 0, locked: false, confirming: false }
    }

    pub fn push(&mut self, b: u8) -> Option<SbusPacket> {
        if self.confirming {
            self.confirming = false;
            if b == SBUS_HEADER {
                // La trame suivante commence bien ici : synchro trouvée.
                self.locked = true;
                self.buf[0] = b;
                self.len = 1;
                return None;
            }
            // Fausse synchro : on réanalyse la fenêtre, puis on traite `b` normalement.
            self.resync();
        }

        if self.len == 0 && b != SBUS_HEADER {
            self.locked = false;
            return None;
        }
        self.buf[self.len] = b;
        self.len += 1;
        if self.len < SBUS_FRAME_LENGTH {
            return None;
        }
        // Décodage des voies et drapeaux par sbus-rs (vérifie aussi en-tête et fin).
        match SbusPacket::from_array(&self.buf) {
            Ok(packet) if self.locked => {
                self.len = 0;
                Some(packet)
            }
            Ok(_) => {
                self.confirming = true;
                None
            }
            Err(_) => {
                self.locked = false;
                self.resync();
                None
            }
        }
    }

    /// Repart du prochain 0x0F présent dans la fenêtre (hors premier octet).
    fn resync(&mut self) {
        match self.buf[1..self.len].iter().position(|&x| x == SBUS_HEADER) {
            Some(i) => {
                let start = i + 1;
                self.buf.copy_within(start..self.len, 0);
                self.len -= start;
            }
            None => self.len = 0,
        }
    }
}

/// Valeur brute d'une voie → consigne dans [-ZERO, ZERO], avec zone morte.
fn normalize(raw: u16) -> i32 {
    let v = (raw as i32 - SBUS_MID) * ZERO / ((SBUS_MAX - SBUS_MIN) / 2);
    let v = v.clamp(-ZERO, ZERO);
    if v.abs() < DEADBAND { 0 } else { v }
}

/// Mixage « arcade » : un manche pour l'avance, un pour la direction.
fn to_command(ch: &[u16; 16]) -> MotorCommand {
    let throttle = normalize(ch[CH_THROTTLE]);
    let steer = normalize(ch[CH_STEER]);
    let left = (throttle + steer).clamp(-ZERO, ZERO);
    let right = (throttle - steer).clamp(-ZERO, ZERO);
    MotorCommand::SetSpeed(left as i16, right as i16)
}

static SBUS_DMA_BUF: StaticCell<[u8; 128]> = StaticCell::new();

#[embassy_executor::task]
pub async fn sbus_task(rx: UartRx<'static, Async>) {
    let mut rx = rx.into_ring_buffered(SBUS_DMA_BUF.init([0; 128]));
    let mut sync = FrameSync::new();
    let mut chunk = [0u8; 32];
    let mut link_ok = false;
    let mut last_valid = Instant::now();
    let mut frames_lost: u32 = 0;

    loop {
        // La lecture expire au bout de LINK_TIMEOUT : la boucle tourne même sans radio.
        watchdog::feed(Task::Sbus);
        match with_timeout(LINK_TIMEOUT, rx.read(&mut chunk)).await {
            Ok(Ok(n)) => {
                for &b in &chunk[..n] {
                    let Some(packet) = sync.push(b) else { continue };
                    if packet.flags.failsafe {
                        if link_ok {
                            warn!("SBUS: failsafe du récepteur ({} trames perdues)", frames_lost);
                            link_ok = false;
                            RC_EVENT.signal(RcEvent::LinkLost);
                        }
                        continue;
                    }
                    if packet.flags.frame_lost {
                        frames_lost = frames_lost.wrapping_add(1);
                        continue;
                    }
                    if !link_ok {
                        info!("SBUS: liaison radio établie");
                        debug!("SBUS: {}", packet.channels);
                        link_ok = true;
                    }
                    last_valid = Instant::now();
                    RC_EVENT.signal(RcEvent::Frame {
                        override_on: TriState::from_channel(packet.channels[CH_OVERRIDE]) == TriState::High,
                        cmd: to_command(&packet.channels),
                    });
                }
            }
            // Overrun, parité... : on attend simplement la trame suivante.
            Ok(Err(e)) => debug!("SBUS: erreur UART {:?}", e),
            Err(_) => {} // rien reçu pendant LINK_TIMEOUT
        }

        if link_ok && last_valid.elapsed() > LINK_TIMEOUT {
            warn!("SBUS: plus de trames valides");
            link_ok = false;
            RC_EVENT.signal(RcEvent::LinkLost);
        }
    }
}
