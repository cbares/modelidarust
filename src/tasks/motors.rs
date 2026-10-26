use core::cell::Cell;

use defmt::{Format, debug, info, warn};
use embassy_futures::select::{Either4, select4};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Instant, Ticker};
use motor_driver_hal::MotorDriver;

use crate::state::{ODOM_STATE, OdomState};
use crate::tasks::odometry::TICKS_PER_REV;
use crate::tasks::watchdog::{self, Task};
use crate::{board::MotorDriverType, tasks::uart_usb::ZERO};

// --- Sécurité ---
/// Rapport cyclique maximal accepté par le driver DBH-1A (au-delà : instable, voire destructeur).
const MAX_DUTY_PERCENT: i32 = 98;
/// Période des contrôles de sécurité.
const CHECK_PERIOD: Duration = Duration::from_millis(50);
/// Sans aucune trame USB pendant ce délai, les moteurs pilotés par l'USB s'arrêtent.
const USB_TIMEOUT: Duration = Duration::from_millis(500);
/// Détection de roue bloquée par les codeurs. À désactiver pour un essai sans codeurs :
/// sinon toute consigne franche s'arrête au bout de STALL_TIME.
const STALL_DETECTION: bool = true;
/// Blocage : consigne d'au moins 30 % (sur ±ZERO)...
const STALL_MIN_CMD: i32 = 150;
/// ... et roue quasi immobile : moins de 0,1 tour/s (6 tr/min, ≈ 54 pas/s,
/// soit 5 pas par mesure d'odométrie de 100 ms)...
const STALL_MAX_SPEED: f32 = 0.1 * TICKS_PER_REV;
/// ... pendant ce délai.
const STALL_TIME: Duration = Duration::from_millis(300);
/// Après un blocage, une consigne sous ce seuil (ou Stop / Brake) réarme les moteurs.
const NEUTRAL_CMD: i32 = 20;

#[derive(Clone, Copy, PartialEq, Format)]
pub enum MotorCommand {
    SetSpeed(i16, i16), // (alpha_left, alpha_right), dans [-ZERO, ZERO]
    Stop,
    Brake,
}

/// Événements venant de la radiocommande.
#[derive(Clone, Copy, PartialEq, Format)]
pub enum RcEvent {
    /// Trame valide : position de l'interrupteur de prise de main et consigne des manches.
    Frame { override_on: bool, cmd: MotorCommand },
    /// Failsafe du récepteur, ou plus de trames.
    LinkLost,
}

/// Qui pilote les moteurs.
#[derive(Clone, Copy, PartialEq, Format)]
pub enum Owner {
    Usb,
    Rc,
    /// Liaison radio perdue pendant la prise de main : moteurs arrêtés,
    /// l'USB reste bloqué jusqu'à ce que la radio revienne interrupteur relâché.
    RcLost,
}

impl Owner {
    pub fn tag(self) -> &'static str {
        match self {
            Owner::Usb => "USB",
            Owner::Rc => "RC",
            Owner::RcLost => "LOST",
        }
    }
}

/// Défaut de sécurité actif, renvoyé au PC.
#[derive(Clone, Copy, PartialEq, Format)]
pub enum Fault {
    None,
    /// Le PC s'est tu pendant que les moteurs tournaient : arrêt.
    /// Effacé par la prochaine trame USB.
    UsbTimeout,
    /// Roue bloquée : arrêt. Effacé par une consigne neutre (voir NEUTRAL_CMD).
    Stall,
}

impl Fault {
    pub fn tag(self) -> &'static str {
        match self {
            Fault::None => "-",
            Fault::UsbTimeout => "USB_TIMEOUT",
            Fault::Stall => "STALL",
        }
    }
}

/// État publié par la tâche moteurs.
#[derive(Clone, Copy, PartialEq, Format)]
pub struct MotorStatus {
    pub owner: Owner,
    /// Dernière commande appliquée aux moteurs.
    pub applied: MotorCommand,
    pub fault: Fault,
}

/// Commandes reçues sur la liaison USB.
pub static USB_CMD: Signal<CriticalSectionRawMutex, MotorCommand> = Signal::new();
/// Trame USB sans commande moteur (requête `?`) : le PC est toujours là.
pub static USB_ALIVE: Signal<CriticalSectionRawMutex, ()> = Signal::new();
/// Événements de la radiocommande.
pub static RC_EVENT: Signal<CriticalSectionRawMutex, RcEvent> = Signal::new();
/// État courant (lu par le compte rendu série).
pub static MOTOR_STATE: Mutex<CriticalSectionRawMutex, Cell<MotorStatus>> =
    Mutex::new(Cell::new(MotorStatus {
        owner: Owner::Usb,
        applied: MotorCommand::SetSpeed(0, 0),
        fault: Fault::None,
    }));

/// Nouvel état de pilotage, et la commande moteur à appliquer s'il y en a une.
pub fn arbitrate(owner: Owner, ev: RcEvent) -> (Owner, Option<MotorCommand>) {
    match (owner, ev) {
        // Interrupteur actif : la radio prend (ou garde) la main.
        (_, RcEvent::Frame { override_on: true, cmd }) => (Owner::Rc, Some(cmd)),
        // Interrupteur relâché : on rend la main à l'USB, moteurs arrêtés.
        (Owner::Rc, RcEvent::Frame { override_on: false, .. }) => (Owner::Usb, Some(MotorCommand::Stop)),
        (Owner::RcLost, RcEvent::Frame { override_on: false, .. }) => (Owner::Usb, None),
        (Owner::Usb, RcEvent::Frame { override_on: false, .. }) => (Owner::Usb, None),
        // Radio perdue pendant la prise de main : arrêt, USB toujours bloqué.
        (Owner::Rc, RcEvent::LinkLost) => (Owner::RcLost, Some(MotorCommand::Stop)),
        (o, RcEvent::LinkLost) => (o, None),
    }
}

fn is_moving(cmd: MotorCommand) -> bool {
    matches!(cmd, MotorCommand::SetSpeed(l, r) if l != 0 || r != 0)
}

fn is_neutral(cmd: MotorCommand) -> bool {
    match cmd {
        MotorCommand::SetSpeed(l, r) => (l as i32).abs() < NEUTRAL_CMD && (r as i32).abs() < NEUTRAL_CMD,
        MotorCommand::Stop | MotorCommand::Brake => true,
    }
}

/// Une roue est commandée franchement mais ne tourne pas.
fn wheel_blocked(applied: MotorCommand, odom: &OdomState) -> bool {
    let MotorCommand::SetSpeed(l, r) = applied else { return false };
    let blocked = |alpha: i16, velocity: f32| {
        (alpha as i32).abs() >= STALL_MIN_CMD && velocity.abs() < STALL_MAX_SPEED
    };
    blocked(l, odom.odom_left.velocity) || blocked(r, odom.odom_right.velocity)
}

#[embassy_executor::task]
pub async fn motors_task(mut left: MotorDriverType, mut right: MotorDriverType, max_duty: u16) {
    left.enable().unwrap();
    right.enable().unwrap();
    // Consigne dans [-ZERO, ZERO] → rapport cyclique dans [-max_duty, max_duty],
    // plafonné à MAX_DUTY_PERCENT.
    let coef = max_duty as i32 / ZERO;
    let limit = max_duty as i32 * MAX_DUTY_PERCENT / 100;

    let mut status = MotorStatus {
        owner: Owner::Usb,
        applied: MotorCommand::SetSpeed(0, 0),
        fault: Fault::None,
    };
    let mut last_usb = Instant::now();
    let mut stall_since: Option<Instant> = None;
    let mut ticker = Ticker::every(CHECK_PERIOD);

    loop {
        // Commande demandée par l'USB ou la radio, avant filtrage de sécurité.
        let mut requested = None;
        // Arrêt imposé par un contrôle de sécurité.
        let mut safety_stop = None;

        match select4(USB_CMD.wait(), RC_EVENT.wait(), USB_ALIVE.wait(), ticker.next()).await {
            Either4::First(cmd) => {
                last_usb = Instant::now();
                clear_usb_timeout(&mut status);
                if status.owner == Owner::Usb {
                    requested = Some(cmd);
                } else {
                    debug!("Commande USB ignorée : pilotage {}", status.owner);
                }
            }
            Either4::Second(ev) => {
                let (new_owner, cmd) = arbitrate(status.owner, ev);
                if new_owner != status.owner {
                    match new_owner {
                        Owner::RcLost => warn!("Pilotage {} -> {} : radio perdue", status.owner, new_owner),
                        _ => info!("Pilotage {} -> {}", status.owner, new_owner),
                    }
                    status.owner = new_owner;
                }
                requested = cmd;
            }
            Either4::Third(()) => {
                last_usb = Instant::now();
                clear_usb_timeout(&mut status);
            }
            Either4::Fourth(()) => {
                let now = Instant::now();
                if status.owner == Owner::Usb
                    && is_moving(status.applied)
                    && now - last_usb > USB_TIMEOUT
                {
                    warn!("Plus de trame USB depuis {} ms : arrêt", (now - last_usb).as_millis());
                    safety_stop = Some(Fault::UsbTimeout);
                }
                let odom = ODOM_STATE.lock(|s| s.get());
                if STALL_DETECTION && wheel_blocked(status.applied, &odom) {
                    let since = *stall_since.get_or_insert(now);
                    if now - since >= STALL_TIME {
                        warn!("Roue bloquée : arrêt jusqu'à une consigne neutre");
                        safety_stop = Some(Fault::Stall);
                    }
                } else {
                    stall_since = None;
                }
            }
        }

        if let Some(fault) = safety_stop {
            apply(&mut left, &mut right, MotorCommand::Stop, coef, limit);
            status.applied = MotorCommand::Stop;
            status.fault = fault;
            stall_since = None;
        } else if let Some(cmd) = requested {
            if status.fault == Fault::Stall {
                if is_neutral(cmd) {
                    info!("Consigne neutre : moteurs réarmés");
                    status.fault = Fault::None;
                } else {
                    debug!("Blocage : consigne ignorée jusqu'au neutre");
                }
            }
            // La radio renvoie la même consigne à chaque trame : on n'applique que les changements.
            if status.fault != Fault::Stall && cmd != status.applied {
                apply(&mut left, &mut right, cmd, coef, limit);
                status.applied = cmd;
                stall_since = None;
            }
        }
        MOTOR_STATE.lock(|s| s.set(status));
        watchdog::feed(Task::Motors);
    }
}

/// Le PC est revenu : le défaut est effacé (les moteurs restent arrêtés
/// jusqu'à sa prochaine consigne).
fn clear_usb_timeout(status: &mut MotorStatus) {
    if status.fault == Fault::UsbTimeout {
        info!("Trame USB reçue : défaut USB_TIMEOUT effacé");
        status.fault = Fault::None;
    }
}

fn duty(alpha: i16, coef: i32, limit: i32) -> i16 {
    (alpha as i32 * coef).clamp(-limit, limit) as i16
}

fn apply(left: &mut MotorDriverType, right: &mut MotorDriverType, cmd: MotorCommand, coef: i32, limit: i32) {
    match cmd {
        MotorCommand::SetSpeed(alpha_left, alpha_right) => {
            left.enable().unwrap();
            right.enable().unwrap();
            left.set_speed(duty(alpha_left, coef, limit)).unwrap();
            right.set_speed(duty(alpha_right, coef, limit)).unwrap();
            debug!("Motor speeds: alpha_l={}, alpha_r={}", alpha_left, alpha_right);
        }
        // Roue libre : driver désactivé (EN bas), les quatre transistors sont ouverts.
        MotorCommand::Stop => {
            left.stop().unwrap();
            left.disable().unwrap();
            right.stop().unwrap();
            right.disable().unwrap();
            debug!("Motors stopped");
        }
        // Freinage : driver activé et deux entrées à 0, les deux transistors bas du pont
        // court-circuitent le moteur. On n'utilise pas `brake()` de motor-driver-hal, qui
        // met les deux entrées à 100 % : sur le DBH (IR2104), les transistors hauts ne
        // tiennent pas un rapport cyclique de 100 % et le freinage lâche.
        MotorCommand::Brake => {
            left.stop().unwrap();
            right.stop().unwrap();
            left.enable().unwrap();
            right.enable().unwrap();
            debug!("Motors braking");
        }
    }
}
