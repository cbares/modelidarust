use core::cell::Cell;

use defmt::{Format, debug, info, warn};
use embassy_futures::select::{Either, select};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use motor_driver_hal::MotorDriver;

use crate::{board::MotorDriverType, tasks::uart_usb::ZERO};

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

/// Commandes reçues sur la liaison USB.
pub static USB_CMD: Signal<CriticalSectionRawMutex, MotorCommand> = Signal::new();
/// Événements de la radiocommande.
pub static RC_EVENT: Signal<CriticalSectionRawMutex, RcEvent> = Signal::new();
/// Pilote actuel et dernière commande appliquée (lu par le compte rendu série).
pub static MOTOR_STATE: Mutex<CriticalSectionRawMutex, Cell<(Owner, MotorCommand)>> =
    Mutex::new(Cell::new((Owner::Usb, MotorCommand::SetSpeed(0, 0))));

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

#[embassy_executor::task]
pub async fn motors_task(mut left: MotorDriverType, mut right: MotorDriverType, max_duty: u16) {
    left.enable().unwrap();
    right.enable().unwrap();
    // Consigne dans [-ZERO, ZERO] → rapport cyclique dans [-max_duty, max_duty].
    let coef = max_duty as i32 / ZERO;

    let mut owner = Owner::Usb;
    let mut applied = MotorCommand::SetSpeed(0, 0);

    loop {
        let cmd = match select(USB_CMD.wait(), RC_EVENT.wait()).await {
            Either::First(cmd) if owner == Owner::Usb => Some(cmd),
            Either::First(_) => {
                debug!("Commande USB ignorée : pilotage {}", owner);
                None
            }
            Either::Second(ev) => {
                let (new_owner, cmd) = arbitrate(owner, ev);
                if new_owner != owner {
                    match new_owner {
                        Owner::RcLost => warn!("Pilotage {} -> {} : radio perdue", owner, new_owner),
                        _ => info!("Pilotage {} -> {}", owner, new_owner),
                    }
                    owner = new_owner;
                }
                cmd
            }
        };

        // La radio renvoie la même consigne à chaque trame : on n'applique que les changements.
        if let Some(cmd) = cmd
            && cmd != applied
        {
            apply(&mut left, &mut right, cmd, coef);
            applied = cmd;
        }
        MOTOR_STATE.lock(|s| s.set((owner, applied)));
    }
}

fn apply(left: &mut MotorDriverType, right: &mut MotorDriverType, cmd: MotorCommand, coef: i32) {
    match cmd {
        MotorCommand::SetSpeed(alpha_left, alpha_right) => {
            left.enable().unwrap();
            right.enable().unwrap();
            left.set_speed((alpha_left as i32 * coef) as i16).unwrap();
            right.set_speed((alpha_right as i32 * coef) as i16).unwrap();
            debug!("Motor speeds: alpha_l={}, alpha_r={}", alpha_left, alpha_right);
        }
        MotorCommand::Stop => {
            left.stop().unwrap();
            left.disable().unwrap();
            right.stop().unwrap();
            right.disable().unwrap();
            debug!("Motors stopped");
        }
        MotorCommand::Brake => {
            left.enable().unwrap();
            right.enable().unwrap();
            left.brake().unwrap();
            right.brake().unwrap();
            debug!("Motors braking");
        }
    }
}
