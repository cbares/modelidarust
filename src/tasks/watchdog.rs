//! Watchdog matériel (IWDG) qui surveille les tâches, pas seulement l'exécuteur.
//!
//! Chaque tâche qui doit tourner en permanence appelle `feed()` à chaque tour de
//! boucle. Toutes les `WINDOW`, la tâche watchdog vérifie que toutes se sont
//! signalées : si oui elle rafraîchit l'IWDG, sinon elle le laisse expirer et la
//! carte redémarre. Une tâche bloquée (attente infinie, interblocage, DMA figé)
//! provoque donc un reset, comme un exécuteur figé.
//!
//! La tâche USB n'est pas surveillée : elle attend légitimement sans fin quand le
//! PC se tait (ce cas est géré par l'arrêt USB_TIMEOUT de la tâche moteurs).

use core::sync::atomic::{AtomicU32, Ordering};

use defmt::{Format, warn};
use embassy_stm32::Peri;
use embassy_stm32::peripherals::IWDG;
use embassy_stm32::wdg::IndependentWatchdog;
use embassy_time::{Duration, Ticker};

/// Fenêtre de contrôle : chaque tâche surveillée doit s'être signalée dans ce délai.
/// Les tâches tournent toutes les 50 à 100 ms.
const WINDOW: Duration = Duration::from_millis(300);
/// Délai de l'IWDG : sans rafraîchissement, reset. Tolère une fenêtre ratée.
const TIMEOUT_US: u32 = 1_000_000;

/// Tâches surveillées.
#[derive(Clone, Copy)]
#[repr(u32)]
pub enum Task {
    Motors = 1 << 0,
    Odometry = 1 << 1,
    Sbus = 1 << 2,
}

const ALL: u32 = Task::Motors as u32 | Task::Odometry as u32 | Task::Sbus as u32;
const NAMES: [(Task, &str); 3] = [
    (Task::Motors, "moteurs"),
    (Task::Odometry, "odométrie"),
    (Task::Sbus, "sbus"),
];

static CHECKINS: AtomicU32 = AtomicU32::new(0);

/// À appeler par une tâche surveillée à chaque tour de boucle.
pub fn feed(task: Task) {
    CHECKINS.fetch_or(task as u32, Ordering::Relaxed);
}

/// Gèle l'IWDG quand le cœur est arrêté par un débogueur (point d'arrêt, pas à pas) :
/// sinon la carte redémarre au bout de TIMEOUT_US. Sans débogueur, aucun effet.
fn freeze_when_debugging() {
    use embassy_stm32::pac::DBGMCU;
    #[cfg(feature = "stm32l476rg")]
    DBGMCU.apb1fzr1().modify(|w| w.set_iwdg(true));
    #[cfg(not(feature = "stm32l476rg"))]
    DBGMCU.apb1fzr().modify(|w| w.set_iwdg(true));
}

#[embassy_executor::task]
pub async fn watchdog_task(wdg: Peri<'static, IWDG>) {
    freeze_when_debugging();
    let mut iwdg = IndependentWatchdog::new(wdg, TIMEOUT_US);
    iwdg.unleash();
    let mut ticker = Ticker::every(WINDOW);

    loop {
        ticker.next().await;
        let seen = CHECKINS.swap(0, Ordering::Relaxed);
        if seen == ALL {
            iwdg.pet();
        } else {
            for (task, name) in NAMES {
                if seen & task as u32 == 0 {
                    warn!("Watchdog : la tâche {} ne répond plus", name);
                }
            }
        }
    }
}

/// Cause du dernier redémarrage.
#[derive(Clone, Copy, PartialEq, Format)]
pub enum ResetCause {
    /// Watchdog indépendant : une tâche (ou l'exécuteur) était bloquée.
    Watchdog,
    /// Watchdog fenêtré.
    WindowWatchdog,
    /// Reset logiciel (ex. après un flash par probe-rs).
    Software,
    /// Mise sous tension ou chute d'alimentation.
    PowerOn,
    /// Bouton reset ou broche NRST.
    Pin,
    /// Autre ou inconnue.
    Other,
}

impl ResetCause {
    pub fn tag(self) -> &'static str {
        match self {
            ResetCause::Watchdog => "watchdog",
            ResetCause::WindowWatchdog => "window-watchdog",
            ResetCause::Software => "software",
            ResetCause::PowerOn => "power-on",
            ResetCause::Pin => "pin",
            ResetCause::Other => "other",
        }
    }
}

/// Lit la cause du dernier reset dans RCC_CSR, puis efface les drapeaux
/// pour que le prochain reset soit lu correctement.
pub fn reset_cause() -> ResetCause {
    use embassy_stm32::pac::RCC;
    let csr = RCC.csr().read();

    #[cfg(feature = "stm32l476rg")]
    let (iwdg, pin, power) = (csr.iwdgrstf(), csr.pinrstf(), csr.borrstf());
    #[cfg(not(feature = "stm32l476rg"))]
    let (iwdg, pin, power) = (csr.wdgrstf(), csr.padrstf(), csr.porrstf() || csr.borrstf());

    // Le reset broche est aussi levé par les autres causes : on le teste en dernier.
    let cause = if iwdg {
        ResetCause::Watchdog
    } else if csr.wwdgrstf() {
        ResetCause::WindowWatchdog
    } else if csr.sftrstf() {
        ResetCause::Software
    } else if power {
        ResetCause::PowerOn
    } else if pin {
        ResetCause::Pin
    } else {
        ResetCause::Other
    };

    RCC.csr().modify(|w| w.set_rmvf(true));
    cause
}
