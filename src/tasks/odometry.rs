use embassy_stm32::timer::{GeneralInstance4Channel, qei::Qei};
use embassy_time::{Ticker, Duration};

use crate::board::{QeiLeft, QeiRight};
use crate::state::{ODOM_STATE, OdomState, Odom};
use crate::tasks::watchdog::{self, Task};

/// Pas codeur par tour de roue, tels que comptés par le timer en mode QEI.
pub const TICKS_PER_REV: f32 = 544.0;

/// Période de lecture des codeurs (100 Hz, cadence de la régulation de vitesse).
pub const ODOM_PERIOD: Duration = Duration::from_millis(10);

/// La vitesse est calculée sur au moins MIN_WINDOW lectures (50 ms). À basse vitesse,
/// la fenêtre s'allonge jusqu'à contenir MIN_TICKS pas, sans dépasser HISTORY - 1
/// lectures (200 ms) : à 0,1 tr/s, 50 ms ne contiennent que 2 ou 3 pas et la mesure
/// sauterait de ±20 %.
const MIN_WINDOW: usize = 5;
const MIN_TICKS: i32 = 3;
const HISTORY: usize = 21;

/// Les timers des encodeurs sont fixés par la carte (voir `board::QeiLeft` / `QeiRight`).
#[embassy_executor::task]
pub async fn odometry_task(qei_left: QeiLeft, qei_right: QeiRight) {
    let mut ticker = Ticker::every(ODOM_PERIOD);
    let mut enc_left = Encoder::new(qei_left);
    let mut enc_right = Encoder::new(qei_right);

    loop {
        // Lit les encodeurs et publie l'odométrie
        let odom = OdomState {
            odom_left:  enc_left.read(),
            odom_right: enc_right.read(),
        };
        ODOM_STATE.lock(|s| s.set(odom));
        watchdog::feed(Task::Odometry);
        ticker.next().await;
    }
}

pub struct Encoder<T: GeneralInstance4Channel> {
    qei:  Qei<'static, T>,
    /// Dernière valeur brute du compteur matériel (16 bits).
    previous: u16,
    /// Position cumulée, sans débordement à 65535.
    position: i32,
    /// Positions des HISTORY dernières lectures (tampon circulaire).
    history: [i32; HISTORY],
    /// Prochaine case à écrire.
    index: usize,
    /// Nombre de lectures disponibles (au démarrage, moins que HISTORY).
    filled: usize,
}

impl<T: GeneralInstance4Channel> Encoder<T> {
    pub fn new(qei: Qei<'static, T>) -> Self {
        let previous = qei.count();
        Self { qei, previous, position: 0, history: [0; HISTORY], index: 0, filled: 0 }
    }

    pub fn read(&mut self) -> Odom {
        let count = self.qei.count();
        // Différence en arithmétique modulo 2^16 : correcte au passage 65535 <-> 0,
        // tant que l'encodeur fait moins de 32768 pas entre deux lectures.
        let delta = count.wrapping_sub(self.previous) as i16 as i32;
        self.previous = count;
        self.position = self.position.wrapping_add(delta);

        self.history[self.index] = self.position;
        self.index = (self.index + 1) % HISTORY;
        self.filled = (self.filled + 1).min(HISTORY);

        // Position d'il y a k lectures.
        let ago = |k: usize| self.history[(self.index + HISTORY - 1 - k) % HISTORY];
        let max_k = self.filled - 1;
        let mut k = MIN_WINDOW.min(max_k);
        while k < max_k && self.position.wrapping_sub(ago(k)).abs() < MIN_TICKS {
            k += 1;
        }
        let velocity = if k == 0 {
            0.0
        } else {
            let span_s = (k as u64 * ODOM_PERIOD.as_micros()) as f32 / 1_000_000.0;
            self.position.wrapping_sub(ago(k)) as f32 / span_s
        };

        Odom { position: self.position, velocity }
    }
}
