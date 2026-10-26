use embassy_stm32::timer::{GeneralInstance4Channel, qei::Qei};
use embassy_time::{Ticker, Duration};

use crate::board::{QeiLeft, QeiRight};
use crate::state::{ODOM_STATE, OdomState, Odom};

/// Les timers des encodeurs sont fixés par la carte (voir `board::QeiLeft` / `QeiRight`).
#[embassy_executor::task]
pub async fn odometry_task(interval: Duration, qei_left: QeiLeft, qei_right: QeiRight) {
    let mut ticker = Ticker::every(interval);
    let mut enc_left = Encoder::new(qei_left, interval);
    let mut enc_right = Encoder::new(qei_right, interval);

    loop {
        // Lit les encodeurs et publie l'odométrie
        let odom = OdomState {
            odom_left:  enc_left.read(),
            odom_right: enc_right.read(),
        };
        ODOM_STATE.lock(|s| s.set(odom));
        ticker.next().await;
    }
}

pub struct Encoder<T: GeneralInstance4Channel> {
    qei:  Qei<'static, T>,
    /// Dernière valeur brute du compteur matériel (16 bits).
    previous: u16,
    /// Position cumulée, sans débordement à 65535.
    position: i32,
    interval: Duration,
}

impl<T: GeneralInstance4Channel> Encoder<T> {
    pub fn new(qei: Qei<'static, T>, interval: Duration) -> Self {
        let previous = qei.count();
        Self { qei, previous, position: 0, interval }
    }

    pub fn read(&mut self) -> Odom {
        let count = self.qei.count();
        // Différence en arithmétique modulo 2^16 : correcte au passage 65535 <-> 0,
        // tant que l'encodeur fait moins de 32768 pas entre deux lectures.
        let delta = count.wrapping_sub(self.previous) as i16 as i32;
        self.previous = count;
        self.position = self.position.wrapping_add(delta);
        let velocity = delta as f32 / self.interval.as_micros() as f32 * 1_000_000.0;
        Odom { position: self.position, velocity }
    }
}
