use embassy_stm32::timer::{GeneralInstance4Channel, qei::Qei};
use embassy_time::{Ticker, Duration};

use crate::state::{ODOM_STATE, OdomState, Odom};

macro_rules! make_odometry_task {
    ($tim_l:ty, $tim_r:ty) => {
        #[embassy_executor::task]
        pub async fn odometry_task(interval: Duration, qei_left: Qei<'static,$tim_l>, qei_right: Qei<'static,$tim_r>) {

            let mut ticker = Ticker::every(interval);
            let mut enc_left = Encoder::new(qei_left, interval);
            let mut enc_right = Encoder::new(qei_right, interval );

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
    };
}

pub struct Encoder<T: GeneralInstance4Channel> {
    qei:  Qei<'static, T>,
    previous: i32,
    interval: Duration,
}

impl<T: GeneralInstance4Channel> Encoder<T> {
    pub fn new(qei: Qei<'static, T>, interval: Duration) -> Self {
        let previous = qei.count() as i32;
        Self { qei, previous, interval }
    }

    pub fn read(&mut self) -> Odom {
        let position = self.qei.count() as i32;
        let velocity = (position - self.previous) as f32 / self.interval.as_micros() as f32 * 1_000_000.0;
        self.previous = position;
        Odom { position, velocity }
    }
}

#[cfg(feature = "qei-tim1")]
use embassy_stm32::peripherals::TIM1;
#[cfg(feature = "qei-tim2")]
use embassy_stm32::peripherals::TIM2;
#[cfg(feature = "qei-tim3")]
use embassy_stm32::peripherals::TIM3;
#[cfg(feature = "qei-tim4")]
use embassy_stm32::peripherals::TIM4;
#[cfg(feature = "qei-tim5")]
use embassy_stm32::peripherals::TIM5;
#[cfg(feature = "qei-tim9")]
use embassy_stm32::peripherals::TIM9;


cfg_select! {
    feature = "qei-tim1" => cfg_select!{
        feature = "qei-tim2" => {make_odometry_task!(TIM1, TIM2);},
        feature = "qei-tim3" => {make_odometry_task!(TIM1, TIM3);},
        feature = "qei-tim4" => {make_odometry_task!(TIM1, TIM4);},
        feature = "qei-tim5" => {make_odometry_task!(TIM1, TIM5);},
        feature = "qei-tim9" => {make_odometry_task!(TIM1, TIM9);},
    },
    feature = "qei-tim2" => cfg_select!{
        feature = "qei-tim3" => {make_odometry_task!(TIM2, TIM3);},
        feature = "qei-tim4" => {make_odometry_task!(TIM2, TIM4);},
        feature = "qei-tim5" => {make_odometry_task!(TIM2, TIM5);},
        feature = "qei-tim9" => {make_odometry_task!(TIM2, TIM9);},
    },
    feature = "qei-tim3" => cfg_select!{
        feature = "qei-tim4" => {make_odometry_task!(TIM3, TIM4);},
        feature = "qei-tim5" => {make_odometry_task!(TIM3, TIM5);},
        feature = "qei-tim9" => {make_odometry_task!(TIM3, TIM9);},
    },
    feature = "qei-tim4" => cfg_select!{
        feature = "qei-tim5" => {make_odometry_task!(TIM4, TIM5);},
        feature = "qei-tim9" => {make_odometry_task!(TIM4, TIM9);},
    },
    feature = "qei-tim5" => cfg_select!{
        feature = "qei-tim9" => {make_odometry_task!(TIM5, TIM9);},
    },
}
