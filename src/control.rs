//! Régulation de vitesse d'une roue : estimation directe du PWM + correction PI.
//!
//! Le terme intégral porte sur l'écart de position (consigne intégrée moins pas codeur
//! réellement parcourus), et non sur l'intégrale de la vitesse mesurée : c'est la même
//! grandeur, mais sans le bruit de quantification de la vitesse à basse allure.
//!
//! Sans dépendance matérielle : unités physiques (rad/s) en entrée,
//! rapport cyclique normalisé dans [-1, 1] en sortie.

use core::f32::consts::PI;

use defmt::Format;

/// Gains du régulateur, réglables à chaud par la commande série `K`.
#[derive(Clone, Copy, PartialEq, Format)]
pub struct Gains {
    /// Proportionnel : rapport cyclique par rad/s d'erreur.
    pub kp: f32,
    /// Intégral : rapport cyclique par rad d'avance ou de retard sur la consigne.
    pub ki: f32,
    /// Estimation directe : rapport cyclique par rad/s de consigne
    /// (≈ 1 / vitesse à vide à 100 % de PWM).
    pub kff: f32,
    /// Rapport cyclique minimal pour vaincre le frottement au démarrage,
    /// ajouté dans le sens de la consigne.
    pub dmin: f32,
}

/// Valeurs de départ prudentes, à régler sur le robot (voir README).
pub const DEFAULT_GAINS: Gains = Gains { kp: 0.05, ki: 0.5, kff: 0.05, dmin: 0.2 };

/// En dessous de cette consigne (rad/s), la roue est considérée à l'arrêt.
const ZERO_SPEED: f32 = 0.01;

/// Pas codeur par seconde → rad/s.
pub fn ticks_to_rad(ticks_per_s: f32, ticks_per_rev: f32) -> f32 {
    ticks_per_s * 2.0 * PI / ticks_per_rev
}

/// Régulateur d'une roue.
#[derive(Clone, Copy, Default)]
pub struct WheelPi {
    /// Retard de la roue sur la consigne intégrée (rad) : positif si elle est en retard.
    position_error: f32,
    /// Position codeur au dernier appel (pas).
    last_ticks: Option<i32>,
    /// Dernière sortie, dans [-umax, umax].
    pub output: f32,
}

impl WheelPi {
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// Calcule le rapport cyclique pour suivre `target` (rad/s), toutes les `dt` secondes,
    /// connaissant la vitesse mesurée `measured` (rad/s) et la position codeur `ticks`
    /// (`rad_per_tick` rad par pas). Plafonné à ±`umax`.
    #[allow(clippy::too_many_arguments)]
    pub fn update(
        &mut self,
        g: &Gains,
        target: f32,
        measured: f32,
        ticks: i32,
        rad_per_tick: f32,
        dt: f32,
        umax: f32,
    ) -> f32 {
        if target.abs() < ZERO_SPEED {
            self.reset();
            return 0.0;
        }
        let moved = self.last_ticks.map_or(0.0, |t| ticks.wrapping_sub(t) as f32 * rad_per_tick);
        self.last_ticks = Some(ticks);

        let feedforward = g.kff * target + g.dmin * target.signum();
        let proportional = g.kp * (target - measured);
        let error = self.position_error + target * dt - moved;

        // Anti-emballement : quand la sortie sature déjà dans le sens de l'écart,
        // on ne laisse pas l'écart grandir (il peut seulement se réduire).
        let unclamped = feedforward + proportional + g.ki * error;
        if unclamped.abs() <= umax
            || unclamped.signum() != error.signum()
            || error.abs() < self.position_error.abs()
        {
            self.position_error = error;
        }
        // Après un blocage, pas de rattrapage brutal : le terme intégral reste dans ±umax.
        if g.ki > 0.0 {
            let cap = umax / g.ki;
            self.position_error = self.position_error.clamp(-cap, cap);
        }

        self.output = (feedforward + proportional + g.ki * self.position_error).clamp(-umax, umax);
        self.output
    }

    /// La sortie est en butée : la roue reçoit le maximum.
    pub fn saturated(&self, umax: f32) -> bool {
        self.output.abs() >= 0.95 * umax
    }
}
