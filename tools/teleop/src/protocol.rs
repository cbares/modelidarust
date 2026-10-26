//! Protocole série du firmware (src/tasks/uart_usb.rs), utilisé tel quel.
//!
//! - `?`            état seul
//! - `#AAAA,BBBB`   consigne gauche, droite dans [0, 1000], 500 = arrêt
//! - `/`            arrêt (roue libre)
//! - `*`            freinage
//!
//! Réponse : `!AAAA,BBBB L:vit,pos R:vit,pos C:<pilote> F:<défaut>`

/// Consigne neutre du protocole.
pub const ZERO: i32 = 500;

/// Mesure d'une roue.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Wheel {
    /// Pas codeur par seconde.
    pub velocity: f32,
    /// Position cumulée, en pas codeur.
    pub position: i32,
}

/// Réponse décodée de la carte.
#[derive(Debug, Clone, PartialEq)]
pub struct Reply {
    /// Consigne appliquée, centrée : [-ZERO, ZERO].
    pub alpha: (i32, i32),
    pub left: Wheel,
    pub right: Wheel,
    /// Qui pilote : USB, RC ou LOST.
    pub owner: String,
    /// Défaut actif : -, USB_TIMEOUT ou STALL.
    pub fault: String,
}

/// Décode une ligne de réponse, ou `None` si ce n'en est pas une.
pub fn parse(line: &str) -> Option<Reply> {
    let mut fields = line.trim().strip_prefix('!')?.split(' ');
    let (al, ar) = fields.next()?.split_once(',')?;
    let left = wheel(fields.next()?.strip_prefix("L:")?)?;
    let right = wheel(fields.next()?.strip_prefix("R:")?)?;
    let owner = fields.next()?.strip_prefix("C:")?.to_string();
    let fault = fields.next()?.strip_prefix("F:")?.to_string();
    if fields.next().is_some() {
        return None;
    }
    Some(Reply {
        alpha: (al.parse::<i32>().ok()? - ZERO, ar.parse::<i32>().ok()? - ZERO),
        left,
        right,
        owner,
        fault,
    })
}

fn wheel(field: &str) -> Option<Wheel> {
    let (v, p) = field.split_once(',')?;
    Some(Wheel { velocity: v.parse().ok()?, position: p.parse().ok()? })
}

/// Trame de consigne avec mixage « arcade », comme la radiocommande :
/// gauche = avance + direction, droite = avance - direction.
pub fn speed_frame(throttle: i32, steer: i32) -> String {
    let left = (throttle + steer).clamp(-ZERO, ZERO);
    let right = (throttle - steer).clamp(-ZERO, ZERO);
    format!("#{:04},{:04}", left + ZERO, right + ZERO)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_reply() {
        let r = parse("!0700,0600 L:+600.5,+420 R:-3,-210 C:USB F:-\r").unwrap();
        assert_eq!(r.alpha, (200, 100));
        assert_eq!(r.left, Wheel { velocity: 600.5, position: 420 });
        assert_eq!(r.right, Wheel { velocity: -3.0, position: -210 });
        assert_eq!(r.owner, "USB");
        assert_eq!(r.fault, "-");
    }

    #[test]
    fn parse_fault_and_owner() {
        let r = parse("!0500,0500 L:+0,+0 R:+0,+0 C:LOST F:USB_TIMEOUT").unwrap();
        assert_eq!(r.owner, "LOST");
        assert_eq!(r.fault, "USB_TIMEOUT");
    }

    #[test]
    fn reject_other_lines() {
        assert_eq!(parse("MCU started"), None);
        assert_eq!(parse("!0500,0500 L:+0,+0 R:+0,+0 C:USB"), None);
        assert_eq!(parse("!0500,0500 L:x,+0 R:+0,+0 C:USB F:-"), None);
    }

    #[test]
    fn frames() {
        assert_eq!(speed_frame(0, 0), "#0500,0500");
        assert_eq!(speed_frame(150, 50), "#0700,0600");
        assert_eq!(speed_frame(500, 500), "#1000,0500");
        assert_eq!(speed_frame(-500, 200), "#0200,0000");
    }
}
