use core::cell::Cell;

use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

#[derive(Clone, Copy)]
pub struct OdomState {
    pub odom_left:  Odom,
    pub odom_right: Odom,
}

#[derive(Clone, Copy)]
pub struct Odom {
    pub velocity: f32,
    pub position: i32,
}


pub static ODOM_STATE: Mutex<CriticalSectionRawMutex, Cell<OdomState>> =
    Mutex::new(Cell::new(OdomState {
        odom_left:  Odom { position: 0, velocity: 0.0 },
        odom_right: Odom { position: 0, velocity: 0.0 },
    }));