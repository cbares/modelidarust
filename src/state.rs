use core::cell::Cell;

use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

#[derive(Clone, Copy)]
pub struct MotorCmd {
    pub alpha_left: i32,
    pub alpha_right: i32,
    max_speed: i32,
}

impl  MotorCmd {
    pub fn new(alpha_left: i32, alpha_right: i32, max_speed: i32) -> Self {
        Self { alpha_left, alpha_right, max_speed }
    }
    
}

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


pub static MOTOR_CMD: Mutex<CriticalSectionRawMutex, Cell<MotorCmd>> =
    Mutex::new(Cell::new(MotorCmd {
        alpha_left: 0,
        alpha_right: 0,
        max_speed: 0,
    }));


pub static ODOM_STATE: Mutex<CriticalSectionRawMutex, Cell<OdomState>> =
    Mutex::new(Cell::new(OdomState {
        odom_left:  Odom { position: 0, velocity: 0.0 },
        odom_right: Odom { position: 0, velocity: 0.0 },
    }));