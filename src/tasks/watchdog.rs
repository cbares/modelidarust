use embassy_stm32::wdg::IndependentWatchdog;
use embassy_stm32::peripherals::IWDG;
use embassy_stm32::Peri;
use embassy_time::Timer;

#[embassy_executor::task]
pub async fn  watchdog_task(wdg:Peri<'static, IWDG>, timeout: u32) {
    let mut iwdg = IndependentWatchdog::new(wdg, timeout); // 1s timeout
    iwdg.unleash();

    loop {
        iwdg.pet();
        Timer::after_micros((timeout / 2).into()).await;
    }
}
