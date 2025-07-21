#![no_std]
#![no_main]

use core::cell::{Cell, RefCell};
use core::task::Waker;
// use cortex_m::asm::nop;
use critical_section::CriticalSection;
use defmt::*;
use embassy_executor::Spawner;
use embassy_nxp::{
    gpio::{Level, Output},
    pac::{interrupt, PMC, RTC, SYSCON},
};
use embassy_sync::blocking_mutex::CriticalSectionMutex as Mutex;
use embassy_time::Timer;
use embassy_time_driver::time_driver_impl;
use embassy_time_driver::Driver;
use embassy_time_queue_utils::Queue;
use {defmt_rtt as _, panic_halt as _};

struct AlarmState {
    timestamp: Cell<u64>,
}

unsafe impl Send for AlarmState {}

impl AlarmState {
    const fn new() -> Self {
        Self {
            timestamp: Cell::new(u64::MAX),
        }
    }
}

pub struct RtcDriver {
    alarms: Mutex<AlarmState>,
    queue: Mutex<RefCell<Queue>>,
}

time_driver_impl!(static DRIVER: RtcDriver = RtcDriver {
    alarms: Mutex::new(AlarmState::new()),
    queue: Mutex::new(RefCell::new(Queue::new())),
});
impl RtcDriver {
    fn init(&'static self) {
        defmt::info!("init");
        let syscon = unsafe { &*SYSCON::ptr() };
        let pmc = unsafe { &*PMC::ptr() };
        let rtc = unsafe { &*RTC::ptr() };

        syscon.ahbclkctrl0.modify(|_, w| w.rtc().enable());
        rtc.ctrl.modify(|_, w| w.swreset().set_bit());
        rtc.ctrl.modify(|_, w| w.swreset().clear_bit());
        pmc.rtcosc32k.write(|w| w.sel().xtal32k());
        rtc.ctrl.modify(|_, w| w.rtc_osc_pd().power_up());
        //reset/clear(?) conter
        rtc.count.reset();
        //en rtc main counter
        rtc.ctrl.modify(|_, w| w.rtc_en().set_bit());
        rtc.ctrl.modify(|_, w| w.rtc1khz_en().set_bit());
        // subsec counter enable
        rtc.ctrl.modify(|_, w| w.rtc_subsec_ena().set_bit());
        let mut cp = cortex_m::peripheral::Peripherals::take().unwrap();
        unsafe { cp.NVIC.set_priority(interrupt::RTC, 3) };
        unsafe { cortex_m::peripheral::NVIC::unmask(interrupt::RTC) };
    }

    fn set_alarm(&self, cs: CriticalSection, timestamp: u64) -> bool {
        defmt::info!("set");
        let rtc = unsafe { &*RTC::ptr() };
        let alarm = &self.alarms.borrow(cs);
        alarm.timestamp.set(timestamp);
        let now = self.now();

        if timestamp <= now {
            alarm.timestamp.set(u64::MAX);
            return false;
        }

        //time diff in sub-sec not ticks (32kHz)
        let diff = timestamp - now;
        let sec = (diff / 32768) as u32;
        let subsec = (diff % 32768) as u32;

        let current_sec = rtc.count.read().val().bits();
        let target_sec = current_sec.wrapping_add(sec as u32);

        rtc.match_.write(|w| unsafe { w.matval().bits(target_sec) });
        rtc.wake.write(|w| unsafe {
            let ms = ((subsec * 1000) + 16384) / 32768;
            w.val().bits(ms as u16)
        });
        if subsec > 0 {
            let ms = ((subsec * 1000) + 16384) / 32768;
            rtc.wake.write(|w| unsafe { w.val().bits(ms as u16) });
        }
        rtc.ctrl.modify(|_, w| w.alarm1hz().clear_bit().wake1khz().clear_bit());
        true
    }

    fn on_interrupt(&self) {
        defmt::info!("on inter");
        critical_section::with(|cs| {
            let rtc = unsafe { &*RTC::ptr() };
            let flags = rtc.ctrl.read();
            if flags.alarm1hz().bit_is_clear() {
                rtc.ctrl.modify(|_, w| w.alarm1hz().set_bit());
                self.trigger_alarm(cs);
            }

            if flags.wake1khz().bit_is_clear() {
                rtc.ctrl.modify(|_, w| w.wake1khz().set_bit());
                self.trigger_alarm(cs);
            }
        });
    }

    fn trigger_alarm(&self, cs: CriticalSection) {
        defmt::info!("trigger");
        let alarm = &self.alarms.borrow(cs);
        alarm.timestamp.set(u64::MAX);
        let mut next = self.queue.borrow(cs).borrow_mut().next_expiration(self.now());
        if next == u64::MAX {
            defmt::warn!("no scheduled events, skipping");
            return;
        }
        while !self.set_alarm(cs, next) {
            next = self.queue.borrow(cs).borrow_mut().next_expiration(self.now());
            if next == u64::MAX {
                defmt::warn!("no next event found after retry");
                return;
            }
        }
    }
}

impl Driver for RtcDriver {
    //read time:sec+wake count in ms
    fn now(&self) -> u64 {
        defmt::info!("now");
        let rtc = unsafe { &*RTC::ptr() };
        let sec = rtc.count.read().val().bits() as u64;
        let sub = rtc.subsec.read().subsec().bits() as u64;
        sec * 32768 + sub
    }

    fn schedule_wake(&self, at: u64, waker: &Waker) {
        defmt::info!("wake");
        critical_section::with(|cs| {
            let mut queue = self.queue.borrow(cs).borrow_mut();

            if queue.schedule_wake(at, waker) {
                let mut next = queue.next_expiration(self.now());
                while !self.set_alarm(cs, next) {
                    next = queue.next_expiration(self.now());
                }
            }
        })
    }
}
#[cortex_m_rt::interrupt]
fn RTC() {
    // info!("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA");
    // critical_section::with(|cs| {
    //     let rtc = unsafe { &*RTC::ptr() };
    //     let flags = rtc.ctrl.read();
    //     if flags.alarm1hz().bit_is_clear() {
    //         rtc.ctrl.modify(|_, w| w.alarm1hz().set_bit());
    //     }

    //     if flags.wake1khz().bit_is_clear() {
    //         rtc.ctrl.modify(|_, w| w.wake1khz().set_bit());
    //     }
    // });
    DRIVER.on_interrupt();
}
pub(crate) fn init() {
    DRIVER.init();
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // defmt::info!("init");
    // let syscon = unsafe { &*SYSCON::ptr() };
    // let pmc = unsafe { &*PMC::ptr() };
    // let rtc = unsafe { &*RTC::ptr() };

    // syscon.ahbclkctrl0.modify(|_, w| w.rtc().enable());
    // rtc.ctrl.modify(|_, w| w.swreset().set_bit());
    // rtc.ctrl.modify(|_, w| w.swreset().clear_bit());
    // pmc.rtcosc32k.write(|w| w.sel().xtal32k());
    // rtc.ctrl.modify(|_, w| w.rtc_osc_pd().power_up());
    // //reset/clear(?) conter
    // // rtc.count.write(|w| unsafe {w.bits(0)});
    // rtc.count.reset();
    // //en rtc main counter
    // rtc.ctrl.modify(|_, w| w.rtc_en().set_bit());
    // rtc.ctrl.modify(|_, w| w.rtc1khz_en().set_bit());
    // // subsec counter enable
    // rtc.ctrl.modify(|_, w| w.rtc_subsec_ena().set_bit());
    // let mut cp = cortex_m::peripheral::Peripherals::take().unwrap();
    // unsafe { cp.NVIC.set_priority(interrupt::RTC, 3) };
    // unsafe { cortex_m::peripheral::NVIC::unmask(interrupt::RTC) };
    // let n = {
    //     let rtc = unsafe { &*RTC::ptr() };
    //     let sec = rtc.count.read().val().bits() as u64;
    //     let sub = rtc.subsec.read().subsec().bits() as u64;
    //     sec * 32768 + sub
    // };
    // let timestamp = n + 32768 * 5;
    // //time diff in sub-sec not ticks (32kHz)
    // let diff = timestamp - n;
    // let sec = (diff / 32768) as u32;
    // let subsec = (diff % 32768) as u32;

    // let current_sec = rtc.count.read().val().bits();
    // let target_sec = current_sec.wrapping_add(sec as u32);

    // rtc.match_.write(|w| unsafe { w.matval().bits(target_sec) });
    // rtc.wake.write(|w| unsafe {
    //     let ms = ((subsec * 1000) + 16384) / 32768;
    //     w.val().bits(ms as u16)
    // });
    // rtc.ctrl.modify(|_, w| w.alarm1hz().clear_bit().wake1khz().clear_bit());

    let p = embassy_nxp::init(Default::default());

    init();
    Timer::after_millis(200);
    // info!("{:?}", &t);
    // t.await;
    // embassy_nxp::time_driver::init();

    info!("1 AAAAAAAA");
    let mut led = Output::new(p.PIO1_6, Level::Low);
    info!("2 AAAAAAAA");
    loop {
        // info!("led off!");
        // led.set_high();
        // Timer::after_secs(1).await;

        // info!("led on!");
        // led.set_low();
        // Timer::after_secs(1).await;
    }
}
