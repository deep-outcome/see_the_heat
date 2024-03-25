#![no_main]
#![no_std]

use microbit::Peripherals;

use cortex_m_rt::entry;

use microbit::hal::pwm::{
    Channel::{self, C0 as CHRED, C1 as CHGREEN, C2 as CHBLUE},
    CounterMode, Prescaler, Pwm,
};

use microbit::pac::PWM0;
#[cfg(feature = "panic_halt")]
use panic_halt as _;

use microbit::hal::gpio::{p0, Level};
use microbit::hal::time::U32Ext;

#[cfg(feature = "run")]
use microbit::hal::Temp;

const ABS_MAX_DUTY: u16 = 256;

#[entry]
fn main() -> ! {
    const NEG_CEL_DEGS: f32 = 50.0;
    const POS_CEL_DEGS: f32 = 60.0;

    #[cfg(feature = "test")]
    rtt_target::rtt_init_print!();

    let p = Peripherals::take().unwrap();

    #[cfg(feature = "test")]
    let temp = {
        use microbit::hal::Timer;
        let tim = Timer::new(p.TIMER0);

        const OVERLIMIT: f32 = 10.0;

        TestTemp {
            val: -60.0,
            min: -NEG_CEL_DEGS - OVERLIMIT,
            max: POS_CEL_DEGS + OVERLIMIT,
            lo_l: -NEG_CEL_DEGS,
            up_l: POS_CEL_DEGS,
            ste: 0.1,
            tim,
            del: 5,
        }
    };

    #[cfg(feature = "run")]
    let temp = Temp::new(p.TEMP);

    let mut temp_reader = TempReader { temp };

    let pwm0 = Pwm::new(p.PWM0);
    let p0 = p0::Parts::new(p.P0);

    pwm0.set_counter_mode(CounterMode::Up)
        .set_prescaler(Prescaler::Div8)
        .set_period(7800.hz());

    set_max_duty_on_all(&pwm0);

    pwm0.set_output_pin(CHRED, p0.p0_02.into_push_pull_output(Level::Low).degrade())
        .set_output_pin(
            CHGREEN,
            p0.p0_03.into_push_pull_output(Level::Low).degrade(),
        )
        .set_output_pin(CHBLUE, p0.p0_04.into_push_pull_output(Level::Low).degrade());

    #[cfg(feature = "run")]
    temp_reader.start_measurement();

    const RGB_POIS: f32 = 256.0;
    const CEL_DEGS: f32 = 110.0;
    pub const POI_PER_DEG: f32 = RGB_POIS / CEL_DEGS;
    pub const DOU_POI_PER_DEG: f32 = POI_PER_DEG * 2.0;

    #[cfg(feature = "test")]
    assert_eq!(ABS_MAX_DUTY, pwm0.max_duty());

    loop {
        let be_val = temp_reader.read();

        if let Some(val) = be_val {
            if val < -NEG_CEL_DEGS || val > POS_CEL_DEGS {
                set_max_duty_on_all(&pwm0);
                continue;
            }

            let blue_amount = if val < 5.0 {
                (val + NEG_CEL_DEGS) * DOU_POI_PER_DEG
            } else {
                ABS_MAX_DUTY as f32
            };
            let green_amount = {
                let magic_corrector = 0.6 - if val > 30.0 { (val - 30.0) * 0.02 } else { 0.0 };

                #[cfg(feature = "test")]
                {
                    assert!(magic_corrector >= 0.0)
                }

                (val + NEG_CEL_DEGS) * (POI_PER_DEG + magic_corrector)
            };

            let red_amount = POI_PER_DEG * {
                if val < 5.0 {
                    val + NEG_CEL_DEGS
                } else {
                    POS_CEL_DEGS - val
                }
            };

            #[cfg(feature = "test")]
            {
                assert!((red_amount as u16) < 256);
                assert!((blue_amount as u16) < 257);
                assert!((green_amount as u16) < 256);
            }

            pwm0.set_duty_on(CHRED, red_amount as u16);
            pwm0.set_duty_on(CHBLUE, blue_amount as u16);
            pwm0.set_duty_on(CHGREEN, green_amount as u16);
        }
    }
}

fn set_max_duty_on_all(pwm0: &Pwm<PWM0>) {
    for ch in [CHRED, CHGREEN, CHBLUE] {
        set_max_duty(&pwm0, ch);
    }
}

fn set_max_duty(pwm: &Pwm<PWM0>, ch: Channel) {
    if pwm.duty_on(ch) != 0 {
        pwm.set_duty_on(ch, ABS_MAX_DUTY);
    }
}

#[cfg(feature = "test")]
struct TempReader {
    temp: TestTemp,
}

#[cfg(feature = "run")]
struct TempReader {
    temp: Temp,
}

#[cfg(feature = "test")]
struct TestTemp {
    val: f32,
    min: f32,
    max: f32,
    lo_l: f32,
    up_l: f32,
    ste: f32,
    tim: microbit::hal::Timer<microbit::hal::pac::TIMER0>,
    del: u32,
}

#[cfg(feature = "run")]
impl TempReader {
    fn start_measurement(&mut self) {
        self.temp.start_measurement();
    }

    fn read(&mut self) -> Option<f32> {
        if let Ok(val) = self.temp.read() {
            let conv = val.to_num::<f32>();
            self.start_measurement();
            Some(conv)
        } else {
            None
        }
    }
}

#[cfg(feature = "test")]
impl TempReader {
    fn read(&mut self) -> Option<f32> {
        use microbit::hal::prelude::_embedded_hal_blocking_delay_DelayMs;

        let temp = &mut self.temp;

        let min = temp.min;
        let lo_l = temp.lo_l;
        let up_l = temp.up_l;
        let max = temp.max;
        let del = temp.del;

        let mut val = temp.val;

        temp.val = if val >= max { min } else { val + temp.ste };

        if val < lo_l {
            val = if (val * -1.0) as u8 % 2 == 0 {
                lo_l
            } else {
                lo_l - 1000.0 * f32::EPSILON
            }
        }

        if val > up_l {
            val = if val as u8 % 2 == 0 {
                up_l
            } else {
                up_l + 1000.0 * f32::EPSILON
            }
        }

        rtt_target::rprintln!("val --> {}", val);

        temp.tim.delay_ms(del);

        Some(val)
    }
}

#[cfg(feature = "panic_abort")]
mod panic_abort {
    use core::panic::PanicInfo;

    #[panic_handler]
    fn panic(_info: &PanicInfo) -> ! {
        loop {}
    }
}

// cargo flash --target thumbv7em-none-eabihf --chip nRF52833_xxAA --release --features run
// cargo flash --target thumbv7em-none-eabihf --chip nRF52833_xxAA --features test
// cargo embed --target thumbv7em-none-eabihf --features test
// cargo build --target thumbv7em-none-eabihf --release --features run
// cargo build --target thumbv7em-none-eabihf --features test
