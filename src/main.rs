// LED control: https://www.alexdwilson.dev/learning-in-public/pwm-output-how-to-program-a-raspberry-pi-pico-with-rust
// Audio: https://pinoysa.us/codes/pico_audio.txt

#![no_std]
#![no_main]
extern crate panic_halt;
extern crate embedded_hal;
extern crate rp2040_hal;
extern crate fugit;
extern crate embedded_al\loc;
extern crate alloc;
extern crate rand;

use alloc::vec::Vec;
use embedded_alloc::Heap;
use fugit::RateExtU32;
use embedded_hal::PwmPin;
use rp2040_hal::{self as hal, clocks::{ClocksManager, InitError}, gpio::{bank0::{Gpio13, Gpio25}, Pin, PinId}, multicore::{Multicore, Stack}, pac::{self, interrupt}, pll::{common_configs::PLL_USB_48MHZ, PLLConfig}, pwm::{FreeRunning, Pwm0, Pwm4, Pwm6, Slice, SliceId}, Sio, Timer, Watchdog};
use rand::{rngs::SmallRng, Rng, SeedableRng};

mod values;
use values::*;

// Enforce halt (freeze) on panic
#[allow(unused_imports)]
use panic_halt as _;

/// Allocate bootloader
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

/// Allocate heap
#[global_allocator]
static mut HEAP: Heap = Heap::empty();

/// Allocate stack for the second core
static mut CORE1_STACK: Stack<4096> = Stack::new();

/// The audio file to play.
/// Format: WAV 8-bit unsigned mono 8_000 Hz
const AUDIO: &[u8] = include_bytes!("track.wav");

/// The frequency of the on-board crystal
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

/// This clock rate is closest to 176,400,000 Hz, which is a multiple of 44,100 Hz.
#[allow(dead_code)]
const PLL_SYS_176MHZ: PLLConfig = PLLConfig {
    vco_freq: fugit::Rate::<u32, 1, 1>::MHz(528),
    refdiv: 1,
    post_div1: 3,
    post_div2: 1,
};

/// This clock rate is closest to 131,072,000 Hz, which is a multiple of 32,000 Hz (the audio sample rate).
const PLL_SYS_131MHZ: PLLConfig = PLLConfig {
    vco_freq: fugit::Rate::<u32, 1, 1>::MHz(1572),
    refdiv: 1,
    post_div1: 6,
    post_div2: 2,
};

/// Initialize system clocks and PLLs according to specified configs
#[allow(clippy::too_many_arguments)]
fn init_clocks_and_plls_cfg(
    xosc_crystal_freq: u32,
    xosc_dev: pac::XOSC,
    clocks_dev: pac::CLOCKS,
    pll_sys_dev: pac::PLL_SYS,
    pll_usb_dev: pac::PLL_USB,
    pll_sys_cfg: PLLConfig,
    pll_usb_cfg: PLLConfig,
    resets: &mut pac::RESETS,
    watchdog: &mut Watchdog,
) -> Result<ClocksManager, InitError> {
    let xosc = hal::xosc::setup_xosc_blocking(xosc_dev, xosc_crystal_freq.Hz())
        .map_err(InitError::XoscErr)?;

    // Configure watchdog tick generation to tick over every microsecond
    watchdog.enable_tick_generation((xosc_crystal_freq / 1_000_000) as u8);

    let mut clocks = ClocksManager::new(clocks_dev);

    let pll_sys = hal::pll::setup_pll_blocking(
        pll_sys_dev,
        xosc.operating_frequency().into(),
        pll_sys_cfg,
        &mut clocks,
        resets,
    )
    .map_err(InitError::PllError)?;
    let pll_usb = hal::pll::setup_pll_blocking(
        pll_usb_dev,
        xosc.operating_frequency().into(),
        pll_usb_cfg,
        &mut clocks,
        resets,
    )
    .map_err(InitError::PllError)?;

    clocks
        .init_default(&xosc, &pll_sys, &pll_usb)
        .map_err(InitError::ClockError)?;
    Ok(clocks)
}

static mut RNG: Option<SmallRng> = None;

struct Effect {}
impl Effect {
    const NONE   : u32 = 0x0000_0000;
    const DEFAULT: u32 = 0x0001_0000;
    const CUT_OUT: u32 = 0x0002_0000;
    const NOISE  : u32 = 0x0003_0000;
    const MUTATED: u32 = 0x0004_0000;

    const fn new(instr: u32, value: u16) -> u32 {
        instr ^ value as u32
    }

    const fn decode(data: u32) -> (u32, u16) {
        (data & 0xFFFF_0000, (data & 0x0000_FFFF) as u16)
    }

    fn obtain_effect_mix() -> Vec<u32> {
        let mut result = Vec::new();
        result.append(&mut [Effect::CUT_OUT; CUT_OUT_WEIGHT].to_vec());
        result.append(&mut [Effect::NOISE; NOISE_WEIGHT].to_vec());
        result.append(&mut [Effect::MUTATED; MUTATED_WEIGHT].to_vec());
        result
    }
}


/* SHARED WITH INTERRUPT */

/// The hardware PWM driver that is shared with the interrupt routine.
static mut PWM: Option<hal::pwm::Slice<Pwm0, FreeRunning>> = None;

#[interrupt]
fn PWM_IRQ_WRAP() {
    // SAFETY: This is not used outside of interrupt critical sections in the main thread.
    let pwm = unsafe { PWM.as_mut() }.unwrap();

    // Clear the interrupt (so we don't immediately re-enter this routine)
    pwm.clear_interrupt();
}


/* FIRST CORE */

#[rp2040_hal::entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    // let core = pac::CorePeripherals::take().unwrap();

    // Initialize heap
    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 1024;
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
    }

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    let _clocks = init_clocks_and_plls_cfg(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        PLL_SYS_131MHZ,
        PLL_USB_48MHZ,
        &mut pac.RESETS,
        &mut watchdog,).ok().unwrap();

    // Initialize RNG with microseconds since boot as seed
    let time = Timer::new(pac.TIMER, &mut pac.RESETS).get_counter().ticks();
    unsafe { RNG = Some(SmallRng::seed_from_u64(time)) }

    // The single-cycle I/O block controls our GPIO pins
    let mut sio = hal::Sio::new(pac.SIO);

    // Set the pins to their default state
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Init PWMs
    let mut pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);

    let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
    let cores = mc.cores();
    let core1 = &mut cores[1];
    let _test = core1.spawn(
        unsafe {&mut CORE1_STACK.mem},
        move || {core1_task(
            &mut pwm_slices.pwm4,
            &mut pwm_slices.pwm6,
            pins.gpio25,
            pins.gpio13,
        )}
    );

    let mut pwm = pwm_slices.pwm0;
    pwm.default_config();

    // 131,000,000 Hz divided by (top * div.int).
    //
    // fPWM = fSYS / ((TOP + 1) * (CSR_PH_CORRECT + 1) * (DIV_INT + (DIV_FRAC / 16)))
    //
    // 32kHz ~= 131,000,000 / ((4096 + 1) * 1 * 1)
    const TOP: u16 = 4096;
    const HALF: u16 = TOP >>1;
    pwm.set_top(TOP);
    pwm.set_div_int(1);

    pwm.enable_interrupt();
    pwm.enable();

    // Output channel A on PWM0 to GPIO16
    pwm.channel_a.output_to(pins.gpio16);

    unsafe {
        // Share the PWM with our interrupt routine.
        PWM = Some(pwm);

        // Unmask the PWM_IRQ_WRAP interrupt so we start receiving events.
        pac::NVIC::unmask(pac::Interrupt::PWM_IRQ_WRAP);
    }

    let mut rng = SmallRng::from_rng(unsafe { RNG.clone() }.unwrap_or(SmallRng::seed_from_u64(0))).unwrap();

    let effects = Effect::obtain_effect_mix();
    let mut effect = Effect::NONE;
    let mut effect_duration: u16 = 0;

    loop {
        // N.B: Skip the WAV header here. We're going to assume the format is what we expect.
        for i in 0x2C..(AUDIO.len() << 2) {
            let val = &AUDIO[i >> 2];
            // Rescale from unsigned u8 numbers to 0..4096 (the TOP register we specified earlier)
            //
            // The PWM channel will increment an internal counter register, and if the counter is
            // above or equal to this number, the PWM will output a logic high signal.
            let val = ((*val as u16) << 4) & 0xFFF;

            if effect == Effect::NONE {
                effect = if rng.gen_bool(PART_DEFAULT) {
                        Effect::DEFAULT
                    }
                    else {
                        effects[rng.gen_range(0..effects.len())]
                    };
                
                effect_duration = match effect {
                        Effect::NONE =>    0,
                        Effect::DEFAULT => DEFAULT_DURATION,
                        Effect::CUT_OUT => CUT_OUT_DURATION,
                        Effect::NOISE =>   NOISE_DURATION,
                        Effect::MUTATED => MUTATED_DURATION,
                        _ =>               0,
                    }
            }

            let val = match effect {
                Effect::NONE => HALF,
                Effect::DEFAULT => val,
                Effect::CUT_OUT => HALF,
                Effect::NOISE =>
                    rng.gen_range((HALF - NOISE_OFFSET_LOW)..(HALF + NOISE_OFFSET_HIGH)),
                Effect::MUTATED => val.saturating_mul(2),
                _ => TOP,
            };

            cortex_m::interrupt::free(|_| {
                // SAFETY: Interrupt cannot currently use this while we're in a critical section.
                let channel = &mut unsafe { PWM.as_mut() }.unwrap().channel_a;
                channel.set_duty(val);
            });

            if sio.fifo.is_write_ready() {
                sio.fifo.write(Effect::new(effect, effect_duration));
            }

            effect_duration = effect_duration.saturating_sub(1);
            if effect_duration == 0 {
                effect = Effect::NONE;
            }

            if i != AUDIO.len() {
                // Throttle until the PWM channel delivers us an interrupt saying it's done
                // with this cycle (the internal counter wrapped). The interrupt handler will
                // clear the interrupt and we'll send out the next sample.
                cortex_m::asm::wfi();
            }
        }
    }
}


/* SECOND CORE */

fn core1_task(pwm4: &mut Slice<Pwm4, <Pwm4 as SliceId>::Reset>, pwm6: &mut Slice<Pwm6, <Pwm6 as SliceId>::Reset>, gpio25: Pin<Gpio25, <Gpio25 as PinId>::Reset>, gpio13: Pin<Gpio13, <Gpio13 as PinId>::Reset>) -> ! {
    let pac = unsafe {
        pac::Peripherals::steal()
    };
    let mut sio = Sio::new(pac.SIO);

    
    // Configure PWM4
    pwm4.set_top(HIGH);
    pwm4.set_ph_correct();
    pwm4.enable();

    // Output channel B on PWM4 to GPIO 25
    let led_chan = &mut pwm4.channel_b;
    led_chan.output_to(gpio25);

    // Configure PWM6
    pwm6.set_top(HIGH);
    pwm6.set_ph_correct();
    pwm6.enable();

    // Output channel B on PWM6 to GPIO 13
    let lights_chan = &mut pwm6.channel_b;
    lights_chan.output_to(gpio13);

    let mut light_iter = 
        (LOW..=HIGH).skip(SPEED_UP)
        .chain(
            (LOW..=HIGH).skip(SPEED_DOWN).rev()
        ).cycle().into_iter();

    let mut rng = SmallRng::from_rng(unsafe { RNG.clone() }.unwrap_or(SmallRng::seed_from_u64(1))).unwrap_or(SmallRng::seed_from_u64(1));

    loop {
        let data = sio.fifo.read_blocking();
        let (instruction, duration) = Effect::decode(data);
        
        let duty = light_iter.next().unwrap_or(HIGH);
        
        let value = match instruction {
            Effect::NONE => LOW,
            Effect::DEFAULT => duty,
            Effect::CUT_OUT => 0,
            Effect::NOISE => {
                let val = rng.r#gen();
                led_chan.set_duty(val);
                lights_chan.set_duty(val);

                if duration >>NOISE_THRESH_DIV >= 1 {
                    for _ in 0..NOISE_THRESHOLD {
                        sio.fifo.read_blocking();
                    }
                }
                val
            }
            Effect::MUTATED => {
                if duration & MUTATED_PERIOD == MUTATED_PERIOD {
                    duty.saturating_add(MUTATED_AMOUNT)
                } else {
                    duty.saturating_sub(MUTATED_AMOUNT)
                }
            }
            _ => HIGH,
        };

        led_chan.set_duty(value);
        lights_chan.set_duty(value);
    }
}
