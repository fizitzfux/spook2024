/* EFFECTS */
pub const PART_DEFAULT:      f64 = 0.85;
pub const CUT_OUT_WEIGHT:  usize = 1;
pub const NOISE_WEIGHT:    usize = 4;
pub const MUTATED_WEIGHT:  usize = 8;

// DEFAULT
pub const DEFAULT_DURATION:  u16 = CYCLE_LEN;

// CUT_OUT
pub const CUT_OUT_DURATION:  u16 = CYCLE_LEN /2;

// NOISE
pub const NOISE_DURATION:    u16 = CYCLE_LEN /3;
pub const NOISE_THRESHOLD:   u32 = 0xFF;
pub const NOISE_THRESH_DIV:  u16 = 8;
pub const NOISE_OFFSET_LOW:  u16 = 0xFF;
pub const NOISE_OFFSET_HIGH: u16 = 0xFF;

// MUTATED
pub const MUTATED_DURATION:  u16 = CYCLE_LEN;
pub const MUTATED_AMOUNT:    u16 = 0x0FFF;
pub const MUTATED_PERIOD:    u16 = 0b1000_0000_0000;


/* SOUND */
const CYCLE_LEN:             u16 = 31975;


/* LIGHTING */
pub const HIGH:              u16 = 0x7FFF;
pub const LOW:               u16 = 0x0FFF;
pub const SPEED_UP:        usize = 19;
pub const SPEED_DOWN:      usize = 19;
