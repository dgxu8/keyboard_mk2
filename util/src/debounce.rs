use heapless::Vec;

pub const RB_ROW_LEN: usize = 8;
pub const RB_COL_LEN: usize = 8;

const FLIP: u8 = 5;
const MAX: u8 = 10;
const MIN: u8 = 0;
const OVERSHOOT: u8 = 25;

#[inline(always)]
pub fn rb_pack_state(state: &[[u8; RB_ROW_LEN]; RB_COL_LEN]) -> Vec<u8, RB_COL_LEN> {
    state.iter().map(|col| {
        // col.iter().rev().fold(0, |acc, &x| {
        col.iter().rev().fold(0, |acc, &x| {
            acc << 1 | ((x > FLIP) as u8)
        })
    }).collect()
}

#[inline(always)]
pub fn rb_unpack_state(buf: &[u8], state: &mut [[u8; RB_ROW_LEN]; RB_COL_LEN]) {
    for (i, packed) in buf.iter().enumerate() {
        let mut packed = *packed;
        for val in state[i].iter_mut() {
            *val = packed & 0x1;
            packed >>= 1;
        }
    }
}

#[inline(always)]
pub fn debounce<F>(state: &mut [u8], mut reg: u32, mut notify: F)
where
    F: FnMut(u8, bool) -> (),
{
    for i in 0..state.len() {
        let val = reg & 0x1 != 0;
        integrate(&mut state[i], val, |x| notify(i as _, x));
        reg >>= 1;
    }
}

#[inline(always)]
pub fn integrate<F>(cnt: &mut u8, pressed: bool, mut notify: F)
where
    F: FnMut(bool) -> (),
{
    if pressed {
        if *cnt < MAX {
            *cnt += 1;
            if *cnt == FLIP {
                *cnt = MAX + OVERSHOOT;
                notify(true);
            }
        }
    } else {
        if *cnt > MIN {
            *cnt -= 1;
            if *cnt == FLIP {
                *cnt = MIN;
                notify(false);
            }
        }
    }
}

#[inline(always)]
pub fn shift<F>(reg: &mut u8, pressed: bool, mut notify: F)
where
    F: FnMut(bool) -> (),
{
    let mut cnt = *reg & 0b1000_0000;
    cnt |= (*reg << 1) & 0b0001_1111;
    cnt |= pressed as u8;
    if cnt == 0b0001_1111 {
        *reg = 0b1001_1111;
        notify(true);
    } else if cnt == 0b1000_0000 {
        *reg = 0;
        notify(false);
    } else {
        *reg = cnt;
    }
}

#[inline(always)]
pub fn none<F>(prev_state: &mut u8, pressed: bool, mut notify: F)
where
    F: FnMut(bool) -> (),
{
    if *prev_state != pressed as u8 {
        *prev_state = pressed as u8;
        notify(pressed);
    }
}
