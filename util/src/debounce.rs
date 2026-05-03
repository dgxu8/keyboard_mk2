use crate::keycom::{FullState, KeyType};

pub const RB_ROW_LEN: usize = 8;
pub const RB_COL_LEN: usize = 8;

const FLIP: u8 = 5;
const MAX: u8 = 10;
const MIN: u8 = 0;
const OVERSHOOT: u8 = 25;

#[inline(always)]
pub fn rb_encode<F>(state: &[[u8; RB_ROW_LEN]; RB_COL_LEN], map: F) -> FullState
where
    F: Fn(usize, usize) -> KeyType
{
    let mut full_state = FullState::new();
    for (col, inner) in state.iter().enumerate() {
        for (row, val) in inner.iter().enumerate() {
            if *val > FLIP {
                full_state.set(map(col, row));
            }
        }
    }
    full_state
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
pub fn debounce_update<F>(state: &mut [u8], reg: u32, notify: F)
where
    F: FnMut(usize, bool) -> (),
{
    debounce(state, reg, notify, |_, _| {});
}

#[inline(always)]
pub fn debounce_full<F>(state: &mut [u8], reg: u32, notify: F)
where
    F: FnMut(usize, bool) -> (),
{
    debounce(state, reg, |_, _| {}, notify);
}

#[inline(always)]
pub fn debounce<F, G>(state: &mut [u8], mut reg: u32, mut notify_change: F, mut notify_full: G)
where
    F: FnMut(usize, bool) -> (),
    G: FnMut(usize, bool) -> (),
{
    for i in 0..state.len() {
        let val = reg & 0x1 != 0;
        integrate(
            &mut state[i], val,
            |pressed| notify_change(i as _, pressed),
            |changed| notify_full(i as _, changed),
        );
        reg >>= 1;
    }
}

#[inline(always)]
pub fn integrate_update<F>(cnt: &mut u8, pressed: bool, notify: F)
where
    F: FnMut(bool) -> (),
{
    integrate(cnt, pressed, notify, |_| {});
}

#[inline(always)]
pub fn integrate_full<F>(cnt: &mut u8, pressed: bool, notify: F)
where
    F: FnMut(bool) -> (),
{
    integrate(cnt, pressed, |_| {}, notify);
}

pub fn integrate<F, G>(cnt: &mut u8, pressed: bool, mut notify_change: F, mut notify_full: G)
where
    F: FnMut(bool) -> (),
    G: FnMut(bool) -> (),
{
    let mut changed = false;
    if pressed {
        if *cnt < MAX {
            *cnt += 1;
            if *cnt == FLIP {
                *cnt = MAX + OVERSHOOT;
                changed = true;
                notify_change(true);
            }
        }
    } else {
        if *cnt > MIN {
            *cnt -= 1;
            if *cnt == FLIP {
                *cnt = MIN;
                changed = true;
                notify_change(false);
            }
        }
    }
    if *cnt > FLIP {
        notify_full(changed);
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
