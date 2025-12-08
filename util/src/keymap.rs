use core::{cmp::Ordering, usize};
use heapless::Vec;

#[derive(PartialEq, Eq, Clone, Copy, defmt::Format)]
#[derive(Default)]
pub enum KeyType {
    Keycode(u8),
    Mediacode(u8),
    EnableNum,
    #[default]
    NoCode,
}

impl TryFrom<&[u8]> for KeyType {
    type Error = ();

    fn try_from(value: &[u8]) -> Result<Self, Self::Error> {
        match (value[0], value[1]) {
            (0, code) => Ok(KeyType::Keycode(code)),
            (1, code) => Ok(KeyType::Mediacode(code)),
            (2, 0) => Ok(KeyType::EnableNum),
            (2, 1) => Ok(KeyType::NoCode),
            _ => Err(()),
        }
    }
}

impl KeyType {
    // Encode update to send and over serial
    pub fn encode_update(&self, pressed: bool) -> Result<u8, ()> {
        let mut val = match *self {
            KeyType::Keycode(val) if val < 0x66 => val,
            KeyType::Keycode(val) if val < 0xE8 => (val - 0xE0) + 0x66,
            KeyType::Mediacode(0xE2) => 0x6E,
            KeyType::Mediacode(0xE9) => 0x6F,
            KeyType::Mediacode(0x70) => 0x70,
            _ => return Err(()),
        };
        val |= if pressed { 0x80 } else { 0x00 };
        Ok(val)
    }

    pub fn decode_update(mut val: u8) -> (KeyType, bool) {
        let pressed: bool = (val & 0x80) != 0;

        // 0 <= normal <= 0x65 | 0x66 <= modifier <= 0x6D | 0x6E <= audio <= 0x70
        val = val & 0x7f;
        if val < 0x66 {
            return (KeyType::Keycode(val), pressed);
        }

        // 0xE0 | 0xE1 | 0xE2 | 0xE3 | 0xE4 | 0xE5 | 0xE6 | 0xE7
        // 0x66 | 0x67 | 0x68 | 0x69 | 0x6A | 0x6B | 0x6C | 0x6D
        // 0x00 | 0x01 | 0x02 | 0x03 | 0x04 | 0x05 | 0x06 | 0x07
        val -= 0x66;
        if val < 8 {
            return (KeyType::Keycode(val + 0xE0), pressed);
        }

        // Mute | VolInc | VolDec
        // 0xE2 | 0xE9   | 0xEA
        // 0x6E | 0x6F   | 0x70
        val -= 8;
        let key = match val {
            0x00 => KeyType::Mediacode(0xE2),
            0x01 => KeyType::Mediacode(0xE9),
            0x02 => KeyType::Mediacode(0x70),
            _ => KeyType::NoCode,
        };
        (key, pressed)
    }
}

pub struct FullState {
    pub key: Vec<u8, 14>,  // Sized 1 bigger so we can send it over serial
    pub modifier: u8,
    pub audio: u8,
}

impl FullState {
    pub fn new() -> Self {
        Self {
            key: Vec::from_array([0u8; 13]),
            modifier: 0,
            audio: 0,
        }
    }

    pub fn any_set(&self) -> bool {
        if self.modifier != 0 {
            true
        } else if self.audio != 0 {
            true
        } else {
            for val in &self.key {
                if *val != 0 {
                    return true;
                }
            }
            false
        }
    }

    pub fn clear(&mut self) {
        self.key.clear();
        // This will always be less then the size of the Vec
        unsafe { self.key.resize_default(13).unwrap_unchecked(); }
        self.modifier = 0;
        self.audio = 0;
    }

    #[inline(always)]
    pub fn merge(&self, other: &Self) -> [u8; 13] {
        let combined: Vec<u8, 13> = self.key[..13]
            .iter().zip(&other.key[..13])
            .map(|(val1, val2)| *val1 | *val2)
            .collect();
        // This shouldn't do anything under the hood since the size is the same
        unsafe { combined[..].try_into().unwrap_unchecked() }
    }

    #[inline(always)]
    pub fn is_set(&self, key: KeyType) -> bool {
        match key {
            KeyType::Keycode(val) if val < 0x66 => {
                (self.key[(val as usize) / 8] & (1 << (val % 8))) != 0
            },
            KeyType::Keycode(val) if val >= 0xE0 && val <= 0xE7 => {
                (self.modifier & 1 << (val - 0xE0)) != 0
            },
            KeyType::Mediacode(val) if val == 0xE2 => {
                (self.audio & (1 << 0)) != 0
            },
            KeyType::Mediacode(val) if val == 0xE9 => {
                (self.audio & (1 << 1)) != 0
            },
            KeyType::Mediacode(val) if val == 0xEA => {
                (self.audio & (1 << 2)) != 0
            },
            _ => false,
        }
    }

    #[inline(always)]
    pub fn set(&mut self, key: KeyType) {
        match key {
            KeyType::Keycode(val) if val < 0x66 => {
                self.key[val as usize / 8] |= 1 << (val % 8);
            },
            KeyType::Keycode(val) if val >= 0xE0 && val <= 0xE7 => {
                self.modifier |= 1 << (val - 0xE0);
            },
            KeyType::Mediacode(val) if val == 0xE2 => {
                self.audio |= 1 << 0;
            },
            KeyType::Mediacode(val) if val == 0xE9 => {
                self.audio |= 1 << 1;
            },
            KeyType::Mediacode(val) if val == 0xEA => {
                self.audio |= 1 << 2;
            },
            _ => (),
        }
    }

    #[inline(always)]
    pub fn reset(&mut self, key: KeyType) {
        match key {
            KeyType::Keycode(val) if val < 0x66 => {
                self.key[val as usize / 8] &= !(1 << (val % 8));
            },
            KeyType::Keycode(val) if val >= 0xE0 && val <= 0xE7 => {
                self.modifier &= !(1 << (val - 0xE0));
            },
            KeyType::Mediacode(val) if val == 0xE2 => {
                self.audio &= !(1 << 0);
            },
            KeyType::Mediacode(val) if val == 0xE9 => {
                self.audio &= !(1 << 1);
            },
            KeyType::Mediacode(val) if val == 0xEA => {
                self.audio &= !(1 << 2);
            },
            _ => (),
        }
    }

    #[inline(always)]
    pub fn serialize(mut self) -> Vec<u8, 14> {
        self.key[12] |= self.audio << 5;  // Pack audio keycode
        // There will always have been a free space at the end
        unsafe { self.key.push_unchecked(self.modifier); }
        self.key
    }

    #[inline(always)]
    pub fn deserialize(&mut self, buf: &[u8]) -> bool {
        let mut changed = false;

        let modifier_new = buf[13];
        changed = changed || self.modifier != modifier_new;
        self.modifier = modifier_new;

        let audio_new = (buf[12] >> 5) & 0x7;
        changed = changed || self.audio != audio_new;
        self.audio = audio_new;

        changed = changed || (self.key[..12].iter().cmp(&buf[..12]) != Ordering::Equal);
        changed = changed || (self.key[12] != (buf[12] & 0x1F));
        self.key.copy_from_slice(&buf[..13]);
        self.key[12] &= 0x1F;
        changed
    }
}
