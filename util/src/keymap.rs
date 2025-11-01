#[derive(defmt::Format)]
#[derive(Default)]
pub enum KeyType {
    Keycode(u8),
    Mediacode(u8),
    EnableNum,
    #[default]
    None,
}

impl TryFrom<&[u8]> for KeyType {
    type Error = ();

    fn try_from(value: &[u8]) -> Result<Self, Self::Error> {
        match (value[0], value[1]) {
            (0, code) => Ok(KeyType::Keycode(code)),
            (1, code) => Ok(KeyType::Mediacode(code)),
            (2, 0) => Ok(KeyType::EnableNum),
            (2, 1) => Ok(KeyType::None),
            _ => Err(()),
        }
    }
}
