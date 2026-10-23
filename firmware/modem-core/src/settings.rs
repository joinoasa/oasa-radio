//! Radio settings, limits and air time.

/// LoRa bandwidths the SX1262 supports. The number is the protocol code.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Bandwidth {
    Khz7_8 = 0,
    Khz10_4 = 1,
    Khz15_6 = 2,
    Khz20_8 = 3,
    Khz31_25 = 4,
    Khz41_7 = 5,
    Khz62_5 = 6,
    Khz125 = 7,
    Khz250 = 8,
    Khz500 = 9,
}

impl Bandwidth {
    pub const ALL: [Bandwidth; 10] = [
        Bandwidth::Khz7_8,
        Bandwidth::Khz10_4,
        Bandwidth::Khz15_6,
        Bandwidth::Khz20_8,
        Bandwidth::Khz31_25,
        Bandwidth::Khz41_7,
        Bandwidth::Khz62_5,
        Bandwidth::Khz125,
        Bandwidth::Khz250,
        Bandwidth::Khz500,
    ];

    /// From the protocol code.
    pub fn from_code(code: u8) -> Option<Self> {
        Self::ALL.get(usize::from(code)).copied()
    }

    /// The protocol code.
    pub fn code(self) -> u8 {
        self as u8
    }

    /// Bandwidth in hertz (rounded).
    pub fn hz(self) -> u32 {
        [
            7_813, 10_417, 15_625, 20_833, 31_250, 41_667, 62_500, 125_000, 250_000, 500_000,
        ][usize::from(self as u8)]
    }

    /// The SX126x register value for `SetModulationParams`.
    pub fn chip_code(self) -> u8 {
        [0x00, 0x08, 0x01, 0x09, 0x02, 0x0A, 0x03, 0x04, 0x05, 0x06][usize::from(self as u8)]
    }
}

/// What a board and chip allow. A setting outside these is refused.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Limits {
    pub freq_min_hz: u32,
    pub freq_max_hz: u32,
    pub sf_min: u8,
    pub sf_max: u8,
    pub power_min_dbm: i8,
    pub power_max_dbm: i8,
}

impl Limits {
    /// The SX1262 chip: 150 to 960 MHz, SF5 to SF12, -9 to +22 dBm
    /// (Semtech SX1261/2 datasheet).
    pub const SX1262: Limits = Limits {
        freq_min_hz: 150_000_000,
        freq_max_hz: 960_000_000,
        sf_min: 5,
        sf_max: 12,
        power_min_dbm: -9,
        power_max_dbm: 22,
    };
    /// The Heltec WiFi LoRa 32 V3 sold for 863 to 928 MHz (HTIT-WB32LA).
    pub const HELTEC_V3: Limits = Limits {
        freq_min_hz: 863_000_000,
        freq_max_hz: 928_000_000,
        ..Limits::SX1262
    };
}

/// Setting keys on the control channel.
pub mod key {
    pub const FREQ: u8 = 0x01;
    pub const SF: u8 = 0x02;
    pub const BW: u8 = 0x03;
    pub const CR: u8 = 0x04;
    pub const POWER: u8 = 0x05;
    pub const SYNC: u8 = 0x06;
    pub const PREAMBLE: u8 = 0x07;
    pub const AIR: u8 = 0x08;
}

/// The radio settings. Packet format is fixed: explicit header, CRC on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Settings {
    /// Carrier frequency in hertz.
    pub freq_hz: u32,
    /// Spreading factor, 5 to 12.
    pub sf: u8,
    pub bw: Bandwidth,
    /// Coding rate denominator: 5 means 4/5, up to 8 for 4/8.
    pub cr: u8,
    /// Output power in dBm.
    pub tx_power_dbm: i8,
    /// SX126x LoRa sync word register. 0x1424 is private (SX127x 0x12),
    /// 0x3444 is public (SX127x 0x34).
    pub sync_word: u16,
    /// Preamble length in symbols.
    pub preamble: u16,
    /// Promiscuous mode: report every received frame, CRC failures included.
    pub air_mode: bool,
}

/// Size of [`Settings::to_bytes`].
pub const SETTINGS_LEN: usize = 13;

impl Settings {
    /// The lab profile `lora-sf7-bw125`: SF7, 125 kHz, CR 4/5, 8 symbol
    /// preamble, 915 MHz. Power starts low (2 dBm) for bench tests over coax.
    pub const LAB_SF7_BW125: Settings = Settings {
        freq_hz: 915_000_000,
        sf: 7,
        bw: Bandwidth::Khz125,
        cr: 5,
        tx_power_dbm: 2,
        sync_word: 0x1424,
        preamble: 8,
        air_mode: false,
    };

    /// Check every field. `Err` holds the key of the first one that is out of range.
    pub fn validate(&self, l: &Limits) -> Result<(), u8> {
        if !(l.freq_min_hz..=l.freq_max_hz).contains(&self.freq_hz) {
            return Err(key::FREQ);
        }
        if !(l.sf_min..=l.sf_max).contains(&self.sf) {
            return Err(key::SF);
        }
        if !(5..=8).contains(&self.cr) {
            return Err(key::CR);
        }
        if !(l.power_min_dbm..=l.power_max_dbm).contains(&self.tx_power_dbm) {
            return Err(key::POWER);
        }
        // SF5 and SF6 need at least 12 preamble symbols (Semtech).
        let min_pre = if self.sf <= 6 { 12 } else { 6 };
        if self.preamble < min_pre {
            return Err(key::PREAMBLE);
        }
        Ok(())
    }

    /// Wire form: freq u32, sf, bw code, cr, power i8, sync u16, preamble u16, air. Big-endian.
    pub fn to_bytes(&self) -> [u8; SETTINGS_LEN] {
        let mut b = [0u8; SETTINGS_LEN];
        b[0..4].copy_from_slice(&self.freq_hz.to_be_bytes());
        b[4] = self.sf;
        b[5] = self.bw.code();
        b[6] = self.cr;
        b[7] = self.tx_power_dbm as u8;
        b[8..10].copy_from_slice(&self.sync_word.to_be_bytes());
        b[10..12].copy_from_slice(&self.preamble.to_be_bytes());
        b[12] = u8::from(self.air_mode);
        b
    }

    /// Parse the wire form. `None` if short or the bandwidth code is unknown.
    pub fn from_bytes(b: &[u8]) -> Option<Self> {
        if b.len() < SETTINGS_LEN {
            return None;
        }
        Some(Settings {
            freq_hz: u32::from_be_bytes([b[0], b[1], b[2], b[3]]),
            sf: b[4],
            bw: Bandwidth::from_code(b[5])?,
            cr: b[6],
            tx_power_dbm: b[7] as i8,
            sync_word: u16::from_be_bytes([b[8], b[9]]),
            preamble: u16::from_be_bytes([b[10], b[11]]),
            air_mode: b[12] != 0,
        })
    }

    /// Does the chip need low data rate optimization (symbol over 16.38 ms)?
    pub fn low_data_rate(&self) -> bool {
        // symbol time in microseconds = 2^sf * 1e6 / bw
        let t = (1u64 << self.sf) * 1_000_000 / u64::from(self.bw.hz());
        t > 16_380
    }

    /// Air time of a packet of `payload_len` bytes in microseconds
    /// (Semtech SX1261/2 datasheet, LoRa time on air; explicit header, CRC on).
    pub fn time_on_air_us(&self, payload_len: usize) -> u32 {
        let sf = u64::from(self.sf);
        let bw = u64::from(self.bw.hz());
        let tsym_ns = (1u64 << sf) * 1_000_000_000 / bw;
        let de = u64::from(self.low_data_rate());
        let num = (8 * payload_len as u64 + 28 + 16).saturating_sub(4 * sf);
        let den = 4 * (sf - 2 * de);
        let groups = num.div_ceil(den);
        let symbols = 8 + groups * u64::from(self.cr);
        // preamble + 4.25 symbols (6.25 for SF5 and SF6), in quarter symbols
        let pre_q = u64::from(self.preamble) * 4 + if self.sf <= 6 { 25 } else { 17 };
        let total_ns = tsym_ns * (pre_q + symbols * 4) / 4;
        (total_ns / 1000).min(u64::from(u32::MAX)) as u32
    }
}

/// Why a `SET` was refused.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SetError {
    /// Malformed list or unknown key. Holds the key (0 if the list was cut off).
    BadArgs(u8),
    /// A value outside the limits. Holds the key.
    OutOfRange(u8),
}

/// Apply a list of `key value` pairs to `base`. Nothing changes unless all
/// pairs are good. A key may appear once.
pub fn apply_pairs(base: &Settings, args: &[u8], limits: &Limits) -> Result<Settings, SetError> {
    let mut s = *base;
    let mut seen: u16 = 0;
    let mut i = 0;
    while i < args.len() {
        let k = args[i];
        let size = match k {
            key::FREQ => 4,
            key::SF | key::BW | key::CR | key::POWER | key::AIR => 1,
            key::SYNC | key::PREAMBLE => 2,
            _ => return Err(SetError::BadArgs(k)),
        };
        let v = args.get(i + 1..i + 1 + size).ok_or(SetError::BadArgs(0))?;
        if seen & (1 << k) != 0 {
            return Err(SetError::BadArgs(k));
        }
        seen |= 1 << k;
        match k {
            key::FREQ => s.freq_hz = u32::from_be_bytes([v[0], v[1], v[2], v[3]]),
            key::SF => s.sf = v[0],
            key::BW => s.bw = Bandwidth::from_code(v[0]).ok_or(SetError::OutOfRange(k))?,
            key::CR => s.cr = v[0],
            key::POWER => s.tx_power_dbm = v[0] as i8,
            key::SYNC => s.sync_word = u16::from_be_bytes([v[0], v[1]]),
            key::PREAMBLE => s.preamble = u16::from_be_bytes([v[0], v[1]]),
            _ => {
                if v[0] > 1 {
                    return Err(SetError::OutOfRange(k));
                }
                s.air_mode = v[0] == 1;
            }
        }
        i += 1 + size;
    }
    s.validate(limits).map_err(SetError::OutOfRange)?;
    Ok(s)
}

/// Expand an SX127x style one byte sync word (0x12, 0x34) to the SX126x
/// register value (0x1424, 0x3444).
pub fn sync_from_byte(b: u8) -> u16 {
    let hi = (b & 0xF0) | 0x04;
    let lo = ((b & 0x0F) << 4) | 0x04;
    u16::from_be_bytes([hi, lo])
}

#[cfg(test)]
mod tests {
    use super::*;

    const L: Limits = Limits::HELTEC_V3;

    #[test]
    fn lab_profile_is_valid_and_low_power() {
        let s = Settings::LAB_SF7_BW125;
        assert_eq!(s.validate(&L), Ok(()));
        assert_eq!(s.validate(&Limits::SX1262), Ok(()));
        assert_eq!((s.sf, s.cr, s.preamble), (7, 5, 8));
        assert_eq!(s.bw.hz(), 125_000);
        assert!((0..=2).contains(&s.tx_power_dbm));
        assert!(!s.air_mode);
    }

    #[test]
    fn bytes_round_trip() {
        let mut s = Settings::LAB_SF7_BW125;
        s.tx_power_dbm = -9;
        s.air_mode = true;
        s.bw = Bandwidth::Khz500;
        assert_eq!(Settings::from_bytes(&s.to_bytes()), Some(s));
        assert_eq!(Settings::from_bytes(&s.to_bytes()[..12]), None);
        let mut b = s.to_bytes();
        b[5] = 10;
        assert_eq!(Settings::from_bytes(&b), None);
    }

    #[test]
    fn limits_are_enforced_at_both_edges() {
        let ok = Settings::LAB_SF7_BW125;
        let t = |f: &dyn Fn(&mut Settings), want: Result<(), u8>| {
            let mut s = ok;
            f(&mut s);
            assert_eq!(s.validate(&L), want);
        };
        t(&|s| s.freq_hz = 863_000_000, Ok(()));
        t(&|s| s.freq_hz = 928_000_000, Ok(()));
        t(&|s| s.freq_hz = 862_999_999, Err(key::FREQ));
        t(&|s| s.freq_hz = 928_000_001, Err(key::FREQ));
        t(&|s| s.sf = 5, Err(key::PREAMBLE)); // preamble 8 is too short for SF5
        t(
            &|s| {
                s.sf = 5;
                s.preamble = 12
            },
            Ok(()),
        );
        t(&|s| s.sf = 4, Err(key::SF));
        t(&|s| s.sf = 12, Ok(()));
        t(&|s| s.sf = 13, Err(key::SF));
        t(&|s| s.cr = 4, Err(key::CR));
        t(&|s| s.cr = 8, Ok(()));
        t(&|s| s.cr = 9, Err(key::CR));
        t(&|s| s.tx_power_dbm = -9, Ok(()));
        t(&|s| s.tx_power_dbm = -10, Err(key::POWER));
        t(&|s| s.tx_power_dbm = 22, Ok(()));
        t(&|s| s.tx_power_dbm = 23, Err(key::POWER));
        t(&|s| s.preamble = 5, Err(key::PREAMBLE));
        t(&|s| s.preamble = 6, Ok(()));
        // The chip itself reaches further than the board.
        let mut s = ok;
        s.freq_hz = 433_000_000;
        assert_eq!(s.validate(&Limits::SX1262), Ok(()));
        assert_eq!(s.validate(&L), Err(key::FREQ));
    }

    #[test]
    fn air_time_matches_the_semtech_calculator() {
        let s = Settings::LAB_SF7_BW125;
        // 20 bytes: 43 symbols + 12.25 preamble symbols at 1.024 ms = 56.576 ms.
        assert_eq!(s.time_on_air_us(20), 56_576);
        // 255 bytes: 378 symbols + 12.25 = 399.616 ms.
        assert_eq!(s.time_on_air_us(255), 399_616);
        // SF12 / 125 kHz uses low data rate optimization. 10 bytes: 18 symbols
        // of 32.768 ms plus 12.25 preamble symbols.
        let mut slow = s;
        slow.sf = 12;
        assert!(slow.low_data_rate());
        assert!(!s.low_data_rate());
        assert_eq!(slow.time_on_air_us(10), 991_232);
    }

    fn pairs(v: &[u8]) -> Result<Settings, SetError> {
        apply_pairs(&Settings::LAB_SF7_BW125, v, &L)
    }

    #[test]
    fn set_applies_a_list_atomically() {
        let s = pairs(&[key::SF, 9, key::BW, 8, key::POWER, 0xFE]).unwrap();
        assert_eq!((s.sf, s.bw, s.tx_power_dbm), (9, Bandwidth::Khz250, -2));
        assert_eq!(s.freq_hz, 915_000_000);
        let s = pairs(&[key::FREQ, 0x36, 0x6B, 0x46, 0x40]).unwrap(); // 913_000_000
        assert_eq!(s.freq_hz, 913_000_000);
        assert_eq!(pairs(&[key::SYNC, 0x34, 0x44]).unwrap().sync_word, 0x3444);
        assert!(pairs(&[key::AIR, 1]).unwrap().air_mode);
        assert_eq!(pairs(&[]).unwrap(), Settings::LAB_SF7_BW125);
    }

    #[test]
    fn set_refuses_bad_lists() {
        assert_eq!(pairs(&[key::SF, 13]), Err(SetError::OutOfRange(key::SF)));
        assert_eq!(pairs(&[key::BW, 10]), Err(SetError::OutOfRange(key::BW)));
        assert_eq!(pairs(&[key::AIR, 2]), Err(SetError::OutOfRange(key::AIR)));
        assert_eq!(pairs(&[0x77, 1]), Err(SetError::BadArgs(0x77)));
        assert_eq!(pairs(&[key::FREQ, 1, 2]), Err(SetError::BadArgs(0)));
        assert_eq!(pairs(&[key::SF]), Err(SetError::BadArgs(0)));
        assert_eq!(
            pairs(&[key::SF, 8, key::SF, 9]),
            Err(SetError::BadArgs(key::SF))
        );
        // Cross check after merging: SF6 alone fails because of the preamble.
        assert_eq!(
            pairs(&[key::SF, 6]),
            Err(SetError::OutOfRange(key::PREAMBLE))
        );
        assert!(pairs(&[key::SF, 6, key::PREAMBLE, 0, 12]).is_ok());
        // One bad pair rejects the whole list.
        assert!(pairs(&[key::SF, 9, key::POWER, 30]).is_err());
    }

    #[test]
    fn sync_word_bytes() {
        assert_eq!(sync_from_byte(0x12), 0x1424);
        assert_eq!(sync_from_byte(0x34), 0x3444);
        assert_eq!(sync_from_byte(0x2B), 0x24B4);
    }
}
