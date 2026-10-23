//! Counters. All wrap; the host should take differences.

/// Size of [`Stats::to_bytes`].
pub const STATS_LEN: usize = 76;

/// Radio state codes in [`Stats::radio_state`].
pub mod state {
    /// The radio failed and is being restarted.
    pub const FAULT: u8 = 0;
    /// Listening.
    pub const RX: u8 = 1;
    /// Transmitting.
    pub const TX: u8 = 2;
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Stats {
    /// Packets sent on the air.
    pub tx_packets: u32,
    pub tx_bytes: u32,
    /// Packets received with a good CRC.
    pub rx_packets: u32,
    pub rx_bytes: u32,
    /// Packets received with a bad CRC.
    pub rx_crc_err: u32,
    /// LoRa headers that failed their check.
    pub rx_header_err: u32,
    /// Received packets with no payload (not carried).
    pub rx_empty: u32,
    /// Frames reported in air mode.
    pub air_frames: u32,
    /// Packets from the host dropped because the TX queue was full.
    pub tx_queue_drop: u32,
    /// Packets from the host over 255 bytes.
    pub tx_oversize: u32,
    /// Transmissions the radio never finished.
    pub tx_timeouts: u32,
    /// Transmissions cut short by a settings change.
    pub tx_aborted: u32,
    /// Radio calls that failed.
    pub radio_errors: u32,
    /// Frames for the host dropped because the serial output was full.
    pub serial_out_drop: u32,
    /// Bad frames on the serial input (all reasons).
    pub serial_bad_frame: u32,
    /// Bytes lost because the serial input overran.
    pub serial_overrun: u32,
    /// Milliseconds since boot.
    pub uptime_ms: u32,
    pub queue_len: u8,
    pub queue_cap: u8,
    /// See [`state`].
    pub radio_state: u8,
    /// Is the `last_*` data valid (a packet has been heard)?
    pub last_valid: u8,
    /// RSSI of the last packet, in quarter dB (dBm times 4).
    pub last_rssi_qdb: i16,
    /// SNR of the last packet, in quarter dB.
    pub last_snr_qdb: i16,
}

impl Stats {
    /// Wire form: 17 u32 in the field order above, then the four u8, then two i16. Big-endian.
    pub fn to_bytes(&self) -> [u8; STATS_LEN] {
        let words = [
            self.tx_packets,
            self.tx_bytes,
            self.rx_packets,
            self.rx_bytes,
            self.rx_crc_err,
            self.rx_header_err,
            self.rx_empty,
            self.air_frames,
            self.tx_queue_drop,
            self.tx_oversize,
            self.tx_timeouts,
            self.tx_aborted,
            self.radio_errors,
            self.serial_out_drop,
            self.serial_bad_frame,
            self.serial_overrun,
            self.uptime_ms,
        ];
        let mut b = [0u8; STATS_LEN];
        for (i, w) in words.iter().enumerate() {
            b[i * 4..i * 4 + 4].copy_from_slice(&w.to_be_bytes());
        }
        b[68] = self.queue_len;
        b[69] = self.queue_cap;
        b[70] = self.radio_state;
        b[71] = self.last_valid;
        b[72..74].copy_from_slice(&self.last_rssi_qdb.to_be_bytes());
        b[74..76].copy_from_slice(&self.last_snr_qdb.to_be_bytes());
        b
    }

    pub fn from_bytes(b: &[u8]) -> Option<Self> {
        if b.len() < STATS_LEN {
            return None;
        }
        let w = |i: usize| u32::from_be_bytes([b[i * 4], b[i * 4 + 1], b[i * 4 + 2], b[i * 4 + 3]]);
        Some(Stats {
            tx_packets: w(0),
            tx_bytes: w(1),
            rx_packets: w(2),
            rx_bytes: w(3),
            rx_crc_err: w(4),
            rx_header_err: w(5),
            rx_empty: w(6),
            air_frames: w(7),
            tx_queue_drop: w(8),
            tx_oversize: w(9),
            tx_timeouts: w(10),
            tx_aborted: w(11),
            radio_errors: w(12),
            serial_out_drop: w(13),
            serial_bad_frame: w(14),
            serial_overrun: w(15),
            uptime_ms: w(16),
            queue_len: b[68],
            queue_cap: b[69],
            radio_state: b[70],
            last_valid: b[71],
            last_rssi_qdb: i16::from_be_bytes([b[72], b[73]]),
            last_snr_qdb: i16::from_be_bytes([b[74], b[75]]),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trip_and_field_order() {
        let s = Stats {
            tx_packets: 1,
            tx_bytes: 2,
            rx_packets: 3,
            rx_bytes: 4,
            rx_crc_err: 5,
            rx_header_err: 6,
            rx_empty: 7,
            air_frames: 8,
            tx_queue_drop: 9,
            tx_oversize: 10,
            tx_timeouts: 11,
            tx_aborted: 12,
            radio_errors: 13,
            serial_out_drop: 14,
            serial_bad_frame: 15,
            serial_overrun: 16,
            uptime_ms: 0xDEAD_BEEF,
            queue_len: 3,
            queue_cap: 8,
            radio_state: state::TX,
            last_valid: 1,
            last_rssi_qdb: -400,
            last_snr_qdb: -22,
        };
        let b = s.to_bytes();
        assert_eq!(&b[0..4], &[0, 0, 0, 1]);
        assert_eq!(&b[64..68], &[0xDE, 0xAD, 0xBE, 0xEF]);
        assert_eq!(Stats::from_bytes(&b), Some(s));
        assert_eq!(Stats::from_bytes(&b[..75]), None);
    }
}
