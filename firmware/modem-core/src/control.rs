//! The control channel: requests from the host, replies from the modem.
//!
//! Both ride in extended frames ([`crate::frame`]).
//!
//! ```text
//! request  (kind 01)  req_id | cmd | args
//! response (kind 02)  req_id | cmd | status | data
//! air      (kind 03)  flags | t_ms u32 | rssi i16 | snr i16 | signal_rssi i16 | payload
//! ```
//!
//! Numbers are big-endian. The modem answers every request that has at least
//! an id and a command, and echoes both.

use crate::settings::{key, Settings};

pub mod cmd {
    /// Protocol and firmware version, and the board name.
    pub const VERSION: u8 = 0x01;
    /// Read the settings.
    pub const GET: u8 = 0x02;
    /// Change settings: a list of `key value` pairs, applied all or nothing.
    pub const SET: u8 = 0x03;
    /// Read the counters.
    pub const STATS: u8 = 0x04;
    /// Zero the counters.
    pub const CLEAR_STATS: u8 = 0x05;
    /// Echo up to 32 bytes.
    pub const PING: u8 = 0x06;
}

pub mod status {
    pub const OK: u8 = 0;
    pub const UNKNOWN_CMD: u8 = 1;
    /// Malformed arguments. `data` holds the offending key, or nothing.
    pub const BAD_ARGS: u8 = 2;
    /// A value outside the limits. `data` holds the key. Nothing changed.
    pub const OUT_OF_RANGE: u8 = 3;
    /// The radio did not accept the settings. The old ones are back.
    pub const RADIO_ERROR: u8 = 4;
}

/// Longest `PING` echo.
pub const MAX_PING: usize = 32;
/// Largest reply body the modem builds.
pub const MAX_RESPONSE: usize = 3 + 96;

/// Longest `SET` argument list: every key once.
pub const SET_ARGS_MAX: usize = 21;

/// Size of the fixed part of an air frame body.
pub const AIR_HEADER: usize = 11;
/// Air frame flag: the packet's CRC was good.
pub const AIR_CRC_OK: u8 = 0x01;

/// A reply, borrowed from a frame body.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Response<'a> {
    pub req_id: u8,
    pub cmd: u8,
    pub status: u8,
    pub data: &'a [u8],
}

impl<'a> Response<'a> {
    pub fn parse(body: &'a [u8]) -> Option<Self> {
        if body.len() < 3 {
            return None;
        }
        Some(Response {
            req_id: body[0],
            cmd: body[1],
            status: body[2],
            data: &body[3..],
        })
    }
}

/// Write a reply body into `out`. Returns the length.
pub fn build_response(req_id: u8, cmd: u8, status: u8, data: &[u8], out: &mut [u8]) -> usize {
    let n = data.len().min(out.len().saturating_sub(3));
    out[0] = req_id;
    out[1] = cmd;
    out[2] = status;
    out[3..3 + n].copy_from_slice(&data[..n]);
    3 + n
}

/// Write a request body into `out`. Returns the length.
pub fn build_request(req_id: u8, cmd: u8, args: &[u8], out: &mut [u8]) -> usize {
    let n = args.len().min(out.len().saturating_sub(2));
    out[0] = req_id;
    out[1] = cmd;
    out[2..2 + n].copy_from_slice(&args[..n]);
    2 + n
}

/// Build the argument list of a `SET`: only the fields that are `Some`.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct SetArgs {
    pub freq_hz: Option<u32>,
    pub sf: Option<u8>,
    pub bw_code: Option<u8>,
    pub cr: Option<u8>,
    pub tx_power_dbm: Option<i8>,
    pub sync_word: Option<u16>,
    pub preamble: Option<u16>,
    pub air_mode: Option<bool>,
}

impl SetArgs {
    /// Write `key value` pairs into `out` (at most [`SET_ARGS_MAX`] bytes). Returns the length.
    pub fn encode(&self, out: &mut [u8; SET_ARGS_MAX]) -> usize {
        let mut n = 0;
        let mut put = |k: u8, v: &[u8]| {
            out[n] = k;
            out[n + 1..n + 1 + v.len()].copy_from_slice(v);
            n += 1 + v.len();
        };
        if let Some(v) = self.freq_hz {
            put(key::FREQ, &v.to_be_bytes());
        }
        if let Some(v) = self.sf {
            put(key::SF, &[v]);
        }
        if let Some(v) = self.bw_code {
            put(key::BW, &[v]);
        }
        if let Some(v) = self.cr {
            put(key::CR, &[v]);
        }
        if let Some(v) = self.tx_power_dbm {
            put(key::POWER, &[v as u8]);
        }
        if let Some(v) = self.sync_word {
            put(key::SYNC, &v.to_be_bytes());
        }
        if let Some(v) = self.preamble {
            put(key::PREAMBLE, &v.to_be_bytes());
        }
        if let Some(v) = self.air_mode {
            put(key::AIR, &[u8::from(v)]);
        }
        n
    }
}

/// What `VERSION` returns.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Version<'a> {
    pub protocol: u8,
    pub major: u8,
    pub minor: u8,
    pub patch: u8,
    /// Board name, ASCII.
    pub board: &'a [u8],
}

impl<'a> Version<'a> {
    pub fn parse(data: &'a [u8]) -> Option<Self> {
        if data.len() < 4 {
            return None;
        }
        Some(Version {
            protocol: data[0],
            major: data[1],
            minor: data[2],
            patch: data[3],
            board: &data[4..],
        })
    }
}

/// Parse an air frame body: `(flags, t_ms, rssi_qdb, snr_qdb, signal_rssi_qdb, payload)`.
pub fn parse_air(body: &[u8]) -> Option<AirFrame<'_>> {
    if body.len() < AIR_HEADER {
        return None;
    }
    Some(AirFrame {
        crc_ok: body[0] & AIR_CRC_OK != 0,
        t_ms: u32::from_be_bytes([body[1], body[2], body[3], body[4]]),
        rssi_qdb: i16::from_be_bytes([body[5], body[6]]),
        snr_qdb: i16::from_be_bytes([body[7], body[8]]),
        signal_rssi_qdb: i16::from_be_bytes([body[9], body[10]]),
        payload: &body[AIR_HEADER..],
    })
}

/// A frame heard on the air.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AirFrame<'a> {
    pub crc_ok: bool,
    /// Modem uptime in ms when it was received.
    pub t_ms: u32,
    pub rssi_qdb: i16,
    pub snr_qdb: i16,
    pub signal_rssi_qdb: i16,
    pub payload: &'a [u8],
}

/// Write an air frame body. Returns the length.
pub fn build_air(f: &AirFrame<'_>, out: &mut [u8]) -> usize {
    out[0] = if f.crc_ok { AIR_CRC_OK } else { 0 };
    out[1..5].copy_from_slice(&f.t_ms.to_be_bytes());
    out[5..7].copy_from_slice(&f.rssi_qdb.to_be_bytes());
    out[7..9].copy_from_slice(&f.snr_qdb.to_be_bytes());
    out[9..11].copy_from_slice(&f.signal_rssi_qdb.to_be_bytes());
    out[AIR_HEADER..AIR_HEADER + f.payload.len()].copy_from_slice(f.payload);
    AIR_HEADER + f.payload.len()
}

/// Read the settings out of a `GET` or `SET` reply.
pub fn settings_from(data: &[u8]) -> Option<Settings> {
    Settings::from_bytes(data)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn request_and_response_bytes() {
        let mut b = [0u8; 32];
        let n = build_request(7, cmd::PING, &[1, 2, 3], &mut b);
        assert_eq!(&b[..n], &[7, 0x06, 1, 2, 3]);
        let n = build_response(7, cmd::PING, status::OK, &[1, 2, 3], &mut b);
        assert_eq!(&b[..n], &[7, 0x06, 0, 1, 2, 3]);
        let r = Response::parse(&b[..n]).unwrap();
        assert_eq!(
            (r.req_id, r.cmd, r.status, r.data),
            (7, 6, 0, &[1u8, 2, 3][..])
        );
        assert!(Response::parse(&[1, 2]).is_none());
    }

    #[test]
    fn set_args_encode_to_the_documented_bytes() {
        let a = SetArgs {
            freq_hz: Some(915_000_000),
            sf: Some(9),
            bw_code: Some(8),
            cr: Some(6),
            tx_power_dbm: Some(-3),
            sync_word: Some(0x3444),
            preamble: Some(10),
            air_mode: Some(true),
        };
        let mut out = [0u8; SET_ARGS_MAX];
        let n = a.encode(&mut out);
        assert_eq!(
            &out[..n],
            &[
                0x01, 0x36, 0x89, 0xCA, 0xC0, // 915_000_000 = 0x3689CAC0
                0x02, 9, 0x03, 8, 0x04, 6, 0x05, 0xFD, 0x06, 0x34, 0x44, 0x07, 0, 10, 0x08, 1
            ]
        );
        assert_eq!(n, SET_ARGS_MAX);
        let none = SetArgs::default().encode(&mut out);
        assert_eq!(none, 0);
    }

    #[test]
    fn air_frame_round_trip() {
        let f = AirFrame {
            crc_ok: false,
            t_ms: 123_456,
            rssi_qdb: -380,
            snr_qdb: -12,
            signal_rssi_qdb: -390,
            payload: &[9, 8, 7],
        };
        let mut b = [0u8; 64];
        let n = build_air(&f, &mut b);
        assert_eq!(n, AIR_HEADER + 3);
        assert_eq!(parse_air(&b[..n]), Some(f));
        assert!(parse_air(&b[..10]).is_none());
    }

    #[test]
    fn version_parse() {
        let v = Version::parse(&[1, 0, 1, 2, b'h', b'v']).unwrap();
        assert_eq!(
            (v.protocol, v.major, v.minor, v.patch, v.board),
            (1, 0, 1, 2, &b"hv"[..])
        );
        assert!(Version::parse(&[1, 2, 3]).is_none());
    }
}
