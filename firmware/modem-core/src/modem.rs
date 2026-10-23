//! The modem engine: serial frames in, LoRa packets out, and the reverse.
//!
//! The engine owns no hardware. It talks to a [`RadioDriver`] and writes
//! frames for the host to a [`FrameSink`]. The firmware calls four things:
//!
//! - [`Modem::on_serial`] with bytes from the host,
//! - [`Modem::on_irq`] when the radio's interrupt line is high,
//! - [`Modem::on_tick`] now and then (at least every 50 ms),
//! - [`Modem::led_on`] to drive the LED.
//!
//! Time is a wrapping millisecond count (`u32`).
//!
//! Half duplex: the radio listens. A queued packet stops the listening, goes
//! out, and listening starts again. At most `Q` packets wait; more are dropped
//! and counted.

use crate::control::{self, cmd, status, AirFrame, MAX_PING, MAX_RESPONSE};
use crate::frame::{self, Decoder, Frame, KIND_AIR, KIND_REQUEST, KIND_RESPONSE, MAX_LORA_PAYLOAD};
use crate::queue::TxQueue;
use crate::settings::{apply_pairs, Limits, SetError, Settings};
use crate::stats::{state, Stats};

/// SX126x interrupt flags the modem uses.
pub mod irq {
    pub const TX_DONE: u16 = 0x0001;
    pub const RX_DONE: u16 = 0x0002;
    pub const HEADER_ERR: u16 = 0x0020;
    pub const CRC_ERR: u16 = 0x0040;
    pub const TIMEOUT: u16 = 0x0200;
    /// All the flags above.
    pub const ALL: u16 = TX_DONE | RX_DONE | HEADER_ERR | CRC_ERR | TIMEOUT;
}

/// A radio call failed. The text is for debugging only.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RadioFault(pub &'static str);

/// What the radio knows about a received packet.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct RxMeta {
    pub len: usize,
    /// Quarter dB (dBm times 4).
    pub rssi_qdb: i16,
    /// Quarter dB.
    pub snr_qdb: i16,
    pub signal_rssi_qdb: i16,
}

/// The radio, as the engine sees it. The SX1262 driver and the test
/// simulation both implement this.
pub trait RadioDriver {
    /// Reset the chip and set up the board (regulator, RF switch, clock).
    fn init(&mut self) -> Result<(), RadioFault>;
    /// Stop whatever the radio is doing and apply all settings.
    fn configure(&mut self, s: &Settings) -> Result<(), RadioFault>;
    /// Listen until a packet arrives, forever.
    fn start_rx(&mut self) -> Result<(), RadioFault>;
    /// Send one packet. The radio gives up after `timeout_ms`.
    fn start_tx(&mut self, payload: &[u8], timeout_ms: u32) -> Result<(), RadioFault>;
    /// Read and clear the interrupt flags ([`irq`]).
    fn irq_status(&mut self) -> Result<u16, RadioFault>;
    /// Read the packet that just arrived.
    fn read_packet(&mut self, buf: &mut [u8; MAX_LORA_PAYLOAD]) -> Result<RxMeta, RadioFault>;
}

/// Where frames for the host go. `false` means there was no room.
pub trait FrameSink {
    fn frame(&mut self, wire: &[u8]) -> bool;
}

/// Who the firmware says it is.
#[derive(Debug, Clone, Copy)]
pub struct Info {
    pub fw: [u8; 3],
    pub board: &'static str,
}

/// How long to wait before restarting a failed radio.
pub const RETRY_MS: u32 = 1000;
/// Scratch for one outgoing wire frame.
const WIRE: usize = 300;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum State {
    Fault { retry_at: u32 },
    Rx,
    Tx { deadline: u32, len: usize },
}

fn reached(now: u32, t: u32) -> bool {
    (now.wrapping_sub(t) as i32) >= 0
}

struct Core<R, const Q: usize> {
    radio: R,
    settings: Settings,
    limits: Limits,
    info: Info,
    stats: Stats,
    txq: TxQueue<Q>,
    state: State,
    flash_until: Option<u32>,
    tx_buf: [u8; MAX_LORA_PAYLOAD],
    rx_buf: [u8; MAX_LORA_PAYLOAD],
    wire: [u8; WIRE],
}

/// The engine. `Q` is the TX queue length.
pub struct Modem<R, const Q: usize> {
    dec: Decoder,
    core: Core<R, Q>,
}

impl<R: RadioDriver, const Q: usize> Modem<R, Q> {
    /// Build the engine. The radio is not touched until [`Modem::start`].
    pub fn new(radio: R, settings: Settings, limits: Limits, info: Info) -> Self {
        Self {
            dec: Decoder::new(),
            core: Core {
                radio,
                settings,
                limits,
                info,
                stats: Stats::default(),
                txq: TxQueue::new(),
                state: State::Fault { retry_at: 0 },
                flash_until: None,
                tx_buf: [0; MAX_LORA_PAYLOAD],
                rx_buf: [0; MAX_LORA_PAYLOAD],
                wire: [0; WIRE],
            },
        }
    }

    /// Bring the radio up and start listening. If that fails the modem keeps
    /// trying every [`RETRY_MS`].
    pub fn start(&mut self, now: u32) {
        self.core.bring_up(now);
    }

    /// Bytes from the host.
    pub fn on_serial<S: FrameSink>(&mut self, bytes: &[u8], now: u32, sink: &mut S) {
        for &b in bytes {
            self.core.stats.serial_bad_frame = self.dec.stats().bad_total();
            match self.dec.push_byte(b) {
                Some(Frame::Data(p)) => self.core.on_data(p),
                Some(Frame::Ext { kind, body }) => self.core.on_ext(kind, body, now, sink),
                None => {}
            }
        }
        self.core.stats.serial_bad_frame = self.dec.stats().bad_total();
        self.core.try_start_tx(now);
    }

    /// The serial port lost `n` bytes (its buffer overran).
    pub fn note_serial_overrun(&mut self, n: u32) {
        let s = &mut self.core.stats;
        s.serial_overrun = s.serial_overrun.wrapping_add(n);
    }

    /// Should the firmware watch the radio's interrupt line? False while the
    /// radio is down, so a stuck line cannot spin the CPU.
    pub fn wants_irq(&self) -> bool {
        !matches!(self.core.state, State::Fault { .. })
    }

    /// The radio's interrupt line is high.
    pub fn on_irq<S: FrameSink>(&mut self, now: u32, sink: &mut S) {
        self.core.on_irq(now, sink);
    }

    /// Time passed. Call at least every 50 ms.
    pub fn on_tick(&mut self, now: u32) {
        self.core.on_tick(now);
    }

    /// Should the LED be on?
    pub fn led_on(&self, now: u32) -> bool {
        match self.core.state {
            State::Fault { .. } => (now / 100) % 2 == 0,
            State::Tx { .. } => true,
            State::Rx => self.core.flash_until.is_some_and(|t| !reached(now, t)) || now % 2000 < 50,
        }
    }

    /// The counters now.
    pub fn stats(&self, now: u32) -> Stats {
        self.core.snapshot(now)
    }

    pub fn settings(&self) -> &Settings {
        &self.core.settings
    }

    /// The radio, for tests.
    pub fn radio(&mut self) -> &mut R {
        &mut self.core.radio
    }

    pub fn is_down(&self) -> bool {
        matches!(self.core.state, State::Fault { .. })
    }
}

impl<R: RadioDriver, const Q: usize> Core<R, Q> {
    fn bump(c: &mut u32) {
        *c = c.wrapping_add(1);
    }

    fn fault(&mut self, now: u32) {
        Self::bump(&mut self.stats.radio_errors);
        self.state = State::Fault {
            retry_at: now.wrapping_add(RETRY_MS),
        };
    }

    fn listen(&mut self, now: u32) {
        match self.radio.start_rx() {
            Ok(()) => self.state = State::Rx,
            Err(_) => self.fault(now),
        }
    }

    fn bring_up(&mut self, now: u32) {
        let s = self.settings;
        if self.radio.init().is_err() || self.radio.configure(&s).is_err() {
            return self.fault(now);
        }
        self.listen(now);
    }

    fn on_data(&mut self, packet: &[u8]) {
        if packet.len() > MAX_LORA_PAYLOAD {
            Self::bump(&mut self.stats.tx_oversize);
        } else if !self.txq.push(packet) {
            Self::bump(&mut self.stats.tx_queue_drop);
        }
    }

    fn try_start_tx(&mut self, now: u32) {
        if self.state != State::Rx {
            return;
        }
        let Some(n) = self.txq.pop_into(&mut self.tx_buf) else {
            return;
        };
        let toa_ms = self.settings.time_on_air_us(n) / 1000;
        let timeout = toa_ms.saturating_mul(2).saturating_add(100);
        match self.radio.start_tx(&self.tx_buf[..n], timeout) {
            Ok(()) => {
                self.state = State::Tx {
                    deadline: now.wrapping_add(timeout).wrapping_add(50),
                    len: n,
                }
            }
            Err(_) => self.fault(now),
        }
    }

    fn on_irq<S: FrameSink>(&mut self, now: u32, sink: &mut S) {
        if matches!(self.state, State::Fault { .. }) {
            return;
        }
        let flags = match self.radio.irq_status() {
            Ok(f) => f,
            Err(_) => return self.fault(now),
        };
        match self.state {
            State::Tx { len, .. } => {
                if flags & irq::TX_DONE != 0 {
                    Self::bump(&mut self.stats.tx_packets);
                    self.stats.tx_bytes = self.stats.tx_bytes.wrapping_add(len as u32);
                    self.listen(now);
                } else if flags & irq::TIMEOUT != 0 {
                    Self::bump(&mut self.stats.tx_timeouts);
                    self.listen(now);
                }
            }
            State::Rx => {
                if flags & irq::HEADER_ERR != 0 {
                    Self::bump(&mut self.stats.rx_header_err);
                }
                if flags & irq::RX_DONE != 0 {
                    self.on_rx(flags & irq::CRC_ERR == 0, now, sink);
                }
            }
            State::Fault { .. } => {}
        }
        self.try_start_tx(now);
    }

    fn on_rx<S: FrameSink>(&mut self, crc_ok: bool, now: u32, sink: &mut S) {
        let meta = match self.radio.read_packet(&mut self.rx_buf) {
            Ok(m) => m,
            Err(_) => return self.fault(now),
        };
        let n = meta.len.min(MAX_LORA_PAYLOAD);
        self.stats.last_valid = 1;
        self.stats.last_rssi_qdb = meta.rssi_qdb;
        self.stats.last_snr_qdb = meta.snr_qdb;
        if crc_ok {
            Self::bump(&mut self.stats.rx_packets);
            self.stats.rx_bytes = self.stats.rx_bytes.wrapping_add(n as u32);
            self.flash_until = Some(now.wrapping_add(60));
        } else {
            Self::bump(&mut self.stats.rx_crc_err);
        }
        if self.settings.air_mode {
            Self::bump(&mut self.stats.air_frames);
            let mut body = [0u8; control::AIR_HEADER + MAX_LORA_PAYLOAD];
            let len = control::build_air(
                &AirFrame {
                    crc_ok,
                    t_ms: now,
                    rssi_qdb: meta.rssi_qdb,
                    snr_qdb: meta.snr_qdb,
                    signal_rssi_qdb: meta.signal_rssi_qdb,
                    payload: &self.rx_buf[..n],
                },
                &mut body,
            );
            self.send_ext(KIND_AIR, &body[..len], sink);
        } else if crc_ok {
            if n == 0 {
                Self::bump(&mut self.stats.rx_empty);
                return;
            }
            match frame::encode_data(&self.rx_buf[..n], &mut self.wire) {
                Ok(w) if sink.frame(&self.wire[..w]) => {}
                _ => Self::bump(&mut self.stats.serial_out_drop),
            }
        }
    }

    fn send_ext<S: FrameSink>(&mut self, kind: u8, body: &[u8], sink: &mut S) {
        match frame::encode_ext(kind, body, &mut self.wire) {
            Ok(w) if sink.frame(&self.wire[..w]) => {}
            _ => Self::bump(&mut self.stats.serial_out_drop),
        }
    }

    fn on_tick(&mut self, now: u32) {
        match self.state {
            State::Fault { retry_at } if reached(now, retry_at) => self.bring_up(now),
            State::Tx { deadline, .. } if reached(now, deadline) => {
                // The radio never said it was done. Start over.
                Self::bump(&mut self.stats.tx_timeouts);
                let s = self.settings;
                if self.radio.configure(&s).is_err() {
                    return self.fault(now);
                }
                self.listen(now);
            }
            _ => {}
        }
        self.try_start_tx(now);
    }

    fn snapshot(&self, now: u32) -> Stats {
        let mut s = self.stats;
        s.uptime_ms = now;
        s.queue_len = self.txq.len() as u8; // Q is small
        s.queue_cap = self.txq.capacity() as u8;
        s.radio_state = match self.state {
            State::Fault { .. } => state::FAULT,
            State::Rx => state::RX,
            State::Tx { .. } => state::TX,
        };
        s
    }

    /// Apply new settings to the radio. `false` if it refused (old ones restored).
    fn apply(&mut self, new: Settings, now: u32) -> bool {
        if matches!(self.state, State::Fault { .. }) {
            // Taken when the radio comes back.
            self.settings = new;
            return true;
        }
        if matches!(self.state, State::Tx { .. }) {
            Self::bump(&mut self.stats.tx_aborted);
        }
        if self.radio.configure(&new).is_ok() && self.radio.start_rx().is_ok() {
            self.settings = new;
            self.state = State::Rx;
            return true;
        }
        Self::bump(&mut self.stats.radio_errors);
        let old = self.settings;
        if self.radio.configure(&old).is_ok() {
            self.listen(now);
        } else {
            self.fault(now);
        }
        false
    }

    fn on_ext<S: FrameSink>(&mut self, kind: u8, body: &[u8], now: u32, sink: &mut S) {
        if kind != KIND_REQUEST || body.len() < 2 {
            return;
        }
        let (id, c, args) = (body[0], body[1], &body[2..]);
        let mut data = [0u8; 96];
        let mut n = 0;
        let put = |d: &mut [u8; 96], bytes: &[u8]| {
            d[..bytes.len()].copy_from_slice(bytes);
            bytes.len()
        };
        let st = match c {
            cmd::VERSION => {
                let board = self.info.board.as_bytes();
                let board = &board[..board.len().min(32)];
                let head = [
                    crate::PROTOCOL_VERSION,
                    self.info.fw[0],
                    self.info.fw[1],
                    self.info.fw[2],
                ];
                n = put(&mut data, &head);
                data[n..n + board.len()].copy_from_slice(board);
                n += board.len();
                status::OK
            }
            cmd::GET => {
                n = put(&mut data, &self.settings.to_bytes());
                status::OK
            }
            cmd::SET => match apply_pairs(&self.settings, args, &self.limits) {
                Ok(new) => {
                    let ok = self.apply(new, now);
                    n = put(&mut data, &self.settings.to_bytes());
                    if ok {
                        status::OK
                    } else {
                        status::RADIO_ERROR
                    }
                }
                Err(SetError::BadArgs(k)) => {
                    n = if k == 0 { 0 } else { put(&mut data, &[k]) };
                    status::BAD_ARGS
                }
                Err(SetError::OutOfRange(k)) => {
                    n = put(&mut data, &[k]);
                    status::OUT_OF_RANGE
                }
            },
            cmd::STATS => {
                n = put(&mut data, &self.snapshot(now).to_bytes());
                status::OK
            }
            cmd::CLEAR_STATS => {
                self.stats = Stats::default();
                status::OK
            }
            cmd::PING if args.len() <= MAX_PING => {
                n = put(&mut data, args);
                status::OK
            }
            cmd::PING => status::BAD_ARGS,
            _ => status::UNKNOWN_CMD,
        };
        let mut out = [0u8; MAX_RESPONSE];
        let len = control::build_response(id, c, st, &data[..n], &mut out);
        self.send_ext(KIND_RESPONSE, &out[..len], sink);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::control::{parse_air, Response, SetArgs, Version};
    use crate::frame::{encode_data, encode_ext, MAX_WIRE};
    use crate::settings::{key, Bandwidth};
    use crate::sim::{Air, Fault, SimRadio};
    use std::vec::Vec;

    #[derive(Default)]
    struct Sink {
        frames: Vec<Vec<u8>>,
        room: Option<usize>,
    }
    impl FrameSink for Sink {
        fn frame(&mut self, wire: &[u8]) -> bool {
            if self.room.is_some_and(|r| self.frames.len() >= r) {
                return false;
            }
            self.frames.push(wire.to_vec());
            true
        }
    }

    #[derive(Debug, PartialEq, Eq, Clone)]
    enum Out {
        Data(Vec<u8>),
        Resp(u8, u8, u8, Vec<u8>),
        Air(bool, Vec<u8>),
    }

    /// What the host would read from the modem's serial port.
    fn decode(sink: &mut Sink) -> Vec<Out> {
        let mut d = Decoder::new();
        let mut v = Vec::new();
        for f in sink.frames.drain(..) {
            for b in f {
                match d.push_byte(b) {
                    Some(Frame::Data(p)) => v.push(Out::Data(p.to_vec())),
                    Some(Frame::Ext { kind, body }) if kind == KIND_RESPONSE => {
                        let r = Response::parse(body).unwrap();
                        v.push(Out::Resp(r.req_id, r.cmd, r.status, r.data.to_vec()));
                    }
                    Some(Frame::Ext { kind, body }) if kind == KIND_AIR => {
                        let a = parse_air(body).unwrap();
                        v.push(Out::Air(a.crc_ok, a.payload.to_vec()));
                    }
                    Some(other) => panic!("unexpected {other:?}"),
                    None => {}
                }
            }
        }
        assert_eq!(d.stats().bad_total(), 0);
        v
    }

    fn data(p: &[u8]) -> Vec<u8> {
        let mut o = std::vec![0u8; MAX_WIRE];
        let n = encode_data(p, &mut o).unwrap();
        o.truncate(n);
        o
    }

    fn request(id: u8, c: u8, args: &[u8]) -> Vec<u8> {
        let mut body = std::vec![0u8; 2 + args.len()];
        control::build_request(id, c, args, &mut body);
        let mut o = std::vec![0u8; MAX_WIRE];
        let n = encode_ext(KIND_REQUEST, &body, &mut o).unwrap();
        o.truncate(n);
        o
    }

    const INFO: Info = Info {
        fw: [0, 1, 2],
        board: "test-board",
    };
    type M = Modem<SimRadio, 4>;

    struct Rig {
        a: M,
        b: M,
        sa: Sink,
        sb: Sink,
        now: u32,
        air: Air,
    }

    impl Rig {
        fn new() -> Self {
            let air = Air::new();
            let mk = || {
                let mut m: M = Modem::new(
                    air.radio(),
                    Settings::LAB_SF7_BW125,
                    Limits::HELTEC_V3,
                    INFO,
                );
                m.start(0);
                m
            };
            let (a, b) = (mk(), mk());
            Rig {
                a,
                b,
                sa: Sink::default(),
                sb: Sink::default(),
                now: 0,
                air,
            }
        }

        /// Run interrupts until both radios are quiet.
        fn pump(&mut self) {
            for _ in 0..1000 {
                let mut any = false;
                if self.a.wants_irq() && self.a.radio().irq_pending() {
                    self.a.on_irq(self.now, &mut self.sa);
                    any = true;
                }
                if self.b.wants_irq() && self.b.radio().irq_pending() {
                    self.b.on_irq(self.now, &mut self.sb);
                    any = true;
                }
                if !any {
                    return;
                }
            }
            panic!("radios never went quiet");
        }

        fn a_in(&mut self, bytes: &[u8]) {
            self.a.on_serial(bytes, self.now, &mut self.sa);
            self.pump();
        }
    }

    #[test]
    fn starts_listening_with_the_lab_profile() {
        let mut r = Rig::new();
        assert!(r.a.radio().is_listening());
        assert_eq!(r.a.radio().configures(), 1);
        assert_eq!(r.a.settings(), &Settings::LAB_SF7_BW125);
        assert!(!r.a.is_down());
    }

    #[test]
    fn a_packet_goes_across_and_arrives_unchanged() {
        let mut r = Rig::new();
        let p: Vec<u8> = (0..=254u8).collect(); // 255 bytes, zeros included
        r.a_in(&data(&p));
        assert_eq!(r.a.radio().sent(), std::slice::from_ref(&p));
        assert_eq!(decode(&mut r.sb), [Out::Data(p)]);
        assert!(decode(&mut r.sa).is_empty());
        let (sa, sb) = (r.a.stats(10), r.b.stats(10));
        assert_eq!((sa.tx_packets, sa.tx_bytes), (1, 255));
        assert_eq!((sb.rx_packets, sb.rx_bytes), (1, 255));
        assert_eq!(
            (sb.last_valid, sb.last_rssi_qdb, sb.last_snr_qdb),
            (1, -400, 36)
        );
        // And back.
        r.b.on_serial(&data(&[7, 7]), 0, &mut r.sb);
        r.pump();
        assert_eq!(decode(&mut r.sa), [Out::Data(std::vec![7, 7])]);
    }

    #[test]
    fn full_queue_drops_and_counts_and_the_rest_arrive_in_order() {
        let mut r = Rig::new();
        let mut burst = Vec::new();
        for i in 0..10u8 {
            burst.extend(data(&[i, i]));
        }
        r.a_in(&burst);
        let got = decode(&mut r.sb);
        let want: Vec<Out> = (0..4u8).map(|i| Out::Data(std::vec![i, i])).collect();
        assert_eq!(got, want);
        let s = r.a.stats(0);
        assert_eq!(
            (s.tx_queue_drop, s.tx_packets, s.queue_len, s.queue_cap),
            (6, 4, 0, 4)
        );
    }

    #[test]
    fn over_255_bytes_is_refused_and_counted() {
        let mut r = Rig::new();
        r.a_in(&data(&[1; 256]));
        r.a_in(&data(&[1; 512]));
        assert!(r.a.radio().sent().is_empty());
        assert_eq!(r.a.stats(0).tx_oversize, 2);
        r.a_in(&data(&[1; 255]));
        assert_eq!(r.a.radio().sent().len(), 1);
    }

    #[test]
    fn serial_noise_is_counted_and_ignored() {
        let mut r = Rig::new();
        let mut s = std::vec![0x55u8, 0x66, 0, 1, 2, 3, 0];
        s.extend(data(&[9]));
        r.a_in(&s);
        assert_eq!(decode(&mut r.sb), [Out::Data(std::vec![9])]);
        assert!(r.a.stats(0).serial_bad_frame >= 2);
        r.a.note_serial_overrun(5);
        assert_eq!(r.a.stats(0).serial_overrun, 5);
    }

    fn ask(r: &mut Rig, id: u8, c: u8, args: &[u8]) -> Out {
        r.a_in(&request(id, c, args));
        let mut v = decode(&mut r.sa);
        assert_eq!(v.len(), 1, "one reply");
        v.remove(0)
    }

    #[test]
    fn version_get_ping() {
        let mut r = Rig::new();
        match ask(&mut r, 5, cmd::VERSION, &[]) {
            Out::Resp(5, c, status::OK, d) => {
                assert_eq!(c, cmd::VERSION);
                let v = Version::parse(&d).unwrap();
                assert_eq!((v.protocol, v.major, v.minor, v.patch), (1, 0, 1, 2));
                assert_eq!(v.board, b"test-board");
            }
            o => panic!("{o:?}"),
        }
        match ask(&mut r, 6, cmd::GET, &[]) {
            Out::Resp(6, _, status::OK, d) => {
                assert_eq!(Settings::from_bytes(&d), Some(Settings::LAB_SF7_BW125))
            }
            o => panic!("{o:?}"),
        }
        assert_eq!(
            ask(&mut r, 7, cmd::PING, &[1, 2, 3]),
            Out::Resp(7, cmd::PING, status::OK, std::vec![1, 2, 3])
        );
        assert_eq!(
            ask(&mut r, 8, cmd::PING, &[0; 33]),
            Out::Resp(8, cmd::PING, status::BAD_ARGS, Vec::new())
        );
        assert_eq!(
            ask(&mut r, 9, 0x77, &[]),
            Out::Resp(9, 0x77, status::UNKNOWN_CMD, Vec::new())
        );
        // A request with no command gets no reply (there is nothing to echo).
        let mut o = std::vec![0u8; MAX_WIRE];
        let n = encode_ext(KIND_REQUEST, &[1], &mut o).unwrap();
        r.a_in(&o[..n]);
        assert!(decode(&mut r.sa).is_empty());
    }

    fn set(r: &mut Rig, a: SetArgs) -> Out {
        let mut buf = [0u8; crate::control::SET_ARGS_MAX];
        let n = a.encode(&mut buf);
        ask(r, 1, cmd::SET, &buf[..n])
    }

    #[test]
    fn set_changes_the_radio_and_set_is_atomic() {
        let mut r = Rig::new();
        let out = set(
            &mut r,
            SetArgs {
                sf: Some(9),
                bw_code: Some(Bandwidth::Khz250.code()),
                tx_power_dbm: Some(-3),
                ..Default::default()
            },
        );
        let Out::Resp(_, _, status::OK, d) = out else {
            panic!("{out:?}")
        };
        let s = Settings::from_bytes(&d).unwrap();
        assert_eq!((s.sf, s.bw, s.tx_power_dbm), (9, Bandwidth::Khz250, -3));
        assert_eq!(r.a.settings(), &s);
        assert_eq!(r.a.radio().configures(), 2);
        assert!(r.a.radio().is_listening());
        // The other radio is still on SF7: the two no longer hear each other.
        r.a_in(&data(&[1, 2, 3]));
        assert!(decode(&mut r.sb).is_empty());
        assert_eq!(r.a.stats(0).tx_packets, 1);
        // Out of range: refused, names the key, nothing changed, radio untouched.
        let out = set(
            &mut r,
            SetArgs {
                sf: Some(7),
                freq_hz: Some(433_000_000),
                ..Default::default()
            },
        );
        assert_eq!(
            out,
            Out::Resp(1, cmd::SET, status::OUT_OF_RANGE, std::vec![key::FREQ])
        );
        assert_eq!(r.a.settings(), &s);
        assert_eq!(r.a.radio().configures(), 2);
        // Malformed.
        assert_eq!(
            ask(&mut r, 2, cmd::SET, &[0x66, 1]),
            Out::Resp(2, cmd::SET, status::BAD_ARGS, std::vec![0x66])
        );
        assert_eq!(
            ask(&mut r, 2, cmd::SET, &[key::FREQ, 1]),
            Out::Resp(2, cmd::SET, status::BAD_ARGS, Vec::new())
        );
        // Put B on the same settings and they talk again.
        r.b.on_serial(
            &request(3, cmd::SET, &[key::SF, 9, key::BW, 8, key::POWER, 0xFD]),
            0,
            &mut r.sb,
        );
        decode(&mut r.sb);
        r.a_in(&data(&[4, 5, 6]));
        assert_eq!(decode(&mut r.sb), [Out::Data(std::vec![4, 5, 6])]);
    }

    #[test]
    fn stats_and_clear() {
        let mut r = Rig::new();
        r.a_in(&data(&[1]));
        let Out::Resp(_, _, status::OK, d) = ask(&mut r, 1, cmd::STATS, &[]) else {
            panic!()
        };
        let s = Stats::from_bytes(&d).unwrap();
        assert_eq!((s.tx_packets, s.tx_bytes, s.radio_state), (1, 1, state::RX));
        assert_eq!(
            ask(&mut r, 2, cmd::CLEAR_STATS, &[]),
            Out::Resp(2, cmd::CLEAR_STATS, status::OK, Vec::new())
        );
        assert_eq!(r.a.stats(0).tx_packets, 0);
    }

    #[test]
    fn bad_crc_is_counted_and_not_delivered() {
        let mut r = Rig::new();
        r.air.set_fault(Fault {
            corrupt: 1,
            drop: 0,
        });
        r.a_in(&data(&[1, 2, 3]));
        assert!(decode(&mut r.sb).is_empty(), "a bad packet is not data");
        let s = r.b.stats(0);
        assert_eq!((s.rx_packets, s.rx_crc_err), (0, 1));
        // The next good one is fine.
        r.a_in(&data(&[1, 2, 3]));
        assert_eq!(decode(&mut r.sb), [Out::Data(std::vec![1, 2, 3])]);
    }

    #[test]
    fn air_mode_reports_every_frame_with_its_flag() {
        let mut r = Rig::new();
        r.b.on_serial(&request(1, cmd::SET, &[key::AIR, 1]), 0, &mut r.sb);
        assert_eq!(decode(&mut r.sb).len(), 1);
        r.a_in(&data(&[1, 2, 3]));
        r.air.set_fault(Fault {
            corrupt: 1,
            drop: 0,
        });
        r.a_in(&data(&[4, 5, 6]));
        r.a_in(&data(&[7]));
        let got = decode(&mut r.sb);
        assert_eq!(
            got,
            [
                Out::Air(true, std::vec![1, 2, 3]),
                Out::Air(false, std::vec![4 ^ 0x10, 5, 6]),
                Out::Air(true, std::vec![7]),
            ]
        );
        let s = r.b.stats(0);
        assert_eq!((s.air_frames, s.rx_packets, s.rx_crc_err), (3, 2, 1));
        // An empty frame is also shown.
        r.b.radio().inject(&[], true, -100, 8);
        r.pump();
        assert_eq!(decode(&mut r.sb), [Out::Air(true, Vec::new())]);
        // Back to normal: no more air frames.
        r.b.on_serial(&request(2, cmd::SET, &[key::AIR, 0]), 0, &mut r.sb);
        decode(&mut r.sb);
        r.a_in(&data(&[8]));
        assert_eq!(decode(&mut r.sb), [Out::Data(std::vec![8])]);
    }

    #[test]
    fn header_errors_and_empty_packets_are_counted() {
        let mut r = Rig::new();
        r.a.radio().inject_header_error();
        r.a.radio().inject(&[], true, -90, 0);
        r.pump();
        let s = r.a.stats(0);
        assert_eq!((s.rx_header_err, s.rx_empty, s.rx_packets), (1, 1, 1));
        assert!(decode(&mut r.sa).is_empty());
    }

    #[test]
    fn a_full_serial_output_is_counted() {
        let mut r = Rig::new();
        r.sb.room = Some(1);
        r.a_in(&data(&[1]));
        r.a_in(&data(&[2]));
        assert_eq!(decode(&mut r.sb), [Out::Data(std::vec![1])]);
        assert_eq!(r.b.stats(0).serial_out_drop, 1);
    }

    #[test]
    fn radio_that_fails_to_start_is_retried() {
        let air = Air::new();
        let radio = air.radio();
        radio.fail_init(2);
        let mut m: M = Modem::new(radio, Settings::LAB_SF7_BW125, Limits::HELTEC_V3, INFO);
        let mut sink = Sink::default();
        m.start(0);
        assert!(m.is_down() && !m.wants_irq());
        assert_eq!(m.stats(0).radio_errors, 1);
        assert_eq!(m.stats(0).radio_state, state::FAULT);
        // Queued while down, no loss up to the queue size.
        m.on_serial(&data(&[1]), 10, &mut sink);
        m.on_tick(999);
        assert!(m.is_down());
        m.on_tick(1000);
        assert!(m.is_down(), "second try fails too");
        m.on_tick(2000);
        assert!(!m.is_down());
        assert!(m.wants_irq());
        assert_eq!(m.stats(0).radio_errors, 2);
        // The packet queued while down goes out now.
        assert_eq!(m.radio().sent(), [std::vec![1]]);
    }

    #[test]
    fn tx_that_fails_takes_the_radio_down_then_it_recovers() {
        let mut r = Rig::new();
        r.a.radio().fail_tx(1);
        r.a_in(&data(&[1]));
        assert!(r.a.is_down());
        r.now = 1000;
        r.a.on_tick(r.now);
        assert!(!r.a.is_down());
        assert!(r.a.radio().is_listening());
    }

    #[test]
    fn tx_that_never_finishes_times_out() {
        let mut r = Rig::new();
        r.a.radio().hang_tx(true);
        r.a_in(&data(&[1; 100]));
        assert_eq!(r.a.stats(0).radio_state, state::TX);
        // 100 bytes at SF7 is 167 ms on air; the deadline is 2x plus 150 ms.
        let toa_ms = Settings::LAB_SF7_BW125.time_on_air_us(100) / 1000;
        let deadline = toa_ms * 2 + 100 + 50;
        r.a.on_tick(deadline - 1);
        assert_eq!(r.a.stats(0).tx_timeouts, 0);
        r.a.radio().hang_tx(false);
        r.a.on_tick(deadline);
        let s = r.a.stats(0);
        assert_eq!((s.tx_timeouts, s.radio_state), (1, state::RX));
    }

    #[test]
    fn settings_change_during_tx_aborts_it() {
        let mut r = Rig::new();
        r.a.radio().hang_tx(true);
        r.a_in(&data(&[1; 10]));
        r.a.radio().hang_tx(false);
        let out = ask(&mut r, 1, cmd::SET, &[key::SF, 8]);
        assert!(matches!(out, Out::Resp(_, _, status::OK, _)));
        let s = r.a.stats(0);
        assert_eq!((s.tx_aborted, s.radio_state), (1, state::RX));
    }

    #[test]
    fn led_patterns() {
        let mut r = Rig::new();
        // Idle: a short blip every two seconds.
        assert!(r.a.led_on(2000) && r.a.led_on(2049));
        assert!(!r.a.led_on(2050) && !r.a.led_on(3000));
        // A good packet flashes it.
        r.a_in(&data(&[1]));
        r.now = 5000;
        r.b.radio().inject(&[1], true, -50, 4);
        r.pump();
        // (flash set at now=5000)
        assert!(r.b.led_on(5059));
        assert!(!r.b.led_on(5060));
        // Solid while sending.
        r.a.radio().hang_tx(true);
        r.a_in(&data(&[2]));
        assert!(r.a.led_on(1234) && r.a.led_on(1999));
        // Fast blink when down.
        let air = Air::new();
        let radio = air.radio();
        radio.fail_init(9);
        let mut m: M = Modem::new(radio, Settings::LAB_SF7_BW125, Limits::HELTEC_V3, INFO);
        m.start(0);
        assert!(m.led_on(0) && !m.led_on(100) && m.led_on(200));
    }

    #[test]
    fn time_wraps_without_trouble() {
        assert!(reached(5, 5) && reached(6, 5) && !reached(4, 5));
        assert!(reached(3, u32::MAX - 2)); // 6 ms later, across the wrap
        assert!(!reached(u32::MAX - 5, 2));
    }
}
