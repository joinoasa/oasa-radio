//! CDC-NCM class requests that phones send and the USB library does not
//! answer as needed (USB NCM 1.0, section 6.2).
//!
//! embassy-usb 0.6.0 (`src/class/cdc_ncm/mod.rs`) stalls
//! `SetEthernetPacketFilter`, stalls `GetNtbInputSize`, and accepts
//! `SetNtbInputSize` without reading it. iOS sends the packet filter and
//! input size requests while it sets up the link. The board crate puts
//! [`NcmRequests`] in front of the library's own handler, so these three
//! requests are answered here and every other one still goes to the library.
//!
//! Only class requests to the NCM control interface reach these functions;
//! the board crate checks the request type and recipient.

/// `SetEthernetPacketFilter` (bRequest 0x43). wValue: the filter bits.
pub const SET_ETHERNET_PACKET_FILTER: u8 = 0x43;
/// `GetNtbInputSize` (bRequest 0x85).
pub const GET_NTB_INPUT_SIZE: u8 = 0x85;
/// `SetNtbInputSize` (bRequest 0x86). Data: dwNtbInMaxSize, and optionally
/// wNtbInMaxDatagrams and a reserved word (8 bytes).
pub const SET_NTB_INPUT_SIZE: u8 = 0x86;

/// The smallest IN NTB size a host may choose (NCM 1.0; Linux
/// `USB_CDC_NCM_NTB_MIN_IN_SIZE`).
pub const NTB_MIN_IN_SIZE: u32 = 2048;
/// The IN NTB size embassy-usb 0.6.0 reports in `GetNtbParameters`, so the
/// size until the host sets one.
pub const NTB_IN_DEFAULT_SIZE: u32 = 2048;
/// The largest NTB this firmware sends: one 1514-byte Ethernet frame and
/// the library's 28-byte header. Any host size from [`NTB_MIN_IN_SIZE`] up
/// is at least this.
pub const NTB_IN_SENT_MAX: u32 = 1514 + 28;

/// The answer to one request.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Reply {
    /// Not one of ours: let the next handler (the library) answer.
    Pass,
    /// OUT request accepted.
    Accept,
    /// IN request: this many bytes of the buffer are the answer.
    Data(usize),
    /// Refuse (the USB stack stalls).
    Stall,
}

/// What the host has asked for, for the test page.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Seen {
    /// Last `SetEthernetPacketFilter` value, if any.
    pub packet_filter: Option<u16>,
    /// Last accepted `SetNtbInputSize` value, if any.
    pub ntb_in_size: Option<u32>,
    /// wNtbInMaxDatagrams from an 8-byte `SetNtbInputSize` (0 if none).
    pub ntb_in_max_datagrams: u16,
    /// `GetNtbInputSize` requests answered.
    pub get_input_size: u16,
    /// Requests refused here.
    pub stalled: u16,
}

/// Answers the packet filter and input size requests for one NCM function.
#[derive(Clone, Debug)]
pub struct NcmRequests {
    comm_if: u8,
    seen: Seen,
    /// Length of the last `SetNtbInputSize` data (4 or 8), echoed by
    /// `GetNtbInputSize`.
    in_size_len: usize,
}

impl NcmRequests {
    /// `comm_if`: the NCM control (communication) interface number.
    pub const fn new(comm_if: u8) -> Self {
        Self {
            comm_if,
            seen: Seen {
                packet_filter: None,
                ntb_in_size: None,
                ntb_in_max_datagrams: 0,
                get_input_size: 0,
                stalled: 0,
            },
            in_size_len: 4,
        }
    }

    pub fn seen(&self) -> Seen {
        self.seen
    }

    /// The IN NTB size now in force.
    pub fn ntb_in_size(&self) -> u32 {
        self.seen.ntb_in_size.unwrap_or(NTB_IN_DEFAULT_SIZE)
    }

    /// Forget everything (USB reset).
    pub fn reset(&mut self) {
        *self = Self::new(self.comm_if);
    }

    /// A class OUT request to an interface. `index` is wIndex, `data` the
    /// data stage (its length is wLength).
    pub fn control_out(&mut self, request: u8, value: u16, index: u16, data: &[u8]) -> Reply {
        if index != u16::from(self.comm_if) {
            return Reply::Pass;
        }
        match request {
            SET_ETHERNET_PACKET_FILTER => {
                // No data stage. The firmware does not filter: it only ever
                // sends frames addressed to the host or broadcast, which every
                // filter value the host may set lets through.
                if !data.is_empty() {
                    return self.stall();
                }
                self.seen.packet_filter = Some(value);
                Reply::Accept
            }
            SET_NTB_INPUT_SIZE => {
                if data.len() != 4 && data.len() != 8 {
                    return self.stall();
                }
                let size = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
                if size < NTB_MIN_IN_SIZE {
                    return self.stall();
                }
                self.seen.ntb_in_size = Some(size);
                self.seen.ntb_in_max_datagrams = if data.len() == 8 {
                    u16::from_le_bytes([data[4], data[5]])
                } else {
                    0
                };
                self.in_size_len = data.len();
                Reply::Accept
            }
            _ => Reply::Pass,
        }
    }

    /// A class IN request to an interface. `length` is wLength; the answer is
    /// written to `buf`.
    pub fn control_in(&mut self, request: u8, index: u16, length: u16, buf: &mut [u8]) -> Reply {
        if index != u16::from(self.comm_if) || request != GET_NTB_INPUT_SIZE {
            return Reply::Pass;
        }
        let mut answer = [0u8; 8];
        answer[..4].copy_from_slice(&self.ntb_in_size().to_le_bytes());
        answer[4..6].copy_from_slice(&self.seen.ntb_in_max_datagrams.to_le_bytes());
        let n = self.in_size_len.min(usize::from(length));
        if buf.len() < n {
            return self.stall();
        }
        buf[..n].copy_from_slice(&answer[..n]);
        self.seen.get_input_size = self.seen.get_input_size.saturating_add(1);
        Reply::Data(n)
    }

    fn stall(&mut self) -> Reply {
        self.seen.stalled = self.seen.stalled.saturating_add(1);
        Reply::Stall
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const IF: u8 = 0;

    #[test]
    fn packet_filter_is_accepted_and_kept() {
        let mut n = NcmRequests::new(IF);
        // Linux and macOS send 0x000F or 0x000E: directed, broadcast, multicast.
        assert_eq!(
            n.control_out(SET_ETHERNET_PACKET_FILTER, 0x000E, 0, &[]),
            Reply::Accept
        );
        assert_eq!(n.seen().packet_filter, Some(0x000E));
        assert_eq!(
            n.control_out(SET_ETHERNET_PACKET_FILTER, 0x000F, 0, &[]),
            Reply::Accept
        );
        assert_eq!(n.seen().packet_filter, Some(0x000F));
    }

    #[test]
    fn packet_filter_with_data_stalls() {
        let mut n = NcmRequests::new(IF);
        assert_eq!(
            n.control_out(SET_ETHERNET_PACKET_FILTER, 0x0F, 0, &[1]),
            Reply::Stall
        );
        assert_eq!(n.seen().packet_filter, None);
        assert_eq!(n.seen().stalled, 1);
    }

    #[test]
    fn input_size_four_bytes() {
        let mut n = NcmRequests::new(IF);
        let data = 16384u32.to_le_bytes();
        assert_eq!(
            n.control_out(SET_NTB_INPUT_SIZE, 0, 0, &data),
            Reply::Accept
        );
        assert_eq!(n.ntb_in_size(), 16384);
        let mut buf = [0xAA; 64];
        assert_eq!(
            n.control_in(GET_NTB_INPUT_SIZE, 0, 4, &mut buf),
            Reply::Data(4)
        );
        assert_eq!(&buf[..4], &data);
        assert_eq!(n.seen().get_input_size, 1);
    }

    #[test]
    fn input_size_eight_bytes() {
        let mut n = NcmRequests::new(IF);
        let data = [0x00, 0x40, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00];
        assert_eq!(
            n.control_out(SET_NTB_INPUT_SIZE, 0, 0, &data),
            Reply::Accept
        );
        assert_eq!(n.ntb_in_size(), 0x4000);
        assert_eq!(n.seen().ntb_in_max_datagrams, 32);
        let mut buf = [0; 64];
        assert_eq!(
            n.control_in(GET_NTB_INPUT_SIZE, 0, 8, &mut buf),
            Reply::Data(8)
        );
        assert_eq!(&buf[..8], &data);
        // A shorter wLength gets a shorter answer.
        assert_eq!(
            n.control_in(GET_NTB_INPUT_SIZE, 0, 4, &mut buf),
            Reply::Data(4)
        );
    }

    #[test]
    fn input_size_default_before_set() {
        let mut n = NcmRequests::new(IF);
        let mut buf = [0; 64];
        assert_eq!(
            n.control_in(GET_NTB_INPUT_SIZE, 0, 4, &mut buf),
            Reply::Data(4)
        );
        assert_eq!(&buf[..4], &NTB_IN_DEFAULT_SIZE.to_le_bytes());
    }

    #[test]
    fn input_size_too_small_or_bad_length_stalls() {
        let mut n = NcmRequests::new(IF);
        assert_eq!(
            n.control_out(SET_NTB_INPUT_SIZE, 0, 0, &2047u32.to_le_bytes()),
            Reply::Stall
        );
        assert_eq!(
            n.control_out(SET_NTB_INPUT_SIZE, 0, 0, &[0, 8]),
            Reply::Stall
        );
        assert_eq!(
            n.control_out(SET_NTB_INPUT_SIZE, 0, 0, &[0; 6]),
            Reply::Stall
        );
        assert_eq!(n.seen().ntb_in_size, None);
        assert_eq!(n.seen().stalled, 3);
        // The smallest allowed size is enough for what the firmware sends.
        const { assert!(NTB_MIN_IN_SIZE >= NTB_IN_SENT_MAX) };
    }

    #[test]
    fn other_requests_and_interfaces_pass() {
        let mut n = NcmRequests::new(IF);
        // SendEncapsulatedCommand and GetNtbParameters stay with the library.
        assert_eq!(n.control_out(0x00, 0, 0, &[]), Reply::Pass);
        let mut buf = [0; 64];
        assert_eq!(n.control_in(0x80, 0, 28, &mut buf), Reply::Pass);
        // Another interface.
        assert_eq!(
            n.control_out(SET_ETHERNET_PACKET_FILTER, 0x0F, 1, &[]),
            Reply::Pass
        );
        assert_eq!(
            n.control_in(GET_NTB_INPUT_SIZE, 1, 4, &mut buf),
            Reply::Pass
        );
        assert_eq!(n.seen(), Seen::default());
    }

    #[test]
    fn reset_forgets() {
        let mut n = NcmRequests::new(IF);
        n.control_out(SET_ETHERNET_PACKET_FILTER, 0x0F, 0, &[]);
        n.control_out(SET_NTB_INPUT_SIZE, 0, 0, &8192u32.to_le_bytes());
        n.reset();
        assert_eq!(n.seen(), Seen::default());
        assert_eq!(n.ntb_in_size(), NTB_IN_DEFAULT_SIZE);
    }
}
