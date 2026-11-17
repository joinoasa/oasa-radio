//! A DHCP server for one client (RFC 2131, options from RFC 2132).
//!
//! It hands out [`PHONE_IP`](crate::net::PHONE_IP) to whoever asks, with a
//! subnet mask and a lease time, and nothing else: **no router (option 3)
//! and no name server (option 6)**, so the phone does not route its internet
//! traffic or DNS through the board.
//!
//! [`reply`] takes the UDP payload of a client message (port 67) and builds
//! the payload to send back. The board sends every reply to
//! 255.255.255.255 port 68: the client has no address yet, and RFC 2131
//! allows a broadcast reply in every case.

use crate::net::{BOARD_IP, NETMASK, PHONE_IP};

pub const SERVER_PORT: u16 = 67;
pub const CLIENT_PORT: u16 = 68;

/// Lease length in seconds (one day).
pub const LEASE_SECS: u32 = 86_400;
/// Every reply is this long or shorter. Old BOOTP clients want at least 300
/// bytes, so replies are padded to 300.
pub const REPLY_LEN: usize = 300;

const OP_REQUEST: u8 = 1;
const OP_REPLY: u8 = 2;
const HTYPE_ETHERNET: u8 = 1;
const MAGIC: [u8; 4] = [99, 130, 83, 99];
const OPTIONS_AT: usize = 240;

const OPT_PAD: u8 = 0;
const OPT_SUBNET_MASK: u8 = 1;
const OPT_REQUESTED_IP: u8 = 50;
const OPT_LEASE_TIME: u8 = 51;
const OPT_MESSAGE_TYPE: u8 = 53;
const OPT_SERVER_ID: u8 = 54;
const OPT_END: u8 = 255;

/// DHCP message types (option 53).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    Discover = 1,
    Offer = 2,
    Request = 3,
    Decline = 4,
    Ack = 5,
    Nak = 6,
    Release = 7,
    Inform = 8,
}

impl Kind {
    fn from_u8(v: u8) -> Option<Self> {
        Some(match v {
            1 => Self::Discover,
            2 => Self::Offer,
            3 => Self::Request,
            4 => Self::Decline,
            5 => Self::Ack,
            6 => Self::Nak,
            7 => Self::Release,
            8 => Self::Inform,
            _ => return None,
        })
    }
}

/// The parts of a client message the server uses.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Request {
    kind: Kind,
    requested_ip: Option<[u8; 4]>,
    server_id: Option<[u8; 4]>,
    ciaddr: [u8; 4],
}

fn ip_at(b: &[u8], at: usize) -> [u8; 4] {
    [b[at], b[at + 1], b[at + 2], b[at + 3]]
}

fn parse(msg: &[u8]) -> Option<Request> {
    if msg.len() < OPTIONS_AT + 1
        || msg[0] != OP_REQUEST
        || msg[1] != HTYPE_ETHERNET
        || msg[2] != 6
        || msg[236..240] != MAGIC
    {
        return None;
    }
    let mut kind = None;
    let mut requested_ip = None;
    let mut server_id = None;
    let mut i = OPTIONS_AT;
    while i < msg.len() {
        let code = msg[i];
        if code == OPT_PAD {
            i += 1;
            continue;
        }
        if code == OPT_END {
            break;
        }
        let len = usize::from(*msg.get(i + 1)?);
        let val = msg.get(i + 2..i + 2 + len)?;
        match (code, len) {
            (OPT_MESSAGE_TYPE, 1) => kind = Kind::from_u8(val[0]),
            (OPT_REQUESTED_IP, 4) => requested_ip = Some(ip_at(val, 0)),
            (OPT_SERVER_ID, 4) => server_id = Some(ip_at(val, 0)),
            _ => {}
        }
        i += 2 + len;
    }
    Some(Request {
        kind: kind?,
        requested_ip,
        server_id,
        ciaddr: ip_at(msg, 12),
    })
}

/// Builds the reply to one client message in `out` (at least [`REPLY_LEN`]
/// bytes). Returns the reply's type and length, or `None` when no reply is
/// due: not a DHCP request, a message for another server, a release or a
/// decline.
pub fn reply(msg: &[u8], out: &mut [u8]) -> Option<(Kind, usize)> {
    let req = parse(msg)?;
    if out.len() < REPLY_LEN {
        return None;
    }
    let kind = match req.kind {
        Kind::Discover => Kind::Offer,
        Kind::Request => {
            if req.server_id.is_some_and(|s| s != BOARD_IP) {
                return None; // the client chose another server
            }
            // Selecting or init-reboot: option 50. Renewing: ciaddr.
            let asked = req.requested_ip.unwrap_or(req.ciaddr);
            if asked == PHONE_IP {
                Kind::Ack
            } else {
                Kind::Nak
            }
        }
        Kind::Inform => Kind::Ack,
        _ => return None,
    };

    let out = &mut out[..REPLY_LEN];
    out.fill(0);
    out[0] = OP_REPLY;
    out[1] = HTYPE_ETHERNET;
    out[2] = 6;
    out[4..8].copy_from_slice(&msg[4..8]); // xid
    out[10..12].copy_from_slice(&msg[10..12]); // flags
    out[24..28].copy_from_slice(&msg[24..28]); // giaddr
    out[28..44].copy_from_slice(&msg[28..44]); // chaddr
    out[236..240].copy_from_slice(&MAGIC);
    let inform = req.kind == Kind::Inform;
    if inform {
        out[12..16].copy_from_slice(&req.ciaddr);
    } else if kind != Kind::Nak {
        out[16..20].copy_from_slice(&PHONE_IP); // yiaddr
    }

    let mut w = OPTIONS_AT;
    let mut opt = |code: u8, val: &[u8]| {
        out[w] = code;
        out[w + 1] = val.len() as u8;
        out[w + 2..w + 2 + val.len()].copy_from_slice(val);
        w += 2 + val.len();
    };
    opt(OPT_MESSAGE_TYPE, &[kind as u8]);
    opt(OPT_SERVER_ID, &BOARD_IP);
    if kind != Kind::Nak {
        if !inform {
            opt(OPT_LEASE_TIME, &LEASE_SECS.to_be_bytes());
        }
        opt(OPT_SUBNET_MASK, &NETMASK);
    }
    // Deliberately no router (3) and no name server (6).
    out[w] = OPT_END;
    Some((kind, REPLY_LEN))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::vec::Vec;

    const MAC: [u8; 6] = [0x88, 0x88, 0x88, 1, 2, 3];
    const XID: [u8; 4] = [0xDE, 0xAD, 0xBE, 0xEF];

    fn client(kind: Kind, extra: &[(u8, &[u8])], ciaddr: [u8; 4]) -> Vec<u8> {
        let mut m = std::vec![0u8; 240];
        m[0] = OP_REQUEST;
        m[1] = HTYPE_ETHERNET;
        m[2] = 6;
        m[4..8].copy_from_slice(&XID);
        m[10] = 0x80; // broadcast flag
        m[12..16].copy_from_slice(&ciaddr);
        m[28..34].copy_from_slice(&MAC);
        m[236..240].copy_from_slice(&MAGIC);
        m.extend_from_slice(&[OPT_MESSAGE_TYPE, 1, kind as u8]);
        for (code, val) in extra {
            m.push(*code);
            m.push(val.len() as u8);
            m.extend_from_slice(val);
        }
        // Parameter request list asking for router and DNS, as phones do.
        m.extend_from_slice(&[55, 4, 1, 3, 6, 15]);
        m.push(OPT_PAD);
        m.push(OPT_END);
        m
    }

    /// Options of a reply as (code, value) pairs.
    fn options(r: &[u8]) -> Vec<(u8, Vec<u8>)> {
        let mut v = Vec::new();
        let mut i = OPTIONS_AT;
        while r[i] != OPT_END {
            let len = r[i + 1] as usize;
            v.push((r[i], r[i + 2..i + 2 + len].to_vec()));
            i += 2 + len;
        }
        v
    }

    fn get(opts: &[(u8, Vec<u8>)], code: u8) -> Option<Vec<u8>> {
        opts.iter()
            .find(|(c, _)| *c == code)
            .map(|(_, v)| v.clone())
    }

    fn check_common(r: &[u8]) {
        assert_eq!(r[0], OP_REPLY);
        assert_eq!(&r[4..8], &XID);
        assert_eq!(r[10], 0x80, "flags copied");
        assert_eq!(&r[28..34], &MAC);
        assert_eq!(&r[236..240], &MAGIC);
        let o = options(r);
        assert_eq!(get(&o, OPT_SERVER_ID).unwrap(), BOARD_IP);
        assert_eq!(get(&o, 3), None, "no router");
        assert_eq!(get(&o, 6), None, "no name server");
        assert_eq!(get(&o, 15), None, "no domain name");
    }

    #[test]
    fn discover_gets_offer() {
        let mut out = [0xFF; 400];
        let (k, n) = reply(&client(Kind::Discover, &[], [0; 4]), &mut out).unwrap();
        assert_eq!((k, n), (Kind::Offer, REPLY_LEN));
        let r = &out[..n];
        check_common(r);
        assert_eq!(&r[16..20], &PHONE_IP);
        let o = options(r);
        assert_eq!(get(&o, OPT_MESSAGE_TYPE).unwrap(), [Kind::Offer as u8]);
        assert_eq!(get(&o, OPT_SUBNET_MASK).unwrap(), NETMASK);
        assert_eq!(get(&o, OPT_LEASE_TIME).unwrap(), LEASE_SECS.to_be_bytes());
    }

    #[test]
    fn request_for_our_address_gets_ack() {
        let mut out = [0; REPLY_LEN];
        let m = client(
            Kind::Request,
            &[(OPT_REQUESTED_IP, &PHONE_IP), (OPT_SERVER_ID, &BOARD_IP)],
            [0; 4],
        );
        let (k, n) = reply(&m, &mut out).unwrap();
        assert_eq!(k, Kind::Ack);
        check_common(&out[..n]);
        assert_eq!(&out[16..20], &PHONE_IP);
        let o = options(&out);
        assert_eq!(get(&o, OPT_MESSAGE_TYPE).unwrap(), [Kind::Ack as u8]);
        assert_eq!(get(&o, OPT_SUBNET_MASK).unwrap(), NETMASK);
    }

    #[test]
    fn renewal_by_ciaddr_gets_ack() {
        let mut out = [0; REPLY_LEN];
        let (k, _) = reply(&client(Kind::Request, &[], PHONE_IP), &mut out).unwrap();
        assert_eq!(k, Kind::Ack);
    }

    #[test]
    fn request_for_another_address_gets_nak() {
        // A phone that last had a different address on this port.
        let mut out = [0; REPLY_LEN];
        let m = client(
            Kind::Request,
            &[(OPT_REQUESTED_IP, &[192, 168, 1, 50])],
            [0; 4],
        );
        let (k, n) = reply(&m, &mut out).unwrap();
        assert_eq!(k, Kind::Nak);
        check_common(&out[..n]);
        assert_eq!(&out[16..20], &[0; 4]);
        let o = options(&out);
        assert_eq!(get(&o, OPT_LEASE_TIME), None);
        assert_eq!(get(&o, OPT_SUBNET_MASK), None);
    }

    #[test]
    fn request_for_another_server_is_ignored() {
        let mut out = [0; REPLY_LEN];
        let m = client(
            Kind::Request,
            &[
                (OPT_REQUESTED_IP, &PHONE_IP),
                (OPT_SERVER_ID, &[10, 0, 0, 1]),
            ],
            [0; 4],
        );
        assert_eq!(reply(&m, &mut out), None);
    }

    #[test]
    fn inform_gets_ack_without_lease() {
        let mut out = [0; REPLY_LEN];
        let (k, _) = reply(&client(Kind::Inform, &[], PHONE_IP), &mut out).unwrap();
        assert_eq!(k, Kind::Ack);
        check_common(&out);
        assert_eq!(&out[12..16], &PHONE_IP);
        assert_eq!(&out[16..20], &[0; 4]);
        assert_eq!(get(&options(&out), OPT_LEASE_TIME), None);
    }

    #[test]
    fn release_and_decline_get_nothing() {
        let mut out = [0; REPLY_LEN];
        assert_eq!(reply(&client(Kind::Release, &[], PHONE_IP), &mut out), None);
        assert_eq!(reply(&client(Kind::Decline, &[], [0; 4]), &mut out), None);
    }

    #[test]
    fn junk_is_ignored() {
        let mut out = [0; REPLY_LEN];
        assert_eq!(reply(&[], &mut out), None);
        assert_eq!(reply(&[0; 300], &mut out), None);
        // A server reply (op 2) is not a request.
        let mut m = client(Kind::Discover, &[], [0; 4]);
        m[0] = OP_REPLY;
        assert_eq!(reply(&m, &mut out), None);
        // No message type.
        let mut m = client(Kind::Discover, &[], [0; 4]);
        m[OPTIONS_AT] = OPT_PAD;
        m[OPTIONS_AT + 1] = OPT_PAD;
        m[OPTIONS_AT + 2] = OPT_PAD;
        assert_eq!(reply(&m, &mut out), None);
        // An option that runs off the end.
        let mut m = client(Kind::Discover, &[], [0; 4]);
        m.truncate(OPTIONS_AT + 3);
        m.extend_from_slice(&[OPT_REQUESTED_IP, 4, 10]);
        assert_eq!(reply(&m, &mut out), None);
        // Output buffer too small.
        let mut small = [0; 100];
        assert_eq!(
            reply(&client(Kind::Discover, &[], [0; 4]), &mut small),
            None
        );
    }

    #[test]
    fn every_truncation_is_safe() {
        let m = client(Kind::Request, &[(OPT_REQUESTED_IP, &PHONE_IP)], [0; 4]);
        let mut out = [0; REPLY_LEN];
        for n in 0..m.len() {
            let _ = reply(&m[..n], &mut out);
        }
    }
}
