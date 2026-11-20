//! Addresses for the USB link.
//!
//! The link is its own /30: the board is .1, the phone gets .2. No router and
//! no name server are offered, so the phone keeps its own internet route and
//! DNS. 10.71.44.0/30 is a private range picked to be unlikely on home or
//! carrier networks; if a phone's other network uses it, change it here.

/// The board's address.
pub const BOARD_IP: [u8; 4] = [10, 71, 44, 1];
/// The one address handed out.
pub const PHONE_IP: [u8; 4] = [10, 71, 44, 2];
/// /30: only the two addresses above.
pub const PREFIX_LEN: u8 = 30;
pub const NETMASK: [u8; 4] = [255, 255, 255, 252];

/// The board's own MAC (its side of the link): locally administered,
/// unicast, from the chip ID.
pub fn board_mac(chip_id: u64) -> [u8; 6] {
    let b = chip_id.to_le_bytes();
    [0x02, b[0], b[1], b[2], b[3], b[4]]
}

/// The MAC the phone uses for its USB network interface (sent in the NCM
/// descriptor). embassy-usb notes that Android refuses a locally
/// administered host MAC, so this one is not: it uses the 88:88:88 prefix
/// from embassy's own USB Ethernet examples. A test value, not ours to
/// give out: a product needs its own.
pub fn phone_mac(chip_id: u64) -> [u8; 6] {
    let b = chip_id.to_le_bytes();
    [0x88, 0x88, 0x88, b[0], b[1], b[2]]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn macs_are_unicast_and_differ() {
        for id in [0, u64::MAX, 0x0123_4567_89AB_CDEF] {
            let (b, p) = (board_mac(id), phone_mac(id));
            assert_eq!(b[0] & 1, 0, "board MAC must be unicast");
            assert_eq!(p[0] & 1, 0, "phone MAC must be unicast");
            assert_eq!(b[0] & 2, 2, "board MAC is locally administered");
            assert_eq!(p[0] & 2, 0, "phone MAC is not (Android)");
            assert_ne!(b, p);
        }
    }

    #[test]
    fn netmask_matches_prefix() {
        let m = u32::MAX << (32 - PREFIX_LEN);
        assert_eq!(NETMASK, m.to_be_bytes());
        let net = |ip: [u8; 4]| u32::from_be_bytes(ip) & m;
        assert_eq!(net(BOARD_IP), net(PHONE_IP));
        assert_ne!(BOARD_IP, PHONE_IP);
    }
}
