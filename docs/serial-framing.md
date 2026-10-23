# Serial framing

How `oasa-radio` carries packets over a byte-stream radio (SiK, mLRS, RFD900x, any UART modem). A serial line has no packet edges, so each packet gets a frame.

**The frame is for line errors only.** The length and CRC detect noise and let the receiver find the next frame. They add no security: the link above (oasa-link) authenticates every packet, and a damaged or forged packet that got through would fail there.

## Wire format

```
00 | COBS( len | packet | crc ) | 00
```

| Part | Size | What |
|---|---|---|
| `00` | 1 | Opening delimiter. Ends any noise before the frame |
| `len` | 2 | Packet length, big-endian, 1 to MTU |
| `packet` | 1 to MTU | The bytes oasa-link gave us, untouched |
| `crc` | 2 | CRC-16 over `len` and `packet`, big-endian |
| COBS | | Rewrites `len, packet, crc` with no zero bytes ([COBS](https://en.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing)) |
| `00` | 1 | Closing delimiter |

- **CRC-16/CCITT-FALSE:** polynomial `0x1021`, initial value `0xFFFF`, no reflection, no final XOR. Check: CRC of `"123456789"` is `0x29B1`.
- **COBS:** the standard algorithm. Zero never appears inside a frame, so a zero byte always means a frame edge.
- **Empty packets** are not carried.
- **MTU:** `--mtu`, default **252** (the SiK packet size), at most 65535. Both ends should use the same value; a receiver drops frames longer than its own.

Example: packet `01 02 03` becomes `00 01 07 03 01 02 03 EB E1 00` (inside the COBS part: `00 03 | 01 02 03 | EB E1`).

## Size on the wire

A packet of `n` bytes takes at most `n + 4 + 1 + (n + 4) / 254 + 2` bytes (integer division). At the default MTU of 252 that is 260 bytes. Two delimiters, four bytes of length and CRC, one or two COBS bytes.

## Receiving

Collect bytes until a `00`. Then:

1. Empty? Ignore it (back-to-back delimiters are normal).
2. Too long for the MTU (it kept growing past the longest legal frame)? Drop it: `oversize`.
3. COBS decode fails? Drop it: `bad_cobs`.
4. Shorter than 5 bytes, or `len` doesn't match the size? Drop it: `bad_length`.
5. CRC doesn't match? Drop it: `bad_crc`.
6. `len` over the MTU? Drop it: `oversize`.
7. Otherwise deliver `packet`.

Every drop adds one to the `bad_frame` count. The receiver then starts over at the next byte, so one bad frame never damages the one after it. Memory stays bounded: a long run of bytes with no `00` is thrown away as it arrives.

## What is and isn't promised

- A delivered packet is almost always the packet that was sent. CRC-16 catches every single-bit error and every burst up to 16 bits, and about 65,535 in 65,536 other changes. The rest is caught by oasa-link's authentication.
- Loss, damage and duplicates are possible on a radio; packets are not retried here. Order is kept.
- A flipped bit that turns a delimiter into data merges two frames, and both are lost. A flipped bit that makes a zero inside a frame splits it; both halves are dropped.
- Noise between frames costs nothing but a `bad_frame` count, because the next frame opens with its own `00`.

Tested in `src/framing.rs` (round trips, known vectors, every truncation, every single-bit flip, noise between frames, seeded random damage) and with real pseudo-terminals in `tests/serial_pty.rs`.
