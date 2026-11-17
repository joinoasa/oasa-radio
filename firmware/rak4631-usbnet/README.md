# RAK4631 USB network test firmware

Test firmware for the P0 phone USB test (T-MSG1). A RAK4631 with this firmware shows up to a phone as a USB network adapter (CDC-NCM). It gives the phone one address by DHCP and serves one test page. Nothing else.

- **The LoRa radio is never used.** No SPI, no radio pins, no antenna switch power. Nothing is transmitted over the air.
- Board address 10.71.44.1, phone address 10.71.44.2, mask /30 (255.255.255.252).
- DHCP offers **no router and no name server**, so the phone keeps its own internet route and DNS.
- Test page: `http://10.71.44.1/`. It shows what the board saw from the phone: the NCM packet filter and input size requests, and the DHCP exchange.
- No Bluetooth (the SoftDevice in flash is never enabled), no logging, no heap, no secrets, no cryptography. LED1 (green) on means the firmware is running.
- USB ID 1209:0001 is the pid.codes **test ID**, the same as [`../rak4631`](../rak4631/README.md). For development only.

Logic that runs on a PC is in [`../usbnet-core`](../usbnet-core) with host tests (`cargo test -p oasa-usbnet-core` from the repo root): the NCM request answers, the DHCP reply and the page. This crate is the board: clocks, USB, the network stack and sockets.

**Nothing here has run on a board or a phone yet.** See the list at the end.

## Build

Same toolchain and memory layout as [`../rak4631`](../rak4631/README.md) (stable Rust, `thumbv7em-none-eabihf`, application at 0x26000, RAM from 0x20008000, for RAK's stock Adafruit UF2 bootloader).

```
cd firmware/rak4631-usbnet
cargo build --release --locked
cargo clippy --release --locked -- -D warnings
cargo fmt --check
cargo deny --locked --target thumbv7em-none-eabihf check
```

CI builds it in the `firmware build (rak4631-usbnet, thumbv7em)` job. The release build is about 50 kB of flash and 37 kB of static RAM.

## Flash

As for the modem firmware ([`../rak4631/README.md`](../rak4631/README.md), "Flash over USB"), with this crate's file names. Double-press reset for the `RAK4631` drive, then:

```
cargo objcopy --release --locked -- -O ihex rak4631-usbnet.hex
python3 uf2conv.py rak4631-usbnet.hex -c -f 0xADA52840 -o rak4631-usbnet.uf2
cp rak4631-usbnet.uf2 /media/$USER/RAK4631/
```

To go back to the modem firmware, double-press reset and flash that one the same way. Not run here.

## Phone test steps (part 2)

Bench check on Linux first, so a phone failure is not a board failure:

1. Plug the board into the PC. `ip link` shows a new `usb0` or `enx…` interface; `dmesg` shows `cdc_ncm`.
2. Get an address: NetworkManager does it, or `sudo dhclient -v <iface>`. Expect 10.71.44.2/30, no default route through it (`ip route`), no DNS server added.
3. `curl http://10.71.44.1/` returns the page. Packet filter and input size lines show what Linux sent.
4. Capture the USB setup traffic (`usbmon` and Wireshark) for the record.

Each phone (iPhone with USB-C, Android with USB-C):

1. Note the phone model and OS version. Cellular or Wi-Fi on, internet working.
2. Plug the board in with a USB-C data cable. LED1 green.
3. iPhone: Settings shows an Ethernet entry for the adapter; it lists IP 10.71.44.2. Android: tethering/Ethernet settings, or a network info app. Record what is shown.
4. Check the phone's internet still works (open any website). This is the "no router, no DNS" check.
5. Open `http://10.71.44.1/` in the browser. Record the page (screenshot): packet filter value, input size, DHCP counts.
6. Unplug, plug back in, reload the page. Up time restarts only if the board lost power.

Write results down per phone (the owner's). Charging the phone is out of scope.

## USB library differences

The firmware uses `embassy-usb` 0.6.0 (crates.io). Its CDC-NCM class is `src/class/cdc_ncm/mod.rs`. What it does with the requests in USB NCM 1.0 section 6.2 that iPhones rely on, read from that file:

| Request | bRequest | embassy-usb 0.6.0 | This firmware |
|---|---|---|---|
| SetEthernetPacketFilter | 0x43 | **Stalled.** Its constant is commented out (line 47) and the request falls through to `_ => Some(OutResponse::Rejected)` in `control_out` (line 184) | Accepted; the value is kept and shown on the page. No filtering is done: the board only sends frames to the phone or broadcast |
| SetNtbInputSize | 0x86 | **Ignored.** Accepted without reading the data (`// TODO`, lines 180 to 183) | Accepted with 4 or 8 data bytes and a size of at least 2048; kept and shown. Smaller sizes or other lengths stall |
| GetNtbInputSize | 0x85 | **Stalled.** Constant commented out (line 54); `control_in` falls through to `Rejected` (line 217) | Returns the size in force (2048 until the phone sets one), 4 or 8 bytes as last set |

So the test notes ("ignored one and rejected the other") match 0.6.0, with one addition: the read-back request (0x85) is also refused.

**No patch to embassy-usb.** The library asks its handlers in the order they were added and stops at the first that answers (`handle_control_out_delegated` and `handle_control_in_delegated` in `src/lib.rs`, lines 693 to 717; the class adds its own handler at `cdc_ncm/mod.rs` line 335). This firmware adds its own handler (`NcmFix` in `src/main.rs`) *before* creating the NCM class, so it answers these three requests and passes everything else on. The answers are `oasa_usbnet_core::ncm`, with host tests.

Left as the library has them (not changed, noted for part 2):

- `bmNetworkCapabilities` is 0 in the NCM functional descriptor (line 312): no optional requests advertised. Hosts that look at bit 5 then send a 4-byte SetNtbInputSize; 8 bytes are accepted anyway.
- The library reports 2048-byte NTBs in both directions and reads one datagram per NTB (`max_datagram_count: 1`, line 212). The board's largest NTB is 1542 bytes, below any size a host may set (at least 2048).
- A datagram longer than the 1514-byte buffer would panic in `read_packet` (line 497) and so reset the board. Hosts keep to the descriptor's 1514-byte maximum segment size.
- Handler count: the default limit is 4 (`build.rs`); this firmware uses 2.

The phone MAC (sent to the phone in the descriptor) starts 88:88:88, as in embassy's own examples, because embassy notes Android refuses a locally administered one. It is a test value, not an address block we own. The board's own MAC is locally administered (02:…). Both end in bytes of the chip ID.

## Dependencies

The same as [`../rak4631`](../rak4631/README.md), without `embedded-hal` and `embassy-futures`: all MIT OR Apache-2.0, with the same two exceptions (`nrf-pac` BSD-3-Clause, `unicode-ident` Unicode-3.0) and the same `bare-metal` advisory ignore. Added:

- `embassy-net` 0.9.1 (MIT OR Apache-2.0): IPv4, UDP and TCP only. No DHCP client, DNS or IPv6.
- **New exception, needs the owner's approval:** `smoltcp` 0.13.1 and `managed` 0.8.0 (the stack under embassy-net) are **0BSD**, a public-domain-like license with no conditions. It is not on the allowed list. `deny.toml` allows it for these two crates only. There is no CDC-NCM network stack in Rust without smoltcp; the alternative is writing our own TCP.
- `oasa-usbnet-core` (ours) and `oasa-modem-core` (ours, for `hex_id`).

This crate writes no `unsafe` code; `oasa-usbnet-core` forbids it.

## Not run on hardware

All of it, until part 2. In order of first-flash risk:

1. Bootloader hand-off, VTOR, clocks, USB power detect: as for the modem firmware, also not yet run.
2. USB enumeration as CDC-NCM on Linux, iOS and Android.
3. Whether iOS sends the packet filter and input size requests, with what values, and whether this firmware's answers are enough for iOS to bring the link up.
4. Whether the 88:88:88 phone MAC works on each phone.
5. The DHCP exchange with each phone, including broadcast replies, and that no phone takes a route or DNS from the board.
6. The page in each phone's browser, and two connections at once.
7. Throughput, reconnects after unplug, and USB suspend.
