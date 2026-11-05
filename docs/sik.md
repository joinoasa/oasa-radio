# SiK radios

How to set up a SiK telemetry pair (Holybro SiK V3, 3DR, RFD900x and other SiK radios) for `oasa-radio bridge`, and how the bridge checks the radio before it starts. This is the fix for H1 in the [R-COM1 security review](security-review-r-com1.md).

**Status:** written and tested against a simulated SiK radio on a pseudo-terminal (`tests/sik_pty.rs`) and the lab's simulated bench (`l2-sik-link`). Not yet run on a real radio.

## Why a check

SiK radios link to any radio with the same NETID and air settings. There is no authentication between radios. A linked radio can send remote AT commands (`RT...`), for example `RTS3=99` (a new NETID) and `RT&W` (save), and ours runs them ([`tdm.c`](https://github.com/ArduPilot/SiK/blob/master/Firmware/radio/tdm.c), `handle_at_command`: the only checks are the `RT` prefix and the length). So anyone in range with a SiK radio can change our NETID, frequency band or power.

Packets stay safe: oasa-link encrypts and authenticates them. What is at risk is the link itself (taken down until someone fixes the radio) and the radio's settings (power or band outside what we intended).

What we do:

1. **A NETID of your own per pair** (below). Not a defence: fewer accidents with other people's radios.
2. **The bridge reads the radio's settings before it starts** (`--sik-expect`) and refuses to run on a mismatch. A change made while the bridge was down, or one that was saved and took effect after a reboot, is caught at the next start.
3. Radios over cable or in the shielded box for bench tests, as always.

What this does not do: SiK has no setting that turns off remote commands, so a change made **while the bridge runs** is only seen at the next start (or as a link that stops). The bridge never sends AT commands while it carries packets. Our own LoRa firmware (`firmware/heltec-v3`) has no remote settings at all.

## Set up a pair

Do this once for each radio, one at a time, with nothing else using the port. Use the same values on both radios of a pair.

```
NETID=$(shuf -i 1-65535 -n 1)          # one random number per pair; write it down
oasa-radio sik show --device /dev/ttyUSB0
oasa-radio sik set  --device /dev/ttyUSB0 --netid $NETID --mavlink 0 --air-speed 64 --ecc 0
# the second radio:
oasa-radio sik set  --device /dev/ttyUSB1 --netid $NETID --mavlink 0 --air-speed 64 --ecc 0
```

`sik set` changes the settings (`ATSn=v`), saves them (`AT&W`), reboots the radio (`ATZ`), reads them back and prints both, as one JSON line. It finds each setting's register number from the radio's own `ATI5` list, so firmware with extra settings works.

| Setting | For the bridge | Why |
|---|---|---|
| `MAVLINK` | **0** | No MAVLink framing: the radio moves our bytes as they come. With 1 or 2 it looks for MAVLink frames in them, which oasa-link's packets are not |
| `NETID` | a random number from 1 to 65535, the same on both radios, a new one per pair | Not the default 25. NETID is not security: it is on the air, and there are only 65536 |
| `AIR_SPEED` | 64 (the default) | The lab's `sik-64k` profile |
| `ECC` | 0 (the default) | ArduPilot no longer recommends it; it halves the data rate. `sik-64k-ecc` models it |
| `SERIAL_SPEED` | 57 (57600 baud, the default) | Give the bridge the same `--baud` |
| `TXPOWER` | 20 dBm default; lower it on the bench | Over cable, keep 60 dB or more between the radios |
| others | the defaults, the same on both radios | `MIN_FREQ`, `MAX_FREQ`, `NUM_CHANNELS`, `LBT_RSSI`, `MAX_WINDOW` must match for the radios to link |

Set the band and power for your country (`MIN_FREQ`, `MAX_FREQ`, `NUM_CHANNELS`, `TXPOWER`). That is your responsibility; the code does not know the rules where you are.

## Run the bridge with the check

```
oasa-radio bridge --radio serial --device /dev/ttyUSB0 --baud 57600 --mtu 242 \
  --sik-expect netid=$NETID,air_speed=64,ecc=0,mavlink=0,txpower=20,min_freq=915000,max_freq=928000,num_channels=50 \
  --udp-listen 127.0.0.1:7821 --udp-send 127.0.0.1:7801
```

Before it opens the port for packets the bridge enters command mode, reads `ATI` and `ATI5`, leaves with `ATO` and prints one line:

```
{"ts":"...","event":"sik_check","device":"/dev/ttyUSB0","ok":true,"version":"SiK 2.2 on HM-TRP","settings":{"FORMAT":25,"SERIAL_SPEED":57,...},"mismatch":[]}
```

On a mismatch it prints `"ok":false` with each difference (`{"name":"NETID","want":4242,"got":25}`), says why on stderr, and exits 1 without bridging. List every setting you care about: what is not listed is not checked. `TXPOWER`, `MIN_FREQ`, `MAX_FREQ` and `NUM_CHANNELS` catch a radio pushed outside its intended power or band.

**MTU 242:** a SiK packet carries at most 250 bytes with ECC off ([`tdm.c`](https://github.com/ArduPilot/SiK/blob/master/Firmware/radio/tdm.c)); our framing makes a packet of 242 bytes at most 249 on the wire ([serial-framing.md](serial-framing.md), "Size on the wire"), so one framed packet fits one SiK packet. Larger packets work too (the stream is transparent) but are split over two. Give oasa-link the same `--mtu`.

The check costs about 3 seconds: SiK wants a second of silence before and after `+++`.

## Packet bytes and the AT code

- The check runs only before bridging. Once the bridge has the port, no AT code runs.
- Bytes that arrive from the air while we talk to the radio (the other side may already be sending) are thrown away and never printed. Only lines that look exactly like `Sn:NAME=value` are read; anything else makes the check fail with a count, not the bytes. A test sends marked packets from the other side during the check and looks for them in the output.
- The `+++` escape also goes out on the air (SiK queues every byte it gets in data mode, `serial.c`). The other side's bridge sees three bytes of noise and counts at most one `bad_frame`.
- The bridge itself cannot send `+++`: every write is a whole frame that starts and ends with `00` (see the review).

## SiK facts used

From the ArduPilot [SiK advanced configuration](https://ardupilot.org/copter/docs/common-3dr-radio-advanced-configuration-and-technical-information.html) page and the [SiK firmware](https://github.com/ArduPilot/SiK) (`Firmware/radio/`), read 2026-11-05:

| Fact | Source |
|---|---|
| `+++` with 1 s of silence before and after enters command mode; the radio then prints `OK` | ArduPilot page; `at.c` (`ATP_COUNT_1S` = 100 ticks of the 100 Hz timer, `at_plus_detector`, `at_timer` injects `AT`) |
| Command mode echoes input (upper case), runs a line on CR, drops a line over 16 characters (69 with AES) | `at.c` `at_input`; `at.h` `AT_CMD_MAXLEN` |
| Lines end in CR LF | `at.c` prints `\n`; `serial.c` `putchar` adds `\r` |
| `ATI` version, `ATI5` all settings as `S%u:%s=%lu`, `ATSn=v` set (`OK`/`ERROR`, S0 is read-only), `AT&W` save, `ATZ` reboot (no reply), `ATO` leave (no reply) | ArduPilot page; `at.c` |
| Settings S0 to S15: FORMAT, SERIAL_SPEED, AIR_SPEED, NETID, TXPOWER, ECC, MAVLINK, OPPRESEND, MIN_FREQ, MAX_FREQ, NUM_CHANNELS, DUTY_CYCLE, LBT_RSSI, MANCHESTER, RTSCTS, MAX_WINDOW (S16 ENCRYPTION_LEVEL on AES builds) | `parameters.c` |
| Value rules: MAVLINK 0 to 2, ECC and OPPRESEND 0 or 1, MAX_WINDOW at most 131, AIR_SPEED at most 256 | `parameters.c` `param_check` |
| NETID is 16 bits | `radio.h` `radio_set_network_id(uint16_t)` |
| These must match on both radios: firmware version, AIR_SPEED, MIN_FREQ, MAX_FREQ, NUM_CHANNELS, NETID, ECC, LBT_RSSI, MAX_WINDOW | ArduPilot page |
| `RT` + any command runs on the linked radio (not `RTO`); no authentication | ArduPilot page ("all of these commands, except for ATO, may be used on a connected remote radio"); `tdm.c` `handle_at_command` |
| MAVLINK 0 sends serial bytes as they come ("simple framing") | `packet.c` `packet_get_next` |
| In command mode, data from the air is not passed to the serial port | `tdm.c` (`!at_mode_active`) |

Air speeds: 2, 4, 8, 16, 19, 24, 32, 48, 64, 96, 128, 192, 250 kbps (ArduPilot page). Serial speeds: 1, 2, 4, 9, 19, 38, 57, 115, 230 (`serial.c` `serial_rates`; others fall back to 57600). `oasa-radio` refuses other values. The other value limits in `oasa-radio` (TXPOWER at most 30, NUM_CHANNELS 1 to 50, MAX_WINDOW at least 20) are our own sanity bounds, *est.* How long a reboot takes (we wait 3 s) is an estimate.

## To confirm on the first real radios

- `ATI5` prints exactly the lines above on the Holybro V3 firmware (extra settings are fine).
- Whether `ATI5` ends with `OK` (the code does not wait for one).
- `RT` changes our radio as described, and the bridge refuses to start afterwards (`l2-sik-link`, `test_remote_change_is_caught`).
- `+++` never triggers while bridging.
