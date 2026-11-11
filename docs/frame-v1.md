# Frame format v1: a protected header

The W0 header (a length byte and its complement) goes out as 16 bare bits. One flipped bit loses the frame. v1 protects the header with a code, so the header is never the weak link. Code: [`waveform/src/frame.rs`](../waveform/src/frame.rs) (layout, CRC-8, header), [`waveform/src/fec.rs`](../waveform/src/fec.rs) (the code). Select it with `ModemConfig::frame = FrameFormat::V1` or `--frame v1` on `oasa-radio waveform`. The default stays W0, bit for bit as before.

## Why

Measured with W0, BPSK, no frequency offset, 64-byte frames, no `expected_len` (the receiver reads the length itself): the preamble detector finds about every frame down to 0 dB Eb/N0, but the header takes 16 bits at the raw bit error rate, so a frame survives with probability `(1 - BER)^16`:

| Eb/N0 | 0 dB | 2 dB | 4 dB | 6 dB |
|---|---|---|---|---|
| W0 found, theory | 27 % | 54 % | 82 % | 96 % |

This is the bench result "frames missed below about 4 dB". The receive report below shows it directly: in W0 runs, `header_rejected` holds nearly all the losses.

## Layout

```
| preamble 63 | sync word 16 | header 152 symbols | payload | CRC-16 16 |
  one bit per symbol          | one bit per symbol | whitened, payload modulation
```

Preamble and sync word are unchanged. Header fields, 24 bits, most significant bit first:

| Field | Bits | Meaning |
|---|---|---|
| `version` | 2 | 0 for this format |
| `extension` | 1 | 1: more header follows. Not defined yet; v1 receivers reject it |
| `modulation` | 2 | 0 BPSK, 1 QPSK, 2 2-FSK, 3 reserved. The modulation of the payload |
| `payload_code` | 3 | 0: none. 1: light error correction, K=7 rate 1/2 and an interleaver over the body ([fec-light.md](fec-light.md)). 2 to 7 are reserved (heavy tier); receivers reject them |
| `pilots` | 1 | Not defined yet; v1 receivers reject 1 |
| `length` | 8 | Payload bytes, 0 to 255 |
| `channel_plan` | 4 | Reserved. Sent as 0, ignored and reported by the receiver |
| `coexistence` | 3 | Reserved flags. Sent as 0, ignored and reported by the receiver |

Fields go out as three bytes (`length` straddles bytes 1 and 2), then the header is built in four steps:

1. **CRC-8** over the three field bytes: polynomial 0x07, initial value 0xFF, no reflection, no final xor. Check values (pinned in unit tests): `crc8(b"123456789") = 0xFB`, `crc8([0, 0, 0]) = 0x2B`. Both match an independent Python implementation.
2. **Scramble** the 32 bits (fields and CRC-8) with PN9 (the whitener W0 uses), so a mostly-zero header is not a run of equal symbols.
3. **Code** them: convolutional, constraint length 7, generators 171 and 133 (octal), rate 1/2, six zero tail bits: (32 + 6) x 2 = 76 coded bits. The encoder is checked against an independent Python implementation.
4. **Send twice.** The 76 coded bits, then the same 76 bits xored with the PN9 sequence (`whiten` of 76 zeros): 152 symbols, one bit per symbol. Effective rate 1/4.

The receiver turns each header symbol into a soft value (PSK: the real part of the matched filter output, positive for bit 0; 2-FSK: energy of the bit 0 tone minus the bit 1 tone), undoes the xor of the second copy, adds the two values of each coded bit, runs a soft Viterbi decoder, unscrambles, and checks the CRC-8. It rejects the frame if the CRC-8 fails, `version` is not 0, `extension` is 1, `pilots` is 1, `payload_code` is not 0 or 1, or `modulation` is not the one the modem is set up for. Rejected frames are counted as `header_rejected`. Then it decodes the body as in W0, or with the light code if `payload_code` is 1.

**Body.** Payload bytes, then CRC-16/CCITT-FALSE over *the three header field bytes and the payload*, whitened with a fresh PN9 run, at the payload modulation. No length or complement bytes. Because the CRC-16 covers the header bytes, a header that decoded to the wrong length (a miscorrection that got past the CRC-8) still fails the CRC-16.

Header symbols are always one bit per symbol, so a QPSK frame has BPSK-like preamble, sync word and header, then QPSK payload. (Eb/N0 refers to payload bits, as in W0. A QPSK header symbol carries twice the energy of a payload bit, so QPSK is 3 dB better placed than BPSK in the tables below.)

## Receiver details worth knowing

* The header's soft values come from the preamble's phase, frequency and amplitude estimate. Once the header decodes, it is known, so the receiver estimates again over preamble, sync word and header (231 symbols instead of 79). This is not in the format; it is there because the body now starts 152 symbols later. Without it a small frequency offset that the preamble alone cannot see (below its noise) turned the phase too far by the start of a long QPSK body: in a test run (40 QPSK frames at 12 dB, offsets within +-0.002 cycles per symbol) 3 frames lost their payload; with it, none. A seeded test (`v1_qpsk_long_body_needs_the_estimate_over_the_header`: 255-byte payloads, offsets within +-0.002) loses 4 of 40 frames when the step is disabled and none with it.
* With `expected_len` set (BER measurement) the header is still decoded, but its checks are skipped and the given length is used. `RxFrame::header` holds the decoded fields when the CRC-8 matched, otherwise `None`, and then `crc_ok` is false. `oasa-radio waveform ber --frame v1` prints `header_ok`: frames whose header decoded to what was sent.
* `RxFrame::header` carries all fields, including `channel_plan` and `coexistence`.

## Receive report

`Modem::demodulate_report(samples, rx)` returns the frames and an `RxReport`. **The counts are of candidates, not of frames.** A candidate is a peak of the detection metric past the threshold. A real frame is one candidate, but its own body adds more (random data correlates with the preamble now and then: about 3 per 64-byte frame at 6 dB) and so does noise. To keep the counts about lost frames, a candidate that lies inside a frame already found is counted as `inside_frame` and not as a rejection.

| Count | Meaning |
|---|---|
| `preambles` | all candidates past the detection threshold |
| `inside_frame` | candidates inside a frame already found, or inside the span a header failure reserves (below), that gave no frame |
| `sync_rejected` | too many wrong bits in the sync word |
| `header_rejected` | v1: CRC-8 or a field rule; W0: the length complement check |
| `too_short` | the frame ran off the end of the buffer |

`preambles = frames + sync_rejected + header_rejected + too_short + inside_frame` (a test checks it, both formats, both paths). How the spans work:

* **v1:** a header that passes its CRC-8 and the field rules gives the length, and the frame's span is reserved for counting whatever its CRC-16 says. With a good CRC-16, candidates before its end are not tried and count as `inside_frame`. With a bad CRC-16 only the preamble, sync word and header are taken as sure: the header is not authenticated, so the length it claims does not hide what follows. Candidates inside the claimed span are tried (a frame found there is returned, and delivered only if its own CRC-16 is good) with a quicker decode (no body phase refinement or channel tracking) and within a work budget: frames decoded inside such spans may cover at most as many samples as the stream brings plus four longest frames, so a run of headers each claiming a long span costs at most about twice a channel full of real frames. Past the budget, candidates there are skipped and counted as `inside_frame`.
* **W0:** the receiver still trusts only a good CRC-16 to skip ahead (unchanged behaviour), but a frame whose length was read also reserves its span for counting.
* **A header failure** (either format) has no length. It reserves the longest possible frame, 255 bytes, after its start, for counting only: candidates inside are still tried, and a real frame found there is returned. Their failures are counted as `inside_frame`. So a second real frame lost within that span is not counted as lost.
* **A sync failure** reserves nothing, so a real frame lost at the sync word adds its body candidates to `sync_rejected`. This is small where it matters (W0 below 4 dB loses frames at the header, not at sync).

Measured, BPSK, 200 frames, seed 5, no `expected_len`, no frequency offset (the acceptance tests):

| Format, Eb/N0 | found | lost | `header_rejected` | `sync_rejected` | `too_short` | `inside_frame` | `preambles` |
|---|---|---|---|---|---|---|---|
| W0, 0 dB | 39 | 161 | 147 | 6 | 8 | 3 | 203 |
| W0, 2 dB | 100 | 100 | 98 | 2 | 1 | 29 | 230 |
| W0, 4 dB | 166 | 34 | 34 | 0 | 0 | 162 | 362 |
| v1, 6 dB | 200 | 0 | 0 | 0 | 0 | 635 | 835 |

For W0, `header_rejected` is within 10 % of frames sent minus frames found (the rest of the loss is `sync_rejected` and `too_short`). `oasa-radio waveform decode` prints the counts in its last line (`decode_done`).

## Found-frame rates

Frames with a good header (a frame returned at the right place) out of 400 sent. 64-byte payloads, random start delay and carrier phase, no `expected_len`, seed 11, release build. The payload may still be damaged (a 64-byte frame at 0 dB has a payload error rate near 1); this is about finding frames. Reproduce: `cargo test --release -p oasa-waveform --test frames found_rate_table -- --ignored --nocapture` (about 90 s).

No frequency offset:

| | -2 dB | -1 dB | 0 dB | 1 dB | 2 dB | 4 dB | 6 dB |
|---|---|---|---|---|---|---|---|
| BPSK W0 | 35 | 62 | 106 | 148 | 211 | 330 | 388 |
| BPSK v1 | 335 | 374 | 386 | 398 | 398 | 400 | 400 |
| QPSK W0 | 47 | 50 | 93 | 136 | 208 | 305 | 385 |
| QPSK v1 | 395 | 399 | 399 | 400 | 400 | 400 | 400 |

With `cfo_search 0.1` and a random offset per frame in +-0.018 cycles per symbol (the bench's 2.5 ppm at 125 kbaud):

| | -2 dB | -1 dB | 0 dB | 1 dB | 2 dB | 4 dB | 6 dB |
|---|---|---|---|---|---|---|---|
| BPSK W0 | 35 | 58 | 102 | 146 | 183 | 323 | 379 |
| BPSK v1 | 264 | 333 | 363 | 382 | 395 | 400 | 400 |
| QPSK W0 | 28 | 52 | 92 | 120 | 186 | 299 | 381 |
| QPSK v1 | 381 | 391 | 398 | 398 | 400 | 400 | 400 |

The acceptance test (`v1_finds_frames_where_w0_loses_them_to_the_header`, BPSK, 200 frames, seed 5) measured: W0 39/200 at 0 dB and 100/200 at 2 dB; v1 197/200 and 200/200. For W0 the report attributes the losses to the header: at 0 dB 147 `header_rejected` against 9 `sync_rejected` and 8 `too_short`.

Where v1 still loses frames:

* **Below about 0 dB, the sync word.** The receiver accepts up to 3 wrong sync bits of 16. At -2 dB the raw bit error rate is 0.13, and v1's losses are `sync_rejected` (62 of 400 for BPSK at -2 dB; none are header losses). With `sync_max_errors` 5 the same run finds 392, 399 and 399 of 400 at -2, -1 and 0 dB; with 8, 397, 400 and 400. A looser check lets more false candidates reach the header decoder, which costs time. Reproduce: `cargo test --release -p oasa-waveform --test frames sync_tolerance_table -- --ignored --nocapture` (BPSK, v1, seed 11, 400 frames, -2, -1, 0, 2, 6 dB; 3, 5 and 8 tolerated errors). The default is unchanged (3) and is a receiver setting, not part of the format. With v1 the CRC-8 and the code make a looser sync check affordable; decide when the bench says where it matters.
* **With a frequency offset at very low Eb/N0.** The preamble's frequency estimate is noisy at 0 dB and below, which turns the header's phase over its 152 symbols (47 of 400 BPSK frames `header_rejected` at -2 dB). It is rare above 2 dB.

## Airtime

Symbols for a 64-byte payload (`Modem::frame_samples` divides by samples per symbol):

| | W0 | v1 | Added |
|---|---|---|---|
| BPSK, 2-FSK | 79 + 68 x 8 = 623 | 79 + 152 + 66 x 8 = 759 | +136 (+22 %) |
| QPSK | 79 + 68 x 4 = 351 | 79 + 152 + 66 x 4 = 495 | +144 (+41 %) |

Fixed overhead (everything but the payload) goes from 111 symbols (BPSK) to 247. At 25 kbaud a 64-byte BPSK frame takes 24.9 ms in W0 and 30.4 ms in v1. Short frames pay most: a 4-byte payload goes from 143 to 279 symbols (BPSK). The header also costs 152 symbols at a fixed rate whatever the payload modulation is.

## Reserved fields

* `channel_plan` (4 bits): later the sender's hop schedule, which will come from the session key and is never sent in the clear. Until then it is 0, and receivers report whatever they see.
* `coexistence` (3 bits): later flags for sharing the band. Same rule.
* `extension`, `pilots`, `payload_code` 2 to 7 and `modulation` 3 are refused today so that a future sender cannot be misread by an old receiver. `payload_code` 1 (light) is defined in [fec-light.md](fec-light.md); it reuses [`fec.rs`](../waveform/src/fec.rs). The heavy tier will take another value.

## Security

The header is **not encrypted and not authenticated**. The CRC-8 and the code protect against noise, not against an attacker. A forged or altered header can only make a frame fail to decode (a wrong CRC-16, a rejection, or a frame lost to a bad length); it cannot make a receiver accept a different payload, because OASA's encryption and authentication sit above this layer, in oasa-sec. Later the link will bind the header bytes into its authenticated data, so a header changed in flight is detected there. Do not put anything secret in the header fields, and do not put a secret schedule in `channel_plan` unless it is the keyed value the session derives.

## Tests

* `fec.rs` unit tests: encoder known answers, round trip, any 2 (and 4) coded bit errors in a 32-bit block corrected over random patterns, soft values beating hard decisions.
* `frame.rs` unit tests: CRC-8 known answers, field layout, header round trip, rejection of damage the code cannot fix, field rules, CRC-16 covering the header bytes.
* `waveform/tests/frames.rs`: v1 round trips (payloads 0 to 255 bytes, all modulations, random phase, delay, sub-sample delay, small offset; the coarse search at +-1.5 symbol rates), every rejection rule and reserved fields on both paths, damaged body, miscorrected length, noise only, the loss-attribution test above.
* `tests/cli.rs`: `--frame v1` through `iq`, `decode` and `ber`.

## Bench check plan (oasa-lab, B200mini to E310 over cable)

1. Rerun `l2-waveform-w1` with `--frame v1` on both radios (same sample rate, same attenuator steps as the W0 run that missed frames). Expect found frames near 100 % at 4 dB Eb/N0 and below, down to about 0 dB. Below that, expect losses to be `sync_rejected` (see above), not headers.
2. Decode the same recordings from the W0 runs with the receive report: `oasa-radio waveform decode --in REC --mod bpsk` (add `--cfo-search` as the bench did). Read `decode_done` like this: the counts are of *candidates* (peaks past the detection threshold), including the ones a frame's own body makes; those are counted as `inside_frame` and are not losses. Compare `header_rejected` with frames sent minus frames found (`frames` in `decode_done`, against the number the sender transmitted). Expect `header_rejected` to be close to that difference below 4 dB (in simulation, within 10 %), with `sync_rejected` and `too_short` small. If it is not, the loss is somewhere else and v1 will not fix it. Where the count is not a clean number: a frame lost at the sync word adds its body candidates to `sync_rejected`; a second frame lost within 255 bytes after a header failure counts as `inside_frame`.
3. Record all the counts for every run next to the found-frame rate, so a result says which stage lost the frames.
4. Check the cost: airtime per frame from the table above against the lab's duty cycle limits.
