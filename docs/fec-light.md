# Light error correction: `payload_code` 1

Roadmap step B4. Frame v1's body gets a convolutional code and an interleaver. Uncoded, a 64-byte frame needs about 9.5 dB Eb/N0 for 1 % packet loss in simulation; with the light code, 3.8 dB (QPSK) and 4.3 dB (BPSK; 3.5 dB with a looser sync check). The code alone, with ideal soft decisions, needs 3.45 dB. Code: [`waveform/src/frame.rs`](../waveform/src/frame.rs) (body, interleaver), [`waveform/src/fec.rs`](../waveform/src/fec.rs) (encoder, soft Viterbi; shared with the [v1 header](frame-v1.md)), [`waveform/src/modem.rs`](../waveform/src/modem.rs) (soft values). Select it with `ModemConfig::code = PayloadCode::Light` or `--frame v1 --code light`. Uncoded stays the default.

## Format

The v1 header (unchanged) says `payload_code` 1. The body:

1. Payload bytes, then CRC-16/CCITT-FALSE over the three header field bytes and the payload (as v1 without a code).
2. Whiten with PN9.
3. Code: K=7, generators 171 and 133 (octal), rate 1/2, six zero tail bits. `n` payload bytes give `2 ((n + 2) 8 + 6)` coded bits: 1068 for 64 bytes.
4. Interleave (below).
5. Send at the payload modulation (BPSK and 2-FSK one coded bit per symbol, QPSK two).

`payload_code` 2 to 7 stay reserved for the heavy tier. Receivers accept 0 and 1 and reject the rest. A v1 receiver reads the code from each header, so coded and uncoded frames can share a channel; `--code` on the receiving side matters only when the length is given (`--payload-len`, BER measurement), where the header is not trusted.

## Interleaver

Row-column block interleaver over the coded body. Nothing is sent for it: it follows from the number of coded bits `L`.

* Columns: `C = min(floor(sqrt(L)), floor((L - 1) / 32))`, at least 1.
* Write the coded bits row by row into rows of `C` (the last row may be short), read them column by column.

What it gives:

* Two coded bits that are neighbours in the code go out at least 32 bits apart: 32 symbols in BPSK and 2-FSK, 16 in QPSK (a test checks every payload length from 2 to 255 bytes).
* A burst on air of up to one column (about `sqrt(L)` bits, 34 for 64 bytes) reaches the decoder as single errors `C` coded bits apart (32 for 64 bytes).
* Square-ish, so both distances grow with the frame. For 64 bytes: 32 columns of 33 or 34.
* Frames under 2 payload bytes (`L` < 65) get `C = 1`: no interleaving.

Why: fades. At 915 MHz and 20 m/s the coherence time is about 7 ms. A fade 20 dB deep lasts much less than that (on average about 0.7 ms for a Rayleigh channel with a 61 Hz Doppler; 10 dB deep, about 2 ms), and a fade wipes out a run of symbols. Without the interleaver the run reaches the decoder in one piece and beats the code (free distance 10); with it, the run is spread into single weak bits.

Measured (`fade_table`, ignored test): 64-byte frames at 6 dB Eb/N0, the body faded 20 dB (amplitude 0.1) over `L` symbols at a random place, frames delivered of 200, seed 50 + L:

| L (symbols) | 0 | 16 | 32 | 48 | 64 | 96 | 128 | 192 | 256 |
|---|---|---|---|---|---|---|---|---|---|
| BPSK light | 200 | 199 | 200 | 200 | 200 | 200 | 200 | 181 | 67 |
| QPSK light | 200 | 200 | 199 | 199 | 196 | 179 | 71 | 0 | 0 |

The faded symbols arrive as near-erasures (LLRs near 0), which the code handles better than errors, so a fade of up to about 4 columns (128 coded bits in a row of the matrix get 4 neighbours faded) is survived. An uncoded frame in such a fade has a bit error rate near 0.4 on the faded bits and is lost to a fade of a few symbols. At 125 kbaud, 128 BPSK symbols is 1.0 ms (64 QPSK symbols 0.5 ms): the typical 20 dB fade at 20 m/s, but not the longer, shallower ones. Fades longer than that, or longer than a frame, need an interleaver across frames or retransmission: the heavy tier's job.

## Eb/N0 convention

**Eb is the energy per information bit.** The code has rate 1/2, so a coded symbol carries half an information bit (BPSK): Es/N0 per coded BPSK symbol = Eb/N0 - 3 dB. QPSK: Es/N0 = Eb/N0. `Modem::energy_per_bit()` returns `Es / (bits per symbol x code rate)`, so the simulated channel (`n0_for_ebn0`), `waveform ber`, `waveform per` and `waveform iq` all use this convention. As in W0, overhead is not counted: preamble, sync, header, CRC-16 and the six tail bits (1.1 % of a 64-byte body, 0.05 dB) are paid for but not in Eb.

Consequence: at a given Eb/N0 the preamble, sync word and header of a coded BPSK or 2-FSK frame are 3 dB weaker than in an uncoded frame (every symbol has the same energy). Finding the frame, not decoding it, then limits the packet loss near 3 dB (below).

`waveform ber --code light` measures the payload BER after decoding; its `theory` and `z` are `null` (no closed form). Compare packet loss instead: `waveform per`, or the tables here.

## Receiver

After the header gives length and code, the body symbols become log-likelihood ratios, positive for bit 0 (the convention of [`fec.rs`](../waveform/src/fec.rs)):

* **PSK.** The matched filter output, turned back by the frame's phase line `a + b k`, has amplitude 1 and complex noise variance `1/rho` (`rho` = Es/N0, estimated per frame from preamble, sync and header). BPSK: `LLR = 4 rho Re(y)`. QPSK (each axis carries `1/sqrt 2`): `2 sqrt(2) rho Re(y)` for the first bit, the same of `Im(y)` for the second. The Viterbi decoder only needs the values in proportion; the scale is there so the numbers mean what they say.
* **2-FSK (noncoherent).** With the tone correlations scaled so the sent tone has magnitude 1, the sent tone has likelihood `exp(-rho (|c|^2 + 1)) I0(2 rho |c|)` and the other `exp(-rho |c|^2)`, so `LLR = ln I0(2 rho |c0|) - ln I0(2 rho |c1|)` (`ln I0` from Abramowitz and Stegun 9.8.1 and 9.8.2, tested against its series). **Soft metrics are worth it:** with the signs only (hard decisions), 64-byte frames at 9 dB lost 135 of 200; with the LLRs, 8 of 200 (seed 11, 2026-11-05).

Then de-interleave, soft Viterbi, unwhiten, CRC-16.

One change in the receiver for coded frames only: the phase line's maximum-likelihood check (`ml_line`, see [waveform-w0.md](waveform-w0.md)) counts the decoded header as known symbols too (231 instead of 79). A coded body is twice as long and 3 dB weaker per symbol, and the search over slopes is sized by the known symbols: with the preamble alone it cost 4.8 times the uncoded frame's refinement; with the header, 1.5 times (about the length ratio). Same packet loss in a check run. Uncoded frames keep the preamble only, so they decode bit for bit as before (checked against `main`: 490 frames over all three modulations, both formats, plain and coarse search, known length, 0 to 4 dB).

Longest frame: a v1 receiver that does not know the length must allow for a 255-byte light frame (4355 symbols BPSK, against 2287 uncoded). The stream receiver therefore waits about twice as long before it decides a v1 frame and holds about twice the window; the coarse search derotates that much. A header failure now reserves that longer span for the receive report's counts (in the check above, one 2-FSK run moved 12 of 2262 candidates from `sync_rejected` to `inside_frame`; no frame changed).

## Measured

Packet loss of the real receiver (no known length: sync, header, length and code read from the frame), 64-byte random payloads, random start delay and carrier phase, no offset, 400 frames a point, seed 11, release build, 2026-11-05. Eb/N0 per information bit. Reproduce: `cargo test --release -p oasa-waveform --test fec_light loss_table -- --ignored --nocapture` (about 30 minutes on a busy 2-CPU machine). "Not found" is the part of the loss where no frame came back at the right place (sync word or header); the rest came back with a bad CRC-16.

| Eb/N0 dB | 0 | 0.5 | 1 | 1.5 | 2 | 2.5 | 3 | 3.5 | 4 | 4.5 | 5 | 5.5 | 6 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ideal code (4000 frames) | 99.8 % | 97.4 % | 85.2 % | 57.8 % | 29.8 % | 11.7 % | 3.2 % | 0.90 % | 0.22 % | 0 | 0 | 0 | 0 |
| BPSK light | 100 % | 98.5 % | 90.0 % | 65.0 % | 32.0 % | 13.0 % | 5.75 % | 2.00 % | 2.25 % | 0.25 % | 0 | 0.25 % | 0 |
| of which not found | 30.0 % | 22.0 % | 18.5 % | 10.8 % | 7.0 % | 3.75 % | 2.25 % | 1.00 % | 2.00 % | 0.25 % | 0 | 0.25 % | 0 |
| BPSK light, sync 5 | 99.8 % | 98.0 % | 88.3 % | 60.5 % | 27.5 % | 9.5 % | 3.75 % | 1.00 % | 0.25 % | 0 | 0 | 0 | 0 |
| of which not found | 10.8 % | 7.0 % | 2.5 % | 0.75 % | 0.25 % | 0 | 0.25 % | 0 | 0 | 0 | 0 | 0 | 0 |
| QPSK light | 99.8 % | 98.5 % | 86.3 % | 59.0 % | 27.8 % | 12.5 % | 3.0 % | 2.00 % | 0.25 % | 0 | 0 | 0 | 0 |
| of which not found | 3.25 % | 2.0 % | 0.5 % | 0.75 % | 0.25 % | 0.25 % | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| BPSK uncoded | 100 % | 100 % | 100 % | 100 % | 100 % | 100 % | 100 % | 100 % | 100 % | 99.5 % | 94.5 % | 87.5 % | 70.8 % |
| QPSK uncoded | 100 % | 100 % | 100 % | 100 % | 100 % | 100 % | 100 % | 100 % | 99.8 % | 98.8 % | 95.8 % | 88.0 % | 72.8 % |

"Sync 5" is `sync_max_errors` 5 instead of the default 3 (QPSK light with 5: the same within one frame per point). Uncoded, higher:

| Eb/N0 dB | 6.5 | 7 | 7.5 | 8 | 8.5 | 9 | 9.5 | 10 |
|---|---|---|---|---|---|---|---|---|
| BPSK uncoded | 54.0 % | 31.8 % | 19.8 % | 9.0 % | 3.75 % | 2.00 % | 1.00 % | 0 |
| QPSK uncoded | 50.0 % | 35.5 % | 17.5 % | 10.8 % | 5.0 % | 1.25 % | 0.25 % | 0 |

2-FSK (noncoherent; soft values as above):

| Eb/N0 dB | 7 | 8 | 9 | 10 | 11 | 12 | 13 | 14 |
|---|---|---|---|---|---|---|---|---|
| 2-FSK uncoded | 100 % | 100 % | 99.3 % | 80.3 % | 37.3 % | 9.25 % | 1.25 % | 0 |
| 2-FSK light | 96.0 % | 26.5 % | 2.75 % | 0.50 % | 0 | 0 | 0 | 0 |

Reading it:

* **1 % loss** (interpolated): ideal code 3.45 dB; QPSK light about 3.8 dB; BPSK light about 4.3 dB with the default sync check and 3.5 dB with 5 wrong sync bits allowed; uncoded about 9.5 dB (BPSK and QPSK); 2-FSK about 9.6 dB coded and 13.3 dB uncoded. Gain of the light code at 1 %: about 5.2 dB (BPSK, default), 5.7 dB (QPSK), 3.7 dB (2-FSK).
* **K=5 for comparison:** the owner's earlier analysis put K=5 at about 4.3 dB for 1 % with ideal soft decisions; K=7 is 3.45 dB here, so the extra memory is worth about 0.8 dB.
* **BPSK is limited by finding the frame**, not by the code: at a given Eb/N0 its preamble, sync and header symbols are 3 dB weaker than QPSK's. With the sync check at 5 wrong bits, BPSK sits within 0.1 dB of the ideal code at 1 %. QPSK's sync is strong enough at these points; its 2 % at 3.5 dB against the ideal 0.9 % is all CRC failures after a good header (about 0.3 dB of phase and timing loss on the body).
* 400 frames give about +-1 % at a 2 % loss (one sigma 0.7 %); the 4 dB BPSK point above 3.5 dB is within that.

## Airtime

Symbols for a 64-byte payload (`Modem::frame_symbols`; `waveform iq` and `waveform per` print `frame_symbols`):

| | v1 uncoded | v1 light | Added |
|---|---|---|---|
| BPSK, 2-FSK | 79 + 152 + 528 = 759 | 79 + 152 + 1068 = 1299 | +540 (+71 %) |
| QPSK | 79 + 152 + 264 = 495 | 79 + 152 + 534 = 765 | +270 (+55 %) |

At 125 kbaud (the W1 bench) a 64-byte BPSK frame takes 6.1 ms uncoded and 10.4 ms coded; at 25 kbaud 30.4 and 52.0 ms. What it buys: at the same received power (same Es/N0 per symbol), 1 % loss needs Es/N0 about 9.5 dB uncoded and 1.3 dB coded (BPSK; 0.5 dB with the looser sync check), so about 8 dB less power for 1.7 times the airtime. Counted per information bit (the table above) the gain is 5.2 dB.

Decode cost: a 64-byte light frame takes about 1.7 times the uncoded frame's time in BPSK and 1.5 times in QPSK (detection over a longer frame, refinement, Viterbi). The Viterbi decoder itself is small (534 steps of 64 states).

## Bench check plan (oasa-lab `l2-waveform-w1`, B200mini to E310 over cable)

Same bench, wiring, rates and IF as W1 ([waveform-w0.md](waveform-w0.md), W1 section): 1 MS/s, 125 kbaud, 8 samples per symbol, 250 kHz digital IF, `--cfo-search 0.1`, both directions, BPSK and QPSK.

1. Make the block with `oasa-radio waveform iq --frame v1 --code light` (64-byte payloads; a frame is 10.4 ms BPSK, 6.1 ms QPSK). Record at least 400 frames a point (16 repeats of the 24-frame block): 96 cannot resolve 1 to 2 %.
2. Set TX gains from the W0 probe. At the same gain the symbol power is the same, so the coded frames' Eb/N0 is the W0 point's plus 3.0 dB (`10 log10(1 / rate)`): gain = probe gain + (target - 3.0 - probe Eb/N0). Aim for 2.5, 3.5, 4.5 and 5.5 dB.
3. Decode without the length (`decode --frame v1`, no `--payload-len`): the receiver as it runs on the air. Delivered = frames with a good CRC-16 whose payload matches one sent. Decode the same recordings again with `--sync-max-errors 5` (log only): the table says it moves BPSK's 1 % point from 4.3 to 3.5 dB.
4. Measure Eb/N0 from the recording as W1 does, then divide by the code rate: `Eb/N0 = Es/N0 / (bits per symbol x 0.5)`.
5. Get the simulated reference at the measured Eb/N0 with the bench's offset and timing: `oasa-radio waveform per --mod M --frame v1 --code light --ebn0-db E1,E2,... --frames 400 --cfo-search 0.1 --cfo-random 0.018 --random-frac-delay`.
6. **Expect** the measured packet-loss curve within about 1 dB of the simulation: for each point with a loss between 2 % and 50 % (what 400 frames can resolve), the measured Eb/N0 minus the Eb/N0 at which the simulation gives that loss is within +-1 dB. At 5 dB and above, at least 97 % delivered.
7. Known bad: decode a coded recording as uncoded with the length given (`--payload-len 64 --code none`): no good CRC.
8. Record the decoder's counts per point (`decode_done`) and the frames with a bad CRC-16. Near 3.5 dB the simulation loses BPSK frames about half at the sync word (`sync_rejected`) and half to the CRC-16, QPSK frames all to the CRC-16 (about 2 %). Far more CRC failures than that points at the soft values or the phase tracking on the longer frame, not the code.

What the bench adds over the simulation: real phase noise and the 2.5 ppm offset over a 10 ms frame (the simulation has a constant offset), and the Eb/N0 measurement. The cable has no fading; a fading check needs a channel emulator or the air.

## Tests

* `frame.rs`: interleaver is a permutation and `deinterleave` undoes it (payloads 0 to 255 bytes), neighbours at least 32 apart on air for 2 to 255 bytes, body round trip with errors corrected, CRC-16 still covers the header, `payload_code` names and reserved values.
* `modem.rs`: `ln I0` against its series, LLR signs and scale.
* `waveform/tests/fec_light.rs`: config, energy and airtime; noiseless round trips (0 to 255 bytes, all modulations); round trips with phase, delay, sub-sample delay, and an offset per frame through `cfo_search 0.1`; the coarse search at +-1.4 symbol rates; any v1 receiver takes the code from the header (mixed buffer); **a 32-bit burst on air** is corrected with the interleaver at six places in the body, not without it, and loses an uncoded frame; the header still protects (reserved codes, a header that names the wrong code or length, a bad CRC-8); noise only gives no frames (plain and coarse); BER mode uses the information bit energy; **packet loss: at least 99 % delivered at 4.5 dB and 90 % at 3.5 dB** (BPSK, 300 and 200 frames, seed 11; measured 300/300 and 194/200, about 25 s in a debug build). Ignored: `loss_table` and `fade_table` (the tables above).
* `waveform/tests/frames.rs`: the v1 uncoded waveform pinned (length and fingerprint from `main`), as W0 already was.
* `waveform/tests/stream.rs`: the stream receiver gives what the whole buffer gives, also for light frames (plain and coarse).
* `tests/cli.rs`: `iq`, `decode` and `rx-stream` with `--code light`, `per`, `ber`, and `--code light` refused without `--frame v1`.

## Not done, and what is uncertain

* The heavy tier (`payload_code` 2 and up): a stronger code or a longer interleaver, for fades longer than a frame's columns.
* Sync tolerance. Near 3.5 dB a coded BPSK frame's sync word is at 0.5 dB Es/N0 and the default check (3 wrong bits of 16) loses about 1 % of frames before the code is reached: 1 % loss moves from 3.5 to 4.3 dB. `sync_max_errors` 5 fixes it (table). The default is unchanged (a receiver setting; W0 and uncoded v1 use it too, and a looser check lets more false candidates reach the header decoder). Recommended for coded BPSK; decide on the bench (`decode --sync-max-errors 5`).
* The plain receiver (no `--cfo-search`) with a carrier offset of 0.002 to 0.004 cycles per symbol loses v1 headers at 3 dB Es/N0, coded or not: the preamble's frequency estimate is gated off at that SNR and the header's phase turns (in a check, 2 to 8 of 20 frames, uncoded and coded alike). With `--cfo-search 0.1`, as the bench runs, 59 to 60 of 60. Not new with the code, but the code puts frames at that Es/N0. A fix (retry the header with the ungated estimate) changes uncoded v1 and is left for its own change.
* The 2-FSK figures are for noncoherent detection; a coded noncoherent link loses more to the combining than coherent PSK.
* The fade table uses a rectangular fade, not a Rayleigh channel.
