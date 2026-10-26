# T-RF3 step 1: E310 receive-only test plan

Goal: with an Ettus USRP E310 (2 receive channels), two 915 MHz antennas and a Heltec V3 running the oasa-radio LoRa firmware as the source, (1) calibrate the channel offset, (2) measure direction of arrival (DOA), (3) measure diversity gain. Numbers in "expected" come from this folder's simulation ([results.md](results.md)). Pass criteria are proposals; change them if the first run shows the simulation was too optimistic. Not yet run.

## Facts checked and not checked

Checked (Ettus E310 product page and UHD manual, E3xx page):
- Frequency range 70 MHz to 6 GHz (RX), AD9361 2x2 (two RX, two TX), up to 56 MHz instantaneous bandwidth. 915 MHz is in range.
- Both RX frontends share one RX LO (UHD manual). UHD also warns that tuning one channel can affect the other, so read back the actual frequency.
- The page says the E310 is in limited quantities and not recommended for new designs. Plan a replacement (a 2-channel board with a shared reference) before building on it.

Not verified, treat as unknown:
- Whether the two RX chains keep a constant phase offset between retunes, gain changes and power cycles. The shared LO makes it plausible, but neither page says the channels are phase coherent. This is the first thing to measure.
- Maximum safe RX input level and RX gain range (not on the pages read). Use the datasheet before connecting anything, and keep conducted signals well below the limit.
- ADC resolution (not stated).
- How the E310 behaves with UHD time-aligned streaming of both channels (use one multi-channel RX stream so samples are aligned).

## Radio rules

- The E310 only receives. Do not start any TX stream. Leave TX antennas unconnected or terminated.
- The Heltec transmits in the 902-928 MHz US ISM band. Use the lowest power that gives the SNR needed (the firmware setting `oasa-radio lora set --power`, for example 2 dBm), with its normal antenna for over-the-air runs, and a 50 ohm load plus attenuators for conducted runs. Check the rules where you test (this is not legal advice; US assumed).
- Per repo rules prefer cable or a shielded box where possible. The DOA and diversity runs need air; keep them short, low power, outdoors or in an open room.

## Equipment

- E310 with UHD host software, or a host with the UHD Python/C++ API, set to 915 MHz (use the LoRa channel the Heltec uses), 1 MS/s (8x the 125 kHz bandwidth is fine), same gain on both channels.
- Two identical 915 MHz antennas (vertical dipole or whip) on a non-metal bar at d = lambda/2 = 16.4 cm centre to centre, and two cables of the same type and length.
- A 2-way power splitter (resistive or Wilkinson) and a fixed 40 to 60 dB attenuator chain, plus SMA adapters.
- Heltec V3 with oasa-radio firmware sending a repeating test packet (for example 1 packet per second, 20 byte payload, SF7, BW125k).
- Tape measure, a marked circle or turntable (angles with 1 deg accuracy), height of at least 1.5 m, no large metal within 2 m.

## Step 0: Capture

Record 2-channel complex baseband to file (SigMF, `.sigmf-data` is already git-ignored in this repo). Offline analysis with the same code as `t_rf3/lora_check.py`: find packets by dechirp and FFT peak on either channel, then take the samples of the preamble and payload, compute `r = mean(x2 * conj(x1))` and its angle.

## Step 1: Calibration (conducted)

Purpose: measure the phase and gain difference between the two receive paths including the exact cables that will go to the antennas.

1. Heltec output through the attenuator to the splitter input. Splitter outputs through the two antenna cables to RX channel 1 and 2 (the antenna end of each cable is the reference plane).
2. Capture 100 packets. Compute the phase of r per packet: `psi_a`. Also the power ratio of the channels (gain offset).
3. Swap the two cables at the E310 end (channel 1 cable to channel 2 port and the reverse). Capture again: `psi_b`. The receiver offset is `eps = (psi_a + psi_b) / 2` (circular mean); half the difference is the splitter and cable asymmetry and is discarded.
4. Repeat after: a retune away and back, a gain change, a power cycle, and 30 minutes warm-up. Record how `eps` moves.

Expected: if the chains are stable, repeated `eps` agree to within a degree or two. Per-packet statistical noise at SNR 0 dB over 3840 samples is about 0.4 deg of DOA, which is about 1.3 deg of phase.

Pass: `eps` repeatable to 2 deg (standard deviation) over repeated captures without retuning; its change with retune, gain and power cycle is known (even if not zero). If it jumps randomly by tens of degrees at each retune, the E310 is not usable as a DOA array without an in-run reference. Stop and report that, which is also a result.

## Step 2: DOA over the air

1. Heltec on a stand 5 to 10 m away (far field starts at about 0.16 m), at the antenna height, vertical antenna, transmitting at low power. The array bar centre on a turntable.
2. Rotate in 10 degree steps from -80 to +80 degrees from broadside, then repeat on the back side (180 minus theta) to show the front/back mirror. At each angle capture 50 packets. Calibrate before and after the run and check eps did not move.
3. Estimate theta per packet: `theta = asin((angle(r) - eps) / (2 pi d / lambda))`.
4. Report bias, standard deviation per packet, and RMSE per angle, plus SNR per packet (per-sample, in 125 kHz).

Expected (simulation, ideal free space, SNR 0 dB, 3840 samples): about 0.3 to 0.6 deg statistical error from 0 to 60 degrees; at SNR -7.5 dB 1.6 deg (broadside, 30 degrees). With a calibration residual of 2 deg the total is about 0.7 deg at broadside and 1.4 deg at 60 degrees. Real error will be larger: multipath, antenna coupling and the bar and cable effects are not in the model.

Pass (proposed): after calibration, RMS error 3 deg or less over +/-60 degrees at SNR 5 dB or more, with bias removed by one fixed offset. Stretch: 1.5 deg. Front and back positions give the same reading (the mirror proves the phase reading is right, and shows the expected ambiguity). Fail signs: error growing faster than 1/cos(theta), a different slope than sin(theta) predicts (coupling or wrong spacing), or per-packet scatter far above the CRB (multipath or phase jitter).

## Step 3: Diversity gain

1. Place the Heltec in a room with scattering (a few metres, indoor, people moving or the source moved by a few wavelengths between captures). One packet per position, at least 500 independent positions, or move the source slowly on a rail.
2. For each packet compute the per-branch SNR (signal power from the preamble over noise power from an empty capture) and the combined SNRs: selection (the larger branch) and MRC (weights from the calibrated phases; `SNR_MRC = |w^H h|^2 ...` computed per packet in software).
3. Report mean gain over the average single branch, and the SNR at 10% and 1% outage for each scheme.

Expected for independent Rayleigh branches: mean gain selection 1.76 dB, MRC 3.01 dB; outage gain 5.6 dB (10%) and 10.2 dB (1%) for selection, 7.0 dB and 11.7 dB for MRC. With lambda/2 spacing in rich scattering the branches are nearly uncorrelated (|rho| about 0.3). Line of sight with no fading gives MRC 3 dB (noise averaging only) and selection about 0 dB.

Pass (proposed): MRC mean gain at least 2.5 dB over the mean single branch when the branch average SNRs are within 1 dB of each other; MRC always at least as good as selection. The outage gain is reported but only judged against theory if the fading is close to Rayleigh (check the single-branch distribution first; with a Rician K factor above 0 the gains are smaller).

## What stays unknown until tried

- Channel phase stability of the E310 (Step 1 decides everything).
- Real inter-element coupling and cable/bar effects at lambda/2 spacing.
- Packet SNR range of the Heltec at the chosen power and distance, and whether the preamble alone gives a usable phase estimate.
- Whether indoor fading is Rayleigh enough to compare with theory.
- How the receiver behaves with strong signals (gain setting, saturation).
- Whether DOA bias depends on frequency offset between Heltec crystal and E310 (a carrier offset adds the same phase rotation to both channels, which cancels in r, so it should not matter; verify).

## After the test

Record results in the oasa-lab test log. If Step 1 passes, next is the 3-channel receive array (needs a source of three coherent channels) and a check of ambiguity margin at a side near 0.4 lambda. This uses the same capture format as the T-RF1 scan.
