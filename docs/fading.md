# Fading channels in simulation (B9)

`waveform per`, `waveform ber` and `waveform cable` can pass the signal through a fading channel before the
carrier offset and noise (`waveform/src/fading.rs`). AWGN, the default, is unchanged bit for bit.

```sh
# packet loss vs mean Eb/N0, flat Rayleigh at 50 Hz Doppler, the live link's settings
oasa-radio waveform per --mod qpsk --sps 4 --frame v1 --payload-len 54 --cfo-search 0.1 \
  --symbol-rate 250000 --frames 10000 --ebn0-db 10,20,30,40 --channel rayleigh --doppler-hz 50
# two rays, the second 2 us later and 6 dB weaker, random phase per frame
oasa-radio waveform per ... --channel tworay --ray-delay-us 2 --ray-gain-db -6
# the same on a live stream between two `bridge --radio sdr` processes
oasa-radio waveform cable --in a.tx --out b.rx --sample-rate 1000000 --channel rayleigh --doppler-hz 50
```

## Models

| `--channel` | What it does | Options |
|---|---|---|
| `awgn` (default) | nothing | |
| `tworay` | `y = a x(t) + b x(t - d)`, `a^2 + b^2 = 1`. No Doppler: a random phase on the second ray per frame. With Doppler: each ray its own Rayleigh process (frequency-selective fading) | `--ray-delay-us` (1), `--ray-gain-db` (-3), `--doppler-hz` (0) |
| `rayleigh` | flat Rayleigh fading, Clarke/Jakes spectrum | `--doppler-hz` (above 0) |

- Mean power gain 1: `--ebn0-db` is the mean Eb/N0 over the fading.
- Doppler `fd = v / wavelength`: 5 Hz is 1.6 m/s (walking) at 915 MHz and 0.26 m/s at 5.8 GHz; 50 Hz is 16 m/s (59 km/h)
  at 915 MHz and 2.6 m/s at 5.8 GHz; 200 Hz is 66 m/s at 915 MHz and 10 m/s (37 km/h) at 5.8 GHz.
- Rayleigh: the sum of 16 sinusoids of Zheng and Xiao (IEEE Trans. Commun. 51(6), 2003), seeded. Tests check unit power,
  the autocorrelation against `J0(2 pi fd tau)` and the Rayleigh envelope (P(10 dB fade) = 9.5 %).
- Time runs on across frames: `--frame-interval-ms` (10, the live link's 100 frames/s) between frame starts, so a
  slow fade covers several frames, as on the air. `--symbol-rate` (250000) gives the sample rate for the delay and
  Doppler; the cable uses its `--sample-rate` and rounds the second ray's delay to whole samples.

## What it shows (QPSK, 250 kbaud, v1, 54-byte payload, 10,000 frames per point)

| Channel | 1 % loss | 0.1 % loss | Floor |
|---|---|---|---|
| AWGN | 9.3 dB | 10-11 dB | none |
| 2 rays, 1 us (1/4 symbol), -6 dB | 15.3 dB | 16-18 dB | none |
| 2 rays, 2 us (1/2 symbol), -6 dB | 18.4 dB | 20-24 dB | none |
| 2 rays, 4 us (1 symbol), -6 dB | 20.2 dB | 22.4 dB | none |
| 2 rays, 1 us, -3 dB | not reached by 24 dB | | 2.7 % at 24 dB, still falling |
| 2 rays, 2 us, -3 dB | | | 27.6 % at 24 dB, falling slowly |
| 2 rays, 4 us, -3 dB | | | 88 % at 30 dB |
| Rayleigh, 5 Hz | 26.3 dB | 37.2 dB | none (slow fading: deep fades last many frames) |
| Rayleigh, 50 Hz | | | about 1.9 % |
| Rayleigh, 200 Hz | | | about 26 % |
| 2-ray Rayleigh, 2 us, -3 dB, 50 Hz | | | about 17 % |

Two limits of today's receiver show: there is no equalizer, so an echo near the main ray's strength at a quarter
symbol or more closes the eye; and the body's carrier phase is fitted as one straight line over the frame (an offset
and a constant frequency, `Modem::track`), so when the channel itself turns during a 1.8 ms frame (Doppler of tens
of Hz and up) the phase is no longer a line and part of the frame decodes with the wrong phase. Flat slow
fading costs about 17 dB at 1 % (diversity or coding and retransmission are the answer there).

**Since B12:** `--phase-track-hz` (receiver only, off by default) follows the channel through the frame
instead of one line. With it, Rayleigh 50 Hz no longer has a floor and 200 Hz drops from about 26 % to about 2 %
at 30 dB uncoded (0.4 % with `--code light`); white noise is unchanged. See [phase-tracking.md](phase-tracking.md).
