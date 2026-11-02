//! OASA's own waveform, milestone W0: a simple digital modem in simulation.
//!
//! The modem works on complex baseband samples ([`Complex32`], the same thing as an SDR's `cf32`),
//! so the code that runs in simulation today can take and produce real IQ from a radio later.
//!
//! - [`modem`]: [`Modem::modulate`] (payload bytes to samples) and [`Modem::demodulate`] (samples to frames).
//! - [`channel`]: the simulated channel (AWGN at a given Eb/N0, phase, frequency and sample delay).
//!   Kept apart from the modem on purpose.
//! - [`theory`]: erfc, Q and the textbook BER curves the simulation is checked against.
//! - [`sim`]: the Monte Carlo BER harness used by the tests and `oasa-radio waveform ber`.
//! - [`frame`], [`pulse`], [`rng`], [`iq`]: frame format and CRC, the RRC pulse, the seeded generator,
//!   the sample type and cf32 byte conversion.
//!
//! Energy convention (the one place this goes wrong most often): the modulator makes the energy of
//! one symbol, the sum of `|x|^2` over its samples, equal to 1. So `Es = 1` and `Eb = 1 / bits_per_symbol`
//! whatever the oversampling is. The channel adds complex noise with variance `N0` per sample, which
//! gives `Es/N0 = 1 / N0`. See `docs/waveform-w0.md`.

#![forbid(unsafe_code)]

pub mod channel;
pub mod fft;
pub mod frame;
pub mod iq;
pub mod modem;
pub mod pulse;
pub mod rng;
pub mod sim;
pub mod theory;

pub use iq::Complex32;
pub use modem::{CoarseSearch, ConfigError, Modem, ModemConfig, Modulation, RxConfig, RxFrame};
