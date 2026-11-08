//! OASA's own waveform, milestone W0: a simple digital modem in simulation.
//!
//! The modem works on complex baseband samples ([`Complex32`], the same thing as an SDR's `cf32`),
//! so the code that runs in simulation today can take and produce real IQ from a radio later.
//!
//! - [`modem`]: [`Modem::modulate`] (payload bytes to samples) and [`Modem::demodulate`] (samples to frames).
//! - [`channel`]: the simulated channel (AWGN at a given Eb/N0, phase, frequency and sample delay).
//!   Kept apart from the modem on purpose.
//! - [`fading`]: fading channel models (two-ray multipath, flat Rayleigh with Doppler) for the simulation.
//! - [`theory`]: erfc, Q and the textbook BER curves the simulation is checked against.
//! - [`sim`]: the Monte Carlo BER harness used by the tests and `oasa-radio waveform ber`.
//! - [`stream`]: [`stream::StreamReceiver`], the receiver for a continuous stream of samples in blocks, and
//!   [`stream::LiveReceiver`], its low-delay form for a live radio.
//! - [`df`]: two receive channels' phase difference and the direction of arrival (T-RF3).
//! - [`diversity`]: two-antenna receive diversity (B14): frame selection, preamble-SNR selection, maximal-ratio combining.
//! - [`fec`]: the convolutional code and its Viterbi decoder (the frame v1 header uses them).
//! - [`frame`], [`pulse`], [`rng`], [`iq`]: frame formats (W0, v1) and CRCs, the RRC pulse, the seeded generator,
//!   the sample type and cf32 byte conversion.
//!
//! Energy convention (the one place this goes wrong most often): the modulator makes the energy of
//! one symbol, the sum of `|x|^2` over its samples, equal to 1. So `Es = 1` and `Eb = 1 / bits_per_symbol`
//! whatever the oversampling is. The channel adds complex noise with variance `N0` per sample, which
//! gives `Es/N0 = 1 / N0`. See `docs/waveform-w0.md`.

#![forbid(unsafe_code)]

pub mod channel;
pub mod df;
pub mod diversity;
pub mod fading;
pub mod fec;
pub mod fft;
pub mod frame;
pub mod iq;
pub mod modem;
pub mod pulse;
pub mod rng;
pub mod sim;
pub mod stream;
pub mod theory;
mod track;

pub use frame::{FrameFormat, HeaderFields};
pub use iq::Complex32;
pub use modem::{
    CoarseSearch, ConfigError, Modem, ModemConfig, Modulation, RxConfig, RxFrame, RxReport,
};
