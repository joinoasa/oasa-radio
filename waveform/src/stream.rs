//! A receiver for a continuous stream: samples arrive in blocks, frames come out once, at their
//! position in the whole stream.
//!
//! [`Modem::demodulate_report`] searches one buffer. Run on every new block with all the old samples it
//! would search them again and again; run on each block alone it would miss frames that cross a block
//! boundary. [`StreamReceiver`] keeps a window over the stream and decides a frame only when its start
//! is at least one longest frame (plus a margin) before the window's end, so the frame is surely all
//! there. A start once decided is never decided again. Samples are dropped from the front of the window
//! once nothing can need them: the window keeps one detection reference before the point where the next
//! decision starts (so the detector's peak test sees the same neighbours as on the whole buffer), and a
//! decided frame that is still running (so its body is skipped exactly as on the whole buffer).
//!
//! Cost: every block is searched together with the overlap kept from before, about one longest frame.
//! Blocks of a few frame lengths or more keep that overhead small.

use crate::frame::MAX_PAYLOAD;
use crate::iq::Complex32;
use crate::modem::{Modem, RxConfig, RxFrame};

/// Samples kept beyond the frame itself: timing and frequency refinement look a few symbols around a
/// candidate, and the matched filter runs past the last symbol.
fn margin(m: &Modem) -> usize {
    8 * m.config().sps + 64
}

pub struct StreamReceiver {
    modem: Modem,
    rx: RxConfig,
    /// The window: samples from absolute index `base` on.
    buf: Vec<Complex32>,
    base: u64,
    /// Frames are new only if they start at or after this absolute index.
    next: u64,
    /// Start and end (absolute) of decided frames that may still run past `next`.
    running: Vec<(u64, u64)>,
    /// Samples pushed so far.
    total: u64,
    /// The window's start moves only in steps of this: the coarse search looks at candidates on a grid
    /// counted from the start of the buffer, so the grid must stay where it is on the whole stream.
    align: u64,
}

impl StreamReceiver {
    pub fn new(modem: Modem, rx: RxConfig) -> Self {
        let align = modem.coarse_search(&rx).map_or(1, |c| c.step.max(1)) as u64;
        Self {
            align,
            modem,
            rx,
            buf: Vec::new(),
            base: 0,
            next: 0,
            running: Vec::new(),
            total: 0,
        }
    }

    pub fn modem(&self) -> &Modem {
        &self.modem
    }

    /// Samples pushed so far.
    pub fn samples_seen(&self) -> u64 {
        self.total
    }

    /// Samples held in the window now (for tests and monitoring: stays near two frame lengths).
    pub fn window_len(&self) -> usize {
        self.buf.len()
    }

    /// The longest frame this receiver can be given, in samples.
    fn longest(&self) -> usize {
        let n = self.rx.expected_len.unwrap_or(MAX_PAYLOAD).min(MAX_PAYLOAD);
        self.modem.frame_samples(n) + margin(&self.modem)
    }

    /// Add a block of samples; returns the frames now decided, in order, with `start` and `end` counted
    /// from the first sample of the stream.
    pub fn push(&mut self, block: &[Complex32]) -> Vec<RxFrame> {
        self.buf.extend_from_slice(block);
        self.total += block.len() as u64;
        let need = self.longest();
        if self.buf.len() < need {
            return Vec::new();
        }
        let limit = self.base + (self.buf.len() - need) as u64;
        let out = self.decide(limit);
        self.next = self.next.max(limit);
        self.trim();
        out
    }

    /// End of stream: decide everything left (frames cut off by the end are not returned).
    pub fn finish(&mut self) -> Vec<RxFrame> {
        let end = self.base + self.buf.len() as u64;
        let out = self.decide(end);
        self.next = self.next.max(end);
        self.buf.clear();
        self.base = end;
        self.running.clear();
        out
    }

    /// Search the window; keep the frames that start in `[next, limit)`.
    fn decide(&mut self, limit: u64) -> Vec<RxFrame> {
        let (frames, _) = self.modem.demodulate_report(&self.buf, &self.rx);
        let mut out = Vec::new();
        for mut f in frames {
            let start = self.base + f.start as u64;
            if start < self.next || start >= limit {
                continue;
            }
            let end = self.base + f.end as u64;
            self.running.push((start, end));
            f.start = start as usize;
            f.end = end as usize;
            out.push(f);
        }
        out
    }

    fn trim(&mut self) {
        let next = self.next;
        self.running.retain(|&(_, end)| end > next);
        let back = (self.modem.reference_len() + margin(&self.modem)) as u64;
        let mut keep = next.saturating_sub(back);
        if let Some(&(start, _)) = self.running.iter().min_by_key(|r| r.0) {
            keep = keep.min(start.saturating_sub(back));
        }
        if keep > self.base {
            let drop = ((keep - self.base) / self.align * self.align) as usize;
            self.buf.drain(..drop.min(self.buf.len()));
            self.base += drop as u64;
        }
    }
}
