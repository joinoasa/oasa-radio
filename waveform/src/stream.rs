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

use crate::iq::Complex32;
use std::collections::VecDeque;

use crate::modem::{Modem, Reject, RxConfig, RxFrame, RxReport, Tally};

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

    /// The longest frame this receiver can be given, in samples (for v1 without a known length: a
    /// 255-byte frame with the light code, since any header may announce one).
    fn longest(&self) -> usize {
        self.modem.longest_frame_samples(&self.rx) + margin(&self.modem)
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

/// A low-delay receiver for a live stream (the `sdr` radio). Same frames as
/// [`Modem::demodulate_with`] with the coarse frequency search on the whole stream, but:
///
/// - the coarse detection metric is computed once per position, as its samples arrive (the
///   [`StreamReceiver`] searches its whole window again on every block, so small blocks cost the
///   window each time and the receiver falls behind);
/// - a frame comes out as soon as its own samples are in. The v1 header gives its length, so a
///   short frame does not wait for room for the longest one.
///
/// Candidates are decided in stream order: a frame still arriving holds back the ones after it.
/// Needs `rx.cfo_search > 0`.
pub struct LiveReceiver {
    modem: Modem,
    fine: RxConfig,
    rx: RxConfig,
    cs: crate::modem::CoarseSearch,
    /// Samples a frame of the longest kind takes (what `full` is measured against).
    frame_len: usize,
    /// Peak test reach, in coarse positions.
    reach: u64,
    /// The window: samples from absolute index `base` on (`base` is a whole number of `step`s).
    buf: Vec<Complex32>,
    base: u64,
    total: u64,
    /// Coarse metric of positions `cm_first ..` (position `i` is sample `i * step`).
    cm: VecDeque<(f32, f32)>,
    cm_first: u64,
    /// The next position to test as a candidate.
    test_next: u64,
    /// Candidates found, not decided yet: (sample, frequency).
    pending: VecDeque<(u64, f32)>,
    /// Absolute sample positions.
    tally: Tally,
}

impl LiveReceiver {
    pub fn new(modem: Modem, rx: RxConfig) -> Result<Self, String> {
        let cs = modem
            .coarse_search(&rx)
            .ok_or("the live receiver needs the coarse frequency search (cfo_search > 0)")?;
        let m = modem.reference_len();
        let fine = RxConfig {
            cfo_bins: 0,
            cfo_search: 0.0,
            ..rx.clone()
        };
        Ok(Self {
            frame_len: modem.longest_frame_samples(&rx),
            reach: m.div_ceil(cs.step) as u64,
            cs,
            fine,
            rx,
            modem,
            buf: Vec::new(),
            base: 0,
            total: 0,
            cm: VecDeque::new(),
            cm_first: 0,
            test_next: 0,
            pending: VecDeque::new(),
            tally: Tally::default(),
        })
    }

    pub fn modem(&self) -> &Modem {
        &self.modem
    }

    /// Samples pushed so far.
    pub fn samples_seen(&self) -> u64 {
        self.total
    }

    /// Samples held now (about one longest frame while one arrives, less when idle).
    pub fn window_len(&self) -> usize {
        self.buf.len()
    }

    /// Counts so far, as [`Modem::demodulate_report`] gives them.
    pub fn report(&self) -> &RxReport {
        &self.tally.rep
    }

    /// Add samples; returns the frames now complete, in order, `start` and `end` counted from the
    /// first sample of the stream.
    pub fn push(&mut self, block: &[Complex32]) -> Vec<RxFrame> {
        self.buf.extend_from_slice(block);
        self.total += block.len() as u64;
        self.metric();
        self.candidates(false);
        let out = self.decide(false);
        self.trim();
        out
    }

    /// End of stream: decide what is left, as the whole-buffer receiver would at the buffer's end.
    pub fn finish(&mut self) -> Vec<RxFrame> {
        self.candidates(true);
        let out = self.decide(true);
        self.buf.clear();
        self.base = self.total;
        out
    }

    /// The coarse metric of every new position whose reference window is in.
    fn metric(&mut self) {
        let m = self.modem.reference_len() as u64;
        let step = self.cs.step as u64;
        let next = self.cm_first + self.cm.len() as u64;
        if next * step + m > self.total {
            return;
        }
        let from = (next * step - self.base) as usize;
        let new = self.modem.coarse_metric_with(&self.buf[from..], &self.cs);
        self.cm.extend(new);
    }

    /// Test the positions whose peak neighbourhood is complete (all of them at the end).
    fn candidates(&mut self, end: bool) {
        let have = self.cm_first + self.cm.len() as u64;
        while self.test_next < have && (end || self.test_next + self.reach < have) {
            let i = self.test_next;
            self.test_next += 1;
            let (v, f) = self.cm[(i - self.cm_first) as usize];
            if v < self.cs.threshold {
                continue;
            }
            let lo = i.saturating_sub(self.reach);
            let hi = (i + self.reach + 1).min(have);
            let is_peak = (lo..hi).all(|j| {
                let c = self.cm[(j - self.cm_first) as usize].0;
                c < v || (c == v && j >= i)
            });
            if is_peak {
                self.pending.push_back((i * self.cs.step as u64, f));
            }
        }
        // the peak test of `test_next` looks back `reach` positions; nothing older is needed
        let keep = self.test_next.saturating_sub(self.reach);
        while self.cm_first < keep && !self.cm.is_empty() {
            self.cm.pop_front();
            self.cm_first += 1;
        }
    }

    /// Decode the pending candidates in order, as far as the samples allow.
    fn decide(&mut self, end: bool) -> Vec<RxFrame> {
        let sps = self.modem.config().sps as u64;
        let m = self.modem.reference_len() as u64;
        let step = self.cs.step as u64;
        let mut out = Vec::new();
        while let Some(&(tau_c, f)) = self.pending.front() {
            if tau_c < self.tally.busy_until as u64 {
                self.tally.skipped();
                self.pending.pop_front();
                continue;
            }
            // the fine search around the candidate must be in (at the end: whatever is there)
            if !end && self.total < tau_c + sps + step + m {
                break;
            }
            let busy = (self.tally.busy_until as u64).saturating_sub(self.base) as usize;
            let got = self.modem.coarse_candidate(
                &self.buf,
                (tau_c - self.base) as usize,
                f,
                &self.cs,
                &self.fine,
                self.frame_len,
                busy,
            );
            let Some((res, tau, full)) = got else {
                self.pending.pop_front();
                continue;
            };
            if matches!(res, Err(Reject::TooShort)) && !full && !end {
                break; // the frame is still arriving
            }
            self.pending.pop_front();
            let base = self.base as usize;
            let res = res.map(|mut fr| {
                fr.start += base;
                fr.end += base;
                fr
            });
            if let Some(fr) = self
                .modem
                .record(res, tau + base, &self.rx, &mut self.tally)
            {
                out.push(fr);
            }
        }
        out
    }

    /// Drop samples nothing can need: before the first pending candidate's search window and the
    /// next position's (kept on the `step` grid so positions stay where they are on the stream).
    fn trim(&mut self) {
        let sps = self.modem.config().sps as u64;
        let step = self.cs.step as u64;
        let next_pos = (self.cm_first + self.cm.len() as u64) * step;
        let mut keep = next_pos.min(self.test_next * step);
        if let Some(&(tau_c, _)) = self.pending.front() {
            keep = keep.min(tau_c);
        }
        let keep = keep.saturating_sub(sps + step) / step * step;
        if keep > self.base {
            let drop = ((keep - self.base) as usize).min(self.buf.len());
            self.buf.drain(..drop);
            self.base += drop as u64;
        }
    }
}
