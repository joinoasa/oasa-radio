//! Voice timing (T-MSG1): hold voice packets and release them on the
//! radio's own clock.
//!
//! One type, [`Pacer`], does both jobs with different settings:
//!
//! - **Transmit scheduler** ([`TxScheduler`], [`Config::TX`]): the phone
//!   hands over voice packets whenever its link delivers them (Bluetooth and
//!   USB are bursty). The radio holds each one and releases it for the air at
//!   an even pace on its own clock.
//! - **Receive playout buffer** ([`PlayoutBuffer`], [`Config::RX`]): packets
//!   heard on the air (late, lost, duplicated, out of order) are held and
//!   handed to the phone at an even pace, a fixed delay after the first one.
//!
//! The bytes are opaque (oasa-link has already encrypted them). The logic
//! needs only two numbers per packet: `seq`, the number of the packet's
//! first codec frame (counts frames, wraps at 2^16), and `frames`, how many
//! codec frames it holds. **How those two numbers travel between phone and
//! radio and on the air is not defined here**: it waits for the phone-link
//! spec (oasa-sec #40). Until then they are plain arguments.
//!
//! Time is an argument too: [`Tick`] is microseconds on the radio's clock,
//! wrapping at 2^32. Nothing here reads a clock, allocates or uses `unsafe`.
//! Design and numbers: `docs/voice-timing.md`.

/// Microseconds on the radio's own clock. Wraps every 2^32 µs (71.6 min).
/// Differences are taken with wrapping arithmetic, so any two ticks less than
/// 35 minutes apart compare correctly.
pub type Tick = u32;

/// Codec 2 700C (and 1200) frame: 320 samples at 8 kHz = 40 ms. Source:
/// oasa-lab `docs/voice-lora-sim.md` (Codec 2 `codec2.c`).
pub const FRAME_US: u32 = 40_000;

/// Codec frames per packet on the planned setting: one packet every 80 ms.
/// Source: oasa-lab `docs/voice-lora-sim.md`, the lowest-delay interval that
/// meets the plan for 700 bit/s on SF7 250 kHz, SF7 500 kHz and SF8 500 kHz,
/// direct and relayed. Not used by the logic (each packet states its own
/// `frames`); here so the numbers live in one place.
pub const FRAMES_PER_PACKET: u8 = 2;

/// Largest `frames` one packet may carry: 16 (320 ms of 2400 bit/s at 20 ms
/// frames, the largest packet in the oasa-lab `l1-voice-lora` sweep).
pub const MAX_FRAMES: u8 = 16;

/// Bytes per slot. 128 holds every packet in the `l1-voice-lora` sweep (the
/// largest is 118 B: 2400 bit/s at 320 ms plus 22 B link overhead). The
/// planned packet is 30 B (two 4 B frames plus 22 B).
pub const SLOT_BYTES: usize = 128;

/// Slots per pacer. Chosen here: the longest hold ([`RX_MAX_HOLD_US`],
/// 320 ms) at the shortest interval in the sweep (80 ms) needs 5; 16 leaves
/// room for one-frame packets at 20 ms.
pub const SLOTS: usize = 16;

/// Transmit lead: the first packet of a talk spurt leaves this long after it
/// reaches the radio, so later packets that the phone link delays by up to
/// this much still leave on time. Chosen here (one 700C frame); to be set
/// from the part 2 timing trace of the real phone link.
pub const TX_LEAD_US: u32 = 40_000;

/// Longest a packet may wait in the transmit scheduler. Chosen here: the
/// lead plus one 80 ms packet interval.
pub const TX_MAX_HOLD_US: u32 = 120_000;

/// Playout delay: the first packet of a talk spurt is played this long after
/// it arrives. Absorbs air jitter (listen-before-talk waits, relay queueing).
/// Chosen here: two 80 ms packet intervals; the `l1-voice-lora` sweep has no
/// jitter in it (perfect channel access), so this is not yet measured.
pub const RX_DELAY_US: u32 = 160_000;

/// Longest a packet may wait in the playout buffer; past it the buffer skips
/// a packet to catch up. Chosen here: twice [`RX_DELAY_US`].
pub const RX_MAX_HOLD_US: u32 = 320_000;

/// After this long with nothing held and nothing released, the pacer forgets
/// which frames it has played, so a talker that restarted its count is heard
/// again. Chosen here: far longer than any relay delay, so a late duplicate
/// is never played twice.
pub const FORGET_US: u32 = 10_000_000;

/// Timing settings of one pacer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    /// Length of one codec frame, µs (1 to 60 000).
    pub frame_us: u32,
    /// Hold given to the first packet of a talk spurt, µs.
    pub target_us: u32,
    /// Longest any packet is held, µs (at least `target_us`, at most 60 s).
    pub max_hold_us: u32,
}

impl Config {
    /// Transmit scheduler settings.
    pub const TX: Config = Config {
        frame_us: FRAME_US,
        target_us: TX_LEAD_US,
        max_hold_us: TX_MAX_HOLD_US,
    };

    /// Receive playout settings.
    pub const RX: Config = Config {
        frame_us: FRAME_US,
        target_us: RX_DELAY_US,
        max_hold_us: RX_MAX_HOLD_US,
    };

    pub const fn is_valid(&self) -> bool {
        self.frame_us >= 1
            && self.frame_us <= 60_000
            && self.target_us <= self.max_hold_us
            && self.max_hold_us <= 60_000_000
    }
}

/// A packet was taken.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Accepted {
    /// On the current schedule.
    Queued,
    /// The first packet: the schedule starts, `target_us` from now.
    Started,
    /// Nothing was held and the packet was off the schedule (a new talk
    /// spurt, a gap, or drift): the schedule starts again `target_us` from
    /// now.
    Resynced,
    /// Taken by dropping the oldest held packet and pulling the schedule in
    /// (the hold would have passed `max_hold_us`, or every slot was full).
    CaughtUp,
}

/// A packet was not taken. Nothing changed except a counter.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Refused {
    /// No bytes, more than the slot holds, or `frames` 0 or over
    /// [`MAX_FRAMES`].
    Bad,
    /// Its frames were already played or skipped.
    Stale,
    /// It overlaps a packet already held.
    Duplicate,
    /// Its time has passed while later packets are held.
    Late,
    /// Too far ahead of the schedule even after dropping one packet.
    TooEarly,
    /// Every slot is full and it is not newer than all of them.
    Full,
}

/// A packet released by [`Pacer::poll`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Release {
    pub seq: u16,
    pub frames: u8,
    /// Bytes copied into the output buffer.
    pub len: usize,
    /// The scheduled time. Equal to the poll time when the caller polls at
    /// [`Pacer::next_release`]; earlier if it polled late.
    pub at: Tick,
    /// Frames between the previous release and this one that never came (or
    /// came too late). Their time is kept as a gap.
    pub lost_frames: u16,
}

/// Counters. All wrap.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct VoiceStats {
    pub accepted: u32,
    pub released: u32,
    pub refused_bad: u32,
    pub refused_stale: u32,
    pub refused_duplicate: u32,
    pub refused_late: u32,
    pub refused_too_early: u32,
    pub refused_full: u32,
    /// Times the schedule started again ([`Accepted::Resynced`]).
    pub resyncs: u32,
    /// Held packets dropped to catch up ([`Accepted::CaughtUp`]).
    pub catch_up_drops: u32,
    /// Frames of time removed from the schedule by catching up.
    pub catch_up_frames: u32,
    /// Frames reported in [`Release::lost_frames`] plus frames of packets
    /// dropped to catch up.
    pub lost_frames: u32,
}

#[derive(Clone, Copy)]
struct Slot<const B: usize> {
    used: bool,
    seq: u16,
    frames: u8,
    len: u16,
    data: [u8; B],
}

impl<const B: usize> Slot<B> {
    const EMPTY: Self = Self {
        used: false,
        seq: 0,
        frames: 0,
        len: 0,
        data: [0; B],
    };
}

/// Holds up to `N` voice packets of up to `B` bytes and releases them on an
/// even schedule. Memory is fixed: `N * (B + 6)` bytes of slots plus under
/// 100 bytes of state; [`PlayoutBuffer`] is 2232 bytes (a test pins it).
///
/// The schedule maps a frame number to a time: frame `anchor_seq` plays at
/// `anchor_at`, and each frame after it `frame_us` later. Packets are
/// released at the time of their first frame.
pub struct Pacer<const N: usize, const B: usize> {
    cfg: Config,
    slots: [Slot<B>; N],
    count: usize,
    anchored: bool,
    anchor_seq: u16,
    anchor_at: Tick,
    /// Frames before `next_seq` were released or skipped (valid once
    /// `have_next`).
    have_next: bool,
    next_seq: u16,
    last_activity: Tick,
    stats: VoiceStats,
}

/// The transmit scheduler: phone to air. Use with [`Config::TX`].
pub type TxScheduler = Pacer<SLOTS, SLOT_BYTES>;
/// The receive playout buffer: air to phone. Use with [`Config::RX`].
pub type PlayoutBuffer = Pacer<SLOTS, SLOT_BYTES>;

fn tick_diff(a: Tick, b: Tick) -> i64 {
    i64::from(a.wrapping_sub(b) as i32)
}

fn seq_diff(a: u16, b: u16) -> i64 {
    i64::from(a.wrapping_sub(b) as i16)
}

impl<const N: usize, const B: usize> Pacer<N, B> {
    /// `None` if the settings are out of range or `N` is 0.
    pub const fn new(cfg: Config) -> Option<Self> {
        if !cfg.is_valid() || N == 0 || B == 0 || B > u16::MAX as usize {
            return None;
        }
        Some(Self {
            cfg,
            slots: [Slot::<B>::EMPTY; N],
            count: 0,
            anchored: false,
            anchor_seq: 0,
            anchor_at: 0,
            have_next: false,
            next_seq: 0,
            last_activity: 0,
            stats: VoiceStats {
                accepted: 0,
                released: 0,
                refused_bad: 0,
                refused_stale: 0,
                refused_duplicate: 0,
                refused_late: 0,
                refused_too_early: 0,
                refused_full: 0,
                resyncs: 0,
                catch_up_drops: 0,
                catch_up_frames: 0,
                lost_frames: 0,
            },
        })
    }

    pub fn config(&self) -> Config {
        self.cfg
    }

    pub fn stats(&self) -> VoiceStats {
        self.stats
    }

    /// Packets held.
    pub fn len(&self) -> usize {
        self.count
    }

    pub fn is_empty(&self) -> bool {
        self.count == 0
    }

    pub const fn capacity(&self) -> usize {
        N
    }

    /// Drop everything and forget the schedule (a new stream or talker).
    /// Counters are kept.
    pub fn reset(&mut self) {
        for s in self.slots.iter_mut() {
            s.used = false;
        }
        self.count = 0;
        self.anchored = false;
        self.have_next = false;
    }

    /// When frame `seq` is due on the current schedule.
    fn due(&self, seq: u16) -> Tick {
        let off = seq_diff(seq, self.anchor_seq) * i64::from(self.cfg.frame_us);
        self.anchor_at.wrapping_add(off as u32)
    }

    /// Index of the held packet with the lowest frame number.
    fn oldest(&self) -> Option<usize> {
        let mut best: Option<usize> = None;
        for (i, s) in self.slots.iter().enumerate() {
            if !s.used {
                continue;
            }
            match best {
                Some(b) if seq_diff(s.seq, self.slots[b].seq) >= 0 => {}
                _ => best = Some(i),
            }
        }
        best
    }

    /// Lowest frame number held after `after`, if any.
    fn next_after(&self, after: u16) -> Option<u16> {
        let mut best: Option<u16> = None;
        for s in self.slots.iter() {
            if !s.used || seq_diff(s.seq, after) <= 0 {
                continue;
            }
            match best {
                Some(b) if seq_diff(s.seq, b) >= 0 => {}
                _ => best = Some(s.seq),
            }
        }
        best
    }

    fn anchor(&mut self, now: Tick, seq: u16) {
        self.anchored = true;
        self.anchor_seq = seq;
        self.anchor_at = now.wrapping_add(self.cfg.target_us);
    }

    /// Drop the oldest held packet; the next one (held, or `incoming`) takes
    /// its time. Returns the frames of time removed.
    fn catch_up(&mut self, incoming: u16) -> u16 {
        let Some(o) = self.oldest() else { return 0 };
        let (o_seq, o_frames) = (self.slots[o].seq, self.slots[o].frames);
        let x = match self.next_after(o_seq) {
            Some(h) if seq_diff(h, incoming) < 0 => h,
            _ => incoming,
        };
        let shift = seq_diff(x, o_seq) as u16;
        let o_due = self.due(o_seq);
        self.slots[o].used = false;
        self.count -= 1;
        // Frames skipped before the dropped packet, and the packet itself.
        let before = if self.have_next {
            seq_diff(o_seq, self.next_seq).max(0) as u32
        } else {
            0
        };
        self.stats.lost_frames = self
            .stats
            .lost_frames
            .wrapping_add(before + u32::from(o_frames));
        self.have_next = true;
        self.next_seq = o_seq.wrapping_add(u16::from(o_frames));
        self.anchor_seq = x;
        self.anchor_at = o_due;
        self.stats.catch_up_drops = self.stats.catch_up_drops.wrapping_add(1);
        self.stats.catch_up_frames = self.stats.catch_up_frames.wrapping_add(u32::from(shift));
        shift
    }

    fn store(&mut self, seq: u16, frames: u8, bytes: &[u8]) {
        // A free slot exists: callers check `count < N` first.
        if let Some(s) = self.slots.iter_mut().find(|s| !s.used) {
            s.used = true;
            s.seq = seq;
            s.frames = frames;
            s.len = bytes.len() as u16; // <= B <= u16::MAX, checked in new
            s.data[..bytes.len()].copy_from_slice(bytes);
            self.count += 1;
        }
    }

    fn refuse(&mut self, why: Refused) -> Result<Accepted, Refused> {
        let c = match why {
            Refused::Bad => &mut self.stats.refused_bad,
            Refused::Stale => &mut self.stats.refused_stale,
            Refused::Duplicate => &mut self.stats.refused_duplicate,
            Refused::Late => &mut self.stats.refused_late,
            Refused::TooEarly => &mut self.stats.refused_too_early,
            Refused::Full => &mut self.stats.refused_full,
        };
        *c = c.wrapping_add(1);
        Err(why)
    }

    fn take(&mut self, now: Tick, seq: u16, frames: u8, bytes: &[u8], how: Accepted) -> Accepted {
        self.store(seq, frames, bytes);
        self.last_activity = now;
        self.stats.accepted = self.stats.accepted.wrapping_add(1);
        if how == Accepted::Resynced {
            self.stats.resyncs = self.stats.resyncs.wrapping_add(1);
        }
        how
    }

    /// Offer a packet that arrived at `now`: its first frame number, its
    /// frame count and its (opaque) bytes.
    pub fn push(
        &mut self,
        now: Tick,
        seq: u16,
        frames: u8,
        bytes: &[u8],
    ) -> Result<Accepted, Refused> {
        if bytes.is_empty() || bytes.len() > B || frames == 0 || frames > MAX_FRAMES {
            return self.refuse(Refused::Bad);
        }
        if self.count == 0 && now.wrapping_sub(self.last_activity) >= FORGET_US {
            self.anchored = false;
            self.have_next = false;
        }
        if self.have_next && seq_diff(seq, self.next_seq) < 0 {
            return self.refuse(Refused::Stale);
        }
        for s in self.slots.iter().filter(|s| s.used) {
            if seq_diff(seq, s.seq) < i64::from(s.frames)
                && seq_diff(s.seq, seq) < i64::from(frames)
            {
                return self.refuse(Refused::Duplicate);
            }
        }
        if !self.anchored {
            self.anchor(now, seq);
            return Ok(self.take(now, seq, frames, bytes, Accepted::Started));
        }
        let hold = tick_diff(self.due(seq), now);
        let max = i64::from(self.cfg.max_hold_us);
        if self.count == 0 && (hold < 0 || hold > max) {
            self.anchor(now, seq);
            return Ok(self.take(now, seq, frames, bytes, Accepted::Resynced));
        }
        if hold < 0 {
            return self.refuse(Refused::Late);
        }
        let newest = self
            .slots
            .iter()
            .filter(|s| s.used)
            .all(|s| seq_diff(seq, s.seq) > 0);
        if hold > max {
            // Would one catch-up step bring it inside the hold?
            let Some(o) = self.oldest() else {
                return self.refuse(Refused::TooEarly);
            };
            let o_seq = self.slots[o].seq;
            let x = match self.next_after(o_seq) {
                Some(h) if seq_diff(h, seq) < 0 => h,
                _ => seq,
            };
            // One step removes at most `max_hold_us` of time: a packet far
            // ahead (a jump in the frame count) never empties the buffer.
            let shift = seq_diff(x, o_seq) * i64::from(self.cfg.frame_us);
            if shift > max || hold - shift > max {
                return self.refuse(Refused::TooEarly);
            }
            self.catch_up(seq);
            return Ok(self.take(now, seq, frames, bytes, Accepted::CaughtUp));
        }
        if self.count == N {
            if !newest {
                return self.refuse(Refused::Full);
            }
            self.catch_up(seq);
            return Ok(self.take(now, seq, frames, bytes, Accepted::CaughtUp));
        }
        Ok(self.take(now, seq, frames, bytes, Accepted::Queued))
    }

    /// When the next held packet is due, if any. The firmware sets its timer
    /// to this and calls [`Pacer::poll`] then.
    pub fn next_release(&self) -> Option<Tick> {
        self.oldest().map(|o| self.due(self.slots[o].seq))
    }

    /// Release the next packet if it is due at `now`: its bytes go to `out`.
    /// Call again until it returns `None` (more than one can be due if the
    /// caller polled late).
    pub fn poll(&mut self, now: Tick, out: &mut [u8; B]) -> Option<Release> {
        let o = self.oldest()?;
        let s = self.slots[o];
        let at = self.due(s.seq);
        if tick_diff(at, now) > 0 {
            return None;
        }
        let len = usize::from(s.len);
        out[..len].copy_from_slice(&s.data[..len]);
        self.slots[o].used = false;
        self.count -= 1;
        let lost = if self.have_next {
            seq_diff(s.seq, self.next_seq).max(0) as u16
        } else {
            0
        };
        self.have_next = true;
        self.next_seq = s.seq.wrapping_add(u16::from(s.frames));
        // Re-base the schedule on this packet so frame differences stay small.
        self.anchor_seq = s.seq;
        self.anchor_at = at;
        self.last_activity = now;
        self.stats.released = self.stats.released.wrapping_add(1);
        self.stats.lost_frames = self.stats.lost_frames.wrapping_add(u32::from(lost));
        Some(Release {
            seq: s.seq,
            frames: s.frames,
            len,
            at,
            lost_frames: lost,
        })
    }
}

#[cfg(test)]
mod tests;
