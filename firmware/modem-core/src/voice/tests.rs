//! Host tests for the voice pacer. Time is simulated: a test's clock is an
//! `i64` of microseconds from its start, turned into a [`Tick`] that starts
//! just before the 32-bit wrap so every run crosses it.

use super::*;
use std::vec::Vec;

const F: i64 = FRAME_US as i64;
/// Packet interval on the planned setting (2 frames of 40 ms).
const IV: i64 = 2 * F;
/// Start 2 s before the tick counter wraps.
const T0: Tick = u32::MAX - 2_000_000;

fn tick(t: i64) -> Tick {
    T0.wrapping_add(t as u32)
}

/// Absolute test time of a tick (valid within ±35 min of T0).
fn time(k: Tick) -> i64 {
    i64::from(k.wrapping_sub(T0) as i32)
}

/// Packet bytes: 30 B like the planned 700 bit/s packet, the frame number
/// in front so a test can check it got the right bytes back.
fn bytes(seq: u16) -> [u8; 30] {
    let mut b = [0xA5u8; 30];
    b[..2].copy_from_slice(&seq.to_le_bytes());
    b
}

fn rx() -> PlayoutBuffer {
    PlayoutBuffer::new(Config::RX).unwrap()
}

fn tx() -> TxScheduler {
    TxScheduler::new(Config::TX).unwrap()
}

#[derive(Debug, Clone, Copy)]
struct Out {
    seq: u16,
    frames: u8,
    at: i64,
    lost: u16,
    resyncs: u32,
    catch_up_frames: u32,
}

/// Drives a pacer: polls at every due time up to `t`, then pushes.
struct Drive<const N: usize, const B: usize> {
    p: Pacer<N, B>,
    now: i64,
    out: Vec<Out>,
}

impl<const N: usize, const B: usize> Drive<N, B> {
    fn new(p: Pacer<N, B>) -> Self {
        Self {
            p,
            now: 0,
            out: Vec::new(),
        }
    }

    fn run_to(&mut self, t: i64) {
        let mut buf = [0u8; B];
        while let Some(due) = self.p.next_release() {
            let due = time(due).max(self.now);
            if due > t {
                break;
            }
            self.now = due;
            let r = self
                .p
                .poll(tick(due), &mut buf)
                .expect("due packet released");
            assert_eq!(time(r.at), due, "released at its scheduled time");
            assert_eq!(&buf[..2], &r.seq.to_le_bytes(), "bytes belong to seq");
            let st = self.p.stats();
            self.out.push(Out {
                seq: r.seq,
                frames: r.frames,
                at: due,
                lost: r.lost_frames,
                resyncs: st.resyncs,
                catch_up_frames: st.catch_up_frames,
            });
        }
        self.now = self.now.max(t);
    }

    fn push(&mut self, t: i64, seq: u16, frames: u8) -> Result<Accepted, Refused> {
        self.run_to(t);
        let r = self.p.push(tick(t), seq, frames, &bytes(seq));
        assert!(self.p.len() <= N);
        r
    }

    fn finish(&mut self) {
        self.run_to(self.now + 60_000_000);
        assert!(self.p.is_empty());
    }
}

/// Every pair of consecutive releases on one schedule (no resync between) is
/// spaced exactly by the frames between them, less any caught-up frames.
fn assert_even(out: &[Out]) {
    assert_even_f(out, F);
}

fn assert_even_f(out: &[Out], f: i64) {
    for w in out.windows(2) {
        let (a, b) = (w[0], w[1]);
        let frames = i64::from(b.seq.wrapping_sub(a.seq) as i16);
        assert!(
            frames >= i64::from(a.frames),
            "in order, no overlap: {a:?} {b:?}"
        );
        if b.resyncs == a.resyncs {
            let cu = i64::from(b.catch_up_frames - a.catch_up_frames);
            assert_eq!(b.at - a.at, (frames - cu) * f, "even: {a:?} {b:?}");
        }
    }
}

#[test]
fn in_order_plays_evenly_at_the_set_delay() {
    let mut d = Drive::new(rx());
    let lat = 70_000; // air delay, constant
    for i in 0..50u16 {
        let t = (i64::from(i) + 1) * IV + lat;
        let want = if i == 0 {
            Accepted::Started
        } else {
            Accepted::Queued
        };
        assert_eq!(d.push(t, i * 2, 2), Ok(want));
    }
    d.finish();
    assert_eq!(d.out.len(), 50);
    let first = IV + lat + i64::from(RX_DELAY_US);
    for (i, o) in d.out.iter().enumerate() {
        assert_eq!(o.seq, i as u16 * 2);
        assert_eq!(o.at, first + i as i64 * IV);
        assert_eq!(o.lost, 0);
    }
    assert_eq!(d.p.stats().lost_frames, 0);
}

#[test]
fn lost_packet_leaves_its_time_as_a_gap() {
    let mut d = Drive::new(rx());
    for i in (0..10u16).filter(|&i| i != 4 && i != 5) {
        d.push((i64::from(i) + 1) * IV, i * 2, 2).unwrap();
    }
    d.finish();
    assert_eq!(d.out.len(), 8);
    let six = d.out.iter().find(|o| o.seq == 12).unwrap();
    let three = d.out.iter().find(|o| o.seq == 6).unwrap();
    assert_eq!(six.lost, 4);
    assert_eq!(six.at - three.at, 3 * IV);
    assert_even(&d.out);
}

#[test]
fn late_packet_is_dropped_while_later_ones_are_held() {
    let mut d = Drive::new(rx());
    let mut arrivals: Vec<(i64, u16)> = (0..10u16)
        .map(|i| {
            let t = (i64::from(i) + 1) * IV;
            // Packet 4 comes 200 ms late: past its play time (160 ms delay).
            (if i == 4 { t + 200_000 } else { t }, i * 2)
        })
        .collect();
    arrivals.sort_by_key(|a| a.0);
    for (t, seq) in arrivals {
        let r = d.push(t, seq, 2);
        if seq == 8 {
            assert_eq!(r, Err(Refused::Late));
        } else {
            assert!(r.is_ok());
        }
    }
    d.finish();
    assert_eq!(d.out.len(), 9);
    assert_eq!(d.p.stats().refused_late, 1);
    assert_eq!(d.out.iter().find(|o| o.seq == 10).unwrap().lost, 2);
    assert_even(&d.out);
}

#[test]
fn duplicates_are_refused_held_or_played() {
    let mut d = Drive::new(rx());
    d.push(IV, 0, 2).unwrap();
    d.push(2 * IV, 2, 2).unwrap();
    assert_eq!(d.push(2 * IV + 5_000, 2, 2), Err(Refused::Duplicate));
    // Overlapping but not equal (a frame count that disagrees) is a duplicate too.
    assert_eq!(d.push(2 * IV + 6_000, 3, 2), Err(Refused::Duplicate));
    d.run_to(IV + i64::from(RX_DELAY_US) + 1); // seq 0 plays
    assert_eq!(d.push(d.now + 1, 0, 2), Err(Refused::Stale));
    d.finish();
    assert_eq!(d.out.len(), 2);
    assert_eq!(d.p.stats().refused_duplicate, 2);
    assert_eq!(d.p.stats().refused_stale, 1);
}

#[test]
fn out_of_order_inside_the_delay_plays_in_order() {
    let mut d = Drive::new(rx());
    // The very first packet overtaken too: seq 2 arrives before seq 0.
    d.push(IV, 2, 2).unwrap();
    d.push(IV + 20_000, 0, 2).unwrap();
    d.push(3 * IV + 50_000, 6, 2).unwrap();
    d.push(3 * IV + 60_000, 4, 2).unwrap();
    d.finish();
    let seqs: Vec<u16> = d.out.iter().map(|o| o.seq).collect();
    assert_eq!(seqs, [0, 2, 4, 6]);
    assert_eq!(d.out[1].at - d.out[0].at, IV);
    assert_even(&d.out);
}

#[test]
fn burst_after_gap_restarts_and_catches_up() {
    let mut d = Drive::new(rx());
    let gap = 600_000;
    let mut arrivals: Vec<(i64, u16)> = (0..40u16)
        .map(|i| {
            let made = (i64::from(i) + 1) * IV;
            // Packets 10 to 19 are held up somewhere (a relay) and come in a burst.
            let t = if (10..20).contains(&i) {
                11 * IV + gap + i64::from(i - 10) * 1_000
            } else {
                made
            };
            (t, i * 2)
        })
        .collect();
    arrivals.sort_by_key(|a| a.0);
    for (t, seq) in arrivals {
        let _ = d.push(t, seq, 2);
    }
    d.finish();
    let st = d.p.stats();
    assert!(st.resyncs >= 1, "{st:?}");
    assert!(st.catch_up_drops >= 1, "{st:?}");
    assert_even(&d.out);
    // After the burst the delay is back inside the bound, and the stream ends
    // on time: the last packet plays within max hold of its arrival.
    let last = d.out.last().unwrap();
    assert_eq!(last.seq, 78);
    assert!(last.at - 40 * IV <= i64::from(RX_MAX_HOLD_US));
}

/// A talker whose clock runs `ppm` fast (positive) or slow against ours.
fn drift_run(ppm: i64, secs: i64) -> (Drive<SLOTS, SLOT_BYTES>, i64) {
    let mut d = Drive::new(rx());
    let n = secs * 1_000_000 / IV;
    let mut worst = 0;
    for i in 0..n {
        let made = (i + 1) * IV * 1_000_000 / (1_000_000 + ppm);
        let t = made + 60_000;
        let _ = d.push(t, (i * 2) as u16, 2);
    }
    d.finish();
    for o in &d.out {
        let i = i64::from(o.seq) / 2; // n < 32768 frames, no wrap here
        let arr = (i + 1) * IV * 1_000_000 / (1_000_000 + ppm) + 60_000;
        worst = worst.max(o.at - arr);
        assert!(o.at >= arr);
    }
    assert_even(&d.out);
    (d, worst)
}

#[test]
fn small_drift_costs_nothing() {
    // 100 ppm (two crystals at 50 ppm each) over 60 s.
    for ppm in [-100, 100] {
        let (d, worst) = drift_run(ppm, 60);
        let st = d.p.stats();
        assert_eq!(
            st.resyncs + st.catch_up_drops + st.lost_frames,
            0,
            "{ppm}: {st:?}"
        );
        assert_eq!(d.out.len(), 750);
        assert!(worst <= i64::from(RX_DELAY_US) + 10_000);
    }
}

#[test]
fn large_drift_stays_bounded() {
    // 3000 ppm over 120 s moves the clocks 360 ms apart: more than the delay.
    let (slow, worst) = drift_run(-3000, 120);
    let st = slow.p.stats();
    assert!(st.resyncs >= 1 && st.resyncs <= 4, "slow: {st:?}");
    assert!(worst <= i64::from(RX_MAX_HOLD_US));
    let (fast, worst) = drift_run(3000, 120);
    let st = fast.p.stats();
    assert!(
        st.catch_up_drops >= 1 && st.catch_up_drops <= 6,
        "fast: {st:?}"
    );
    assert!(worst <= i64::from(RX_MAX_HOLD_US));
    // Either way nearly everything plays.
    assert!(slow.out.len() >= 1490 && fast.out.len() >= 1490);
}

#[test]
fn overflow_drops_the_oldest_and_refuses_fill_ins() {
    let cfg = Config {
        frame_us: 20_000,
        target_us: 500_000,
        max_hold_us: 1_000_000,
    };
    let mut d = Drive::new(Pacer::<4, 32>::new(cfg).unwrap());
    for s in 0..4u16 {
        d.push(1_000 + i64::from(s), s * 2, 1).unwrap(); // every other frame
    }
    // Full. A fill-in (seq 1) is refused; a newer packet pushes out seq 0.
    assert_eq!(d.push(2_000, 1, 1), Err(Refused::Full));
    assert_eq!(d.push(2_001, 8, 1), Ok(Accepted::CaughtUp));
    assert_eq!(d.p.len(), 4);
    let st = d.p.stats();
    assert_eq!(
        (st.refused_full, st.catch_up_drops, st.catch_up_frames),
        (1, 1, 2)
    );
    d.finish();
    let seqs: Vec<u16> = d.out.iter().map(|o| o.seq).collect();
    assert_eq!(seqs, [2, 4, 6, 8]);
    // Seq 2 took seq 0's time.
    assert_eq!(d.out[0].at, 1_000 + 500_000);
    assert_even_f(&d.out, 20_000);
}

#[test]
fn too_early_is_refused_while_others_are_held() {
    let mut d = Drive::new(rx());
    d.push(IV, 0, 2).unwrap();
    // A frame number 10 s ahead: no single catch-up step can make room.
    assert_eq!(d.push(IV + 1, 500, 2), Err(Refused::TooEarly));
    d.finish();
    assert_eq!(d.out.len(), 1);
}

#[test]
fn ticks_and_frame_numbers_wrap() {
    let mut d = Drive::new(rx());
    // T0 wraps 2 s in; frame numbers wrap after 10 packets.
    let base: u16 = u16::MAX - 19;
    for i in 0..80u16 {
        d.push((i64::from(i) + 1) * IV, base.wrapping_add(i * 2), 2)
            .unwrap();
    }
    d.finish();
    assert_eq!(d.out.len(), 80);
    assert_eq!(d.out[10].seq, 0);
    assert_eq!(d.p.stats().lost_frames, 0);
    assert_even(&d.out);
}

#[test]
fn talker_restart_is_heard_after_forget_time() {
    let mut d = Drive::new(rx());
    d.push(IV, 1000, 2).unwrap();
    d.push(2 * IV, 1002, 2).unwrap();
    d.run_to(3_000_000);
    // A lower frame number soon after: stale (a late duplicate, say).
    assert_eq!(d.push(3_000_000, 0, 2), Err(Refused::Stale));
    // Long after: a new stream.
    let t = 3_000_000 + i64::from(FORGET_US);
    assert_eq!(d.push(t, 0, 2), Ok(Accepted::Started));
    d.finish();
    assert_eq!(d.out.len(), 3);
}

#[test]
fn new_talk_spurt_resyncs() {
    let mut d = Drive::new(rx());
    for i in 0..5u16 {
        d.push((i64::from(i) + 1) * IV, i * 2, 2).unwrap();
    }
    // 2 s later the frame count carries on where it stopped: the packet is
    // off the schedule (late) and nothing is held, so the schedule restarts.
    let t = 2_000_000;
    assert_eq!(d.push(t, 10, 2), Ok(Accepted::Resynced));
    d.push(t + IV, 12, 2).unwrap();
    d.finish();
    assert_eq!(d.out[5].at, t + i64::from(RX_DELAY_US));
    assert_eq!(d.out[6].at - d.out[5].at, IV);
}

#[test]
fn bad_input_and_bad_config_refused() {
    let mut p = rx();
    assert_eq!(p.push(0, 0, 2, &[]), Err(Refused::Bad));
    assert_eq!(p.push(0, 0, 2, &[0; SLOT_BYTES + 1]), Err(Refused::Bad));
    assert_eq!(p.push(0, 0, 0, &[1]), Err(Refused::Bad));
    assert_eq!(p.push(0, 0, MAX_FRAMES + 1, &[1]), Err(Refused::Bad));
    assert_eq!(p.stats().refused_bad, 4);
    assert!(p.is_empty());
    let bad = Config {
        target_us: 2,
        max_hold_us: 1,
        ..Config::RX
    };
    assert!(PlayoutBuffer::new(bad).is_none());
    assert!(Pacer::<0, 8>::new(Config::RX).is_none());
    assert!(Config::TX.is_valid() && Config::RX.is_valid());
}

#[test]
fn tx_evens_out_a_bursty_phone_link() {
    // The phone makes a packet every 80 ms but its link hands them over two
    // at a time every 160 ms.
    let mut d = Drive::new(tx());
    for i in 0..40u16 {
        let made = (i64::from(i) + 1) * IV;
        let t = (made + 2 * IV - 1) / (2 * IV) * (2 * IV) + 7_000;
        assert!(d.push(t, i * 2, 2).is_ok());
    }
    d.finish();
    assert_eq!(d.out.len(), 40);
    let st = d.p.stats();
    assert_eq!(st.resyncs + st.catch_up_drops, 0, "{st:?}");
    for w in d.out.windows(2) {
        assert_eq!(w[1].at - w[0].at, IV);
    }
}

#[test]
fn tx_restarts_when_the_phone_falls_behind() {
    let mut d = Drive::new(tx());
    for i in 0..5u16 {
        d.push((i64::from(i) + 1) * IV, i * 2, 2).unwrap();
    }
    // The link stalls 200 ms: the next packet is late and nothing is held.
    let t = 6 * IV + 200_000;
    assert_eq!(d.push(t, 10, 2), Ok(Accepted::Resynced));
    d.finish();
    assert_eq!(d.out[5].at, t + i64::from(TX_LEAD_US));
}

#[test]
fn memory_is_fixed() {
    // Pinned so a change in size is a visible decision. Slots: 16 x 134 B.
    let size = core::mem::size_of::<PlayoutBuffer>();
    std::println!("PlayoutBuffer / TxScheduler: {size} bytes");
    assert_eq!(size, 2232);
    assert_eq!(core::mem::size_of::<TxScheduler>(), size);
}

/// A small seeded generator (SplitMix64), so the randomized test needs no
/// outside crate and repeats exactly.
struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }
    fn below(&mut self, n: u64) -> u64 {
        self.next() % n
    }
    fn chance(&mut self, per_mille: u64) -> bool {
        self.below(1000) < per_mille
    }
}

/// Random loss, duplicates, jitter (which reorders), delay spikes, a pause
/// in the talk and drift, over many seeds. Checks: never a frame twice,
/// never out of order, the right bytes, even spacing on one schedule, every
/// release within max hold of its arrival, never more than N held.
#[test]
fn randomized_invariants() {
    let mut seen = VoiceStats::default();
    for seed in 0..300u64 {
        let mut r = Rng(seed);
        let tx_side = r.chance(300);
        let cfg = if tx_side { Config::TX } else { Config::RX };
        let fpp = [1u8, 2, 4][r.below(3) as usize];
        let loss = r.below(200);
        let dup = r.below(100);
        let jitter = r.below(250_000) as i64;
        let spike = r.below(30);
        let ppm = r.below(4001) as i64 - 2000;
        let n = 150 + r.below(250) as i64;
        let pause_at = r.below(n as u64) as i64;
        let pause = if r.chance(500) {
            r.below(3_000_000) as i64
        } else {
            0
        };
        let start_seq = r.next() as u16;

        let mut arrivals: Vec<(i64, u16)> = Vec::new();
        let step = i64::from(fpp) * F;
        for i in 0..n {
            let talk = (i + 1) * step + if i >= pause_at { pause } else { 0 };
            let made = talk * 1_000_000 / (1_000_000 + ppm);
            // Frame numbers count talker time, pause included.
            let seq = start_seq.wrapping_add(((talk / F) as u16).wrapping_sub(u16::from(fpp)));
            if r.chance(loss) {
                continue;
            }
            let mut t = made + 20_000 + r.below(jitter as u64 + 1) as i64;
            if r.chance(spike) {
                t += r.below(800_000) as i64;
            }
            arrivals.push((t, seq));
            if r.chance(dup) {
                arrivals.push((t + r.below(600_000) as i64, seq));
            }
        }
        arrivals.sort_by_key(|a| a.0);

        let mut d = Drive::new(Pacer::<SLOTS, SLOT_BYTES>::new(cfg).unwrap());
        let mut taken: Vec<(u16, i64)> = Vec::new();
        for &(t, seq) in &arrivals {
            if d.push(t, seq, fpp).is_ok() {
                taken.push((seq, t));
            }
        }
        d.finish();

        assert_even(&d.out);
        for w in d.out.windows(2) {
            assert!(
                i64::from(w[1].seq.wrapping_sub(w[0].seq) as i16) > 0,
                "seed {seed}"
            );
        }
        for o in &d.out {
            // The copy that was taken (a refused duplicate never counts).
            let (_, arr) = *taken.iter().rev().find(|x| x.0 == o.seq).unwrap();
            assert!(o.at >= arr, "seed {seed}: released before arrival");
            assert!(
                o.at - arr <= i64::from(cfg.max_hold_us),
                "seed {seed}: held {} µs",
                o.at - arr
            );
        }
        let st = d.p.stats();
        assert_eq!(st.released as usize, d.out.len());
        assert_eq!(st.accepted, st.released + st.catch_up_drops, "seed {seed}");
        seen.refused_duplicate += st.refused_duplicate;
        seen.refused_stale += st.refused_stale;
        seen.refused_late += st.refused_late;
        seen.refused_too_early += st.refused_too_early;
        seen.resyncs += st.resyncs;
        seen.catch_up_drops += st.catch_up_drops;
        seen.lost_frames += st.lost_frames;
    }
    // The runs reached every path, so the checks above mean something.
    std::println!("randomized runs: {seen:?}");
    assert!(seen.refused_duplicate > 0 && seen.refused_stale > 0 && seen.refused_late > 0);
    assert!(seen.resyncs > 0 && seen.catch_up_drops > 0 && seen.lost_frames > 0);
}

/// On a clean link the pacer delivers everything with no extra events.
#[test]
fn randomized_clean_link_delivers_all() {
    for seed in 0..100u64 {
        let mut r = Rng(1000 + seed);
        let jitter = r.below(70_000) as i64; // under the 160 ms delay, with room for drift
        let ppm = r.below(201) as i64 - 100;
        let mut arrivals: Vec<(i64, u16)> = Vec::new();
        for i in 0..300i64 {
            let made = (i + 1) * IV * 1_000_000 / (1_000_000 + ppm);
            arrivals.push((
                made + 30_000 + r.below(jitter as u64 + 1) as i64,
                (i * 2) as u16,
            ));
        }
        arrivals.sort_by_key(|a| a.0);
        let mut d = Drive::new(rx());
        for &(t, seq) in &arrivals {
            d.push(t, seq, 2).unwrap();
        }
        d.finish();
        assert_eq!(d.out.len(), 300, "seed {seed}");
        let st = d.p.stats();
        assert_eq!(
            st.resyncs + st.catch_up_drops + st.lost_frames,
            0,
            "seed {seed}"
        );
        assert_even(&d.out);
    }
}
