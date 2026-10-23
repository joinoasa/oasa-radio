//! The real modem engine (firmware/modem-core) on the far side of a
//! pseudo-terminal, with a simulated radio. The host code under test talks to
//! it exactly as it would talk to the board's serial port.
#![allow(dead_code)]

use std::fs::File;
use std::io::{Read, Write};
use std::os::fd::AsFd;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::Instant;

use nix::poll::{poll, PollFd, PollFlags, PollTimeout};
use oasa_modem_core::modem::{FrameSink, Info, Modem};
use oasa_modem_core::settings::{Limits, Settings};
use oasa_modem_core::sim::{Air, SimRadio};
use oasa_radio::relay::Pty;

struct VecSink(Vec<Vec<u8>>);

impl FrameSink for VecSink {
    fn frame(&mut self, wire: &[u8]) -> bool {
        self.0.push(wire.to_vec());
        true
    }
}

pub struct SimModem {
    /// Give to `--device`.
    pub path: PathBuf,
    /// The simulated radio, for injecting and inspecting.
    pub radio: SimRadio,
    stop: Arc<AtomicBool>,
    handle: Option<JoinHandle<()>>,
}

impl SimModem {
    pub fn start(air: &Air) -> Self {
        Self::start_with(air, Settings::LAB_SF7_BW125)
    }

    pub fn start_with(air: &Air, settings: Settings) -> Self {
        let pty = Pty::open().unwrap();
        let radio = air.radio();
        let path = pty.path.clone();
        let stop = Arc::new(AtomicBool::new(false));
        let (r, st) = (radio.clone(), stop.clone());
        let handle = std::thread::spawn(move || run(pty, r, settings, st));
        Self {
            path,
            radio,
            stop,
            handle: Some(handle),
        }
    }
}

impl Drop for SimModem {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::SeqCst);
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
    }
}

fn run(pty: Pty, radio: SimRadio, settings: Settings, stop: Arc<AtomicBool>) {
    let mut port = File::from(pty.master.try_clone().unwrap());
    let mut m: Modem<SimRadio, 8> = Modem::new(
        radio.clone(),
        settings,
        Limits::HELTEC_V3,
        Info {
            fw: [0, 1, 0],
            board: "sim",
        },
    );
    let t0 = Instant::now();
    let now = || t0.elapsed().as_millis() as u32;
    m.start(now());
    let mut sink = VecSink(Vec::new());
    let mut buf = [0u8; 512];
    while !stop.load(Ordering::SeqCst) {
        let mut fds = [PollFd::new(pty.master.as_fd(), PollFlags::POLLIN)];
        if poll(&mut fds, PollTimeout::from(1u16)).unwrap_or(0) > 0 {
            if let Ok(n) = port.read(&mut buf) {
                if n > 0 {
                    m.on_serial(&buf[..n], now(), &mut sink);
                }
            }
        }
        for _ in 0..100 {
            if m.wants_irq() && radio.irq_pending() {
                m.on_irq(now(), &mut sink);
            } else {
                break;
            }
        }
        m.on_tick(now());
        for f in sink.0.drain(..) {
            let _ = port.write_all(&f);
        }
    }
}
