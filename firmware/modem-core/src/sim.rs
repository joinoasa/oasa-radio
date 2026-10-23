//! A simulated radio and air, for tests. Needs the `sim` feature (and std).
//!
//! [`Air`] joins any number of [`SimRadio`]s. A frame sent by one is heard by
//! every other radio that is listening with the same frequency, spreading
//! factor, bandwidth, coding rate and sync word. Sending takes no time. The
//! test can corrupt or drop frames and inject frames of its own.

use std::collections::VecDeque;
use std::sync::{Arc, Mutex};
use std::vec::Vec;

use crate::frame::MAX_LORA_PAYLOAD;
use crate::modem::{irq, RadioDriver, RadioFault, RxMeta};
use crate::settings::Settings;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Mode {
    Standby,
    Rx,
    Tx,
}

struct Heard {
    payload: Vec<u8>,
    crc_ok: bool,
    meta: RxMeta,
}

struct Node {
    settings: Option<Settings>,
    mode: Mode,
    irq: u16,
    heard: VecDeque<Heard>,
    current: Option<Heard>,
    /// Fail the next calls of this kind.
    fail_init: u32,
    fail_tx: u32,
    /// Never finish a transmission.
    hang_tx: bool,
    sent: Vec<Vec<u8>>,
    configures: u32,
}

/// What happens to the next frames sent on the air.
#[derive(Clone, Copy, Debug, Default)]
pub struct Fault {
    /// Flip a bit and mark the CRC bad, for this many frames.
    pub corrupt: u32,
    /// Lose this many frames.
    pub drop: u32,
}

struct Inner {
    nodes: Vec<Node>,
    fault: Fault,
}

/// The shared air.
#[derive(Clone)]
pub struct Air(Arc<Mutex<Inner>>);

impl Default for Air {
    fn default() -> Self {
        Self::new()
    }
}

impl Air {
    pub fn new() -> Self {
        Air(Arc::new(Mutex::new(Inner {
            nodes: Vec::new(),
            fault: Fault::default(),
        })))
    }

    /// A new radio on this air.
    pub fn radio(&self) -> SimRadio {
        let mut g = self.0.lock().unwrap();
        g.nodes.push(Node {
            settings: None,
            mode: Mode::Standby,
            irq: 0,
            heard: VecDeque::new(),
            current: None,
            fail_init: 0,
            fail_tx: 0,
            hang_tx: false,
            sent: Vec::new(),
            configures: 0,
        });
        SimRadio {
            air: self.clone(),
            id: g.nodes.len() - 1,
        }
    }

    /// Set what happens to the next frames.
    pub fn set_fault(&self, f: Fault) {
        self.0.lock().unwrap().fault = f;
    }
}

/// One simulated radio.
pub struct SimRadio {
    air: Air,
    id: usize,
}

fn same_channel(a: &Settings, b: &Settings) -> bool {
    (a.freq_hz, a.sf, a.bw, a.cr, a.sync_word) == (b.freq_hz, b.sf, b.bw, b.cr, b.sync_word)
}

impl SimRadio {
    fn with<T>(&self, f: impl FnOnce(&mut Node) -> T) -> T {
        let mut g = self.air.0.lock().unwrap();
        f(&mut g.nodes[self.id])
    }

    /// Is the interrupt line high?
    pub fn irq_pending(&self) -> bool {
        self.with(|n| n.irq != 0)
    }

    /// Hand this radio a frame as if it came off the air (needs it to be listening).
    pub fn inject(&self, payload: &[u8], crc_ok: bool, rssi_qdb: i16, snr_qdb: i16) {
        self.with(|n| {
            if n.mode == Mode::Rx {
                n.heard.push_back(Heard {
                    payload: payload.to_vec(),
                    crc_ok,
                    meta: RxMeta {
                        len: payload.len(),
                        rssi_qdb,
                        snr_qdb,
                        signal_rssi_qdb: rssi_qdb,
                    },
                });
                n.irq |= irq::RX_DONE;
            }
        });
    }

    /// Raise a header error, as the chip does for a damaged LoRa header.
    pub fn inject_header_error(&self) {
        self.with(|n| {
            if n.mode == Mode::Rx {
                n.irq |= irq::HEADER_ERR;
            }
        });
    }

    /// Fail the next `n` calls to `init`.
    pub fn fail_init(&self, n: u32) {
        self.with(|x| x.fail_init = n);
    }

    /// Fail the next `n` calls to `start_tx`.
    pub fn fail_tx(&self, n: u32) {
        self.with(|x| x.fail_tx = n);
    }

    /// Start transmissions but never finish them.
    pub fn hang_tx(&self, on: bool) {
        self.with(|n| n.hang_tx = on);
    }

    /// Everything this radio has sent on the air.
    pub fn sent(&self) -> Vec<Vec<u8>> {
        self.with(|n| n.sent.clone())
    }

    /// How many times `configure` ran.
    pub fn configures(&self) -> u32 {
        self.with(|n| n.configures)
    }

    pub fn is_listening(&self) -> bool {
        self.with(|n| n.mode == Mode::Rx)
    }
}

impl RadioDriver for SimRadio {
    fn init(&mut self) -> Result<(), RadioFault> {
        self.with(|n| {
            if n.fail_init > 0 {
                n.fail_init -= 1;
                return Err(RadioFault("sim: init"));
            }
            n.mode = Mode::Standby;
            n.irq = 0;
            Ok(())
        })
    }

    fn configure(&mut self, s: &Settings) -> Result<(), RadioFault> {
        self.with(|n| {
            n.settings = Some(*s);
            n.mode = Mode::Standby;
            n.irq = 0;
            n.configures += 1;
            Ok(())
        })
    }

    fn start_rx(&mut self) -> Result<(), RadioFault> {
        self.with(|n| {
            n.mode = Mode::Rx;
            Ok(())
        })
    }

    fn start_tx(&mut self, payload: &[u8], _timeout_ms: u32) -> Result<(), RadioFault> {
        assert!(payload.len() <= MAX_LORA_PAYLOAD && !payload.is_empty());
        let mut g = self.air.0.lock().unwrap();
        let me = self.id;
        if g.nodes[me].fail_tx > 0 {
            g.nodes[me].fail_tx -= 1;
            return Err(RadioFault("sim: tx"));
        }
        g.nodes[me].mode = Mode::Tx;
        g.nodes[me].sent.push(payload.to_vec());
        if g.nodes[me].hang_tx {
            return Ok(());
        }
        let mine = g.nodes[me].settings.expect("configured before tx");
        g.nodes[me].mode = Mode::Standby;
        g.nodes[me].irq |= irq::TX_DONE;
        let mut dropped = false;
        let mut corrupt = false;
        if g.fault.drop > 0 {
            g.fault.drop -= 1;
            dropped = true;
        } else if g.fault.corrupt > 0 {
            g.fault.corrupt -= 1;
            corrupt = true;
        }
        if dropped {
            return Ok(());
        }
        for (i, n) in g.nodes.iter_mut().enumerate() {
            if i == me || n.mode != Mode::Rx {
                continue;
            }
            if !n.settings.as_ref().is_some_and(|s| same_channel(s, &mine)) {
                continue;
            }
            let mut p = payload.to_vec();
            if corrupt {
                p[0] ^= 0x10;
            }
            n.heard.push_back(Heard {
                payload: p,
                crc_ok: !corrupt,
                meta: RxMeta {
                    len: payload.len(),
                    rssi_qdb: -400,
                    snr_qdb: 36,
                    signal_rssi_qdb: -404,
                },
            });
            n.irq |= irq::RX_DONE;
        }
        Ok(())
    }

    fn irq_status(&mut self) -> Result<u16, RadioFault> {
        self.with(|n| {
            let mut f = n.irq;
            n.irq = 0;
            // The chip holds one packet; the next one waits here.
            if f & irq::RX_DONE != 0 {
                n.current = n.heard.pop_front();
                if n.current.as_ref().is_some_and(|h| !h.crc_ok) {
                    f |= irq::CRC_ERR;
                }
                if !n.heard.is_empty() {
                    n.irq |= irq::RX_DONE;
                }
            }
            Ok(f)
        })
    }

    fn read_packet(&mut self, buf: &mut [u8; MAX_LORA_PAYLOAD]) -> Result<RxMeta, RadioFault> {
        self.with(|n| {
            let h = n.current.take().ok_or(RadioFault("sim: no packet"))?;
            buf[..h.payload.len()].copy_from_slice(&h.payload);
            Ok(h.meta)
        })
    }
}
