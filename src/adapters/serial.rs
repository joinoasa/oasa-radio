//! `serial`: transparent serial radios (SiK, mLRS, RFD900x, any UART modem).
//!
//! The device is a byte stream, so each packet is framed (COBS, length, CRC-16:
//! `docs/serial-framing.md`). Bad frames are dropped and counted; the next
//! delimiter resyncs. No flow control, 8N1, raw mode.

use std::collections::VecDeque;
use std::fs::File;
use std::io::{Read, Write};
use std::os::fd::AsFd;
use std::os::unix::fs::OpenOptionsExt;
use std::time::Duration;

use nix::poll::{poll, PollFd, PollFlags, PollTimeout};
use nix::sys::termios::{
    self, BaudRate, ControlFlags, FlushArg, InputFlags, LocalFlags, OutputFlags, SetArg,
};

use crate::framing::{FrameStats, Framer, DEFAULT_MTU};
use crate::radio::{Counters, Radio, RadioError, RadioInfo};

/// Default UART speed (the SiK factory setting).
pub const DEFAULT_BAUD: u32 = 57_600;

/// Settings for [`SerialRadio::open`].
#[derive(Debug, Clone)]
pub struct SerialConfig {
    pub device: String,
    pub baud: u32,
    /// Largest packet carried.
    pub mtu: usize,
}

impl SerialConfig {
    pub fn new(device: &str) -> Self {
        Self {
            device: device.to_string(),
            baud: DEFAULT_BAUD,
            mtu: DEFAULT_MTU,
        }
    }
}

/// UART speeds Linux terminals accept, by number.
pub fn baud_rate(baud: u32) -> Option<BaudRate> {
    Some(match baud {
        1200 => BaudRate::B1200,
        2400 => BaudRate::B2400,
        4800 => BaudRate::B4800,
        9600 => BaudRate::B9600,
        19_200 => BaudRate::B19200,
        38_400 => BaudRate::B38400,
        57_600 => BaudRate::B57600,
        115_200 => BaudRate::B115200,
        230_400 => BaudRate::B230400,
        460_800 => BaudRate::B460800,
        500_000 => BaudRate::B500000,
        576_000 => BaudRate::B576000,
        921_600 => BaudRate::B921600,
        1_000_000 => BaudRate::B1000000,
        1_152_000 => BaudRate::B1152000,
        1_500_000 => BaudRate::B1500000,
        2_000_000 => BaudRate::B2000000,
        2_500_000 => BaudRate::B2500000,
        3_000_000 => BaudRate::B3000000,
        3_500_000 => BaudRate::B3500000,
        4_000_000 => BaudRate::B4000000,
        _ => return None,
    })
}

/// Put an open terminal into raw 8N1 mode with no flow control at `baud`.
/// Also used on the slave side of test pseudo-terminals.
pub fn set_raw<F: AsFd>(fd: F, baud: BaudRate) -> Result<(), RadioError> {
    let io = |e: nix::Error| RadioError::Io(std::io::Error::from(e));
    let mut t = termios::tcgetattr(&fd).map_err(io)?;
    termios::cfmakeraw(&mut t);
    t.control_flags |= ControlFlags::CLOCAL | ControlFlags::CREAD | ControlFlags::CS8;
    t.control_flags &= !(ControlFlags::CRTSCTS
        | ControlFlags::CSTOPB
        | ControlFlags::PARENB
        | ControlFlags::HUPCL);
    t.input_flags &= !(InputFlags::IXON | InputFlags::IXOFF | InputFlags::IXANY);
    t.output_flags &= !OutputFlags::OPOST;
    t.local_flags &= !(LocalFlags::ECHO | LocalFlags::ICANON | LocalFlags::ISIG);
    termios::cfsetspeed(&mut t, baud).map_err(io)?;
    termios::tcsetattr(&fd, SetArg::TCSANOW, &t).map_err(io)?;
    termios::tcflush(&fd, FlushArg::TCIOFLUSH).map_err(io)?;
    Ok(())
}

pub struct SerialRadio {
    file: File,
    cfg: SerialConfig,
    framer: Framer,
    queue: VecDeque<Vec<u8>>,
    counters: Counters,
    seen_bad: u64,
    read_buf: Vec<u8>,
    wire: Vec<u8>,
}

impl SerialRadio {
    pub fn open(cfg: SerialConfig) -> Result<Self, RadioError> {
        let rate = baud_rate(cfg.baud)
            .ok_or_else(|| RadioError::Config(format!("--baud {} is not supported", cfg.baud)))?;
        let framer = Framer::new(cfg.mtu).map_err(RadioError::Config)?;
        let file = std::fs::OpenOptions::new()
            .read(true)
            .write(true)
            // Don't make the device our controlling terminal.
            .custom_flags(nix::libc::O_NOCTTY)
            .open(&cfg.device)
            .map_err(|e| RadioError::Config(format!("cannot open {}: {e}", cfg.device)))?;
        set_raw(&file, rate)?;
        Ok(Self {
            file,
            cfg,
            framer,
            queue: VecDeque::new(),
            counters: Counters::default(),
            seen_bad: 0,
            read_buf: vec![0; 4096],
            wire: Vec::new(),
        })
    }

    /// Why frames were rejected, by reason.
    pub fn frame_stats(&self) -> FrameStats {
        self.framer.stats()
    }

    fn take_new_bad_frames(&mut self) {
        let bad = self.framer.stats().bad_frame;
        self.counters.add_drop("bad_frame", bad - self.seen_bad);
        self.seen_bad = bad;
    }
}

impl Radio for SerialRadio {
    fn info(&self) -> RadioInfo {
        RadioInfo {
            kind: "serial",
            target: self.cfg.device.clone(),
            mtu: self.cfg.mtu,
            bitrate: Some(self.cfg.baud),
        }
    }

    fn send(&mut self, packet: &[u8]) -> Result<(), RadioError> {
        self.wire.clear();
        match self.framer.encode(packet, &mut self.wire) {
            Ok(()) => {}
            Err(crate::framing::FrameError::Empty) => {
                self.counters.add_drop("empty", 1);
                return Ok(());
            }
            Err(crate::framing::FrameError::TooLarge { .. }) => {
                self.counters.add_drop("too_large", 1);
                return Ok(());
            }
        }
        self.file.write_all(&self.wire)?;
        self.counters.sent += 1;
        self.counters.sent_bytes += packet.len() as u64;
        Ok(())
    }

    fn recv(&mut self, timeout: Duration) -> Result<Option<Vec<u8>>, RadioError> {
        if let Some(p) = self.queue.pop_front() {
            return Ok(Some(p));
        }
        let ms = u16::try_from(timeout.as_millis()).unwrap_or(u16::MAX);
        let mut fds = [PollFd::new(self.file.as_fd(), PollFlags::POLLIN)];
        match poll(&mut fds, PollTimeout::from(ms)) {
            Ok(0) => return Ok(None),
            Ok(_) => {}
            Err(nix::errno::Errno::EINTR) => return Ok(None),
            Err(e) => return Err(std::io::Error::from(e).into()),
        }
        let n = match self.file.read(&mut self.read_buf) {
            Ok(0) => {
                return Err(std::io::Error::new(
                    std::io::ErrorKind::UnexpectedEof,
                    "the serial device closed",
                )
                .into())
            }
            Ok(n) => n,
            Err(e) if e.kind() == std::io::ErrorKind::Interrupted => return Ok(None),
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => return Ok(None),
            Err(e) => return Err(e.into()),
        };
        for p in self.framer.push(&self.read_buf[..n]) {
            self.counters.received += 1;
            self.counters.received_bytes += p.len() as u64;
            self.queue.push_back(p);
        }
        self.take_new_bad_frames();
        Ok(self.queue.pop_front())
    }

    fn counters(&self) -> &Counters {
        &self.counters
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn baud_table_has_the_common_rates() {
        for b in [9600, 57_600, 115_200, 921_600] {
            assert!(baud_rate(b).is_some(), "{b}");
        }
        assert!(baud_rate(12_345).is_none());
        assert!(baud_rate(0).is_none());
    }

    #[test]
    fn open_errors_are_clear() {
        let e = SerialRadio::open(SerialConfig::new("/dev/does-not-exist"))
            .err()
            .unwrap()
            .to_string();
        assert!(e.contains("cannot open /dev/does-not-exist"), "{e}");
        let mut c = SerialConfig::new("/dev/null");
        c.baud = 7;
        assert!(SerialRadio::open(c)
            .err()
            .unwrap()
            .to_string()
            .contains("--baud 7"));
        let mut c = SerialConfig::new("/dev/null");
        c.mtu = 0;
        assert!(SerialRadio::open(c).is_err());
    }
}
