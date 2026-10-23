//! A small blocking SX1262 driver for LoRa.
//!
//! Why not `lora-phy`: version 3.0.1 reports a packet with a failed CRC as a
//! good one (it only logs the CRC error), and it cannot set a sync word other
//! than the two LoRaWAN ones. The modem needs both: CRC failures are counted
//! and shown in air mode, and the sync word is a setting.
//!
//! Command bytes, register addresses and the PA table follow the Semtech
//! SX1261/2 datasheet (DS.SX1261-2.W.APP). `lora-phy` (MIT or Apache-2.0)
//! was used as a cross-check of the same values.
//!
//! The driver is generic over `embedded-hal` 1.0 traits, so the host tests
//! drive it against a model of the chip.

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::spi::SpiBus;

use crate::frame::MAX_LORA_PAYLOAD;
use crate::modem::{irq, RadioDriver, RadioFault, RxMeta};
use crate::settings::{Bandwidth, Settings};

mod op {
    pub const SET_STANDBY: u8 = 0x80;
    pub const SET_TX: u8 = 0x83;
    pub const SET_RX: u8 = 0x82;
    pub const SET_PACKET_TYPE: u8 = 0x8A;
    pub const SET_RF_FREQUENCY: u8 = 0x86;
    pub const SET_TX_PARAMS: u8 = 0x8E;
    pub const SET_PA_CONFIG: u8 = 0x95;
    pub const SET_BUFFER_BASE: u8 = 0x8F;
    pub const SET_MODULATION: u8 = 0x8B;
    pub const SET_PACKET_PARAMS: u8 = 0x8C;
    pub const GET_RX_BUFFER_STATUS: u8 = 0x13;
    pub const GET_PACKET_STATUS: u8 = 0x14;
    pub const CFG_DIO_IRQ: u8 = 0x08;
    pub const GET_IRQ_STATUS: u8 = 0x12;
    pub const CLR_IRQ_STATUS: u8 = 0x02;
    pub const CALIBRATE: u8 = 0x89;
    pub const CALIBRATE_IMAGE: u8 = 0x98;
    pub const SET_REGULATOR_MODE: u8 = 0x96;
    pub const CLEAR_DEVICE_ERRORS: u8 = 0x07;
    pub const SET_TCXO_MODE: u8 = 0x97;
    pub const SET_DIO2_RF_SWITCH: u8 = 0x9D;
    pub const GET_STATUS: u8 = 0xC0;
    pub const WRITE_REGISTER: u8 = 0x0D;
    pub const READ_REGISTER: u8 = 0x1D;
    pub const WRITE_BUFFER: u8 = 0x0E;
    pub const READ_BUFFER: u8 = 0x1E;
}

mod reg {
    pub const IQ_POLARITY: u16 = 0x0736;
    pub const LORA_SYNC_WORD: u16 = 0x0740;
    pub const TX_MODULATION: u16 = 0x0889;
    pub const RX_GAIN: u16 = 0x08AC;
    pub const TX_CLAMP: u16 = 0x08D8;
}

/// Board wiring facts the chip cannot know.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    /// DIO3 powers a TCXO at this voltage code (0x02 is 1.8 V). `None`: crystal.
    pub tcxo_code: Option<u8>,
    /// Use the DC-DC regulator (the board needs its inductor).
    pub dcdc: bool,
    /// DIO2 drives the antenna switch.
    pub dio2_rf_switch: bool,
    /// Boosted receive gain (about 2 mA more, better sensitivity).
    pub rx_boost: bool,
}

/// The Heltec WiFi LoRa 32 V3: 1.8 V TCXO on DIO3, antenna switch on DIO2.
/// Same as Meshtastic's `heltec_v3` variant. DC-DC is the RadioLib and
/// Meshtastic default for this board.
pub const HELTEC_V3: Config = Config {
    tcxo_code: Some(0x02),
    dcdc: true,
    dio2_rf_switch: true,
    rx_boost: true,
};

/// Polls of BUSY (10 us apart) before giving up: 200 ms.
const BUSY_POLLS: u32 = 20_000;

/// Image calibration bytes for the band (datasheet table 9-2).
pub fn image_cal(freq_hz: u32) -> [u8; 2] {
    match freq_hz {
        f if f > 900_000_000 => [0xE1, 0xE9],
        f if f > 850_000_000 => [0xD7, 0xDB],
        f if f > 770_000_000 => [0xC1, 0xC5],
        f if f > 460_000_000 => [0x75, 0x81],
        f if f > 425_000_000 => [0x6B, 0x6F],
        _ => [0x6B, 0x6F],
    }
}

/// The frequency register: `freq * 2^25 / 32 MHz`, rounded.
pub fn pll_step(freq_hz: u32) -> u32 {
    (((u64::from(freq_hz) << 25) + 16_000_000) / 32_000_000) as u32
}

/// PA settings for the SX1262 (datasheet table 13-21):
/// `(paDutyCycle, hpMax, SetTxParams power)`.
pub fn pa_params(dbm: i8) -> (u8, u8, u8) {
    let p = dbm.clamp(-9, 22);
    let (dc, hp, add) = match p {
        21..=22 => (0x04, 0x07, 22 - p),
        18..=20 => (0x03, 0x05, 2),
        15..=17 => (0x02, 0x03, 5),
        _ => (0x02, 0x02, 8),
    };
    let param = if p >= 21 { 22 } else { p + add };
    (dc, hp, param as u8)
}

pub struct Sx126x<SPI, CS, RST, BUSY, D> {
    spi: SPI,
    cs: CS,
    rst: RST,
    busy: BUSY,
    delay: D,
    cfg: Config,
    preamble: u16,
    scratch: [u8; MAX_LORA_PAYLOAD + 8],
}

type R<T> = Result<T, RadioFault>;

impl<SPI, CS, RST, BUSY, D> Sx126x<SPI, CS, RST, BUSY, D>
where
    SPI: SpiBus<u8>,
    CS: OutputPin,
    RST: OutputPin,
    BUSY: InputPin,
    D: DelayNs,
{
    pub fn new(spi: SPI, cs: CS, rst: RST, busy: BUSY, delay: D, cfg: Config) -> Self {
        Self {
            spi,
            cs,
            rst,
            busy,
            delay,
            cfg,
            preamble: 8,
            scratch: [0; MAX_LORA_PAYLOAD + 8],
        }
    }

    /// The SPI bus, for tests.
    pub fn bus(&mut self) -> &mut SPI {
        &mut self.spi
    }

    fn wait_busy(&mut self) -> R<()> {
        for _ in 0..BUSY_POLLS {
            if !self.busy.is_high().map_err(|_| RadioFault("busy pin"))? {
                return Ok(());
            }
            self.delay.delay_us(10);
        }
        Err(RadioFault("busy timeout"))
    }

    /// Run the command in `scratch[..n]`: wait for the chip, then one SPI transaction.
    fn run(&mut self, n: usize) -> R<()> {
        self.wait_busy()?;
        self.cs.set_low().map_err(|_| RadioFault("cs pin"))?;
        let r = self
            .spi
            .transfer_in_place(&mut self.scratch[..n])
            .and_then(|()| self.spi.flush());
        self.cs.set_high().map_err(|_| RadioFault("cs pin"))?;
        r.map_err(|_| RadioFault("spi"))
    }

    fn cmd(&mut self, bytes: &[u8]) -> R<()> {
        self.scratch[..bytes.len()].copy_from_slice(bytes);
        self.run(bytes.len())
    }

    /// Send `head`, clock out `skip + out.len()` more bytes and copy the last
    /// `out.len()` of them into `out`.
    fn query(&mut self, head: &[u8], skip: usize, out: &mut [u8]) -> R<()> {
        let total = head.len() + skip + out.len();
        self.scratch[..head.len()].copy_from_slice(head);
        self.scratch[head.len()..total].fill(0);
        self.run(total)?;
        out.copy_from_slice(&self.scratch[head.len() + skip..total]);
        Ok(())
    }

    fn write_reg(&mut self, addr: u16, data: &[u8]) -> R<()> {
        let a = addr.to_be_bytes();
        self.scratch[..3].copy_from_slice(&[op::WRITE_REGISTER, a[0], a[1]]);
        self.scratch[3..3 + data.len()].copy_from_slice(data);
        self.run(3 + data.len())
    }

    fn read_reg(&mut self, addr: u16) -> R<u8> {
        let a = addr.to_be_bytes();
        let mut v = [0u8];
        self.query(&[op::READ_REGISTER, a[0], a[1]], 1, &mut v)?;
        Ok(v[0])
    }

    /// Read, change, write back one register.
    fn update_reg(&mut self, addr: u16, f: impl FnOnce(u8) -> u8) -> R<()> {
        let v = self.read_reg(addr)?;
        self.write_reg(addr, &[f(v)])
    }

    fn standby(&mut self) -> R<()> {
        self.cmd(&[op::SET_STANDBY, 0x00])
    }

    fn clear_irq(&mut self) -> R<()> {
        self.cmd(&[op::CLR_IRQ_STATUS, 0xFF, 0xFF])
    }

    fn set_packet_params(&mut self, payload_len: u8) -> R<()> {
        let p = self.preamble.to_be_bytes();
        // Explicit header, CRC on, standard IQ.
        self.cmd(&[
            op::SET_PACKET_PARAMS,
            p[0],
            p[1],
            0x00,
            payload_len,
            0x01,
            0x00,
        ])?;
        // Datasheet 15.4: bit 2 set for standard IQ, or long packets get lost.
        self.update_reg(reg::IQ_POLARITY, |v| v | 0x04)
    }

    fn status_mode(&mut self) -> R<u8> {
        let mut s = [0u8];
        self.query(&[op::GET_STATUS], 0, &mut s)?;
        Ok((s[0] >> 4) & 0x07)
    }
}

impl<SPI, CS, RST, BUSY, D> RadioDriver for Sx126x<SPI, CS, RST, BUSY, D>
where
    SPI: SpiBus<u8>,
    CS: OutputPin,
    RST: OutputPin,
    BUSY: InputPin,
    D: DelayNs,
{
    fn init(&mut self) -> R<()> {
        self.cs.set_high().map_err(|_| RadioFault("cs pin"))?;
        self.rst.set_low().map_err(|_| RadioFault("reset pin"))?;
        self.delay.delay_ms(2);
        self.rst.set_high().map_err(|_| RadioFault("reset pin"))?;
        self.delay.delay_ms(10);
        self.wait_busy()?;
        self.standby()?;
        if self.cfg.dcdc {
            self.cmd(&[op::SET_REGULATOR_MODE, 0x01])?;
        }
        if self.cfg.dio2_rf_switch {
            self.cmd(&[op::SET_DIO2_RF_SWITCH, 0x01])?;
        }
        if let Some(v) = self.cfg.tcxo_code {
            // The first start with a TCXO flags an oscillator error. Clear it,
            // then give the TCXO 10 ms (640 steps of 15.625 us) to settle.
            self.cmd(&[op::CLEAR_DEVICE_ERRORS, 0x00, 0x00])?;
            self.cmd(&[op::SET_TCXO_MODE, v & 0x07, 0x00, 0x02, 0x80])?;
        }
        // Calibrate everything, now that the chip knows its clock.
        self.cmd(&[op::CALIBRATE, 0x7F])?;
        self.cmd(&[op::SET_BUFFER_BASE, 0x00, 0x00])?;
        // Is a chip there? Standby RC is mode 2, standby XOSC is 3.
        match self.status_mode()? {
            2 | 3 => Ok(()),
            _ => Err(RadioFault("chip not in standby")),
        }
    }

    fn configure(&mut self, s: &Settings) -> R<()> {
        self.standby()?;
        self.cmd(&[op::SET_PACKET_TYPE, 0x01])?; // LoRa
        let cal = image_cal(s.freq_hz);
        self.cmd(&[op::CALIBRATE_IMAGE, cal[0], cal[1]])?;
        let f = pll_step(s.freq_hz).to_be_bytes();
        self.cmd(&[op::SET_RF_FREQUENCY, f[0], f[1], f[2], f[3]])?;
        let (dc, hp, param) = pa_params(s.tx_power_dbm);
        self.cmd(&[op::SET_PA_CONFIG, dc, hp, 0x00, 0x01])?;
        // Ramp 200 us.
        self.cmd(&[op::SET_TX_PARAMS, param, 0x04])?;
        // Datasheet 15.2: better resistance to antenna mismatch.
        self.update_reg(reg::TX_CLAMP, |v| v | 0x1E)?;
        self.cmd(&[
            op::SET_MODULATION,
            s.sf,
            s.bw.chip_code(),
            s.cr - 4,
            u8::from(s.low_data_rate()),
        ])?;
        // Datasheet 15.1: modulation quality, 500 kHz is the odd one.
        let wide = s.bw == Bandwidth::Khz500;
        self.update_reg(
            reg::TX_MODULATION,
            |v| if wide { v & !0x04 } else { v | 0x04 },
        )?;
        self.preamble = s.preamble;
        self.set_packet_params(0xFF)?;
        self.write_reg(reg::LORA_SYNC_WORD, &s.sync_word.to_be_bytes())?;
        let m = irq::ALL.to_be_bytes();
        self.cmd(&[op::CFG_DIO_IRQ, m[0], m[1], m[0], m[1], 0, 0, 0, 0])?;
        self.clear_irq()
    }

    fn start_rx(&mut self) -> R<()> {
        self.clear_irq()?;
        self.set_packet_params(0xFF)?;
        let gain = if self.cfg.rx_boost { 0x96 } else { 0x94 };
        self.write_reg(reg::RX_GAIN, &[gain])?;
        // Timeout 0xFFFFFF: receive until told otherwise.
        self.cmd(&[op::SET_RX, 0xFF, 0xFF, 0xFF])
    }

    fn start_tx(&mut self, payload: &[u8], timeout_ms: u32) -> R<()> {
        if payload.is_empty() || payload.len() > MAX_LORA_PAYLOAD {
            return Err(RadioFault("bad payload length"));
        }
        self.standby()?;
        self.clear_irq()?;
        self.scratch[..2].copy_from_slice(&[op::WRITE_BUFFER, 0x00]);
        self.scratch[2..2 + payload.len()].copy_from_slice(payload);
        self.run(2 + payload.len())?;
        self.set_packet_params(payload.len() as u8)?; // <= 255
        let t = timeout_ms
            .saturating_mul(64)
            .clamp(1, 0x00FF_FFFF)
            .to_be_bytes();
        self.cmd(&[op::SET_TX, t[1], t[2], t[3]])
    }

    fn irq_status(&mut self) -> R<u16> {
        let mut f = [0u8; 2];
        self.query(&[op::GET_IRQ_STATUS], 1, &mut f)?;
        let flags = u16::from_be_bytes(f);
        if flags != 0 {
            self.cmd(&[op::CLR_IRQ_STATUS, f[0], f[1]])?;
        }
        Ok(flags)
    }

    fn read_packet(&mut self, buf: &mut [u8; MAX_LORA_PAYLOAD]) -> R<RxMeta> {
        let mut st = [0u8; 2];
        self.query(&[op::GET_RX_BUFFER_STATUS], 1, &mut st)?;
        let (len, off) = (usize::from(st[0]), st[1]);
        if len > 0 {
            self.query(&[op::READ_BUFFER, off], 1, &mut buf[..len])?;
        }
        let mut ps = [0u8; 3];
        self.query(&[op::GET_PACKET_STATUS], 1, &mut ps)?;
        Ok(RxMeta {
            len,
            // The chip reports dBm times -2 and SNR times 4. Quarter dB:
            rssi_qdb: -i16::from(ps[0]) * 2,
            snr_qdb: i16::from(ps[1] as i8),
            signal_rssi_qdb: -i16::from(ps[2]) * 2,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::convert::Infallible;
    use std::vec::Vec;

    /// A model of the parts of the chip the driver talks to.
    struct Chip {
        log: Vec<Vec<u8>>,
        regs: [u8; 0x1000],
        mode: u8,
        irq: u16,
        rx: Vec<u8>,
        rx_off: u8,
        pkt_status: [u8; 3],
        dead: bool,
    }

    impl Chip {
        fn new() -> Self {
            Chip {
                log: Vec::new(),
                regs: [0; 0x1000],
                mode: 0,
                irq: 0,
                rx: Vec::new(),
                rx_off: 0,
                pkt_status: [0; 3],
                dead: false,
            }
        }
        fn ops(&self) -> Vec<u8> {
            self.log.iter().map(|w| w[0]).collect()
        }
        fn find(&self, op: u8) -> Vec<&Vec<u8>> {
            self.log.iter().filter(|w| w[0] == op).collect()
        }
    }

    impl embedded_hal::spi::ErrorType for Chip {
        type Error = Infallible;
    }

    impl SpiBus<u8> for Chip {
        fn read(&mut self, _: &mut [u8]) -> Result<(), Infallible> {
            unreachable!()
        }
        fn write(&mut self, _: &[u8]) -> Result<(), Infallible> {
            unreachable!()
        }
        fn transfer(&mut self, _: &mut [u8], _: &[u8]) -> Result<(), Infallible> {
            unreachable!()
        }
        fn transfer_in_place(&mut self, w: &mut [u8]) -> Result<(), Infallible> {
            self.log.push(w.to_vec());
            if self.dead {
                w.fill(0);
                return Ok(());
            }
            match w[0] {
                op::SET_STANDBY => self.mode = 2,
                op::SET_RX => self.mode = 5,
                op::SET_TX => self.mode = 6,
                op::GET_STATUS => w[1] = self.mode << 4,
                op::GET_IRQ_STATUS => {
                    w[1] = self.mode << 4;
                    w[2..4].copy_from_slice(&self.irq.to_be_bytes());
                }
                op::CLR_IRQ_STATUS => {
                    let m = u16::from_be_bytes([w[1], w[2]]);
                    self.irq &= !m;
                }
                op::GET_RX_BUFFER_STATUS => {
                    w[2] = self.rx.len() as u8;
                    w[3] = self.rx_off;
                }
                op::READ_BUFFER => {
                    let n = w.len() - 3;
                    w[3..].copy_from_slice(&self.rx[..n]);
                }
                op::GET_PACKET_STATUS => w[2..5].copy_from_slice(&self.pkt_status),
                op::READ_REGISTER => {
                    let a = usize::from(u16::from_be_bytes([w[1], w[2]]));
                    for (i, b) in w[4..].iter_mut().enumerate() {
                        *b = self.regs[a + i];
                    }
                }
                op::WRITE_REGISTER => {
                    let a = usize::from(u16::from_be_bytes([w[1], w[2]]));
                    for (i, b) in w[3..].iter().enumerate() {
                        self.regs[a + i] = *b;
                    }
                }
                _ => {}
            }
            Ok(())
        }
        fn flush(&mut self) -> Result<(), Infallible> {
            Ok(())
        }
    }

    struct Pin(bool);
    impl embedded_hal::digital::ErrorType for Pin {
        type Error = Infallible;
    }
    impl OutputPin for Pin {
        fn set_low(&mut self) -> Result<(), Infallible> {
            self.0 = false;
            Ok(())
        }
        fn set_high(&mut self) -> Result<(), Infallible> {
            self.0 = true;
            Ok(())
        }
    }
    struct Busy(u32);
    impl embedded_hal::digital::ErrorType for Busy {
        type Error = Infallible;
    }
    impl InputPin for Busy {
        fn is_high(&mut self) -> Result<bool, Infallible> {
            if self.0 == u32::MAX {
                return Ok(true); // stuck
            }
            let b = self.0 > 0;
            self.0 = self.0.saturating_sub(1);
            Ok(b)
        }
        fn is_low(&mut self) -> Result<bool, Infallible> {
            Ok(!self.is_high()?)
        }
    }
    struct NoDelay;
    impl DelayNs for NoDelay {
        fn delay_ns(&mut self, _: u32) {}
    }

    type Drv = Sx126x<Chip, Pin, Pin, Busy, NoDelay>;

    fn drv(busy: u32) -> Drv {
        Sx126x::new(
            Chip::new(),
            Pin(true),
            Pin(true),
            Busy(busy),
            NoDelay,
            HELTEC_V3,
        )
    }

    #[test]
    fn frequency_register_known_answers() {
        assert_eq!(pll_step(915_000_000), 0x3930_0000);
        // 868 MHz: 868e6 * 2^25 / 32e6 = 910163968 (0x36400000)
        assert_eq!(pll_step(868_000_000), 0x3640_0000);
        // 1 Hz of resolution is about 1.05 steps of the 0.9537 Hz grid.
        assert_eq!(pll_step(915_000_001), 0x3930_0001);
    }

    #[test]
    fn pa_table_matches_the_datasheet() {
        assert_eq!(pa_params(22), (0x04, 0x07, 22));
        assert_eq!(pa_params(21), (0x04, 0x07, 22));
        assert_eq!(pa_params(20), (0x03, 0x05, 22));
        assert_eq!(pa_params(18), (0x03, 0x05, 20));
        assert_eq!(pa_params(17), (0x02, 0x03, 22));
        assert_eq!(pa_params(15), (0x02, 0x03, 20));
        assert_eq!(pa_params(14), (0x02, 0x02, 22));
        assert_eq!(pa_params(2), (0x02, 0x02, 10));
        assert_eq!(pa_params(0), (0x02, 0x02, 8));
        assert_eq!(pa_params(-9), (0x02, 0x02, 0xFF)); // -1 as two's complement
        assert_eq!(pa_params(100), pa_params(22));
        assert_eq!(pa_params(-100), pa_params(-9));
    }

    #[test]
    fn image_calibration_bands() {
        assert_eq!(image_cal(915_000_000), [0xE1, 0xE9]);
        assert_eq!(image_cal(868_000_000), [0xD7, 0xDB]);
        assert_eq!(image_cal(433_000_000), [0x6B, 0x6F]);
    }

    #[test]
    fn init_sets_up_the_heltec_board() {
        let mut d = drv(0);
        d.init().unwrap();
        let c = d.bus();
        assert!(c.log.iter().any(|w| w == &[0x96, 0x01]), "DC-DC");
        assert!(
            c.log.iter().any(|w| w == &[0x9D, 0x01]),
            "DIO2 antenna switch"
        );
        assert!(
            c.log.iter().any(|w| w == &[0x97, 0x02, 0x00, 0x02, 0x80]),
            "TCXO 1.8 V, 10 ms"
        );
        assert!(c.log.iter().any(|w| w == &[0x89, 0x7F]), "calibrate all");
        // The TCXO is set before the calibration that depends on it.
        let ops = c.ops();
        let tcxo = ops.iter().position(|&o| o == 0x97).unwrap();
        let cal = ops.iter().position(|&o| o == 0x89).unwrap();
        assert!(tcxo < cal);
    }

    #[test]
    fn init_fails_when_no_chip_answers() {
        let mut d = drv(0);
        d.bus().dead = true;
        assert_eq!(d.init(), Err(RadioFault("chip not in standby")));
    }

    #[test]
    fn init_fails_when_busy_never_clears() {
        let mut d = drv(u32::MAX);
        assert_eq!(d.init(), Err(RadioFault("busy timeout")));
    }

    #[test]
    fn busy_that_clears_late_is_waited_for() {
        let mut d = drv(5);
        d.init().unwrap();
    }

    #[test]
    fn configure_writes_the_lab_profile() {
        let mut d = drv(0);
        d.init().unwrap();
        d.bus().log.clear();
        d.configure(&Settings::LAB_SF7_BW125).unwrap();
        let c = d.bus();
        let one = |op: u8| -> Vec<u8> {
            let f = c.find(op);
            assert_eq!(f.len(), 1, "op {op:02x}");
            f[0].clone()
        };
        assert_eq!(one(op::SET_PACKET_TYPE), [0x8A, 0x01]);
        assert_eq!(one(op::SET_RF_FREQUENCY), [0x86, 0x39, 0x30, 0x00, 0x00]);
        assert_eq!(one(op::CALIBRATE_IMAGE), [0x98, 0xE1, 0xE9]);
        // SF7, 125 kHz (0x04), CR 4/5 (1), no low data rate optimization.
        assert_eq!(one(op::SET_MODULATION), [0x8B, 7, 0x04, 1, 0]);
        // 8 symbol preamble, explicit header, 255 max, CRC on, normal IQ.
        assert_eq!(one(op::SET_PACKET_PARAMS), [0x8C, 0, 8, 0, 0xFF, 1, 0]);
        // 2 dBm.
        assert_eq!(one(op::SET_PA_CONFIG), [0x95, 0x02, 0x02, 0x00, 0x01]);
        assert_eq!(one(op::SET_TX_PARAMS), [0x8E, 10, 0x04]);
        assert_eq!(c.regs[0x0740..0x0742], [0x14, 0x24]);
        assert_eq!(c.regs[0x0736] & 0x04, 0x04);
        assert_eq!(c.regs[0x0889] & 0x04, 0x04);
        assert_eq!(c.regs[0x08D8] & 0x1E, 0x1E);
        assert_eq!(
            one(op::CFG_DIO_IRQ),
            [0x08, 0x02, 0x63, 0x02, 0x63, 0, 0, 0, 0],
            "TxDone RxDone HeaderErr CrcErr Timeout on DIO1"
        );
    }

    #[test]
    fn configure_follows_the_settings() {
        let mut d = drv(0);
        d.init().unwrap();
        let mut s = Settings::LAB_SF7_BW125;
        s.sf = 12;
        s.bw = Bandwidth::Khz500;
        s.cr = 8;
        s.tx_power_dbm = 22;
        s.sync_word = 0x3444;
        s.preamble = 0x0102;
        s.freq_hz = 868_000_000;
        d.configure(&s).unwrap();
        let c = d.bus();
        let last = |op: u8| c.find(op).last().unwrap().to_vec();
        // SF12 at 500 kHz is a 8.2 ms symbol: no low data rate optimization.
        assert_eq!(last(op::SET_MODULATION), [0x8B, 12, 0x06, 4, 0]);
        assert_eq!(last(op::SET_PACKET_PARAMS)[1..3], [0x01, 0x02]);
        assert_eq!(last(op::SET_PA_CONFIG), [0x95, 0x04, 0x07, 0x00, 0x01]);
        assert_eq!(last(op::SET_TX_PARAMS), [0x8E, 22, 0x04]);
        assert_eq!(last(op::CALIBRATE_IMAGE), [0x98, 0xD7, 0xDB]);
        assert_eq!(c.regs[0x0740..0x0742], [0x34, 0x44]);
        assert_eq!(c.regs[0x0889] & 0x04, 0, "500 kHz clears the bit");
        // Slow settings turn low data rate optimization on.
        s.sf = 12;
        s.bw = Bandwidth::Khz125;
        d.configure(&s).unwrap();
        assert_eq!(d.bus().find(op::SET_MODULATION).last().unwrap()[4], 1);
    }

    #[test]
    fn rx_starts_continuous_with_boost() {
        let mut d = drv(0);
        d.init().unwrap();
        d.configure(&Settings::LAB_SF7_BW125).unwrap();
        d.start_rx().unwrap();
        let c = d.bus();
        assert_eq!(c.mode, 5);
        assert_eq!(c.log.last().unwrap(), &[0x82, 0xFF, 0xFF, 0xFF]);
        assert_eq!(c.regs[0x08AC], 0x96);
    }

    #[test]
    fn tx_writes_the_payload_then_starts() {
        let mut d = drv(0);
        d.init().unwrap();
        d.configure(&Settings::LAB_SF7_BW125).unwrap();
        d.bus().log.clear();
        d.start_tx(&[0xAA, 0xBB, 0xCC], 500).unwrap();
        let c = d.bus();
        let ops = c.ops();
        let wb = ops.iter().position(|&o| o == op::WRITE_BUFFER).unwrap();
        let pp = ops
            .iter()
            .position(|&o| o == op::SET_PACKET_PARAMS)
            .unwrap();
        let tx = ops.iter().position(|&o| o == op::SET_TX).unwrap();
        assert!(wb < pp && pp < tx);
        assert_eq!(c.log[wb], [0x0E, 0x00, 0xAA, 0xBB, 0xCC]);
        assert_eq!(c.log[pp], [0x8C, 0, 8, 0, 3, 1, 0], "length 3");
        // 500 ms is 32000 steps of 15.625 us = 0x007D00.
        assert_eq!(c.log[tx], [0x83, 0x00, 0x7D, 0x00]);
        assert_eq!(c.mode, 6);
        assert!(d.start_tx(&[], 10).is_err());
        assert!(d.start_tx(&[0; 256], 10).is_err());
    }

    #[test]
    fn irq_status_reads_and_clears() {
        let mut d = drv(0);
        d.init().unwrap();
        d.bus().irq = irq::TX_DONE | irq::CRC_ERR;
        assert_eq!(d.irq_status().unwrap(), 0x41);
        assert_eq!(d.bus().irq, 0);
        assert_eq!(d.irq_status().unwrap(), 0);
        // Nothing to clear means no clear command.
        let n = d.bus().find(op::CLR_IRQ_STATUS).len();
        d.irq_status().unwrap();
        assert_eq!(d.bus().find(op::CLR_IRQ_STATUS).len(), n);
    }

    #[test]
    fn read_packet_gives_payload_and_signal() {
        let mut d = drv(0);
        d.init().unwrap();
        let c = d.bus();
        c.rx = (1..=200).collect();
        c.rx_off = 0;
        c.pkt_status = [0x50, 0xF4, 0x52]; // -40 dBm, -3 dB, -41 dBm
        let mut buf = [0u8; MAX_LORA_PAYLOAD];
        let m = d.read_packet(&mut buf).unwrap();
        assert_eq!(m.len, 200);
        assert_eq!(&buf[..200], &(1..=200).collect::<Vec<u8>>()[..]);
        assert_eq!(m.rssi_qdb, -160);
        assert_eq!(m.snr_qdb, -12);
        assert_eq!(m.signal_rssi_qdb, -164);
    }
}
