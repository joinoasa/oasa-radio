//! OASA LoRa modem firmware for the RAK4631 (RAK4630 module: nRF52840 and SX1262).
//!
//! Packets in over USB serial, out over LoRa, and the reverse. All logic is in
//! `oasa-modem-core`, which is tested on a PC. This file is only the board:
//! pins, the USB serial port, the SPI bus, the interrupt line and the LED.
//!
//! The board is wired like this (sources in README.md):
//!
//! | Signal | Pin |
//! |---|---|
//! | SX1262 SCK / MOSI / MISO / NSS | P1.11 / P1.12 / P1.13 / P1.10 |
//! | SX1262 RESET / BUSY / DIO1 | P1.06 / P1.14 / P1.15 |
//! | Antenna switch power | P1.05, high = on |
//! | LED1 (green) / LED2 (blue) | P1.03 / P1.04, high = on |
//! | USB | the nRF52840's own USB, CDC-ACM serial port |
//!
//! The USB serial port carries only modem frames. There is no logging. No
//! Bluetooth: the SoftDevice in flash is never enabled. No heap.

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_futures::select::{Either3, select3};
use embassy_nrf::config::{Config as NrfConfig, Debug, HfclkSource, LfclkSource};
use embassy_nrf::gpio::{Input, Level, Output, OutputDrive, Pull};
use embassy_nrf::usb::Driver;
use embassy_nrf::usb::vbus_detect::HardwareVbusDetect;
use embassy_nrf::{bind_interrupts, pac, peripherals, spim, usb};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Instant, Timer};
use embassy_usb::UsbDevice;
use embassy_usb::class::cdc_acm::{CdcAcmClass, Receiver, Sender, State};
use embassy_usb::driver::EndpointError;
use oasa_modem_core::board::{OutFrame, hex_id};
use oasa_modem_core::modem::{FrameSink, Info, Modem};
use oasa_modem_core::settings::{Limits, Settings};
use oasa_modem_core::sx126x::{self, Sx126x};
use static_cell::StaticCell;

bind_interrupts!(struct Irqs {
    USBD => usb::InterruptHandler<peripherals::USBD>;
    CLOCK_POWER => usb::vbus_detect::InterruptHandler;
    SPI2 => spim::InterruptHandler<peripherals::SPI2>;
});

type UsbDriver = Driver<'static, HardwareVbusDetect>;

/// A panic restarts the chip. It comes back through the bootloader with the
/// lab profile and low power.
#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    cortex_m::peripheral::SCB::sys_reset()
}

/// Packets that may wait for the radio.
const TX_QUEUE: usize = 8;
/// Frames waiting for the USB port.
const OUT_FRAMES: usize = 6;
/// USB packets from the host waiting for the engine.
const IN_PACKETS: usize = 4;
/// Full-speed bulk packet size.
const USB_PACKET: usize = 64;
const BOARD: &str = "rak4631";
const FW: [u8; 3] = oasa_modem_core::fw_version!();

/// pid.codes test VID and PID (1209:0001), for development only. A product
/// needs its own PID before boards are given out.
const USB_VID: u16 = 0x1209;
const USB_PID: u16 = 0x0001;

static OUT: Channel<CriticalSectionRawMutex, OutFrame, OUT_FRAMES> = Channel::new();

struct InPacket {
    len: usize,
    buf: [u8; USB_PACKET],
}

static IN: Channel<CriticalSectionRawMutex, InPacket, IN_PACKETS> = Channel::new();

/// Hands frames from the engine to the USB writer task.
struct ChannelSink;

impl FrameSink for ChannelSink {
    fn frame(&mut self, wire: &[u8]) -> bool {
        match OutFrame::new(wire) {
            Some(f) => OUT.try_send(f).is_ok(),
            None => false,
        }
    }
}

#[embassy_executor::task]
async fn usb_task(mut dev: UsbDevice<'static, UsbDriver>) -> ! {
    dev.run().await
}

/// Writes frames to the host. A separate task, so a slow host never stops
/// the radio. Frames are dropped while DTR is low (no program has the port
/// open), so they do not wait and arrive stale.
#[embassy_executor::task]
async fn usb_writer(mut tx: Sender<'static, UsbDriver>) {
    loop {
        let f = OUT.receive().await;
        if tx.dtr() {
            let _ = write_frame(&mut tx, f.bytes()).await;
        }
    }
}

async fn write_frame(
    tx: &mut Sender<'static, UsbDriver>,
    bytes: &[u8],
) -> Result<(), EndpointError> {
    for chunk in bytes.chunks(USB_PACKET) {
        tx.write_packet(chunk).await?;
    }
    if bytes.len().is_multiple_of(USB_PACKET) {
        // A full last packet needs a zero-length one to end the transfer.
        tx.write_packet(&[]).await?;
    }
    Ok(())
}

/// Reads the host's bytes. When the queue is full the USB endpoint stops
/// taking packets, so the host waits instead of bytes being lost.
#[embassy_executor::task]
async fn usb_reader(mut rx: Receiver<'static, UsbDriver>) {
    loop {
        rx.wait_connection().await;
        loop {
            let mut p = InPacket {
                len: 0,
                buf: [0; USB_PACKET],
            };
            match rx.read_packet(&mut p.buf).await {
                Ok(n) => {
                    p.len = n;
                    IN.send(p).await;
                }
                Err(_) => break, // unplugged or reset
            }
        }
    }
}

/// Busy-wait delay for the radio driver, counted in CPU cycles (64 MHz), so
/// short waits are exact. embassy-time's own delay ticks at 32.768 kHz.
struct CycleDelay;

impl embedded_hal::delay::DelayNs for CycleDelay {
    fn delay_ns(&mut self, ns: u32) {
        // At most 2^32 ns: 275 million cycles, fits a u32.
        cortex_m::asm::delay((u64::from(ns) * 64).div_ceil(1000) as u32);
    }
}

/// The chip's 64-bit device ID (FICR), as the USB serial number, so two
/// boards on one host get stable, different names.
fn serial_number() -> &'static str {
    static SERIAL: StaticCell<[u8; 16]> = StaticCell::new();
    let id =
        (u64::from(pac::FICR.deviceid(1).read()) << 32) | u64::from(pac::FICR.deviceid(0).read());
    hex_id(id, SERIAL.init([0; 16]))
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut cfg = NrfConfig::default();
    // The 32 MHz crystal: USB needs it. The 32.768 kHz clock (timers only)
    // comes from the internal RC, which works whether or not a crystal is fitted.
    cfg.hfclk_source = HfclkSource::ExternalXtal;
    cfg.lfclk_source = LfclkSource::InternalRC;
    // Never write UICR: it belongs to the bootloader.
    cfg.debug = Debug::NotConfigured;
    let p = embassy_nrf::init(cfg);

    // USB serial (CDC-ACM) on the nRF52840's own USB port.
    let driver = Driver::new(p.USBD, Irqs, HardwareVbusDetect::new(Irqs));
    let mut uc = embassy_usb::Config::new(USB_VID, USB_PID);
    uc.manufacturer = Some("OASA");
    uc.product = Some("OASA LoRa modem (RAK4631)");
    uc.serial_number = Some(serial_number());
    uc.max_power = 100;
    uc.max_packet_size_0 = 64;

    static CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static MSOS_DESC: StaticCell<[u8; 128]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
    static CDC_STATE: StaticCell<State> = StaticCell::new();
    let mut builder = embassy_usb::Builder::new(
        driver,
        uc,
        CONFIG_DESC.init([0; 256]),
        BOS_DESC.init([0; 256]),
        MSOS_DESC.init([0; 128]),
        CONTROL_BUF.init([0; 64]),
    );
    let class = CdcAcmClass::new(
        &mut builder,
        CDC_STATE.init(State::new()),
        USB_PACKET as u16,
    );
    let (tx, rx) = class.split();
    spawner.spawn(usb_task(builder.build()).unwrap());
    spawner.spawn(usb_writer(tx).unwrap());
    spawner.spawn(usb_reader(rx).unwrap());

    // Power for the antenna switch. DIO2 picks the direction. P1.07 (also
    // wired to the switch) is left alone, as RAK and Meshtastic advise.
    let _ant_power = Output::new(p.P1_05, Level::High, OutputDrive::Standard);
    let mut led = Output::new(p.P1_03, Level::High, OutputDrive::Standard);
    let _led2 = Output::new(p.P1_04, Level::Low, OutputDrive::Standard);

    // SX1262 on SPIM2. NSS is driven by hand so the driver controls the whole transaction.
    let mut sc = spim::Config::default();
    sc.frequency = spim::Frequency::M8;
    sc.mode = spim::MODE_0;
    let spi = spim::Spim::new(p.SPI2, Irqs, p.P1_11, p.P1_13, p.P1_12, sc);
    let cs = Output::new(p.P1_10, Level::High, OutputDrive::Standard);
    let rst = Output::new(p.P1_06, Level::High, OutputDrive::Standard);
    let busy = Input::new(p.P1_14, Pull::None);
    let mut dio1 = Input::new(p.P1_15, Pull::None);

    let radio = Sx126x::new(spi, cs, rst, busy, CycleDelay, sx126x::RAK4631);
    let mut modem: Modem<_, TX_QUEUE> = Modem::new(
        radio,
        Settings::LAB_SF7_BW125,
        Limits::RAK4631,
        Info {
            fw: FW,
            board: BOARD,
        },
    );

    let t0 = Instant::now();
    let now = || t0.elapsed().as_millis() as u32;
    modem.start(now());

    let mut sink = ChannelSink;
    loop {
        // While the radio is down its interrupt line is not watched.
        let watch_irq = modem.wants_irq();
        let irq = async {
            if watch_irq {
                dio1.wait_for_high().await
            } else {
                core::future::pending::<()>().await
            }
        };
        let event = select3(irq, IN.receive(), Timer::after(Duration::from_millis(20))).await;
        match event {
            Either3::First(()) => modem.on_irq(now(), &mut sink),
            Either3::Second(pkt) => modem.on_serial(&pkt.buf[..pkt.len], now(), &mut sink),
            Either3::Third(()) => {}
        }
        let t = now();
        modem.on_tick(t);
        led.set_level(if modem.led_on(t) {
            Level::High
        } else {
            Level::Low
        });
    }
}
