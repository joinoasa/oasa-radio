//! OASA LoRa modem firmware for the Heltec WiFi LoRa 32 V3.
//!
//! Packets in over USB serial, out over LoRa, and the reverse. All logic is in
//! `oasa-modem-core`, which is tested on a PC. This file is only the board:
//! pins, the serial port, the SPI bus, the interrupt line and the LED.
//!
//! The board is wired like this (sources in README.md):
//!
//! | Signal | GPIO |
//! |---|---|
//! | USB serial (CP2102) UART0 TX / RX | 43 / 44 |
//! | SX1262 SCK / MOSI / MISO / NSS | 9 / 10 / 11 / 8 |
//! | SX1262 RESET / BUSY / DIO1 | 12 / 13 / 14 |
//! | LED | 35 |
//!
//! UART0 carries only modem frames. There is no logging: anything printed
//! there would corrupt the stream. No Wi-Fi, no Bluetooth, no heap.

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_futures::select::{Either3, select3};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Instant, Timer};
use esp_hal::Async;
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;
use esp_hal::gpio::{Input, InputConfig, Level, Output, OutputConfig};
use esp_hal::spi::Mode;
use esp_hal::spi::master::{Config as SpiConfig, Spi};
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::uart::{Config as UartConfig, Uart, UartTx};
use oasa_modem_core::modem::{FrameSink, Info, Modem};
use oasa_modem_core::settings::{Limits, Settings};
use oasa_modem_core::sx126x::{self, Sx126x};

// The bootloader wants an app descriptor.
esp_bootloader_esp_idf::esp_app_desc!();

/// A panic restarts the modem. It comes back with the lab profile and low power.
#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    esp_hal::system::software_reset()
}

/// UART0 speed. The CP2102 on the board does up to 921600; the radio is far slower.
const BAUD: u32 = 115_200;
/// Packets that may wait for the radio.
const TX_QUEUE: usize = 8;
/// Longest frame the engine writes (see `oasa_modem_core::modem`).
const FRAME_MAX: usize = 300;
/// Frames waiting for the serial port.
const OUT_FRAMES: usize = 6;
const BOARD: &str = "heltec-wifi-lora-32-v3";

struct WireFrame {
    len: usize,
    buf: [u8; FRAME_MAX],
}

static OUT: Channel<CriticalSectionRawMutex, WireFrame, OUT_FRAMES> = Channel::new();

/// Hands frames from the engine to the serial writer task.
struct ChannelSink;

impl FrameSink for ChannelSink {
    fn frame(&mut self, wire: &[u8]) -> bool {
        if wire.len() > FRAME_MAX {
            return false;
        }
        let mut f = WireFrame {
            len: wire.len(),
            buf: [0; FRAME_MAX],
        };
        f.buf[..wire.len()].copy_from_slice(wire);
        OUT.try_send(f).is_ok()
    }
}

/// Writes frames to the serial port. A separate task, so a slow serial line
/// never stops the radio or the reading of the host's bytes.
#[embassy_executor::task]
async fn serial_writer(mut tx: UartTx<'static, Async>) {
    loop {
        let f = OUT.receive().await;
        let mut rest = &f.buf[..f.len];
        while !rest.is_empty() {
            match tx.write_async(rest).await {
                Ok(n) => rest = &rest[n..],
                Err(_) => break,
            }
        }
    }
}

const fn version_part(s: &str) -> u8 {
    let b = s.as_bytes();
    let (mut i, mut n) = (0, 0u32);
    while i < b.len() {
        n = n * 10 + (b[i] - b'0') as u32;
        i += 1;
    }
    if n > 255 { 255 } else { n as u8 }
}

const FW: [u8; 3] = [
    version_part(env!("CARGO_PKG_VERSION_MAJOR")),
    version_part(env!("CARGO_PKG_VERSION_MINOR")),
    version_part(env!("CARGO_PKG_VERSION_PATCH")),
];

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    let p = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));
    let timg0 = TimerGroup::new(p.TIMG0);
    esp_rtos::start(timg0.timer0, p.FROM_CPU_INTR0);

    // USB serial: UART0 on GPIO43 (TX) and GPIO44 (RX), wired to the CP2102.
    let uart = Uart::new(p.UART0, UartConfig::default().with_baudrate(BAUD))
        .unwrap()
        .with_rx(p.GPIO44)
        .with_tx(p.GPIO43)
        .into_async();
    let (mut rx, tx) = uart.split();
    spawner.spawn(serial_writer(tx).unwrap());

    // SX1262 on SPI2. NSS is driven by hand so the driver controls the whole transaction.
    let spi = Spi::new(
        p.SPI2,
        SpiConfig::default()
            .with_frequency(Rate::from_mhz(8))
            .with_mode(Mode::_0),
    )
    .unwrap()
    .with_sck(p.GPIO9)
    .with_mosi(p.GPIO10)
    .with_miso(p.GPIO11);
    let cs = Output::new(p.GPIO8, Level::High, OutputConfig::default());
    let rst = Output::new(p.GPIO12, Level::High, OutputConfig::default());
    let busy = Input::new(p.GPIO13, InputConfig::default());
    let mut dio1 = Input::new(p.GPIO14, InputConfig::default());
    // GPIO36 (Vext) is left alone: it only powers the OLED and a header.
    let mut led = Output::new(p.GPIO35, Level::High, OutputConfig::default());

    let radio = Sx126x::new(spi, cs, rst, busy, Delay::new(), sx126x::HELTEC_V3);
    let mut modem: Modem<_, TX_QUEUE> = Modem::new(
        radio,
        Settings::LAB_SF7_BW125,
        Limits::HELTEC_V3,
        Info {
            fw: FW,
            board: BOARD,
        },
    );

    let t0 = Instant::now();
    let now = || t0.elapsed().as_millis() as u32;
    modem.start(now());

    let mut sink = ChannelSink;
    let mut buf = [0u8; 128];
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
        let event = select3(
            irq,
            rx.read_async(&mut buf),
            Timer::after(Duration::from_millis(20)),
        )
        .await;
        match event {
            Either3::First(()) => modem.on_irq(now(), &mut sink),
            Either3::Second(Ok(n)) => modem.on_serial(&buf[..n], now(), &mut sink),
            // The FIFO overflowed or the line glitched: bytes were lost.
            Either3::Second(Err(_)) => modem.note_serial_overrun(1),
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
