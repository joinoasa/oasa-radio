//! OASA USB network adapter test firmware for the RAK4631 (nRF52840).
//!
//! The board shows up to a phone as a USB network adapter (CDC-NCM), gives
//! the phone one address by DHCP with no router and no name server, and
//! serves one test page at http://10.71.44.1/. That is all: **the LoRa radio
//! is never touched** (no SPI, no radio pins, no antenna switch power), and
//! nothing is ever transmitted over the air.
//!
//! The logic that can run on a PC (the NCM request answers, the DHCP reply,
//! the page) is in `oasa-usbnet-core` and tested there. This file is the
//! board: clocks, USB, the network stack and the sockets.
//!
//! No logging, no heap, no secrets. No Bluetooth: the SoftDevice in flash is
//! never enabled.

#![no_std]
#![no_main]

use core::cell::Cell;
use core::sync::atomic::{AtomicU32, Ordering};

use embassy_executor::Spawner;
use embassy_net::tcp::TcpSocket;
use embassy_net::udp::{PacketMetadata, UdpSocket};
use embassy_net::{IpEndpoint, Ipv4Address, Ipv4Cidr, Stack, StackResources, StaticConfigV4};
use embassy_nrf::config::{Config as NrfConfig, Debug, HfclkSource, LfclkSource};
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_nrf::rng::Rng;
use embassy_nrf::usb::Driver;
use embassy_nrf::usb::vbus_detect::HardwareVbusDetect;
use embassy_nrf::{bind_interrupts, pac, peripherals, usb};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Duration, Instant, Timer, with_timeout};
use embassy_usb::class::cdc_ncm::embassy_net::{Device, Runner, State as NetState};
use embassy_usb::class::cdc_ncm::{CdcNcmClass, State};
use embassy_usb::control::{InResponse, OutResponse, Recipient, Request, RequestType};
use embassy_usb::{Handler, UsbDevice};
use oasa_modem_core::board::hex_id;
use oasa_usbnet_core::ncm::{NcmRequests, Reply, Seen};
use oasa_usbnet_core::{dhcp, http, net};
use static_cell::StaticCell;

bind_interrupts!(struct Irqs {
    USBD => usb::InterruptHandler<peripherals::USBD>;
    CLOCK_POWER => usb::vbus_detect::InterruptHandler;
});

type UsbDriver = Driver<'static, HardwareVbusDetect>;

/// A panic restarts the chip (through the bootloader).
#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    cortex_m::peripheral::SCB::sys_reset()
}

/// pid.codes test VID and PID (1209:0001), the same test ID as the modem
/// firmware. For development only: boards given out need their own PID.
const USB_VID: u16 = 0x1209;
const USB_PID: u16 = 0x0001;
/// Full-speed bulk packet size.
const USB_PACKET: u16 = 64;
/// Largest Ethernet frame without FCS (the NCM descriptor's wMaxSegmentSize).
const MTU: usize = 1514;
/// The NCM control interface. The NCM function is the only one and is
/// created first, so its control interface is 0 and its data interface 1.
const NCM_COMM_IF: u8 = 0;
const FW: &str = env!("CARGO_PKG_VERSION");

/// What the phone has asked of the NCM function, for the test page.
static NCM_SEEN: Mutex<CriticalSectionRawMutex, Cell<Seen>> = Mutex::new(Cell::new(Seen {
    packet_filter: None,
    ntb_in_size: None,
    ntb_in_max_datagrams: 0,
    get_input_size: 0,
    stalled: 0,
}));
static DHCP_OFFERS: AtomicU32 = AtomicU32::new(0);
static DHCP_ACKS: AtomicU32 = AtomicU32::new(0);
static DHCP_NAKS: AtomicU32 = AtomicU32::new(0);
static PAGES: AtomicU32 = AtomicU32::new(0);
static SERIAL: StaticCell<[u8; 16]> = StaticCell::new();

/// Answers the NCM packet filter and input size requests, which
/// embassy-usb 0.6.0 refuses or ignores (see README, "USB library
/// differences"). Registered before the NCM class, so embassy-usb asks it
/// first; anything it passes on goes to the class's own handler.
struct NcmFix {
    req: NcmRequests,
}

impl NcmFix {
    fn publish(&self) {
        NCM_SEEN.lock(|s| s.set(self.req.seen()));
    }

    fn ours(req: &Request) -> bool {
        req.request_type == RequestType::Class && req.recipient == Recipient::Interface
    }
}

impl Handler for NcmFix {
    fn reset(&mut self) {
        self.req.reset();
        self.publish();
    }

    fn control_out(&mut self, req: Request, data: &[u8]) -> Option<OutResponse> {
        if !Self::ours(&req) {
            return None;
        }
        let r = self
            .req
            .control_out(req.request, req.value, req.index, data);
        self.publish();
        match r {
            Reply::Pass => None,
            Reply::Accept | Reply::Data(_) => Some(OutResponse::Accepted),
            Reply::Stall => Some(OutResponse::Rejected),
        }
    }

    fn control_in<'a>(&'a mut self, req: Request, buf: &'a mut [u8]) -> Option<InResponse<'a>> {
        if !Self::ours(&req) {
            return None;
        }
        let r = self.req.control_in(req.request, req.index, req.length, buf);
        self.publish();
        match r {
            Reply::Pass => None,
            Reply::Data(n) => Some(InResponse::Accepted(&buf[..n])),
            Reply::Accept => Some(InResponse::Accepted(&[])),
            Reply::Stall => Some(InResponse::Rejected),
        }
    }
}

#[embassy_executor::task]
async fn usb_task(mut dev: UsbDevice<'static, UsbDriver>) -> ! {
    dev.run().await
}

#[embassy_executor::task]
async fn ncm_task(runner: Runner<'static, UsbDriver, MTU>) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, Device<'static, MTU>>) -> ! {
    runner.run().await
}

/// The DHCP server: one client, replies always broadcast to port 68.
#[embassy_executor::task]
async fn dhcp_task(stack: Stack<'static>) {
    let mut rx_meta = [PacketMetadata::EMPTY; 4];
    let mut rx_buf = [0u8; 2048];
    let mut tx_meta = [PacketMetadata::EMPTY; 2];
    let mut tx_buf = [0u8; 1024];
    let mut sock = UdpSocket::new(stack, &mut rx_meta, &mut rx_buf, &mut tx_meta, &mut tx_buf);
    // Port only, any address: DHCP requests come from 0.0.0.0 to 255.255.255.255.
    if sock.bind(dhcp::SERVER_PORT).is_err() {
        return;
    }
    let to = IpEndpoint::new(Ipv4Address::BROADCAST.into(), dhcp::CLIENT_PORT);
    let mut msg = [0u8; 600];
    let mut out = [0u8; dhcp::REPLY_LEN];
    loop {
        let Ok((n, _)) = sock.recv_from(&mut msg).await else {
            continue; // too long for the buffer: not a request we answer
        };
        let Some((kind, len)) = dhcp::reply(&msg[..n], &mut out) else {
            continue;
        };
        let counter = match kind {
            dhcp::Kind::Offer => &DHCP_OFFERS,
            dhcp::Kind::Nak => &DHCP_NAKS,
            _ => &DHCP_ACKS,
        };
        if sock.send_to(&out[..len], to).await.is_ok() {
            counter.fetch_add(1, Ordering::Relaxed);
        }
    }
}

/// The test page. Two of these run, so a browser's second connection (for
/// example for the icon) does not wait.
#[embassy_executor::task(pool_size = 2)]
async fn http_task(stack: Stack<'static>, serial: &'static str) {
    let mut rx_buf = [0u8; 1536];
    let mut tx_buf = [0u8; 2048];
    let mut req = [0u8; http::MAX_REQUEST + 1];
    let mut resp = [0u8; http::MAX_RESPONSE];
    loop {
        let mut sock = TcpSocket::new(stack, &mut rx_buf, &mut tx_buf);
        sock.set_timeout(Some(Duration::from_secs(10)));
        if sock.accept(http::PORT).await.is_err() {
            continue;
        }
        // Read the request head, up to MAX_REQUEST + 1 bytes (then it gets 431).
        let mut n = 0;
        while n < req.len() && !http::request_complete(&req[..n]) {
            match sock.read(&mut req[n..]).await {
                Ok(0) | Err(_) => break,
                Ok(k) => n += k,
            }
        }
        if n > 0 {
            let info = http::PageInfo {
                serial,
                fw: FW,
                ncm: NCM_SEEN.lock(|s| s.get()),
                dhcp_offers: DHCP_OFFERS.load(Ordering::Relaxed),
                dhcp_acks: DHCP_ACKS.load(Ordering::Relaxed),
                dhcp_naks: DHCP_NAKS.load(Ordering::Relaxed),
                uptime_s: Instant::now().as_secs() as u32,
                pages: PAGES.load(Ordering::Relaxed),
            };
            if let Some(len) = http::respond(&req[..n], &info, &mut resp) {
                let mut sent = 0;
                while sent < len {
                    match sock.write(&resp[sent..len]).await {
                        Ok(0) | Err(_) => break,
                        Ok(k) => sent += k,
                    }
                }
                if sent == len {
                    PAGES.fetch_add(1, Ordering::Relaxed);
                }
            }
        }
        sock.close();
        // Wait (bounded) for the answer to leave before the socket is dropped.
        let _ = with_timeout(Duration::from_secs(5), sock.flush()).await;
        sock.abort();
        let _ = with_timeout(Duration::from_secs(1), sock.flush()).await;
    }
}

/// The chip's 64-bit device ID (FICR).
fn chip_id() -> u64 {
    (u64::from(pac::FICR.deviceid(1).read()) << 32) | u64::from(pac::FICR.deviceid(0).read())
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

    // LED1 (green) on: the firmware is running. LED2 (blue) off.
    // The radio pins (P1.05 to P1.15) are never configured.
    let _led1 = Output::new(p.P1_03, Level::High, OutputDrive::Standard);
    let _led2 = Output::new(p.P1_04, Level::Low, OutputDrive::Standard);

    let id = chip_id();
    let serial: &'static str = hex_id(id, SERIAL.init([0; 16]));

    // Seed for the network stack (TCP sequence numbers, ports), from the
    // chip's hardware random number generator.
    let mut seed = [0u8; 8];
    Rng::new_blocking(p.RNG).blocking_fill_bytes(&mut seed);
    let seed = u64::from_le_bytes(seed);

    // USB: one CDC-NCM function.
    let driver = Driver::new(p.USBD, Irqs, HardwareVbusDetect::new(Irqs));
    let mut uc = embassy_usb::Config::new(USB_VID, USB_PID);
    uc.manufacturer = Some("OASA");
    uc.product = Some("OASA USB network test (RAK4631)");
    uc.serial_number = Some(serial);
    uc.max_power = 100;
    uc.max_packet_size_0 = 64;

    static CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static MSOS_DESC: StaticCell<[u8; 128]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; 128]> = StaticCell::new();
    let mut builder = embassy_usb::Builder::new(
        driver,
        uc,
        CONFIG_DESC.init([0; 256]),
        BOS_DESC.init([0; 256]),
        MSOS_DESC.init([0; 128]),
        CONTROL_BUF.init([0; 128]),
    );

    // Our handler first, so it sees the NCM class requests before the class does.
    static NCM_FIX: StaticCell<NcmFix> = StaticCell::new();
    builder.handler(NCM_FIX.init(NcmFix {
        req: NcmRequests::new(NCM_COMM_IF),
    }));

    static NCM_STATE: StaticCell<State> = StaticCell::new();
    let class = CdcNcmClass::new(
        &mut builder,
        NCM_STATE.init(State::new()),
        net::phone_mac(id),
        USB_PACKET,
    );
    let usb = builder.build();

    static NET_STATE: StaticCell<NetState<MTU, 4, 4>> = StaticCell::new();
    let (ncm_runner, device) = class
        .into_embassy_net_device::<MTU, 4, 4>(NET_STATE.init(NetState::new()), net::board_mac(id));

    // Static address, no gateway, no DNS.
    let [a, b, c, d] = net::BOARD_IP;
    let config = embassy_net::Config::ipv4_static(StaticConfigV4 {
        address: Ipv4Cidr::new(Ipv4Address::new(a, b, c, d), net::PREFIX_LEN),
        gateway: None,
        dns_servers: Default::default(),
    });
    // One UDP socket, two TCP sockets.
    static RESOURCES: StaticCell<StackResources<3>> = StaticCell::new();
    let (stack, net_runner) =
        embassy_net::new(device, config, RESOURCES.init(StackResources::new()), seed);

    spawner.spawn(usb_task(usb).unwrap());
    spawner.spawn(ncm_task(ncm_runner).unwrap());
    spawner.spawn(net_task(net_runner).unwrap());
    spawner.spawn(dhcp_task(stack).unwrap());
    for _ in 0..2 {
        spawner.spawn(http_task(stack, serial).unwrap());
    }

    loop {
        Timer::after(Duration::from_secs(3600)).await;
    }
}
