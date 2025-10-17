#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use defmt::info;
use embassy_executor::Spawner;
use embassy_net::dns::DnsSocket;
use embassy_net::tcp::client::{TcpClient, TcpClientState};
use embassy_net::{IpEndpoint, Stack};
use embassy_net::{
    IpListenEndpoint, Ipv4Cidr, Runner, StackResources, StaticConfigV4, tcp::TcpSocket,
};

use embassy_time::{Duration, Timer};
use esp_hal::rsa::Rsa;
use esp_hal::time;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::{clock::CpuClock, rng::Rng};
use esp_radio::Controller;
use esp_radio::wifi::{ScanConfig, WifiController, WifiDevice, WifiEvent, WifiStaState};
use esp_radio::{
    ble::controller::BleConnector,
    wifi::{self, ClientConfig, ModeConfig},
};
use reqwless::client::{HttpClient, TlsConfig};
use reqwless::{self, response};
use static_cell::StaticCell;
use trouble_host::prelude::*;
use {esp_backtrace as _, esp_println as _};

extern crate alloc;

const CONNECTIONS_MAX: usize = 1;
const L2CAP_CHANNELS_MAX: usize = 1;
const SSID: &str = "Not a honeypot";
const PASSWORD: &str = "Nbvf12nbvf12";

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    // generator version: 0.6.0

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // esp_alloc::heap_allocator!(#[unsafe(link_section = ".dram2_uninit")] size: 64 * 1024);
    // COEX needs more RAM - so we've added some more
    esp_alloc::heap_allocator!(size: 64 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    info!("Embassy initialized!");

    // let radio_init = esp_radio::init().expect("Failed to initialize Wi-Fi/BLE controller");
    let radio_init = &*mk_static!(Controller<'static>, esp_radio::init().unwrap());

    let (mut _wifi_controller, _interfaces) =
        esp_radio::wifi::new(&radio_init, peripherals.WIFI, Default::default())
            .expect("Failed to initialize Wi-Fi controller");
    // find more examples https://github.com/embassy-rs/trouble/tree/main/examples/esp32
    let transport = BleConnector::new(&radio_init, peripherals.BT, Default::default()).unwrap();
    let ble_controller = ExternalController::<_, 20>::new(transport);
    let mut resources: HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> =
        HostResources::new();
    let _stack = trouble_host::new(ble_controller, &mut resources);

    let sta_device = _interfaces.sta;

    let sta_config = embassy_net::Config::dhcpv4(Default::default());

    let rng = Rng::new();
    let seed = (rng.random() as u64) << 32 | rng.random() as u64;
    let tls_seed = rng.random() as u64 | ((rng.random() as u64) << 32);

    let (sta_stack, sta_runner) = embassy_net::new(
        sta_device,
        sta_config,
        mk_static!(StackResources<3>, StackResources::<3>::new()),
        seed,
    );

    spawner.spawn(connection(_wifi_controller)).ok();
    spawner.spawn(net_task(sta_runner)).ok();

    let sta_address = loop {
        if let Some(config) = sta_stack.config_v4() {
            let address = config.address.address();
            info!("Got IP: {}", address);
            break address;
        }
        info!("Waiting for IP...");
        Timer::after(Duration::from_millis(500)).await;
    };

    info!("Sta addr: {:?}", sta_address);

    access_website(sta_stack, tls_seed).await;

    // let mut rx_buffer = [0; 4096];
    // let mut tx_buffer = [0; 4096];

    // // TODO: Spawn some tasks
    // let _ = spawner;

    // let url = "https://iotjukebox.onrender.com/song";
    // let tcp_state = TcpClientState::<1, 4096, 4096>::new();
    // let tcp_client = TcpClient::new(sta_stack, &tcp_state);
    // let dns_socket = DnsSocket::new(sta_stack);

    // let tls = TlsConfig::new(
    //     tls_seed,
    //     &mut rx_buffer,
    //     &mut tx_buffer,
    //     reqwless::client::TlsVerify::None,
    // );
    // let mut client = HttpClient::new_with_tls(&tcp_client, &dns_socket, tls);

    // let mut http_req = client
    //     .request(reqwless::request::Method::GET, url)
    //     .await
    //     .expect("request failed");

    // let mut buffer = [0u8; 4096];

    // let response = http_req
    //     .send(&mut buffer)
    //     .await
    //     .unwrap();

    // info!("Got response");

    // let res = response.body().read_to_end().await.unwrap();
    // let content = core::str::from_utf8(res).unwrap();
    // info!("{:?}", content);

    loop {
        Timer::after(Duration::from_secs(2)).await;
        //
        let addrs = sta_stack
            .dns_query("iotjukebox.onrender.com", embassy_net::dns::DnsQueryType::A)
            .await
            .expect("No ip address found");

        info!("Ip: {:?}", addrs);

        // let remote_endpoint =
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0-rc.1/examples/src/bin
}

// #[embassy_executor::task]
// async fn get_req(mut controller: WifiController<'static>) {

// }

#[embassy_executor::task(pool_size = 2)]
async fn net_task(mut runner: Runner<'static, WifiDevice<'static>>) {
    runner.run().await
}

#[embassy_executor::task]
async fn connection(mut controller: WifiController<'static>) {
    info!("start connection task");
    info!("Device capabilities: {:?}", controller.capabilities());
    loop {
        if esp_radio::wifi::sta_state() == WifiStaState::Connected {
            // wait until we're no longer connected
            // info!("We were connected, now fuck it");
            info!("We are connected");
            // controller.wait_for_event(WifiEvent::StaDisconnected).await;
            // Timer::after(Duration::from_millis(5000)).await
        }
        if !matches!(controller.is_started(), Ok(true)) {
            let client_config = ModeConfig::Client(
                ClientConfig::default()
                    .with_ssid(SSID.into())
                    .with_password(PASSWORD.into())
                    .with_auth_method(wifi::AuthMethod::WpaWpa2Personal),
            );
            controller.set_config(&client_config).unwrap();
            info!("Starting wifi");
            controller.start_async().await.unwrap();
            info!("Wifi started!");

            info!("Scan");
            let scan_config = ScanConfig::default().with_max(10);
            let result = controller
                .scan_with_config_async(scan_config)
                .await
                .unwrap();
            for ap in result {
                info!("{:?}", ap);
            }
        }
        info!("About to connect...");

        match controller.connect_async().await {
            Ok(_) => info!("Wifi connected!"),
            Err(e) => {
                info!("Failed to connect to wifi: {:?}", e);
                Timer::after(Duration::from_millis(5000)).await
            }
        }
    }
}

// static TLS_RX: StaticCell<[u8; 4096]> = StaticCell::new();
// static TLS_TX: StaticCell<[u8; 4096]> = StaticCell::new();
// static HTTP_BUF: StaticCell<[u8; 4096]> = StaticCell::new();

async fn access_website(stack: Stack<'_>, tls_seed: u64) {
    // let rx_buffer: &mut [u8; 4096] = TLS_RX.init([0; 4096]);
    // let tx_buffer: &mut [u8; 4096] = TLS_TX.init([0; 4096]);
    // let http_buf: &mut [u8; 4096] = HTTP_BUF.init([0; 4096]);

    let mut tx_buffer = [0; 4096];
    let mut rx_buffer = [0; 4096];
    let mut http_buf = [0; 4096*5];
    const TCP_RX: usize = 4096;
    const TCP_TX: usize = 4096;

    let dns = DnsSocket::new(stack);
    let tcp_state = TcpClientState::<1, TCP_RX, TCP_TX>::new();
    let tcp = TcpClient::new(stack, &tcp_state);

    let tls = TlsConfig::new(
        tls_seed,
        &mut rx_buffer,
        &mut tx_buffer,
        reqwless::client::TlsVerify::None,
    );

    info!("Tls config done");

    let mut client = HttpClient::new_with_tls(&tcp, &dns, tls);
    info!("Tls http client ok");

    let url = "https://iotjukebox.onrender.com/song";
    let mut http_req = client
        .request(reqwless::request::Method::GET, url)
        .await
        .unwrap();
    info!("http request ok");

    let response = http_req.send(&mut http_buf).await.unwrap();

    info!("Got response");
    let res = response.body().read_to_end().await.unwrap();

    let content = core::str::from_utf8(res).unwrap();
    info!("{}", content);
}

// #[embassy_executor::task]
// async fn https_get(stack: Stack<'static, WifiDevice<'static>>) {

//     let state = TcpClientState::<1, 4096, 4096>::new();
//     let mut tcp_client = TcpClient::new(stack, &state);
//     let dns_socket = DnsSocket::new(&stack);
//     let mut rsa = Rsa::new(peripherals.RSA);
//     let config = TlsConfig::new(
//         reqwless::TlsVersion::Tls1_3,
//         reqwless::Certificates {
//             ca_chain: reqwless::X509::pem(CERT.as_bytes()).ok(),
//             ..Default::default()
//         },
//         Some(&mut rsa), // Will use hardware acceleration
//     );
//     let mut client = HttpClient::new_with_tls(&tcp_client, &dns_socket, config);

//     let mut request = client
//         .request(reqwless::request::Method::GET, "https://www.google.com")
//         .await
//         .unwrap()
//         .content_type(reqwless::headers::ContentType::TextPlain)
//         .headers(&[("Host", "google.com")])
//         .send(&mut buffer)
//         .await
//         .unwrap();

// }
