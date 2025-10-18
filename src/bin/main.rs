#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use core::cell::RefCell;
use core::str::FromStr;
use heapless::Deque;

use alloc::string::String;

use alloc::vec::Vec;
use bt_hci::cmd::le::LeSetScanParams;
use bt_hci::controller::ControllerCmdSync;
use defmt::{Debug2Format, error, info, warn, Format};
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_net::Stack;
use embassy_net::dns::DnsSocket;
use embassy_net::tcp::client::{TcpClient, TcpClientState};
use embassy_net::{Runner, StackResources};

use embassy_time::{Duration, Timer};
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::i2c::master::{Config, I2c};
use esp_hal::ledc::{LSGlobalClkSource, Ledc, channel, timer};
use esp_hal::peripherals::{self, GPIO4, GPIO15, GPIO16, GPIO21, I2C0, LEDC, Peripherals};
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::{clock::CpuClock, rng::Rng};
use esp_hal_buzzer::Buzzer;
use esp_radio::wifi::{ScanConfig, WifiController, WifiDevice, WifiStaState};
use esp_radio::{
    ble::controller::BleConnector,
    wifi::{self, ClientConfig, ModeConfig},
};
use microjson::JSONValue;
use reqwless::client::{HttpClient, TlsConfig};
use reqwless::{self};
use static_cell::StaticCell;
use trouble_host::prelude::*;
use {esp_backtrace as _, esp_println as _};

extern crate alloc;

const CONNECTIONS_MAX: usize = 1;
const L2CAP_CHANNELS_MAX: usize = 1;
const SSID: &str = "Not a honeypot";
const PASSWORD: &str = "Nbvf12nbvf12";
// const SSID: &str = "Kap";
// const PASSWORD: &str = "Nbvf12nbvf12";

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();


#[derive(Debug, Format)]
pub enum MyError {
    Http(&'static str),
    Network(reqwless::Error),
}

impl From<reqwless::Error> for MyError {
    fn from(value: reqwless::Error) -> Self {
        MyError::Network(value)
    }
}

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

    esp_alloc::heap_allocator!(#[unsafe(link_section = ".dram2_uninit")] size: 96 * 1024);
    // COEX needs more RAM - so we've added some more
    esp_alloc::heap_allocator!(size: 24 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    info!("Embassy initialized!");

    // let radio_init = esp_radio::init().expect("Failed to initialize Wi-Fi/BLE controller");
    static RADIO: StaticCell<esp_radio::Controller<'static>> = StaticCell::new();
    let radio_init = RADIO.init(esp_radio::init().unwrap());

    let (mut _wifi_controller, _interfaces) =
        esp_radio::wifi::new(radio_init, peripherals.WIFI, Default::default())
            .expect("Failed to initialize Wi-Fi controller");
    // find more examples https://github.com/embassy-rs/trouble/tree/main/examples/esp32
    let transport = BleConnector::new(radio_init, peripherals.BT, Default::default()).unwrap();
    let ble_controller: ExternalController<_, 1> = ExternalController::new(transport);

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

    let unparsed_response = send_get(sta_stack, tls_seed).await.unwrap();

    let leddc_pin = peripherals.LEDC;
    let buzzer_pin = peripherals.GPIO21;
    // show_song_name();
    spawner
        .spawn(play_sond(unparsed_response, leddc_pin, buzzer_pin))
        .ok();

    ble_scanner_run(ble_controller).await;

    loop {
        //
        // let addrs = sta_stack
        //     .dns_query("iotjukebox.onrender.com", embassy_net::dns::DnsQueryType::A)
        //     .await
        //     .expect("No ip address found");

        // info!("Ip: {:?}", addrs);
        //
        // buzzer
        //     .play_tones_from_slice(&freqs, &times)
        //     .expect("can't play shit");

        info!("Replay");
        Timer::after(Duration::from_secs(2)).await;

        // let remote_endpoint =
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0-rc.1/examples/src/bin
}

async fn ble_scanner_run<C>(controller: C)
where
    C: Controller + ControllerCmdSync<LeSetScanParams>,
{
    let address: Address = Address::random([0xff, 0x8f, 0x1b, 0x05, 0xe4, 0xff]);

    info!("Our address ={:?}", address);

    let mut resources: HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> =
        HostResources::new();
    let stack = trouble_host::new(controller, &mut resources).set_random_address(address);

    let Host {
        central,
        mut runner,
        ..
    } = stack.build();

    let printer = Printer {
        seen: RefCell::new(Deque::new()),
    };
    let mut scanner = Scanner::new(central);
    let _ = join(runner.run_with_handler(&printer), async {
        let config = trouble_host::connection::ScanConfig::default();
        let mut _session = scanner.scan(&config).await.unwrap();
        // Scan forever
        loop {
            Timer::after(Duration::from_secs(1)).await;
        }
    })
    .await;
}

struct Printer {
    seen: RefCell<Deque<BdAddr, 128>>,
}

impl EventHandler for Printer {
    fn on_adv_reports(&self, mut it: LeAdvReportsIter<'_>) {
        let mut seen = self.seen.borrow_mut();
        while let Some(Ok(report)) = it.next() {
            if seen.iter().find(|b| b.raw() == report.addr.raw()).is_none() {
                info!("discovered: {:?}", report.addr);
                info!("report: {:?}", report);
                if seen.is_full() {
                    seen.pop_front();
                }
                seen.push_back(report.addr).unwrap();
            }
        }
    }
}

#[embassy_executor::task]
async fn play_sond(
    unparsed_response: String,
    leddc_pin: LEDC<'static>,
    buzzer_pin: GPIO21<'static>,
) {
    let response_json = parse_json(&unparsed_response);
    info!("Json: {}", Debug2Format(&response_json));

    let mut freqs = Vec::new();
    let mut times = Vec::new();

    let tempo = response_json
        .get_key_value("tempo")
        .unwrap()
        .read_string()
        .unwrap()
        .trim_matches('"')
        .parse::<u32>()
        .unwrap();

    let wholenote = (600_000 * 4) / tempo;

    for (i, v) in response_json
        .get_key_value("melody")
        .unwrap()
        .iter_array()
        .unwrap()
        .enumerate()
    {
        let val = v
            .read_string()
            .expect("String")
            .trim_matches(['\\', '"'])
            .parse::<i32>();

        match val {
            Ok(val) => {
                if i % 2 == 0 {
                    freqs.push(val as u32);
                } else if val < 0 {
                    let _val = val.abs() - val.abs() / 2;
                    times.push(wholenote / (_val * 10) as u32);
                } else {
                    times.push(wholenote / (val * 10) as u32);
                }
            }
            Err(_) => error!("Parsing error {}", Debug2Format(&v)),
        }
    }

    let mut ledc = Ledc::new(leddc_pin);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

    let mut buzzer = Buzzer::new(
        &ledc,
        timer::Number::Timer0,
        channel::Number::Channel1,
        buzzer_pin,
    );
    buzzer
        .play_tones_from_slice(&freqs, &times)
        .expect("can't play shit");

    info!("Freqs: {:?}", Debug2Format(&freqs));
    info!("Times: {:?}", Debug2Format(&times));
}

fn parse_json<'a>(unparsed_response: &'a str) -> JSONValue<'a> {
    JSONValue::load_and_verify(unparsed_response).unwrap()
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

async fn send_get(stack: Stack<'_>, tls_seed: u64) -> Result<String, MyError> {
    let mut tx_buffer = [0; 4096];
    let mut rx_buffer = [0; 4096 * 5];
    let mut http_buf = [0; 4096];
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

    // let mut attempt = 0;
    // let response;
    // loop {
    //     attempt += 1;

    //     let mut req = match client.request(reqwless::request::Method::GET, url).await {
    //         Ok(r) => r,
    //         Err(e) => {
    //             error!("Request attempt {} failed: {:?}", attempt, e);
    //             if attempt >= 3 {
    //                 return Err(MyError::Http("Failed http after 3 tries"));
    //             }
    //             Timer::after(Duration::from_secs(2)).await;
    //             continue;
    //         }
    //     };

    //     match req.send(&mut http_buf).await {
    //         Ok(r) => {response = r; break},
    //         Err(e) => {
    //             error!("Send attempt {} failed: {:?}", attempt, e);
    //             if attempt >= 3 {
    //                 return Err(MyError::Http("Failed http after 3 tries"));
    //             }
    //             Timer::after(Duration::from_secs(2)).await;
    //             continue;
    //         }
    //     }
    // };

    info!("Got response");
    let res = response.body().read_to_end().await.unwrap();

    let content = core::str::from_utf8(res).unwrap();
    info!("{}", content);

    let content = String::from_str(content).unwrap();
    Ok(content)
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
//

// async fn show_song_name(i2c: I2C0, sda_pin: GPIO4, scl_pin: GPIO15, rst_pin: GPIO16) -> _ {

//     let i2c_conf = Config::default().with_frequency(Rate::from_khz(100));
//     // SDA	GPIO 4
//     // SCL	GPIO 15
//     // RST	GPIO 16
//     let mut i2c = I2c::new(i2c, i2c_conf)
//         .unwrap()
//         .with_sda(sda_pin)
//         .with_scl(scl_pin);

//     // Reset OLED (critical on these boards)
//     let rst_config = OutputConfig::default();
//     let mut rst = Output::new(rst_pin, Level::High, rst_config);
//     rst.set_low();
//     Timer::after(Duration::from_millis(10)).await;
//     rst.set_high();
//     Timer::after(Duration::from_millis(10)).await;

//     // Quick bus scan to prove wiring/address
//     let mut found = false;
//     for addr in 0x03..=0x77 {
//         if i2c.write(addr, &[]).is_ok() {
//             info!("I2C device @ 0x{:02X}", addr);
//             found = true;
//         }
//     }
//     if !found {
//         warn!("No I2C devices found");
//     }

//     let interface = I2CDisplayInterface::new(i2c);
//     let mut display = Ssd1306A::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
//         .into_buffered_graphics_mode();

//     match display.init() {
//         Ok(()) => info!("ssd1306 init ok"),
//         Err(e) => warn!("ssd1306 init failed: {}", Debug2Format(&e)),
//     }

//     display.clear(BinaryColor::Off).unwrap();

//     // Draw some text
//     let text_style = MonoTextStyleBuilder::new()
//         .font(&FONT_6X10)
//         .text_color(BinaryColor::On)
//         .build();

//     // TODO: Spawn some tasks
//     let _ = spawner;

//     let mut counter = 0_i32;

//     loop {
//         display.clear(BinaryColor::Off).unwrap();
//         Text::with_baseline(
//             "David, where's my compiler",
//             Point {
//                 x: (counter),
//                 y: (16),
//             },
//             text_style,
//             Baseline::Top,
//         )
//         .draw(&mut display)
//         .unwrap();
//         display.flush().unwrap();

//         Timer::after(Duration::from_millis(30)).await;

//         counter -= 3;
//         if counter < -256 {
//             counter = 127;
//         }
//     }

// }
