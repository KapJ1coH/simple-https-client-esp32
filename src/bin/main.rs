#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use core::str::FromStr;

use alloc::string::String;

use alloc::vec::Vec;
use defmt::{Debug2Format, error, info};
use embassy_executor::Spawner;
use embassy_net::Stack;
use embassy_net::dns::DnsSocket;
use embassy_net::tcp::client::{TcpClient, TcpClientState};
use embassy_net::{Runner, StackResources};

use embassy_time::{Duration, Timer};
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::timer::timg::TimerGroup;
use esp_hal::{clock::CpuClock, rng::Rng};
use esp_hal::{
    ledc::{LSGlobalClkSource, Ledc, channel, timer},
    main,
};
use esp_hal_buzzer::{Buzzer, ToneValue, notes::*, song};
use esp_radio::Controller;
use esp_radio::wifi::{ScanConfig, WifiController, WifiDevice, WifiStaState};
use esp_radio::{
    ble::controller::BleConnector,
    wifi::{self, ClientConfig, ModeConfig},
};
use microjson::JSONValue;
use reqwless::client::{HttpClient, TlsConfig};
use reqwless::{self};
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

    esp_alloc::heap_allocator!(#[unsafe(link_section = ".dram2_uninit")] size: 32 * 1024);
    // COEX needs more RAM - so we've added some more
    esp_alloc::heap_allocator!(size: 32 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    info!("Embassy initialized!");

    // let radio_init = esp_radio::init().expect("Failed to initialize Wi-Fi/BLE controller");
    let radio_init = &*mk_static!(Controller<'static>, esp_radio::init().unwrap());

    let (mut _wifi_controller, _interfaces) =
        esp_radio::wifi::new(radio_init, peripherals.WIFI, Default::default())
            .expect("Failed to initialize Wi-Fi controller");
    // find more examples https://github.com/embassy-rs/trouble/tree/main/examples/esp32
    let transport = BleConnector::new(radio_init, peripherals.BT, Default::default()).unwrap();
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

    let unparsed_response = send_get(sta_stack, tls_seed).await.unwrap();

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
                    freqs.push(val);
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

    // // let mut buzzer_pin = Output::new(peripherals.GPIO26, Level::Low, OutputConfig::default());

    // let sound_ratio = 0.9;


    let mut ledc = Ledc::new(peripherals.LEDC);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

    let mut buzzer = Buzzer::new(
        &ledc,
        timer::Number::Timer0,
        channel::Number::Channel1,
        peripherals.GPIO21,
    );

    let freqs_u32: Vec<u32> = freqs
        .iter()
        .copied()
        .map(|x| u32::try_from(x).expect("negative freq"))
        .collect();

    let times_u32: Vec<u32> = times
        .iter()
        .copied()
        .map(|x| u32::try_from(x).expect("negative time"))
        .collect();

    // now take slices that live as long as the Vecs
    let freqs_slice: &[u32] = &freqs_u32;
    let times_slice: &[u32] = &times_u32;

    info!("Freqs: {:?}", Debug2Format(&freqs_slice));
    info!("Times: {:?}", Debug2Format(&times_slice));



    // for (freq, time) in freqs.iter().zip(times.iter_mut()) {
    //     let duration: u64 = ms_per_beat as u64 * (4 / *time as u64);

    //     if *time as u64 > 0 {
    //         buzzer_pin.set_high();
    //         let _dur = duration as f64 * sound_ratio;
    //         Timer::after_millis(_dur as u64).await;
    //         buzzer_pin.set_low();
    //         Timer::after_millis((_dur * (1.0-sound_ratio)) as u64).await;

    //     } else {
    //         buzzer_pin.set_low();
    //         Timer::after_millis(duration).await;
    //     }

    // }

    loop {
        //
        // let addrs = sta_stack
        //     .dns_query("iotjukebox.onrender.com", embassy_net::dns::DnsQueryType::A)
        //     .await
        //     .expect("No ip address found");

        // info!("Ip: {:?}", addrs);
        //
        buzzer
            .play_tones_from_slice(freqs_slice, times_slice)
            .expect("can't play shit");

        // buzzer.play_song(&DOOM).unwrap();

        info!("Replay");
        Timer::after(Duration::from_secs(2)).await;

        // let remote_endpoint =
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0-rc.1/examples/src/bin
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

async fn send_get(stack: Stack<'_>, tls_seed: u64) -> Result<String, Error> {
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

pub const DOOM: [ToneValue; 680] = song!(
    225,
    [
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_D3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_C3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_AS2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_B2, EIGHTEENTH_NOTE),
        (NOTE_C3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_D3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_C3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_AS2, DOTTED_HALF_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_D3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_C3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_AS2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_B2, EIGHTEENTH_NOTE),
        (NOTE_C3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_D3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_C3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_AS2, DOTTED_HALF_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_D3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_C3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_AS2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_B2, EIGHTEENTH_NOTE),
        (NOTE_C3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_D3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_C3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_AS2, DOTTED_HALF_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_D3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_C3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_AS2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_B2, EIGHTEENTH_NOTE),
        (NOTE_C3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_D3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_FS3, DOTTED_SIXTEENTH_NOTE),
        (NOTE_D3, DOTTED_SIXTEENTH_NOTE),
        (NOTE_B2, DOTTED_SIXTEENTH_NOTE),
        (NOTE_A3, DOTTED_SIXTEENTH_NOTE),
        (NOTE_FS3, DOTTED_SIXTEENTH_NOTE),
        (NOTE_B2, DOTTED_SIXTEENTH_NOTE),
        (NOTE_D3, DOTTED_SIXTEENTH_NOTE),
        (NOTE_FS3, DOTTED_SIXTEENTH_NOTE),
        (NOTE_A3, DOTTED_SIXTEENTH_NOTE),
        (NOTE_FS3, DOTTED_SIXTEENTH_NOTE),
        (NOTE_D3, DOTTED_SIXTEENTH_NOTE),
        (NOTE_B2, DOTTED_SIXTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_D3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_C3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_AS2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_B2, EIGHTEENTH_NOTE),
        (NOTE_C3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_D3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_C3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_AS2, DOTTED_HALF_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_D3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_C3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_AS2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_B2, EIGHTEENTH_NOTE),
        (NOTE_C3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_D3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_B3, DOTTED_SIXTEENTH_NOTE),
        (NOTE_G3, DOTTED_SIXTEENTH_NOTE),
        (NOTE_E3, DOTTED_SIXTEENTH_NOTE),
        (NOTE_G3, DOTTED_SIXTEENTH_NOTE),
        (NOTE_B3, DOTTED_SIXTEENTH_NOTE),
        (NOTE_E4, DOTTED_SIXTEENTH_NOTE),
        (NOTE_G3, DOTTED_SIXTEENTH_NOTE),
        (NOTE_B3, DOTTED_SIXTEENTH_NOTE),
        (NOTE_E4, DOTTED_SIXTEENTH_NOTE),
        (NOTE_B3, DOTTED_SIXTEENTH_NOTE),
        (NOTE_G4, DOTTED_SIXTEENTH_NOTE),
        (NOTE_B4, DOTTED_SIXTEENTH_NOTE),
        (NOTE_A2, EIGHTEENTH_NOTE),
        (NOTE_A2, EIGHTEENTH_NOTE),
        (NOTE_A3, EIGHTEENTH_NOTE),
        (NOTE_A2, EIGHTEENTH_NOTE),
        (NOTE_A2, EIGHTEENTH_NOTE),
        (NOTE_G3, EIGHTEENTH_NOTE),
        (NOTE_A2, EIGHTEENTH_NOTE),
        (NOTE_A2, EIGHTEENTH_NOTE),
        (NOTE_F3, EIGHTEENTH_NOTE),
        (NOTE_A2, EIGHTEENTH_NOTE),
        (NOTE_A2, EIGHTEENTH_NOTE),
        (NOTE_DS3, EIGHTEENTH_NOTE),
        (NOTE_A2, EIGHTEENTH_NOTE),
        (NOTE_A2, EIGHTEENTH_NOTE),
        (NOTE_E3, EIGHTEENTH_NOTE),
        (NOTE_F3, EIGHTEENTH_NOTE),
        (NOTE_A2, EIGHTEENTH_NOTE),
        (NOTE_A2, EIGHTEENTH_NOTE),
        (NOTE_A3, EIGHTEENTH_NOTE),
        (NOTE_A2, EIGHTEENTH_NOTE),
        (NOTE_A2, EIGHTEENTH_NOTE),
        (NOTE_G3, EIGHTEENTH_NOTE),
        (NOTE_A2, EIGHTEENTH_NOTE),
        (NOTE_A2, EIGHTEENTH_NOTE),
        (NOTE_F3, EIGHTEENTH_NOTE),
        (NOTE_A2, EIGHTEENTH_NOTE),
        (NOTE_A2, EIGHTEENTH_NOTE),
        (NOTE_DS3, DOTTED_HALF_NOTE),
        (NOTE_A2, EIGHTEENTH_NOTE),
        (NOTE_A2, EIGHTEENTH_NOTE),
        (NOTE_A3, EIGHTEENTH_NOTE),
        (NOTE_A2, EIGHTEENTH_NOTE),
        (NOTE_A2, EIGHTEENTH_NOTE),
        (NOTE_G3, EIGHTEENTH_NOTE),
        (NOTE_A2, EIGHTEENTH_NOTE),
        (NOTE_A2, EIGHTEENTH_NOTE),
        (NOTE_F3, EIGHTEENTH_NOTE),
        (NOTE_A2, EIGHTEENTH_NOTE),
        (NOTE_A2, EIGHTEENTH_NOTE),
        (NOTE_DS3, EIGHTEENTH_NOTE),
        (NOTE_A2, EIGHTEENTH_NOTE),
        (NOTE_A2, EIGHTEENTH_NOTE),
        (NOTE_E3, EIGHTEENTH_NOTE),
        (NOTE_F3, EIGHTEENTH_NOTE),
        (NOTE_A2, EIGHTEENTH_NOTE),
        (NOTE_A2, EIGHTEENTH_NOTE),
        (NOTE_A3, EIGHTEENTH_NOTE),
        (NOTE_A2, EIGHTEENTH_NOTE),
        (NOTE_A2, EIGHTEENTH_NOTE),
        (NOTE_G3, EIGHTEENTH_NOTE),
        (NOTE_A2, EIGHTEENTH_NOTE),
        (NOTE_A2, EIGHTEENTH_NOTE),
        (NOTE_A3, DOTTED_SIXTEENTH_NOTE),
        (NOTE_F3, DOTTED_SIXTEENTH_NOTE),
        (NOTE_D3, DOTTED_SIXTEENTH_NOTE),
        (NOTE_A3, DOTTED_SIXTEENTH_NOTE),
        (NOTE_F3, DOTTED_SIXTEENTH_NOTE),
        (NOTE_D3, DOTTED_SIXTEENTH_NOTE),
        (NOTE_C4, DOTTED_SIXTEENTH_NOTE),
        (NOTE_A3, DOTTED_SIXTEENTH_NOTE),
        (NOTE_F3, DOTTED_SIXTEENTH_NOTE),
        (NOTE_A3, DOTTED_SIXTEENTH_NOTE),
        (NOTE_F3, DOTTED_SIXTEENTH_NOTE),
        (NOTE_D3, DOTTED_SIXTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_D3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_C3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_AS2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_B2, EIGHTEENTH_NOTE),
        (NOTE_C3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_D3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_C3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_AS2, DOTTED_HALF_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_D3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_C3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_AS2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_B2, EIGHTEENTH_NOTE),
        (NOTE_C3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_D3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_C3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_AS2, DOTTED_HALF_NOTE),
        (NOTE_CS3, EIGHTEENTH_NOTE),
        (NOTE_CS3, EIGHTEENTH_NOTE),
        (NOTE_CS4, EIGHTEENTH_NOTE),
        (NOTE_CS3, EIGHTEENTH_NOTE),
        (NOTE_CS3, EIGHTEENTH_NOTE),
        (NOTE_B3, EIGHTEENTH_NOTE),
        (NOTE_CS3, EIGHTEENTH_NOTE),
        (NOTE_CS3, EIGHTEENTH_NOTE),
        (NOTE_A3, EIGHTEENTH_NOTE),
        (NOTE_CS3, EIGHTEENTH_NOTE),
        (NOTE_CS3, EIGHTEENTH_NOTE),
        (NOTE_G3, EIGHTEENTH_NOTE),
        (NOTE_CS3, EIGHTEENTH_NOTE),
        (NOTE_CS3, EIGHTEENTH_NOTE),
        (NOTE_GS3, EIGHTEENTH_NOTE),
        (NOTE_A3, EIGHTEENTH_NOTE),
        (NOTE_B2, EIGHTEENTH_NOTE),
        (NOTE_B2, EIGHTEENTH_NOTE),
        (NOTE_B3, EIGHTEENTH_NOTE),
        (NOTE_B2, EIGHTEENTH_NOTE),
        (NOTE_B2, EIGHTEENTH_NOTE),
        (NOTE_A3, EIGHTEENTH_NOTE),
        (NOTE_B2, EIGHTEENTH_NOTE),
        (NOTE_B2, EIGHTEENTH_NOTE),
        (NOTE_G3, EIGHTEENTH_NOTE),
        (NOTE_B2, EIGHTEENTH_NOTE),
        (NOTE_B2, EIGHTEENTH_NOTE),
        (NOTE_F3, DOTTED_HALF_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_D3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_C3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_AS2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_B2, EIGHTEENTH_NOTE),
        (NOTE_C3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_D3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_B3, DOTTED_SIXTEENTH_NOTE),
        (NOTE_G3, DOTTED_SIXTEENTH_NOTE),
        (NOTE_E3, DOTTED_SIXTEENTH_NOTE),
        (NOTE_G3, DOTTED_SIXTEENTH_NOTE),
        (NOTE_B3, DOTTED_SIXTEENTH_NOTE),
        (NOTE_E4, DOTTED_SIXTEENTH_NOTE),
        (NOTE_G3, DOTTED_SIXTEENTH_NOTE),
        (NOTE_B3, DOTTED_SIXTEENTH_NOTE),
        (NOTE_E4, DOTTED_SIXTEENTH_NOTE),
        (NOTE_B3, DOTTED_SIXTEENTH_NOTE),
        (NOTE_G4, DOTTED_SIXTEENTH_NOTE),
        (NOTE_B4, DOTTED_SIXTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_D3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_C3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_AS2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_B2, EIGHTEENTH_NOTE),
        (NOTE_C3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_D3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_C3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_AS2, DOTTED_HALF_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_D3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_C3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_AS2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_B2, EIGHTEENTH_NOTE),
        (NOTE_C3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_D3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_FS3, DOTTED_SIXTEENTH_NOTE),
        (NOTE_DS3, DOTTED_SIXTEENTH_NOTE),
        (NOTE_B2, DOTTED_SIXTEENTH_NOTE),
        (NOTE_FS3, DOTTED_SIXTEENTH_NOTE),
        (NOTE_DS3, DOTTED_SIXTEENTH_NOTE),
        (NOTE_B2, DOTTED_SIXTEENTH_NOTE),
        (NOTE_G3, DOTTED_SIXTEENTH_NOTE),
        (NOTE_D3, DOTTED_SIXTEENTH_NOTE),
        (NOTE_B2, DOTTED_SIXTEENTH_NOTE),
        (NOTE_DS4, DOTTED_SIXTEENTH_NOTE),
        (NOTE_DS3, DOTTED_SIXTEENTH_NOTE),
        (NOTE_B2, DOTTED_SIXTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_D3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_C3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_AS2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_B2, EIGHTEENTH_NOTE),
        (NOTE_C3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_D3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_C3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_AS2, DOTTED_HALF_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_D3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_C3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_AS2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_B2, EIGHTEENTH_NOTE),
        (NOTE_C3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_D3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E4, DOTTED_SIXTEENTH_NOTE),
        (NOTE_B3, DOTTED_SIXTEENTH_NOTE),
        (NOTE_G3, DOTTED_SIXTEENTH_NOTE),
        (NOTE_G4, DOTTED_SIXTEENTH_NOTE),
        (NOTE_E4, DOTTED_SIXTEENTH_NOTE),
        (NOTE_G3, DOTTED_SIXTEENTH_NOTE),
        (NOTE_B3, DOTTED_SIXTEENTH_NOTE),
        (NOTE_D4, DOTTED_SIXTEENTH_NOTE),
        (NOTE_E4, DOTTED_SIXTEENTH_NOTE),
        (NOTE_G4, DOTTED_SIXTEENTH_NOTE),
        (NOTE_E4, DOTTED_SIXTEENTH_NOTE),
        (NOTE_G3, DOTTED_SIXTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_D3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_C3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_AS2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_B2, EIGHTEENTH_NOTE),
        (NOTE_C3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_D3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_C3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_AS2, DOTTED_HALF_NOTE),
        (NOTE_A2, EIGHTEENTH_NOTE),
        (NOTE_A2, EIGHTEENTH_NOTE),
        (NOTE_A3, EIGHTEENTH_NOTE),
        (NOTE_A2, EIGHTEENTH_NOTE),
        (NOTE_A2, EIGHTEENTH_NOTE),
        (NOTE_G3, EIGHTEENTH_NOTE),
        (NOTE_A2, EIGHTEENTH_NOTE),
        (NOTE_A2, EIGHTEENTH_NOTE),
        (NOTE_F3, EIGHTEENTH_NOTE),
        (NOTE_A2, EIGHTEENTH_NOTE),
        (NOTE_A2, EIGHTEENTH_NOTE),
        (NOTE_DS3, EIGHTEENTH_NOTE),
        (NOTE_A2, EIGHTEENTH_NOTE),
        (NOTE_A2, EIGHTEENTH_NOTE),
        (NOTE_E3, EIGHTEENTH_NOTE),
        (NOTE_F3, EIGHTEENTH_NOTE),
        (NOTE_A2, EIGHTEENTH_NOTE),
        (NOTE_A2, EIGHTEENTH_NOTE),
        (NOTE_A3, EIGHTEENTH_NOTE),
        (NOTE_A2, EIGHTEENTH_NOTE),
        (NOTE_A2, EIGHTEENTH_NOTE),
        (NOTE_G3, EIGHTEENTH_NOTE),
        (NOTE_A2, EIGHTEENTH_NOTE),
        (NOTE_A2, EIGHTEENTH_NOTE),
        (NOTE_A3, DOTTED_SIXTEENTH_NOTE),
        (NOTE_F3, DOTTED_SIXTEENTH_NOTE),
        (NOTE_D3, DOTTED_SIXTEENTH_NOTE),
        (NOTE_A3, DOTTED_SIXTEENTH_NOTE),
        (NOTE_F3, DOTTED_SIXTEENTH_NOTE),
        (NOTE_D3, DOTTED_SIXTEENTH_NOTE),
        (NOTE_C4, DOTTED_SIXTEENTH_NOTE),
        (NOTE_A3, DOTTED_SIXTEENTH_NOTE),
        (NOTE_F3, DOTTED_SIXTEENTH_NOTE),
        (NOTE_A3, DOTTED_SIXTEENTH_NOTE),
        (NOTE_F3, DOTTED_SIXTEENTH_NOTE),
        (NOTE_D3, DOTTED_SIXTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_D3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_C3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_AS2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_B2, EIGHTEENTH_NOTE),
        (NOTE_C3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_D3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_C3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_AS2, DOTTED_HALF_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_D3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_C3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_AS2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_B2, EIGHTEENTH_NOTE),
        (NOTE_C3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_D3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_C3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_AS2, DOTTED_HALF_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_D3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_C3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_AS2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_B2, EIGHTEENTH_NOTE),
        (NOTE_C3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_D3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_C3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_AS2, DOTTED_HALF_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_D3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_C3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_AS2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_B2, EIGHTEENTH_NOTE),
        (NOTE_C3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_D3, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_E2, EIGHTEENTH_NOTE),
        (NOTE_B3, DOTTED_SIXTEENTH_NOTE),
        (NOTE_G3, DOTTED_SIXTEENTH_NOTE),
        (NOTE_E3, DOTTED_SIXTEENTH_NOTE),
        (NOTE_B2, DOTTED_SIXTEENTH_NOTE),
        (NOTE_E3, DOTTED_SIXTEENTH_NOTE),
        (NOTE_G3, DOTTED_SIXTEENTH_NOTE),
        (NOTE_C4, DOTTED_SIXTEENTH_NOTE),
        (NOTE_B3, DOTTED_SIXTEENTH_NOTE),
        (NOTE_G3, DOTTED_SIXTEENTH_NOTE),
        (NOTE_B3, DOTTED_SIXTEENTH_NOTE),
        (NOTE_G3, DOTTED_SIXTEENTH_NOTE),
        (NOTE_E3, DOTTED_SIXTEENTH_NOTE)
    ]
);
