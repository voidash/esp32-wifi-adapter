// #[macro_use]
// extern crate lazy_static;

use std::sync::{Arc, Mutex};
use std::time::Duration;
use std::{sync::RwLock, thread};


use esp_idf_svc::eventloop::EspEventLoopType;
// use esp_idf_hal::task::watchdog::{TWDTConfig, self};
// use esp_idf_hal::task::watchdog::TWDTDriver;
use esp_idf_svc::sys::{ ESP_ERR_NVS_NO_FREE_PAGES, ESP_ERR_NVS_NEW_VERSION_FOUND, uart_config_t, uart_word_length_t_UART_DATA_8_BITS, uart_parity_t_UART_PARITY_DISABLE, uart_stop_bits_t_UART_STOP_BITS_2, uart_hw_flowcontrol_t_UART_HW_FLOWCTRL_DISABLE, uart_sclk_t_UART_SCLK_APB, uart_config_t__bindgen_ty_1, uart_set_pin, UART_NUM_0, UART_PIN_NO_CHANGE, uart_port_t, gpio_num_t_GPIO_NUM_1, gpio_num_t_GPIO_NUM_3, uart_driver_install, uart_param_config, uart_read_bytes, vTaskDelay, esp_event_handler_register, WIFI_EVENT, ESP_EVENT_ANY_ID, esp_wifi_init, wifi_init_config_t, esp_wifi_set_storage, wifi_storage_t_WIFI_STORAGE_RAM, wifi_interface_t_WIFI_IF_AP, wifi_scan_method_t_WIFI_FAST_SCAN, esp_netif_init, esp_netif_create_default_wifi_sta, esp_netif_ip_info_t, esp_ip4_addr, esp_netif_dhcpc_stop, esp_netif_set_ip_info, wifi_sort_method_t_WIFI_CONNECT_AP_BY_SIGNAL, wifi_pmf_config_t, esp_wifi_set_ps, wifi_ps_type_t_WIFI_PS_NONE, esp_wifi_set_protocol, wifi_mode_t_WIFI_MODE_AP, wifi_mode_t_WIFI_MODE_STA, esp_event_base_t, esp_wifi_connect, uart_write_bytes} ;

use ringbuffer::{AllocRingBuffer, RingBuffer};
use std::ffi::CString;


use esp_idf_svc::hal::peripherals::Peripherals;
use esp_idf_svc::wifi::{AccessPointConfiguration,BlockingWifi, EspWifi, AuthMethod, Configuration, ClientConfiguration, WifiEvent};
use esp_idf_svc::{eventloop::EspSystemEventLoop, nvs::EspDefaultNvsPartition};

// lazy_static! {
//     static ref PARSED_VALUE: RwLock<NetworkInfo> = RwLock::new(NetworkInfo::AP(NetworkDefaults{
//         ssid: "Hello".into(),
//         password: None,
//         channel: 11,
//         protocol:1
//     },2));
//
//     static ref WIFI_TO_SERIAL: RwLock<AllocRingBuffer<u8>> = RwLock::new(AllocRingBuffer::new(1024*32));    
//     static ref SERIAL_TO_WIFI: RwLock<AllocRingBuffer<u8>> = RwLock::new(AllocRingBuffer::new(1024*16));
// }
//
use anyhow::Result;

macro_rules! error_check {
    ($x:expr) => {
        esp_idf_svc::sys::EspError::convert($x)
    };

    ($x:expr, $y:expr) => {
        esp_idf_svc::sys::EspError::convert($x).expect($y);
    }
}

//utils

fn set_str(buf: &mut [u8], s: &str) {
   let cs = CString::new(s).unwrap(); 
   let ss = cs.as_bytes_with_nul();
   buf[..ss.len()].copy_from_slice(ss);
}

#[derive(Debug,Default)]
struct NetworkDefaults {
    ssid: String,
    password: Option<String>,
    channel: u8,
    protocol: u8
}

#[derive(Debug)]
enum NetworkInfo{
    AP(NetworkDefaults,i32),
    StationMode(NetworkDefaults,Station)
}


#[derive(Debug)]
struct Station {
   ip : (u8,u8,u8,u8),
   gateway: (u8,u8,u8,u8),
   netmask: (u8,u8,u8,u8),
   baud : u32
}

impl Station {
    fn convert(value : (u8,u8,u8,u8)) -> u32 {
        ((value.0 as u32) & 0xff) << 24 |
        ((value.1 as u32) & 0xff) << 16 |
        ((value.2 as u32) & 0xff) << 8 |
        ((value.3 as u32) & 0xff)
    } 

    fn parse_ip(ip: &str) -> Option<(u8,u8,u8,u8)> {
        let values = ip.split(".").filter_map(|val| val.parse::<u8>().ok()).collect::<Vec<u8>>();
        if values.len() == 4 {
            return Some((values[0], values[1], values[2], values[3]));
        }  

        None
    }
}

#[repr(u8)]
#[derive(Debug,Default)]
enum WifiProtocol {
    #[default]
    PROTOCOL11B = 1, 
    PROTOCOL11G = 2, 
    PROTOCOL11N = 4, 
    PROTOCOLLR = 8
}


fn parse_info(line: &str) -> Option<NetworkInfo> {
    use NetworkInfo::*;
    use WifiProtocol::*;
    let mut defaults = NetworkDefaults::default();

    loop {
        if line.starts_with("ssid") {
            defaults.ssid = line.trim_start_matches("ssid").trim().to_string();
        }else if line.starts_with("password") {
            defaults.password = Some(line.trim_start_matches("password").trim().to_string());
        }else if line.starts_with("channel") {
            defaults.channel = line.trim_start_matches("channel").trim().parse::<u8>().unwrap();
        }else if line.starts_with("protocols") {
            let protocol = line.trim_start_matches("protocols").trim().to_string();
            if let Some(_) = protocol.find("b") {
               defaults.protocol |= PROTOCOL11B as u8;
            }else if let Some(_) = protocol.find("g") {
               defaults.protocol |= PROTOCOL11G as u8;
            }else if let Some(_) = protocol.find("n") {
               defaults.protocol |= PROTOCOL11N as u8;
            }else if let Some(_) = protocol.find("l") {
               defaults.protocol |= PROTOCOLLR as u8;
            }
        } else if line.starts_with("ap") {
            return Some(AP(defaults,line.trim_start_matches("ap").trim().parse::<i32>().unwrap()));
        }else if line.starts_with("sta") {
             let station_mode = line.trim_start_matches("sta").trim().split(" ").collect::<Vec<&str>>();
             let ip = Station::parse_ip(station_mode[0]).expect("Ip parsing failed");
             let gateway = Station::parse_ip(station_mode[1]).expect("Gateway parsing failed");
             let netmask = Station::parse_ip(station_mode[2]).expect("Netmask parsing failed");
             let baud_rate = station_mode[3].trim().parse::<u32>().expect("Error getting Baud Rate for station mode");
             return Some(StationMode(defaults,Station { ip, gateway, netmask, baud: baud_rate }));
        }
    }
}


const BAUD_RATE: u32 = 9600;

// alot of unsafe UART stuff here
// fn initiazlize_uart() -> anyhow::Result<()>{
//     let peripherals = Peripherals::take()?;
//
//     let tx = peripherals.pins.gpio1;
//     let rx = peripherals.pins.gpio3;
//
//     let config = config::Config::new().baudrate(Hertz(BAUD_RATE));
//
//     let uart = Arc::new(Mutex::new(UartDriver::new(
//         peripherals.uart0,
//         tx,
//         rx,
//         Option::<gpio::Gpio0>::None,
//         Option::<gpio::Gpio1>::None,
//         &config
//       )?));
//
//     let u1 = uart.clone();
//     let u2 = uart.clone();
//
//
//     Ok(())
// }
//
// fn uart_tx(uart: Arc<Mutex<UartDriver>> ) {
//     loop {
//
//         let buffer = WIFI_TO_SERIAL.read().unwrap().to_vec();
//         // let buffer = [0,1,2,34,5,6,7,8,9];
//         let len: usize = buffer.len();
//
//         let mut header: [u8; 4] = [0xAA, 0 , (len as u8) & 0xFF, (len >> 8) as u8 & 0xFF];
//
//         header[1] = u8::MAX - (header[0] + header[2] + header[3]);
//
//         for i in 0..len {
//             header[1] = header[1].wrapping_sub(buffer[i]);
//         }
//
//         //uart-write-bytes
//         // unsafe{uart_write_bytes(UART_NUM_0 as i32, header.as_mut_ptr().cast() as *const std::ffi::c_void, 4)};
//         // unsafe{uart_write_bytes(UART_NUM_0 as i32, buffer.as_ptr().cast() as *const std::ffi::c_void, 4)};
//
//         let uart = uart.lock().unwrap();
//         uart.write(&header).unwrap();
//         uart.write(&buffer).unwrap();
//     }
// }
//
//
// fn uart_rx(uart: Arc<Mutex<UartDriver>>) {
//     loop {
//         let mut buffer = [0u8;2000];
//         loop {
//             let uart = uart.lock().unwrap();
//             unsafe{uart_read_bytes(UART_NUM_0 as i32, (&mut buffer[0..0]).as_mut_ptr().cast(), 1, 0xFFFFFFFF);}
//             uart.read(&mut buffer[0..0],BLOCK).unwrap();
//
//             if buffer[0] == 0xAA {break;}
//         }
//
//         let uart = uart.lock().unwrap();
//         uart.read(&mut buffer[1..1],BLOCK).unwrap();
//         uart.read(&mut buffer[2..2],BLOCK).unwrap();
//         uart.read(&mut buffer[2..2],BLOCK).unwrap();
//
//
//         let len = buffer[2] as u16 | (buffer[3] as u16) << 8;
//         if 4 + len as usize > buffer.len() * 4 {
//             continue;
//         }
//
//         for i in 0..(len as usize) {
//             // unsafe{uart_read_bytes(UART_NUM_0 as i32, (&mut buffer[(4+i as usize)..(4+i as usize)]).as_mut_ptr().cast(), 1, 0xFFFFFFFF);}
//             uart.read(&mut buffer[4+i..4+i],BLOCK).unwrap();
//         }
//
//         println!("{:?}",buffer);
//
//         let mut acc = 0;
//         for i in 0..(4+len) {
//             acc += buffer[i as usize];
//         }
//
//         if acc == 0 {
//             // unsafe{xRingbufferSend(SERIAL_TO_WIFI, buffer.as_mut_ptr().cast() , len.into(), 0);}
//             for i in 0..len{
//                 SERIAL_TO_WIFI.write().unwrap().push(buffer[i as usize]);
//             }
//         } 
//     }
// }


fn initialize_wifi(wifi: &mut BlockingWifi<EspWifi>) -> Result<()> {

    let parsed_value = NetworkInfo::AP(NetworkDefaults{
        ssid: "Hello".into(),
        password: None,
        channel: 11,
        protocol:1
    },2);

        match &(parsed_value) {
              NetworkInfo::AP(config,_baudrate) => {
                // Access point
                
                let ap_configuration: Configuration = Configuration::AccessPoint(
                                        AccessPointConfiguration {
                                            ssid: config.ssid.as_str().into(),
                                            ssid_hidden: false,
                                            password: if let Some(pass) = &config.password {pass.as_str().into()} else {"".into()},
                                            auth_method: if let None = &config.password { AuthMethod::None} else { AuthMethod::WPA2Personal },
                                            channel: config.channel,
                                            ..Default::default()
                                            }
                );

                wifi.set_configuration(&ap_configuration)?;

                wifi.start()?;

                wifi.wait_netif_up()?;
                log::info!("Wifi netif up");
            }
            NetworkInfo::StationMode(config, _station) => {
               let sta_config: Configuration = Configuration::Client(
                   ClientConfiguration {
                    ssid: config.ssid.as_str().into(),
                    bssid: None,
                    password: if let Some(password) = &config.password { password.as_str().into() } else {"".into()},
                    auth_method: if let None = &config.password {AuthMethod::None} else {AuthMethod::WPA2Personal},
                    channel: Some(config.channel),
                });

                wifi.set_configuration(&sta_config)?;

                wifi.start()?;

                wifi.wait_netif_up()?;
            }
        }

    Ok(())
}

// unsafe extern "C" fn on_wifi_event(event_handler_arg: *mut ::core::ffi::c_void, event_base: esp_event_base_t , event_id: i32, event_data: *mut ::core::ffi::c_void) {
//         use WifiEvent::*;
//
//         let client_counter = 0; 
//
//         if let Some(status) = WifiEvent{
//             match status {
//                 ApStaConnected => {
//                     log::info!("AP STA connected");
//                     // set wifiRxCallback
//                     // error_check!(esp_wifi_internal_reg_rxcb());
//                 }
//                 ApStaDisconnected => {
//                     //we set it to nullptr
//                     log::info!("AP STA disconnected");
//                 }
//                 StaStart => {
//                     //do something with ICMP, IGMP , UDP, UDPLITE, TCP
//                     
//                 }
//
//                 StaConnected => {
//                     log::info!("STA connected");
//                 }
//
//                 StaDisconnected => {
//                     error_check!(esp_wifi_connect(),"unable to connect again after connection was lost");
//                 }
//
//                 _ => { }
//             }
//         }
// } 

// TODO: look into Netstack-lwip
// TODO: look into pbuf 
fn wifi_tx() {
    // create packet buffer 
    // check type of payload and send it
    //
    // let mut len: usize = 0;
    loop {
        // let buffer = SERIAL_TO_WIFI.read().unwrap();
        // let len = buffer.len();

    }
}


fn read_line() -> String {
    let mut pos = 0;
    let mut line: [char;256] = [' ';256];
    loop {
        let ch = unsafe{ esp_idf_svc::sys::getchar() } as u8;

        if ch == 255 {
            unsafe {vTaskDelay(1)};
            continue;
        }
        if ch == '\n' as u8 {
            break;
        }
        log::info!("{}", ch);

        line[pos] = char::from_u32(ch as u32).expect("non char value");
        pos += 1;
    }

    //convert to String
    line.iter().collect()
}

fn main() -> Result<()>{
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();
    log::info!("Awesome ESP32 based wifi dongle");


    // watchdog.feed()?;
    // unsafe { vTaskDelay(3) };

    // let line = read_line();
    // {
    //     let mut parsed_value = PARSED_VALUE.write().unwrap();
    //     *parsed_value = parse_info(&line).unwrap().into();
    // }


    // log::info!("{:?}", PARSED_VALUE.read().unwrap());


    let peripherals = Peripherals::take()?;
    let nvs = EspDefaultNvsPartition::take()?;
    let sys_loop = EspSystemEventLoop::take()?;

    let mut wifi = BlockingWifi::wrap(
            EspWifi::new(peripherals.modem, sys_loop.clone(), Some(nvs))?, 
            sys_loop.clone()
            )?;

    // initiazlize_uart()?;
    initialize_wifi(&mut wifi)?;



    let status = Arc::new(Mutex::new(
        (WifiEvent::StaStopped, WifiEvent::ApStopped),
    ));
    let s_status = status.clone();

    sys_loop.subscribe(move |event: &WifiEvent| {
        let mut guard = s_status.lock().unwrap();

        match event {
            WifiEvent::ApStarted => guard.1 = WifiEvent::ApStarted,
            WifiEvent::ApStopped => guard.1 = WifiEvent::ApStopped,
            WifiEvent::StaStarted => guard.0 = WifiEvent::StaStarted,
            WifiEvent::StaStopped => guard.0 = WifiEvent::StaStopped,
            WifiEvent::StaConnected => guard.0 = WifiEvent::StaConnected,
            WifiEvent::StaDisconnected => guard.0 = WifiEvent::StaDisconnected,
            WifiEvent::ScanDone => guard.0 = WifiEvent::ScanDone,
            _ => (),
        };
        println!(" khusi {:?}", event);
    })?;

    loop {
        println!("Test");
        thread::sleep(Duration::from_secs(1));
    }

    // Ok(())
}
