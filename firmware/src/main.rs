#[macro_use]
extern crate lazy_static;

use std::sync::{RwLock, Mutex, Arc};

use esp_idf_svc::sys::{xRingbufferCreate, RingbufferType_t_RINGBUF_TYPE_NOSPLIT, ESP_ERR_NVS_NO_FREE_PAGES, ESP_ERR_NVS_NEW_VERSION_FOUND, uart_config_t, uart_word_length_t_UART_DATA_8_BITS, uart_parity_t_UART_PARITY_DISABLE, uart_stop_bits_t_UART_STOP_BITS_2, uart_hw_flowcontrol_t_UART_HW_FLOWCTRL_DISABLE, uart_sclk_t_UART_SCLK_APB, uart_config_t__bindgen_ty_1, uart_set_pin, UART_NUM_0, UART_PIN_NO_CHANGE, uart_port_t, gpio_num_t_GPIO_NUM_1, gpio_num_t_GPIO_NUM_3, uart_driver_install, uart_param_config, RingbufHandle_t, xRingbufferReceive, uart_write_bytes, vRingbufferReturnItem, uart_read_bytes, xRingbufferSend, vTaskDelay, esp_event_handler_register, WIFI_EVENT, ESP_EVENT_ANY_ID, esp_wifi_init, wifi_init_config_t, esp_wifi_set_storage, wifi_storage_t_WIFI_STORAGE_RAM, wifi_interface_t_WIFI_IF_AP, wifi_scan_method_t_WIFI_FAST_SCAN, esp_netif_init, esp_netif_create_default_wifi_sta, esp_netif_ip_info_t, esp_ip4_addr, esp_netif_dhcpc_stop, esp_netif_set_ip_info, wifi_sort_method_t_WIFI_CONNECT_AP_BY_SIGNAL, wifi_pmf_config_t, esp_wifi_set_ps, wifi_ps_type_t_WIFI_PS_NONE, esp_wifi_set_protocol, wifi_mode_t_WIFI_MODE_AP, wifi_mode_t_WIFI_MODE_STA, esp_event_base_t, esp_wifi_internal_reg_rxcb, esp_wifi_connect};

use ringbuffer::{AllocRingBuffer, RingBuffer};
use strum_macros::FromRepr;
use std::ffi::CString;



lazy_static! {
    static ref PARSED_VALUE: RwLock<NetworkInfo> = RwLock::new(NetworkInfo::AP(NetworkDefaults::default(),2));

    static ref WIFI_TO_SERIAL: AllocRingBuffer<u8> = AllocRingBuffer::new(1024*32);    
    static ref SERIAL_TO_WIFI: AllocRingBuffer<u8> = AllocRingBuffer::new(1024*16);
}

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

#[derive(FromRepr,Debug)]
#[repr(i32)]
enum WifiEvent {
    WifiReady= 0,
    ScanDone= 1,
    StaStart= 2,
    StaStop= 3,
    StaConnected= 4,
    StaDisconnected= 5,
    StaAuthmodeChange= 6,
    StaWpsErsuccess= 7,
    StaWpsErFailed= 8,
    StaWpsErTimeout= 9,
    StaWpsErPin= 10,
    StaWpsErPbcOverlap= 11,
    ApStart= 12,
    ApStop= 13,
    ApStaConnected= 14,
    ApStaDisconnected= 15,
    ApProbeReqTecved= 16,
    FtmReport= 17,
    StaBssRssiLow= 18,
    ActionTxStatus= 19,
    RocDone= 20,
    StaBeaconTimeout= 21,
    Max= 22,
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


const BAUD_RATE: i32 = 9600;

// alot of unsafe UART stuff here
fn initiazlize_uart() {
    let uart_config = uart_config_t {
        baud_rate: BAUD_RATE,
        data_bits: uart_word_length_t_UART_DATA_8_BITS,
        parity: uart_parity_t_UART_PARITY_DISABLE,
        stop_bits: uart_stop_bits_t_UART_STOP_BITS_2,
        flow_ctrl: 0,
        rx_flow_ctrl_thresh: uart_hw_flowcontrol_t_UART_HW_FLOWCTRL_DISABLE as u8,
        __bindgen_anon_1: uart_config_t__bindgen_ty_1 { source_clk: uart_sclk_t_UART_SCLK_APB },
    };

    error_check!(unsafe{uart_set_pin(UART_NUM_0 as uart_port_t, gpio_num_t_GPIO_NUM_1 ,gpio_num_t_GPIO_NUM_3, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE)}, "cannot set uart");

    error_check!(unsafe{uart_driver_install(UART_NUM_0 as uart_port_t,256,256 , 0, core::ptr::null_mut(), 0)}, "can't install UART");
    error_check!(unsafe{uart_param_config(UART_NUM_0 as uart_port_t, &uart_config)}, "cannot set the UART parameter");
    // initialize Tx and Rx
}



fn uart_tx() {
    let mut len: usize = 0;
    loop {
        let buffer = WIFI_TO_SERIAL.to_vec();

        let mut header: [u8; 4] = [0xAA, 0 , (len as u8) & 0xFF, (len >> 8) as u8 & 0xFF];

        header[1] = u8::MAX - (header[0] + header[2] + header[3]);

        for i in 0..len {
            header[1] = header[1].wrapping_sub(buffer[i]);
        }

        //uart-write-bytes
        unsafe{uart_write_bytes(UART_NUM_0 as i32, header.as_mut_ptr().cast() as *const std::ffi::c_void, 4)};
        unsafe{uart_write_bytes(UART_NUM_0 as i32, buffer.as_ptr().cast() as *const std::ffi::c_void, 4)};

    }
}


fn uart_rx(serial_to_wifi: &RingbufHandle_t) {
    loop {
        let mut buffer = [0u8;2000];
        loop {
            unsafe{uart_read_bytes(UART_NUM_0 as i32, (&mut buffer[0..0]).as_mut_ptr().cast(), 1, 0xFFFFFFFF);}

            if buffer[0] == 0xAA {break;}
        }

        unsafe{uart_read_bytes(UART_NUM_0 as i32, (&mut buffer[1..1]).as_mut_ptr().cast(), 1, 0xFFFFFFFF);}
        unsafe{uart_read_bytes(UART_NUM_0 as i32, (&mut buffer[2..2]).as_mut_ptr().cast(), 1, 0xFFFFFFFF);}
        unsafe{uart_read_bytes(UART_NUM_0 as i32, (&mut buffer[3..3]).as_mut_ptr().cast(), 1, 0xFFFFFFFF);}

        let len = buffer[2] as u16 | (buffer[3] as u16) << 8;
        if 4 + len as usize > buffer.len() * 4 {
            continue;
        }

        for i in 0..len {
            unsafe{uart_read_bytes(UART_NUM_0 as i32, (&mut buffer[(4+i as usize)..(4+i as usize)]).as_mut_ptr().cast(), 1, 0xFFFFFFFF);}
        }

        let mut acc = 0;
        for i in 0..(4+len) {
            acc += buffer[i as usize];
        }

        if acc == 0 {
            // unsafe{xRingbufferSend(SERIAL_TO_WIFI, buffer.as_mut_ptr().cast() , len.into(), 0);}
            for i in 0..len{
                SERIAL_TO_WIFI.push(buffer[i as usize]);
            }
        } 
    }
}


fn initialize_wifi() -> Result<()> {

    unsafe{ error_check!(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, Some(on_wifi_event), std::ptr::null_mut() ), "unable to set event handler for wifi event"); }

    let ret = unsafe {esp_idf_svc::sys::nvs_flash_init()};
    if ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND {
        if let Ok(_) = error_check!(unsafe {esp_idf_svc::sys::nvs_flash_erase()}) {
            error_check!(unsafe {esp_idf_svc::sys::nvs_flash_init()},"can't init nvs");
        }
    }
    unsafe{
        error_check!(esp_wifi_init(&wifi_init_config_t{
             #[cfg(esp_idf_version_major = "4")]
            event_handler: Some(esp_idf_svc::sys::esp_event_send_internal),
            osi_funcs: &mut esp_idf_svc::sys::g_wifi_osi_funcs,
            wpa_crypto_funcs: esp_idf_svc::sys::g_wifi_default_wpa_crypto_funcs,
            static_rx_buf_num: esp_idf_svc::sys::CONFIG_ESP32_WIFI_STATIC_RX_BUFFER_NUM as _,
            dynamic_rx_buf_num: esp_idf_svc::sys::CONFIG_ESP32_WIFI_DYNAMIC_RX_BUFFER_NUM as _,
            tx_buf_type: esp_idf_svc::sys::CONFIG_ESP32_WIFI_TX_BUFFER_TYPE as _,
            static_tx_buf_num: esp_idf_svc::sys::WIFI_STATIC_TX_BUFFER_NUM as _,
            dynamic_tx_buf_num: esp_idf_svc::sys::WIFI_DYNAMIC_TX_BUFFER_NUM as _,
            csi_enable: esp_idf_svc::sys::WIFI_CSI_ENABLED as _,
            ampdu_rx_enable: esp_idf_svc::sys::WIFI_AMPDU_RX_ENABLED as _,
            ampdu_tx_enable: esp_idf_svc::sys::WIFI_AMPDU_TX_ENABLED as _,
            amsdu_tx_enable: esp_idf_svc::sys::WIFI_AMSDU_TX_ENABLED as _,
            nvs_enable: esp_idf_svc::sys::WIFI_NVS_ENABLED as _,
            nano_enable: esp_idf_svc::sys::WIFI_NANO_FORMAT_ENABLED as _,
            rx_ba_win: esp_idf_svc::sys::WIFI_DEFAULT_RX_BA_WIN as _,
            wifi_task_core_id: esp_idf_svc::sys::WIFI_TASK_CORE_ID as _,
            beacon_max_len: esp_idf_svc::sys::WIFI_SOFTAP_BEACON_MAX_LEN as _,
            mgmt_sbuf_num: esp_idf_svc::sys::WIFI_MGMT_SBUF_NUM as _,
            feature_caps: esp_idf_svc::sys::g_wifi_feature_caps,
            sta_disconnected_pm: esp_idf_svc::sys::WIFI_STA_DISCONNECTED_PM_ENABLED != 0,

            espnow_max_encrypt_num: esp_idf_svc::sys::CONFIG_ESP_WIFI_ESPNOW_MAX_ENCRYPT_NUM as i32,
            magic: esp_idf_svc::sys::WIFI_INIT_CONFIG_MAGIC as _,
            ..Default::default()
        }))?;

        error_check!(esp_wifi_set_storage(wifi_storage_t_WIFI_STORAGE_RAM))?;
        
        match *PARSED_VALUE.read().unwrap() {
              NetworkInfo::AP(config,baudrate) => {
                // Access point
                error_check!(esp_idf_svc::sys::esp_wifi_set_mode(esp_idf_svc::sys::wifi_mode_t_WIFI_MODE_AP), "Can't set wifi mode to AP");


                let mut ap_config = esp_idf_svc::sys::wifi_config_t {
                    ap: esp_idf_svc::sys::wifi_ap_config_t {
                        ssid: [0; 32],
                        password: [0; 64],
                        ssid_len: 0,
                        channel: 1,
                        authmode: esp_idf_svc::sys::wifi_auth_mode_t_WIFI_AUTH_WPA2_PSK,
                        // ssid_hidden: 0,
                        // max_connection: 4,
                        // beacon_interval: 60000,
                        ..Default::default()
                    }
                };
                set_str(&mut ap_config.ap.ssid, &config.ssid);
                match config.password {
                    Some(password) => {
                        set_str(&mut ap_config.ap.password, &password);
                    }
                    None => {
                        ap_config.ap.authmode = esp_idf_svc::sys::wifi_auth_mode_t_WIFI_AUTH_OPEN;
                    }
                }

                error_check!(esp_idf_svc::sys::esp_wifi_set_config(wifi_interface_t_WIFI_IF_AP, &mut ap_config), "can't assign configuration");

                error_check!(esp_wifi_set_protocol(wifi_mode_t_WIFI_MODE_STA,config.protocol),"Error Setting the Protocol to STA Mode");

            }

            NetworkInfo::StationMode(config, station)=> {

                error_check!(esp_netif_init(), "failed to initialize the abstraction for TCP/IP");

                let netif_sta = esp_netif_create_default_wifi_sta();
                assert_eq!(netif_sta, std::ptr::null_mut());

                let ip_info : esp_netif_ip_info_t = esp_netif_ip_info_t {
                    ip: esp_ip4_addr {addr: Station::convert(station.ip)},
                    netmask: esp_ip4_addr { addr: Station::convert(station.netmask) },
                    gw: esp_ip4_addr {addr: Station::convert(station.gateway)},
                };

                esp_netif_dhcpc_stop(netif_sta);
                esp_netif_set_ip_info(netif_sta, &ip_info);

                error_check!(esp_idf_svc::sys::esp_wifi_set_mode(esp_idf_svc::sys::wifi_mode_t_WIFI_MODE_STA), "Can't set wifi mode to STA");

                let mut sta_config = esp_idf_svc::sys::wifi_config_t {
                    sta: esp_idf_svc::sys::wifi_sta_config_t {
                        ssid: [0; 32],
                        password: [0; 64],
                        scan_method: wifi_scan_method_t_WIFI_FAST_SCAN,
                        bssid_set: false,
                        bssid: [0;6] ,
                        channel: config.channel,
                        listen_interval: 0,
                        sort_method: wifi_sort_method_t_WIFI_CONNECT_AP_BY_SIGNAL,
                        pmf_cfg: wifi_pmf_config_t {
                            capable: true,
                            required: false,
                        },
                        ..Default::default()
                    }
                };

                
                set_str(&mut sta_config.ap.ssid, &config.ssid);
                match config.password {
                    Some(password) => {
                        set_str(&mut sta_config.ap.password, &password);
                    }
                    None => {
                        sta_config.ap.authmode = esp_idf_svc::sys::wifi_auth_mode_t_WIFI_AUTH_OPEN;
                    }
                }

                error_check!(esp_wifi_set_protocol(wifi_mode_t_WIFI_MODE_AP,config.protocol),"Error Setting the Protocol to AP mode");
            },
        } 

        error_check!(esp_wifi_set_ps(wifi_ps_type_t_WIFI_PS_NONE),"Error setting power save to None");
        error_check!(esp_idf_svc::sys::esp_wifi_start(), "unable to start");
        // spawn thread for wifi_tx

    }
    Ok(())
}

unsafe extern "C" fn on_wifi_event(event_handler_arg: *mut ::core::ffi::c_void, event_base: esp_event_base_t , event_id: i32, event_data: *mut ::core::ffi::c_void) {
        use WifiEvent::*;

        let client_counter = 0; 

        if let Some(status) = WifiEvent::from_repr(event_id) {
            match status {
                ApStaConnected => {
                    log::info!("AP STA connected");
                    // set wifiRxCallback
                    // error_check!(esp_wifi_internal_reg_rxcb());
                }
                ApStaDisconnected => {
                    //we set it to nullptr
                    log::info!("AP STA disconnected");
                }
                StaStart => {
                    //do something with ICMP, IGMP , UDP, UDPLITE, TCP
                    
                }

                StaConnected => {
                    log::info!("STA connected");
                }

                StaDisconnected => {
                    error_check!(esp_wifi_connect(),"unable to connect again after connection was lost");
                }

                _ => { }
            }
        }
                

       todo!()
} 

// TODO: look into Netstack-lwip
// TODO: look into pbuf 
fn wifi_tx() {
    // create packet buffer 
    // check type of payload and send it
    //
    let mut len: usize = 0;

    loop {
        // let buffer = unsafe{ xRingbufferReceive(SERIAL_TO_WIFI, &mut len, 0xFFFFFFFF) };
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
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();


    // let mut buffer = AllocRingBuffer::with_capacity(1024*16);


    log::info!("Awesome ESP32 based wifi dongle");



    let line = read_line();
    {
        let mut parsed_value = PARSED_VALUE.write().unwrap();
        *PARSED_VALUE = parse_info(&line).unwrap().into();
    }

   
    log::info!("{:?}", PARSED_VALUE.read().unwrap());

    initialize_wifi()?;

    Ok(())
}
