use esp_idf_svc::sys::{xRingbufferCreate, RingbufferType_t_RINGBUF_TYPE_NOSPLIT, ESP_ERR_NVS_NO_FREE_PAGES, ESP_ERR_NVS_NEW_VERSION_FOUND, nvs_flash_erase, nvs_flash_init, uart_config_t, uart_word_length_t_UART_DATA_8_BITS, uart_parity_t_UART_PARITY_DISABLE, uart_stop_bits_t_UART_STOP_BITS_2, uart_hw_flowcontrol_t_UART_HW_FLOWCTRL_DISABLE, uart_sclk_t_UART_SCLK_APB, uart_config_t__bindgen_ty_1, uart_set_pin, UART_NUM_0, UART_PIN_NO_CHANGE, uart_port_t, gpio_num_t_GPIO_NUM_2, gpio_num_t_GPIO_NUM_1, gpio_num_t_GPIO_NUM_3, uart_driver_install, uart_param_config, RingbufHandle_t, xRingbufferReceive, uart_write_bytes, vRingbufferReturnItem, uart_read_bytes, xRingbufferSend, vTaskDelay, esp_event_handler_register, WIFI_EVENT, ESP_EVENT_ANY_ID};

macro_rules! error_check {
    ($x:expr) => {
        esp_idf_svc::sys::EspError::convert($x)
    };

    ($x:expr, $y:expr) => {
        esp_idf_svc::sys::EspError::convert($x).expect($y);
    }
}

#[derive(Debug)]
enum NetworkInfo{
    SSID(String),
    Password(String),
    Channel(i32),
    Protocol(WifiProtocol),
    AP(i32),
    StationMode(Station)
}

#[derive(Debug)]
struct Station {
   ip : (u8,u8,u8,u8),
   gateway: (u8,u8,u8,u8),
   netmask: (u8,u8,u8,u8),
   baud : u32
}

fn parse_ip(ip: &str) -> Option<(u8,u8,u8,u8)> {
    let values = ip.split(".").filter_map(|val| val.parse::<u8>().ok()).collect::<Vec<u8>>();
    if values.len() == 4 {
        return Some((values[0], values[1], values[2], values[3]));
    }  

    None
}

#[derive(Debug)]
enum WifiProtocol {
    PROTOCOL11B, PROTOCOL11G, PROTOCOL11N, PROTOCOLLR
}

fn parse_info(line: &str) -> Option<NetworkInfo> {
    use NetworkInfo::*;
    use WifiProtocol::*;

    if line.starts_with("ssid") {
        return Some(SSID(line.trim_start_matches("ssid").trim().to_string()));
    }else if line.starts_with("password") {
        return Some(Password(line.trim_start_matches("password").trim().to_string()));
    }else if line.starts_with("channel") {
        return Some(Channel(line.trim_start_matches("channel").trim().parse::<i32>().unwrap()));
    }else if line.starts_with("protocols") {
        let protocol = line.trim_start_matches("protocols").trim().to_string();
        if let Some(_) = protocol.find("b") {
           return Some(Protocol(PROTOCOL11B));
        }else if let Some(_) = protocol.find("g") {
           return Some(Protocol(PROTOCOL11G));
        }else if let Some(_) = protocol.find("n") {
           return Some(Protocol(PROTOCOL11N));
        }else if let Some(_) = protocol.find("l") {
           return Some(Protocol(PROTOCOLLR));
        }
    } else if line.starts_with("ap") {
        return Some(AP(line.trim_start_matches("ap").trim().parse::<i32>().unwrap()));
    }else if line.starts_with("sta") {
         let station_mode = line.trim_start_matches("sta").trim().split(" ").collect::<Vec<&str>>();
         let ip = parse_ip(station_mode[0]).expect("Ip parsing failed");
         let gateway = parse_ip(station_mode[1]).expect("Gateway parsing failed");
         let netmask = parse_ip(station_mode[2]).expect("Netmask parsing failed");
         let baud_rate = station_mode[3].trim().parse::<u32>().expect("Error getting Baud Rate for station mode");
         return Some(StationMode(Station { ip, gateway, netmask, baud: baud_rate }));
    }
    None
}


const BAUD_RATE: i32 = 9600;

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



fn uart_tx(wifi_to_serial: &RingbufHandle_t) {
    let mut len: usize = 0;
    loop {
        let val = unsafe{xRingbufferReceive(wifi_to_serial.clone(), &mut len, 0xFFFFFFFF) };
        let buffer = unsafe {std::ffi::CStr::from_ptr(val as *const i8)}.to_bytes(); 

        let mut header: [u8; 4] = [0xAA, 0 , (len as u8) & 0xFF, (len >> 8) as u8 & 0xFF];

        header[1] = u8::MAX - (header[0] + header[2] + header[3]);

        for i in 0..len {
            header[1] = header[1].wrapping_sub(buffer[i]);
        }

        //uart-write-bytes
        unsafe{uart_write_bytes(UART_NUM_0 as i32, header.as_mut_ptr().cast() as *const std::ffi::c_void, 4)};
        unsafe{uart_write_bytes(UART_NUM_0 as i32, buffer.as_ptr().cast() as *const std::ffi::c_void, 4)};

        unsafe{vRingbufferReturnItem(wifi_to_serial.clone(), val)};
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
            unsafe{xRingbufferSend(serial_to_wifi.clone(), buffer.as_mut_ptr().cast() , len.into(), 0);}
        } 
    }
}

fn initialize_wifi() {
    // error_check!(unsafe {esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_wifi_event, ) });
}

// TODO: look into Netstack-lwip
// TODO: look into pbuf 
fn wifi_tx() {
    // create packet buffer 
    // check type of payload and send it
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


fn main() -> Result<(), esp_idf_svc::sys::EspError>{
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    // create xRingBufferCreate 
    let wifi_to_serial = unsafe{xRingbufferCreate(1024*32, RingbufferType_t_RINGBUF_TYPE_NOSPLIT)};
    let serial_to_wifi = unsafe{xRingbufferCreate(1024*32, RingbufferType_t_RINGBUF_TYPE_NOSPLIT)};

    let ret = unsafe{nvs_flash_init()};
    if ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND {
        if let Ok(_) = error_check!(unsafe{nvs_flash_erase()}) {
            error_check!(unsafe{nvs_flash_init()},"can't init non volatile storage");
        }
    }


    log::info!("Good things are coming");
    loop {
        let line = read_line();
        let parsed_value = parse_info(&line);
        log::info!("{:?}", parsed_value);
    }

}
