use console::{Term,style};
use dialoguer::{theme::ColorfulTheme, Select,Input};
use std::net::Ipv4Addr;


use anyhow::Result;
use serialport::{available_ports,SerialPortInfo};

fn get_active_ports() -> Vec<SerialPortInfo>{
    available_ports().expect("No ports found")
}

fn interactive_ip_address(prompt: &str, default: &str ) -> String {
     Input::<String>::with_theme(&ColorfulTheme::default())
        .with_prompt(prompt)
        .default(default.into())
        .validate_with(|input: &String| -> Result<(),&str> {
            match input.parse::<Ipv4Addr>() {
                Ok(_) => return Ok(()),
                Err(_) => return Err("Can't validate Ip Address, please type it again")
            }
        })
        .interact_text()
        .unwrap()
}


fn interactive() {
    //get active ports
    let ports = get_active_ports();

    let selected_port_num = Select::with_theme(&ColorfulTheme::default())
        .with_prompt("Select Your Port")
        .items(&ports.iter().map(|port| port.port_name.clone()).collect::<Vec<String>>())
        .interact()
        .unwrap();

    let selected_port = &ports[selected_port_num];


    let baud_rate = Input::<u32>::with_theme(&ColorfulTheme::default())
        .with_prompt("Enter your Baud rate")
        .default(460800)
        .interact_text()
        .unwrap();


    let interface = Input::<String>::with_theme(&ColorfulTheme::default())
        .with_prompt("Set Interface Name")
        .default("wlo-esp32".into())
        .interact_text()
        .unwrap();

    // now connect to STA as a TUN interface
    
    let adapter_mode = Select::with_theme(&ColorfulTheme::default())
        .with_prompt("Station Mode (connect to the Internet) / Access Point Mode (Share an existing connection)")
        .item("STA")
        .item("AP")
        .interact()
        .unwrap();

    println!("{}",adapter_mode);
    

    let ssid = Input::<String>::with_theme(&ColorfulTheme::default())
        .with_prompt("Set Network Name")
        .default("voidash".into())
        .interact_text()
        .unwrap();



    let password = Input::<String>::with_theme(&ColorfulTheme::default())
        .with_prompt("Set Network Name (default: Unprotected network)")
        .default("".into())
        .interact_text()
        .unwrap();

    match adapter_mode {
        0 => {
            // STA mode
            let local_address = interactive_ip_address("Local Static Address, (Dynamic supported in future release)", "192.168.1.14");
            let gateway = interactive_ip_address("Set gateway address (Enter to leave it blank)", "255.255.255.255");
        }
        1 => {
            // AP mode
        }
        _ => { /* not possible */ }

    }

} 

fn main() -> Result<()> {
    interactive();
    println!("Hello from the underside , {}", style("hello").bold().cyan());

    Ok(())
}
