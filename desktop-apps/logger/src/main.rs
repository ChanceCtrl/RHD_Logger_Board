mod random_helpers;
mod rhd_helpers;

use random_helpers::{log_thing, pretty_print_system_time, t_now};

use rhd_helpers::RHD2164;

use std::error::Error;
use std::fs::OpenOptions;
use std::io::Write;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

fn main() -> Result<(), Box<dyn Error>> {
    // Setup our escape
    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();

    ctrlc::set_handler(move || {
        println!("\nStopping before next loop...");
        r.store(false, Ordering::SeqCst);
    })
    .expect("Error setting Ctrl-C handler");

    // Open the file buffer
    let mut data_file = OpenOptions::new()
        .create(true)
        .append(true)
        .open(format!("{}_data_rhd2164.CSV", pretty_print_system_time()))?;

    let mut command_file = OpenOptions::new().create(true).append(true).open(format!(
        "{}_command_rhd2164.CSV",
        pretty_print_system_time()
    ))?;

    // Write the headers
    write!(data_file, "time,")?;
    for i in 0..64 {
        write!(data_file, "channel_{},", i)?;
    }
    writeln!(data_file, "time_end")?;

    writeln!(command_file, "time,command,request,rhd_a,rhd_b")?;

    let mut write_buf = [0_u16; 64];

    // Connect to the RHD uart bridge and configure it
    let mut rhd = RHD2164::init()?;

    while running.load(Ordering::SeqCst) {
        write!(data_file, "{},", t_now())?;

        for i in 0..32 {
            let _ = rhd.get_conversion(i, false);

            match rhd.get_result() {
                Ok(data) => {
                    // If its conversion data, save it to the write buffer
                    if data.0.0 == rhd_helpers::commands::Commands::Convert {
                        write_buf[data.0.1 as usize] = data.1;
                        write_buf[(data.0.1 + 32) as usize] = data.2;
                    }

                    // Log the command we got back
                    writeln!(
                        command_file,
                        "{},{:?},{},{},{}",
                        t_now(),
                        data.0.0,
                        data.0.1,
                        data.1,
                        data.2
                    )?;
                }
                Err(e) => {
                    println!("Got error trying to collect result: {:?}", e);
                }
            }

            std::thread::sleep(std::time::Duration::from_millis(1));
        }

        // Write out the data buffer
        for val in write_buf {
            write!(data_file, "{val},")?;
        }

        // End the line
        writeln!(data_file, "{}", t_now())?;
    }

    return Ok(());
}
