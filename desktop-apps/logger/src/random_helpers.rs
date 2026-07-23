use std::{
    fs::File,
    io::Write,
    time::{SystemTime, UNIX_EPOCH},
};

use crate::rhd_helpers::RHD2164;

pub fn t_now() -> u128 {
    return SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_micros();
}

static DATE_TIME_FORMAT: &str = "%Y-%m-%d_%H:%M:%S";

pub fn pretty_print_system_time() -> String {
    let now = SystemTime::now();
    let duration = now.duration_since(UNIX_EPOCH).expect("Time is being funny");

    let datetime = chrono::DateTime::<chrono::Local>::from(UNIX_EPOCH + duration);
    datetime.format(DATE_TIME_FORMAT).to_string()
}

pub fn log_thing(rhd: &mut RHD2164, file: &mut File) {
    match rhd.get_result() {
        Ok(data) => {
            // Log the command we got back
            writeln!(
                file,
                "{},{:?},{},{},{}",
                t_now(),
                data.0.0,
                data.0.1,
                data.1,
                data.2
            )
            .unwrap();
        }
        Err(e) => {
            println!("Got error trying to collect result: {:?}", e);
        }
    }
}
