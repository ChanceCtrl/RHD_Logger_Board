use std::time::{SystemTime, UNIX_EPOCH};

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
