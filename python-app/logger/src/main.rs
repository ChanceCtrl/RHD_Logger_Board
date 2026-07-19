mod rhd_helpers;

use serial2::SerialPort;

fn main() {
    let mut rhd_data_buffer = [0; 4];

    let rhd = SerialPort::open("/dev/ttyUSB0", 6250000).unwrap();
    let ard = SerialPort::open("/dev/ttyUSB1", 115200).unwrap();
}
