mod commands;

use commands::{Commands, ConfigRegisters, ReadOnlyRegisters};

#[derive(Debug, Clone, Copy)]
pub struct RHD2164 {
    uart_data_buf: [u8; 4],
    command_buf: [(Commands, u8); 2],

    rhd_a_data: u16,
    rhd_b_data: u16,
}

impl RHD2164 {
    pub fn init() -> Self {
        RHD2164 {
            uart_data_buf: [0_u8; 4],
            command_buf: [(Commands::Convert, 0_u8); 2],
            rhd_a_data: 0_u16,
            rhd_b_data: 0_u16,
        }
    }

    pub fn get_conversion(channel: u8) {}

    pub fn calibrate() {}

    pub fn clear_calibration() {}

    pub fn write_register(register: u8) {}

    pub fn read_register(register: u8) {}

    pub fn get_result() -> ((Commands, u8), u16, u16) {}
}
