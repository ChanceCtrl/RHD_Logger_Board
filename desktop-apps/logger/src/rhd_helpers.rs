pub mod commands;

use std::{collections::VecDeque, error::Error};

use commands::{Commands, ConfigRegisters, ReadableRegister};

use serial2::SerialPort;

pub struct RHD2164 {
    rhd_uart: SerialPort,

    uart_data_buf: [u8; 4],
    command_buf: VecDeque<(Commands, u8)>,

    rhd_a_data: u16,
    rhd_b_data: u16,
}

impl RHD2164 {
    pub fn init() -> Result<Self, Box<dyn Error>> {
        let uart = SerialPort::open("/dev/ttyUSB1", 115200).unwrap();

        return Ok(RHD2164 {
            rhd_uart: uart,
            uart_data_buf: [0_u8; 4],
            command_buf: VecDeque::new(),
            rhd_a_data: 0_u16,
            rhd_b_data: 0_u16,
        });
    }

    pub fn get_conversion(&mut self, channel: u8, reset_highpass: bool) -> Result<(), ()> {
        if channel > 63 {
            println!("Too high of a channel");
            return Err(());
        }

        let mut command: u16 = 0b0000_0000_0000_0000;
        command |= (channel as u16) << 8; // 00CC_CCCC_0000_0000
        command |= reset_highpass as u16; // 00CC_CCCC_0000_000H

        self.send_command(Commands::Convert, channel, command);

        return Ok(());
    }

    pub fn calibrate(&mut self) -> Result<(), ()> {
        let command: u16 = 0b0101_0101_0000_0000;

        self.send_command(Commands::Calibrate, 0, command);

        return Ok(());
    }

    pub fn clear_calibration(&mut self) -> Result<(), ()> {
        let command: u16 = 0b0110_1010_0000_0000;

        self.send_command(Commands::ClearCalibration, 0, command);

        return Ok(());
    }

    pub fn write_register(&mut self, register: ConfigRegisters, data: u8) -> Result<(), ()> {
        if register as u8 > 63 {
            println!("Too high of a register");
            return Err(());
        }

        let mut command: u16 = 0b1000_0000_0000_0000;
        command |= (register as u16) << 8; // 10RR_RRRR_0000_0000
        command |= data as u16; // 10RR_RRRR_DDDD_DDDD

        self.send_command(Commands::WriteRegister, register as u8, command);

        return Ok(());
    }

    pub fn read_register<R: ReadableRegister>(&mut self, register: R) -> Result<(), ()> {
        if register.address() > 63 {
            println!("Too high of a register");
            return Err(());
        }

        let mut command: u16 = 0b1100_0000_0000_0000;
        command |= (register.address() as u16) << 8; //11RR_RRRR_0000_0000

        self.send_command(Commands::ReadRegister, register.address(), command);

        return Ok(());
    }

    pub fn get_result(&mut self) -> Result<((Commands, u8), u16, u16), ()> {
        if self.command_buf.len() == 3 {
            self.rhd_a_data = u16::from_le_bytes([self.uart_data_buf[0], self.uart_data_buf[1]]);
            self.rhd_b_data = u16::from_le_bytes([self.uart_data_buf[2], self.uart_data_buf[3]]);

            return Ok((self.command_buf[0], self.rhd_a_data, self.rhd_b_data));
        } else {
            return Err(());
        }
    }

    fn send_command(&mut self, command: Commands, input: u8, payload: u16) {
        self.rhd_uart
            .write_all(payload.to_be_bytes().as_ref())
            .unwrap();

        self.rhd_uart.flush().unwrap();

        self.rhd_uart.read_exact(&mut self.uart_data_buf).unwrap();

        if self.command_buf.len() == 3 {
            self.command_buf.pop_front();
        }

        self.command_buf.push_back((command, input));
    }
}
