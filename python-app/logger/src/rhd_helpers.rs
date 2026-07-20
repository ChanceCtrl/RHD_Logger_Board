mod commands;

use commands::{Commands, ConfigRegisters, ReadableRegister};

use serial2::SerialPort;

pub struct RHD2164 {
    rhd_uart: SerialPort,

    uart_data_buf: [u8; 4],
    command_buf: Vec<(Commands, u8)>,

    rhd_a_data: u16,
    rhd_b_data: u16,
}

impl RHD2164 {
    pub fn init() -> Self {
        RHD2164 {
            rhd_uart: SerialPort::open("/dev/ttyUSB0", 6250000).unwrap(),
            uart_data_buf: [0_u8; 4],
            command_buf: Vec::new(),
            rhd_a_data: 0_u16,
            rhd_b_data: 0_u16,
        }
    }

    pub fn get_conversion(&mut self, channel: u8, reset_highpass: bool) -> Result<(), ()> {
        if channel > 63 {
            println!("Too high of a channel");
            return Err(());
        }

        let mut command: u16 = 0b0000_0000_0000_0000;
        command |= (channel as u16) << 8; // 00CC_CCCC_0000_0000
        command |= reset_highpass as u16; // 00CC_CCCC_0000_000H

        self.send_bytes(command);

        self.command_buf.push((Commands::Convert, channel));

        return Ok(());
    }

    pub fn calibrate(&mut self) -> Result<(), ()> {
        let command: u16 = 0b0101_0101_0000_0000;

        self.send_bytes(command);

        self.command_buf.push((Commands::Calibrate, 0));

        return Ok(());
    }

    pub fn clear_calibration(&mut self) -> Result<(), ()> {
        let command: u16 = 0b0110_1010_0000_0000;

        self.send_bytes(command);

        self.command_buf.push((Commands::ClearCalibration, 0));

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

        self.send_bytes(command);

        self.command_buf
            .push((Commands::WriteRegister, register as u8));

        return Ok(());
    }

    pub fn read_register<R: ReadableRegister>(&mut self, register: R) -> Result<(), ()> {
        if register.address() > 63 {
            println!("Too high of a register");
            return Err(());
        }

        let mut command: u16 = 0b1100_0000_0000_0000;
        command |= (register.address() as u16) << 8; //11RR_RRRR_0000_0000

        self.send_bytes(command);

        self.command_buf
            .push((Commands::ReadRegister, register.address()));

        return Ok(());
    }

    pub fn get_result(self) -> Result<((Commands, u8), u16, u16), ()> {
        if self.command_buf.len() > 2 {
            return Ok((
                self.command_buf[self.command_buf.len() - 2],
                self.rhd_a_data,
                self.rhd_b_data,
            ));
        } else {
            return Err(());
        }
    }

    fn send_bytes(&mut self, data: u16) {
        if self
            .rhd_uart
            .write_all(data.to_le_bytes().as_ref())
            .is_err()
        {
            println!("Got an error writing to the serial port.");
        }

        if self.rhd_uart.read_exact(&mut self.uart_data_buf).is_err() {
            println!("Failed to read the reply on the serial port.")
        }
    }
}
