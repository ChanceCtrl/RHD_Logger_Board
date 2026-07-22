----------------------------------------------------------------------------------
-- Design: CScope_BASYS3
-- Engineer: Chance
-- Description: 
----------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;


entity rhd_logger_arty is
    Port (
        CLK100MHZ       : in    std_logic;
        uart_txd_in     : in    std_logic;
        uart_rxd_out    : out   std_logic;
        btn             : in    std_logic_vector (1 downto 0);
        jb              : inout std_logic_vector (3 downto 0);
        jc              : inout std_logic_vector (3 downto 0)
    );
end rhd_logger_arty;

architecture rhd_logger_arty_arch of rhd_logger_arty is
    -- UART Signals
    signal UARTInput    : std_logic_vector (7 downto 0) := (others => '0');
    signal UARTOutut    : std_logic_vector (7 downto 0) := (others => '0');
    signal UARTEnable   : std_logic := '0';
    signal UARTReady    : std_logic := '0';
    signal UARTRecviced : std_logic := '0';

    -- RHD Signals
    signal RHDBufferD   : std_logic_vector (15 downto 0) := (others => '0');
    signal RHDBufferA   : std_logic_vector (15 downto 0) := (others => '0');
    signal RHDBufferB   : std_logic_vector (15 downto 0) := (others => '0');
    signal RHDHasData   : std_logic := '0';
    signal RHDCommand   : std_logic := '0';
begin   
    jc(3) <= RHDCommand;
    jc(2) <= RHDHasData;
    jc(1) <= UARTReady;
    jc(0) <= UARTEnable;
    
    UART_UUT : entity work.UARTHandler
        generic map(
            BAUD_CLK_TICKS => 16
        )
        port map (
            clk => CLK100MHZ,
            reset => btn(0),
            tx_start => UARTEnable,
            tx_ready => UARTReady,
            rx_done => UARTRecviced,
            data_in => UARTOutut,
            data_out => UARTInput,
            rx => uart_txd_in,
            tx => uart_rxd_out
        );

    RHD_UUT : entity work.RHD2164Handler
        port map (
            sys_clk => CLK100MHZ,
            reset => btn(0),
            spi_line_miso => jb(0),
            spi_line_mosi => jb(1),
            spi_line_sclk => jb(2),
            spi_line_cs => jb(3),
            send_data_buffer => RHDBufferD,
            send_command => RHDCommand,
            a_channel_buffer => RHDBufferA,
            b_channel_buffer => RHDBufferB,
            new_data => RHDHasData
        );

    PM9_UUT : entity work.PacketMaker9000
        port map (
            sys_clk => CLK100MHZ,
            reset => btn(0),
            uart_data_out => UARTOutut,
            uart_tx_ready => UARTReady,
            uart_tx_enable => UARTEnable,
            a_channel_buffer => RHDBufferA,
            b_channel_buffer => RHDBufferB,
            rhd_has_data => RHDHasData
        );

    CMH_UUT : entity work.CommandHandler
        port map (
            sys_clk => CLK100MHZ,
            reset => btn(0),
            uart_buffer => UARTInput,
            uart_rx_done => UARTRecviced,

            rhd_command_buffer => RHDBufferD,
            command_buffer_full => RHDCommand
        );

end rhd_logger_arty_arch;
