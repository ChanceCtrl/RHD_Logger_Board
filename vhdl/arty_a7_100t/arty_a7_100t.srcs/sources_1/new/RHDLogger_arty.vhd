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
        CLK100MHZ       : in  std_logic;
        uart_txd_in     : in  std_logic;
        uart_rxd_out    : out std_logic;
        btn             : in  std_logic_vector (1 downto 0)
    );
end rhd_logger_arty;

architecture rhd_logger_arty_arch of rhd_logger_arty is
    signal UARTInput : std_logic_vector (7 downto 0);
    signal convst, drdy, eoc, eos : std_logic;
    signal spi_clk : std_logic := '0';
begin
    UART_UUT : entity work.UARTHandler
        generic map(
            BAUD_CLK_TICKS => 16
        )
        port map (
            clk => CLK100MHZ,
            reset => btn(0),
            tx_start => btn(1),
            data_in => UARTInput,
            data_out => UARTInput,
            rx => uart_txd_in,
            tx => uart_rxd_out
        );

    RHD_UUT : entity work.RHD2164Handler
        generic map(

        )
        port map (

        );


end rhd_logger_arty_arch;
