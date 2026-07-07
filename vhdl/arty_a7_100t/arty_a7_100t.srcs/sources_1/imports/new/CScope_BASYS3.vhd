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
    signal count : natural range 0 to 1000 := 0;

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
                
--    -- for a freq of 100 kHz, with a clk freq of 100 MHz
--    process(clk) begin
--        if clk'event and clk='1' then
--            count <= count + 1;
--            convst <= '0';
            
--            if count = 999 then
--                count <= 0;
--                convst <= '1';
--            end if;
--        end if;
--    end process;


end rhd_logger_arty_arch;
