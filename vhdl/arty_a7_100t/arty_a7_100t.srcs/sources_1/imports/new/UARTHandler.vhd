library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity UARTHandler is
    generic(
        BAUD_CLK_TICKS: integer := 868 -- clk/baud_rate (100 000 000 / 115 200 = 868.0555)
    );
    
    port(
        clk            : in  std_logic;
        reset          : in  std_logic;
        
        tx_start       : in  std_logic;
        tx_ready       : out std_logic;
        rx_done        : out std_logic;

        data_in        : in  std_logic_vector (7 downto 0);
        data_out       : out std_logic_vector (7 downto 0);

        rx             : in  std_logic;
        tx             : out std_logic
    );
end UARTHandler;


architecture UARTHandler_ARCH of UARTHandler is    
begin
    transmitter: entity work.TXHandler
    generic map(
        BAUD_CLK_TICKS => BAUD_CLK_TICKS
    )
    port map (
        sys_clk => clk,
        reset => reset,
        tx_start => tx_start,
        tx_data_in => data_in,
        tx_data_out => tx,
        tx_ready => tx_ready
    );

    receiver: entity work.RXHandler
    generic map(
        BAUD_CLK_TICKS => BAUD_CLK_TICKS
    )
    port map (
        sys_clk => clk,
        reset => reset,
        rx_done => rx_done,
        rx_data_in => rx,
        rx_data_out => data_out
    );
    
end UARTHandler_ARCH;
