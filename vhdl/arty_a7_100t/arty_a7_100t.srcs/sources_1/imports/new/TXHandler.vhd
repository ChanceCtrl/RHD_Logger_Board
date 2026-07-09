library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;


entity TXHandler is
    generic(
        BAUD_CLK_TICKS: integer := 868 -- clk/baud_rate (100 000 000 / 115 200 = 868.0555)
    );
    
    port(
        sys_clk        : in  std_logic;
        reset          : in  std_logic;
        tx_start       : in  std_logic;
        tx_data_in     : in  std_logic_vector (7 downto 0);
        tx_data_out    : out std_logic;
        tx_ready       : out std_logic;
    );
end TXHandler;


architecture TXHandler_ARCH of TXHandler is
    type tx_states_t is (IDLE, START, DATA, STOP);
    signal tx_state  : tx_states_t := IDLE;

    signal data_index        : integer range 0 to 7 := 0;
    signal data_index_reset  : std_logic := '1';
    signal stored_data       : std_logic_vector(7 downto 0) := (others=>'0');

    signal start_detected    : std_logic := '0';
    signal start_reset       : std_logic := '0';
    
    signal baud_clk          : std_logic := '0';
begin
    baud_rate_clk_generator: process(sys_clk)
        variable baud_count: integer range 0 to (BAUD_CLK_TICKS - 1) := (BAUD_CLK_TICKS - 1);
    begin
        if rising_edge(sys_clk) then
            if (reset = '1') then
                baud_clk <= '0';
                baud_count := (BAUD_CLK_TICKS - 1);
            else
                if (baud_count = 0) then
                    baud_clk <= '1';
                    baud_count := (BAUD_CLK_TICKS - 1);
                else
                    baud_clk <= '0';
                    baud_count := baud_count - 1;
                end if;
            end if;
        end if;
    end process baud_rate_clk_generator;

-- Helps the UART_tx_FSM get started if the enable signal is shorter than the baud_rate_clk_generator
    tx_start_signal: process(sys_clk) begin
        if rising_edge(sys_clk) then
            if (reset ='1') or (start_reset = '1') then
                start_detected <= '0';
            else
                if (tx_start = '1') and (start_detected = '0') then
                    start_detected <= '1';
                    stored_data <= tx_data_in;
                end if;
            end if;
        end if;
    end process tx_start_signal;


-- The data_index_counter process is a simple counter from 0 to 7 working on the baud rate frequency.
-- The data_index signal is used in UART_tx_FSM to go over stored_data and send the bits one by one.
    data_index_counter: process(sys_clk) begin
        if rising_edge(sys_clk) then
            if (reset = '1') or (data_index_reset = '1') then
                data_index <= 0;
            elsif (baud_clk = '1') then
                data_index <= data_index + 1;
            end if;
        end if;
    end process data_index_counter;


-- The UART_tx_FSM process that handles the TX logic
    UART_tx_FSM: process(sys_clk) begin
        if rising_edge(sys_clk) then
            if (reset = '1') then
                tx_state <= IDLE;
                tx_ready <= '0';
                data_index_reset <= '1';   -- keep data_index_counter on hold
                start_reset <= '1';        -- keep tx_start_detector on hold
                tx_data_out <= '1';        -- keep tx line set along the standard
            else
                if (baud_clk = '1') then   -- the FSM works on the baud rate frequency
                    case tx_state is
                        when IDLE =>
                            data_index_reset <= '1';    -- keep data_index_counter on hold
                            start_reset <= '0';         -- enable tx_start_detector to wait for starting impulses
                            tx_data_out <= '1';         -- keep tx line set along the standard
                            tx_ready <= '1';

                            if (start_detected = '1') then
                                tx_state <= START;
                            end if;

                        when START =>
                            tx_ready <= '0';
                            data_index_reset <= '0';   -- enable data_index_counter for DATA state
                            tx_data_out <= '0';        -- send '0' as a start bit

                            tx_state <= DATA;

                        when DATA =>
                            tx_data_out <= stored_data(data_index);   -- send one bit per one baud clock cycle 8 times

                            if (data_index = 7) then
                                data_index_reset <= '1';              -- disable data_index_counter when it has reached 8
                                tx_state <= STOP;
                            end if;

                        when STOP =>
                            tx_data_out <= '1';     -- send '1' as a stop bit
                            start_reset <= '1';     -- prepare tx_start_detector to be ready detecting the next impuls in IDLE

                            tx_state <= IDLE;

                        when others =>
                            tx_state <= IDLE;
                    end case;
                end if;
            end if;
        end if;
    end process UART_tx_FSM;
end TXHandler_ARCH;
