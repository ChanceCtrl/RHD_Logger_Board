library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;


entity RXHandler is
    generic(
        BAUD_CLK_TICKS: integer := 868 -- clk/baud_rate (100 000 000 / 115 200 = 868.0555)
    );
    
    port(
        sys_clk        : in  std_logic;
        reset          : in  std_logic;
        rx_done        : out std_logic;
        rx_data_in     : in  std_logic;
        rx_data_out    : out std_logic_vector (7 downto 0)
    );
end RXHandler;


architecture RXHandler_ARCH of RXHandler is
    type rx_states_t is (IDLE, START, DATA, STOP);
    signal rx_state: rx_states_t := IDLE;
    
    signal rx_stored_data     : std_logic_vector(7 downto 0) := (others => '0');
begin
-- The UART_rx_FSM process represents a Finite State Machine which has
-- four states (IDLE, START, DATA, STOP). See inline comments for more details.
    UART_rx_FSM: process(sys_clk)
        variable bit_count          : integer range 0 to 7  := 0;
        
        constant HALF_BIT_COUNT     : integer := (BAUD_CLK_TICKS/2) - 1;
        variable bit_timer          : integer range 0 to BAUD_CLK_TICKS - 1 := 0;
    begin
        if rising_edge(sys_clk) then
            if (reset = '1') then
                rx_state <= IDLE;
                rx_stored_data <= (others => '0');
                rx_data_out <= (others => '0');
                rx_done <= '0';
                bit_timer := 0;
                bit_count := 0;

            else
                case rx_state is
                    when IDLE =>
                        rx_stored_data <= (others => '0');    -- clean the received data register
                        bit_count := 0;
                        bit_timer := 0;
                        rx_done <= '0';

                        if (rx_data_in = '0') then             -- if the start bit received
                            rx_state <= START;                 -- transit to the START state
                        end if;

                    when START =>
                        if (rx_data_in = '0') then                  -- verify that the start bit is preset
                            if (bit_timer = HALF_BIT_COUNT) then    -- wait a half of the baud rate cycle (it puts the capture point at the middle of duration of the receiving bit)
                                rx_state <= DATA;
                                bit_timer := 0;
                            else
                                bit_timer := bit_timer + 1;
                            end if;
                        else
                            bit_timer := 0;
                            rx_state <= IDLE;                  -- the start bit is not preset (false alarm)
                        end if;

                    when DATA =>
                        if (bit_timer = BAUD_CLK_TICKS-1) then            -- wait for one baud rate cycle
                            rx_stored_data(bit_count) <= rx_data_in;    -- fill in the receiving register one received bit.
                            bit_timer := 0;
                            
                            if (bit_count = 7) then     -- when all 8 bit received, go to the STOP state
                                rx_state <= STOP;
                            else                        -- Otherwise, step the bit count and go again
                                bit_count := bit_count + 1;
                            end if;
                        else
                            bit_timer := bit_timer + 1;
                        end if;

                    when STOP =>
                        if (bit_timer = BAUD_CLK_TICKS-1) then   -- wait for "one" baud rate cycle
                            rx_data_out <= rx_stored_data;          -- transer the received data to the outside world
                            rx_done <= '1';
                            bit_timer := 0;
                            rx_state <= IDLE;
                        else
                            bit_timer := bit_timer + 1;
                        end if;

                    when others =>
                        bit_timer := 0;
                        rx_state <= IDLE;
                end case;
            end if;
        end if;
    end process UART_rx_FSM;
end RXHandler_ARCH;
