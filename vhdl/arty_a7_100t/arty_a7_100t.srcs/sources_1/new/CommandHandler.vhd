library ieee;
use ieee.std_logic_1164.all;

entity CommandHandler is
  Port (
    -- Basic system things
    sys_clk : in std_logic;
    reset   : in std_logic;

    -- UART Related inputs
    uart_buffer         : in  std_logic_vector (7  downto 0);
    uart_rx_done        : in  std_logic;

    -- RHD Outputs
    rhd_command_buffer  : out std_logic_vector (15 downto 0);
    command_buffer_full : out std_logic
  );
end CommandHandler;

architecture CommandHandler_arch of CommandHandler is
  type packet_states_t is (IDLE, RECV, HALT);
  signal packet_state  : packet_states_t := IDLE;

  signal out_buffer   : std_logic_vector (15 downto 0);
begin

  packet_maker_FSM: process(sys_clk, reset)
    variable packet_count : integer range 0 to 1 := 0;
  begin
    if (reset = '1') then
      packet_count    := 0;
      packet_state    <= IDLE;
      command_buffer_full <= '0';
      rhd_command_buffer  <= (others => '0');

    elsif rising_edge(sys_clk) then
      case packet_state is
        when IDLE =>
          command_buffer_full <= '0';
          packet_state <= RECV;

        when RECV =>
          if uart_rx_done = '1' then 
            case packet_count is
              when 0 =>
                out_buffer (15 downto 8) <= uart_buffer;
                packet_state <= HALT;

              when 1 =>
                out_buffer (7 downto 0) <= uart_buffer;
                rhd_command_buffer <= out_buffer;
                command_buffer_full <= '1';
                packet_state <= HALT;
            end case;
          end if;

        when HALT =>
          if packet_count = 1 then
            packet_count := 0;
            packet_state <= IDLE;
          elsif uart_rx_done = '0' then
            packet_count := packet_count + 1;
            packet_state <= RECV;
          end if;
      end case;
    end if;
  end process;
end CommandHandler_arch;
