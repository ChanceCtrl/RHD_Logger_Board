library ieee;
use ieee.std_logic_1164.all;

entity PacketMaker9000 is
  Port (
    -- Generic system inputs
    sys_clk : in  std_logic;
    reset   : in  std_logic;

    -- UART related things
    uart_data_out     : out std_logic_vector (7 downto 0);
    
    uart_tx_ready     : in  std_logic;
    uart_tx_enable    : out std_logic;

    -- RHD related things
    a_channel_buffer  : in  std_logic_vector (15 downto 0);
    b_channel_buffer  : in  std_logic_vector (15 downto 0);

    rhd_has_data      : in  std_logic
  );
end PacketMaker9000;

architecture PacketMaker9000_arch of PacketMaker9000 is
  type packet_states_t is (IDLE, SEND, HALT);
  signal packet_state  : packet_states_t := IDLE;

  signal a_channel_copy : std_logic_vector (15 downto 0);
  signal b_channel_copy : std_logic_vector (15 downto 0);
begin

  packet_maker_FSM: process(sys_clk, reset)
    variable packet_count : integer range 0 to 3 := 0;
  begin
    if (reset = '1') then
      packet_count := 0;
      packet_state <= IDLE;
      uart_tx_enable <= '0';
      uart_data_out <= (others => '0');

    elsif rising_edge(sys_clk) then
      case packet_state is
        when IDLE =>
          uart_tx_enable <= '0';

          if rhd_has_data = '1' then
            a_channel_copy <= a_channel_buffer;
            b_channel_copy <= b_channel_buffer;

            packet_state <= SEND;
          end if;

        when SEND =>
          uart_tx_enable <= '0';

          if uart_tx_ready = '1' then
            case packet_count is
              when 0 =>
                uart_data_out <= a_channel_copy (7 downto 0);
                uart_tx_enable <= '1';
                packet_state <= HALT;

              when 1 => 
                uart_data_out <= a_channel_copy (15 downto 8);
                uart_tx_enable <= '1';
                packet_state <= HALT;

              when 2 => 
                uart_data_out <= b_channel_copy (7 downto 0);
                uart_tx_enable <= '1';
                packet_state <= HALT;

              when 3 => 
                uart_data_out <= b_channel_copy (15 downto 8);
                uart_tx_enable <= '1';
                packet_state <= HALT;
            end case;
          end if;

        when HALT => 
          uart_tx_enable <= '0';

          if packet_count = 3 then
            packet_count := 0;
            packet_state <= IDLE;
          elsif uart_tx_ready = '0' then
            packet_count := packet_count + 1;
            packet_state <= SEND;
          end if;
      end case;
    end if;
  end process packet_maker_FSM;
end PacketMaker9000_arch;
