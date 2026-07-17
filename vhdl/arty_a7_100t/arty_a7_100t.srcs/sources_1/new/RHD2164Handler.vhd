library ieee;
use ieee.std_logic_1164.all;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx leaf cells in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

-- https://intantech.com/files/Intan_RHD2164_datasheet.pdf
entity RHD2164Handler is
  generic(
    -- On the tin, what is the freq. of the sys_clk?
    SYS_CLK_SPEED: integer := 100000000; -- (100mHz)

    -- All of the following values are from this datasheet, on page 11
    -- https://intantech.com/files/Intan_RHD2164_datasheet.pdf

    -- What divider do you want on the sys_clk for the spi_clk
    SPI_CLK_TICKS: integer := 100; -- 1mHz w/ sys_clk @ 100mHz
    -- The max spi freq. the RHD2164 can do is 24mHz

    -- sys_clk Ticks from CS LOW to SCLK HIGH/start of transaction
    T_CS1_TICKS : integer := 3; -- 30ns w/ sys_clk @ 100mHz
    -- Minimum of 20.8 ns

    -- sys_clk Ticks from SCLK LOW to CS HIGH/end of transaction
    T_CS2_TICKS : integer := 3; -- 30ns w/ sys_clk @ 100mHz
    -- Minimum of 20.8 ns

    -- sys_clk Ticks that CS must remain off/HIGH after t_cs2
    T_CSOFF_TICKS : integer := 20 -- 200ns w/ sys_clk @ 100mHz
    -- Minimum of 154 ns
  );

  Port (
    -- System/overall things
    sys_clk   : in  std_logic;
    reset     : in  std_logic;

    -- SPI wire fellas
    spi_line_miso  : in  std_logic;
    spi_line_mosi  : out std_logic;
    spi_line_sclk  : out std_logic;
    spi_line_cs    : out std_logic;

    -- Set info
    send_data_buffer  : in  std_logic_vector (15 downto 0);
    send_command      : in  std_logic;
    
    -- Recived info
    a_channel_buffer  : out std_logic_vector (15 downto 0);
    b_channel_buffer  : out std_logic_vector (15 downto 0);

    -- Feedback signals
    rhd_is_ready    : out std_logic;
    new_data        : out std_logic
  );
end RHD2164Handler;

architecture RHD2164Handler_arch of RHD2164Handler is
  -- Signals for generating the SPI clock
  signal spi_clk_count   : natural := 0;
  signal spi_clk_enable  : std_logic := '0';
  signal spi_pulse_count : integer range 0 to 31 := 0;

  -- State machine stuff for the overall SPI managment
  type rhd_states_t is (IDLE, START, DATA, STOP);
  signal rhd_state  : rhd_states_t := IDLE;

  -- Signals for the buffers
  signal a_channel_done : std_logic := '0';
  signal b_channel_done : std_logic := '0';

  signal send_buff  : std_logic_vector (15 downto 0);
  signal a_buff     : std_logic_vector (15 downto 0);
  signal b_buff     : std_logic_vector (15 downto 0);

  -- Signals for the actual wires
  signal spi_sclk : std_logic := '0';
  signal spi_cs   : std_logic := '1';
  signal spi_mosi : std_logic := '1';
  signal spi_miso : std_logic := '0';

begin
  spi_line_sclk <= spi_sclk;
  spi_line_cs   <= spi_cs;
  spi_line_mosi <= spi_mosi;
  spi_miso      <= spi_line_miso;
  
  -- Handles the MOSI transmitting logic
  process(sys_clk)
    variable bit_count : integer range 0 to 14 := 14;
    variable spi_sclk_prev : std_logic := '0';
  begin
    if rising_edge(sys_clk) then
      case rhd_state is
        when START =>
          spi_mosi <= send_buff(15);
          bit_count := 14;
          
        when DATA =>
          if spi_sclk_prev = '1' and spi_sclk = '0' then
            spi_mosi <= send_buff(bit_count);

            if bit_count = 0 then
              bit_count := 14;
            else
              bit_count := bit_count - 1;
            end if;
          end if;
          
        when others =>
          spi_mosi <= '0';
          bit_count := 14;
      end case;
      
      spi_sclk_prev := spi_sclk;
    end if;
  end process;

  -- Grabs the falling edge bytes for the A Buffer
  process(spi_sclk) 
    variable bit_count : integer range 0 to 15 := 15;
  begin
    if falling_edge(spi_sclk) then
      if rhd_state = DATA then
        if bit_count = 0 then
          a_channel_done <= '1';
          bit_count := 15;
        else
          a_channel_done <= '0';
          a_buff(bit_count) <= spi_miso;
          bit_count := bit_count - 1;
        end if;
      end if;
    end if;
  end process;

  -- Grabs the rising edge bytes for the B Buffer
  process(sys_clk)
    -- Not 0 to 15 because the last bit isn't actually on a SCLK pulse, instead its mapped 
    -- to when CS goes high again... Because this chip hates people is my only guess why.
    variable bit_count : integer range 1 to 15 := 15; 
    
    variable spi_sclk_prev : std_logic := '0';
  begin
    if rising_edge(sys_clK) then
      case rhd_state is 
        when DATA =>
          b_channel_done <= '0';
          
          if spi_sclk_prev = '0' and spi_sclk = '1' then
            if (bit_count = 1) then
              bit_count := 15;
            else
              b_buff(bit_count) <= spi_miso;
              bit_count := bit_count - 1;
            end if;
          end if;
          
        when STOP =>
          b_buff(0) <= spi_miso;
          b_channel_done <= '1';
          
        when others =>
          bit_count := 15;
      end case;
      
      spi_sclk_prev := spi_sclk;
    end if;
  end process;

  -- A simple process that tells other people when we are done collecting data
  process(sys_clk) 
    variable has_new_data  : std_logic := '0';
    variable new_data_prev : std_logic := '0';
  begin
    if rising_edge(sys_clk) then
        has_new_data := a_channel_done and b_channel_done;
        
        new_data <= '0';
        
        if has_new_data = '1' and new_data_prev = '0' then
            a_channel_buffer <= a_buff;
            b_channel_buffer <= b_buff;
            new_data <= '1';
        end if;
        
        new_data_prev := has_new_data;
    end if;
  end process;

  -- This process manages the interaction with the RHD2164
  process(sys_clk, reset) 
    -- Variables for the CS related timing things
    variable t_cs1_count    : integer range 0 to T_CS1_TICKS    := 0;
    variable t_cs2_count    : integer range 0 to T_CS2_TICKS    := 0;
    variable t_csoff_count  : integer range 0 to T_CSOFF_TICKS  := 0;

    -- SCLK counter
  begin
    if reset = '1' then
      rhd_state <= IDLE;
      t_cs1_count     := 0;
      t_cs2_count     := 0;
      t_csoff_count   := 0;
            
      spi_clk_enable <= '0';

    elsif rising_edge(sys_clk) then
      case rhd_state is
        when IDLE =>
          spi_cs <= '1';

          rhd_is_ready <= '1';

          if send_command = '1' then
            rhd_is_ready <= '0';

            send_buff <= send_data_buffer;
            rhd_state <= START;
          end if;

        when START =>
          spi_cs <= '0';
          
          if t_cs1_count = T_CS1_TICKS then
            t_cs1_count := 0;
            
            spi_clk_count <= 0;
            spi_sclk <= '0';
            spi_pulse_count <= 0;
            
            rhd_state <= DATA;
          else
            t_cs1_count := t_cs1_count + 1;
          end if;

        when DATA =>
          -- Generates the SPI_SCLK signal
          if spi_clk_count = (SPI_CLK_TICKS/2) then
            spi_clk_count <= 0;
            spi_sclk <= not spi_sclk;
          
            if spi_pulse_count = 31 then
              spi_pulse_count <= 0;
              rhd_state <= STOP;
            else
              spi_pulse_count <= spi_pulse_count + 1;
            end if;
          else
            spi_clk_count <= spi_clk_count + 1;
          end if;

        when STOP =>
          spi_clk_count <= 0;
          spi_sclk <= '0';
          
          if t_cs2_count = T_CS2_TICKS then
            spi_cs <= '1';

            if t_csoff_count = T_CSOFF_TICKS then
              rhd_state <= IDLE;

              t_cs2_count := 0;
              t_csoff_count := 0;
            else
              t_csoff_count := t_csoff_count + 1;
            end if;
          else
            t_cs2_count := t_cs2_count + 1;
          end if;
      end case;
    end if;
  end process;
end RHD2164Handler_arch;
