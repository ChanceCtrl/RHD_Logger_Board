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
    
    -- What divider do you want on the sys_clk for the ADC sample rate
    ADC_CONV_TICKS: integer := 200000; -- 500hz w/ sys_clk @ 100mHz
    -- The max ADC sample rate is 1.05 MS/s per 32-channel module or 30 kS/s per channel 
    -- for all 64 amplifier channels plus 3 auxiliary channels, which works out to 950ns

    -- sys_clk Ticks from CS LOW to SCLK HIGH/start of transaction
    T_CS1_TICKS : integer := 3; -- 30ns w/ sys_clk @ 100mHz
    -- Minimum of 20.8 ns

    -- sys_clk Ticks from SCLK LOW to CS HIGH/end of transaction
    T_CS2_TICKS : integer := 3; -- 30ns w/ sys_clk @ 100mHz
    -- Minimum of 20.8 ns

    -- sys_clk Ticks that CS must remain off/HIGH after t_cs2
    T_CSOFF_TICKS : integer := 20; -- 200ns w/ sys_clk @ 100mHz
    -- Minimum of 154 ns
  );

  Port (
    -- System/overall things
    sys_clk   : in  std_logic;
    reset     : in  std_logic;

    -- SPI wire fellas
    spi_miso  : in  std_logic;
    spi_mosi  : out std_logic;
    spi_sclk  : out std_logic;
    spi_cs    : out std_logic;

    -- Set info
    data_to_send  : out std_logic_vector (15 downto 0);
    
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
  signal spi_clk_count  : natural := 0;
  signal spi_clk        : std_logic := '0';
  signal spi_clk_enable : std_logic := '0';

begin
  -- This process produces a 100mHz/SPI_CLK_TICKS Hz period, 50% duty cycle signal
  process(sys_clk, spi_clk_enable) begin
    if rising_edge(sys_clk) and spi_clk_enable = '1' then
        if spi_clk_count = (SPI_CLK_TICKS/2) - 1 then
            spi_clk_count <= 0;
            spi_clk <= not spi_clk;
        else
            spi_clk_count <= spi_clk_count + 1;
        end if;
    end if;
  end process;

end RHD2164Handler_arch;
