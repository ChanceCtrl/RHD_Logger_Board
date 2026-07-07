# Ethernet reference clock
set_property PACKAGE_PIN G18 [get_ports eth_ref_clk]
set_property IOSTANDARD LVCMOS33 [get_ports eth_ref_clk]

# Allow non-dedicated routing for generated clock (only if required)
set_property CLOCK_DEDICATED_ROUTE TRUE [get_nets rhd_arty_i/clk_wiz_0/inst/clk_out166]

# 100 MHz system clock
set_property PACKAGE_PIN E3 [get_ports sys_clock]
set_property IOSTANDARD LVCMOS33 [get_ports sys_clock] ;# IO_L12P_T1_MRCC_35

create_clock -name sys_clk_pin \
    -period 10.000 \
    -waveform {0.000 5.000} \
    [get_ports sys_clock]