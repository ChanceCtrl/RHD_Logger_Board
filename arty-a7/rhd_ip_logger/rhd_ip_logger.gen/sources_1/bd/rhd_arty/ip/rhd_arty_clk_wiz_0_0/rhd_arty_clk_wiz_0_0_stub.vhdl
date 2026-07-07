-- Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
-- Copyright 2022-2026 Advanced Micro Devices, Inc. All Rights Reserved.
-- --------------------------------------------------------------------------------
-- Tool Version: Vivado v.2026.1 (lin64) Build 6511674 Tue Jun 16 11:01:26 MDT 2026
-- Date        : Mon Jul  6 18:36:02 2026
-- Host        : prt running 64-bit CachyOS
-- Command     : write_vhdl -force -mode synth_stub
--               /home/ctrl/code/rhd_logger/arty-a7/rhd_ip_logger/rhd_ip_logger.gen/sources_1/bd/rhd_arty/ip/rhd_arty_clk_wiz_0_0/rhd_arty_clk_wiz_0_0_stub.vhdl
-- Design      : rhd_arty_clk_wiz_0_0
-- Purpose     : Stub declaration of top-level module interface
-- Device      : xc7a100tcsg324-1
-- --------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity rhd_arty_clk_wiz_0_0 is
  Port ( 
    clk_out166 : out STD_LOGIC;
    clk_out200 : out STD_LOGIC;
    clk_out25 : out STD_LOGIC;
    resetn : in STD_LOGIC;
    locked : out STD_LOGIC;
    clk_in1 : in STD_LOGIC
  );

  attribute CORE_GENERATION_INFO : string;
  attribute CORE_GENERATION_INFO of rhd_arty_clk_wiz_0_0 : entity is "rhd_arty_clk_wiz_0_0,clk_wiz_v6_0_19_0_0,{component_name=rhd_arty_clk_wiz_0_0,use_phase_alignment=true,use_min_o_jitter=false,use_max_i_jitter=false,use_dyn_phase_shift=false,use_inclk_switchover=false,use_dyn_reconfig=false,enable_axi=0,feedback_source=FDBK_AUTO,PRIMITIVE=MMCM,num_out_clk=3,clkin1_period=10.000,clkin2_period=10.000,use_power_down=false,use_reset=true,use_locked=true,use_inclk_stopped=false,feedback_type=SINGLE,CLOCK_MGR_TYPE=NA,manual_override=false}";
end rhd_arty_clk_wiz_0_0;

architecture stub of rhd_arty_clk_wiz_0_0 is
  attribute syn_black_box : boolean;
  attribute black_box_pad_pin : string;
  attribute syn_black_box of stub : architecture is true;
  attribute black_box_pad_pin of stub : architecture is "clk_out166,clk_out200,clk_out25,resetn,locked,clk_in1";
begin
end;
