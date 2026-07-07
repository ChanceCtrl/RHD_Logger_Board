# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "")
  file(REMOVE_RECURSE
  "/home/ctrl/code/rhd_logger/arty-a7/vitis_workspace/arty_eth_platform/microblaze_0/standalone_microblaze_0/bsp/include/lwipopts.h"
  "/home/ctrl/code/rhd_logger/arty-a7/vitis_workspace/arty_eth_platform/microblaze_0/standalone_microblaze_0/bsp/include/xemac_ieee_reg.h"
  "/home/ctrl/code/rhd_logger/arty-a7/vitis_workspace/arty_eth_platform/microblaze_0/standalone_microblaze_0/bsp/include/xlwipconfig.h"
  "/home/ctrl/code/rhd_logger/arty-a7/vitis_workspace/arty_eth_platform/microblaze_0/standalone_microblaze_0/bsp/lib/liblwip220.a"
  )
endif()
