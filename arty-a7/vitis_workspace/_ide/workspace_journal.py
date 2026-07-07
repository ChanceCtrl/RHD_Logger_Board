# 2026-07-06T19:36:23.098166712
import vitis

client = vitis.create_client()
client.set_workspace(path="vitis_workspace")

platform = client.create_platform_component(name = "arty_eth_platform",hw_design = "$COMPONENT_LOCATION/../../rhd_ip_logger/rhd_arty_wrapper.xsa",os = "standalone",cpu = "microblaze_0",domain_name = "standalone_microblaze_0",compiler = "gcc")

platform = client.get_component(name="arty_eth_platform")
domain = platform.get_domain(name="standalone_microblaze_0")

status = domain.set_lib(lib_name="lwip220", path="$COMPONENT_LOCATION/../../../../../Documents/amd/2026.1/data/embeddedsw/ThirdParty/sw_services/lwip220_v1_4")

status = domain.set_config(option = "lib", param = "lwip220_temac_phy_link_speed", value = "CONFIG_LINKSPEED100", lib_name="lwip220")

