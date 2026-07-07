#include "xemaclite.h"

XEmacLite_Config XEmacLite_ConfigTable[] __attribute__ ((section (".drvcfg_sec"))) = {

	{
		"xlnx,axi-ethernetlite-3.0", /* compatible */
		0x40e00000, /* reg */
		0x1, /* xlnx,tx-ping-pong */
		0x1, /* xlnx,rx-ping-pong */
		0x1, /* xlnx,include-mdio */
		0x0, /* xlnx,include-internal-loopback */
		0x0, /* interrupts */
		0x41200001 /* interrupt-parent */
	},
	 {
		 NULL
	}
};