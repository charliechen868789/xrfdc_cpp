#include "xaxis_switch.h"

XAxis_Switch_Config XAxis_Switch_ConfigTable[] __attribute__ ((section (".drvcfg_sec"))) = {

	{
		"xlnx,axis-switch-1.1", /* compatible */
		0xa0204000, /* reg */
		0x10, /* xlnx,num-si-slots */
		0x1 /* xlnx,num-mi-slots */
	},
	 {
		 NULL
	}
};