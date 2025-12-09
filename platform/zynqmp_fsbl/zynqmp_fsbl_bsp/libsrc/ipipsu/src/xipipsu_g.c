#include "xipipsu.h"

XIpiPsu_Config XIpiPsu_ConfigTable[] __attribute__ ((section (".drvcfg_sec"))) = {

	{
		"xlnx,zynqmp-ipi-mailbox", /* compatible */
		0xff300000, /* reg */
		0x1, /* xlnx,ipi-bitmask */
		0x2, /* xlnx,ipi-buf-index */
		0x4023, /* interrupts */
		0xf9010000, /* interrupt-parent */
		0x9, /* xlnx,ipi-target-count */
		{
			{
				1, /* xlnx,ipi-bitmask */
				2 /* xlnx,ipi-buf-index */
			},
			{
				256, /* xlnx,ipi-bitmask */
				0 /* xlnx,ipi-buf-index */
			},
			{
				512, /* xlnx,ipi-bitmask */
				1 /* xlnx,ipi-buf-index */
			},
			{
				65536, /* xlnx,ipi-bitmask */
				7 /* xlnx,ipi-buf-index */
			},
			{
				131072, /* xlnx,ipi-bitmask */
				7 /* xlnx,ipi-buf-index */
			},
			{
				262144, /* xlnx,ipi-bitmask */
				7 /* xlnx,ipi-buf-index */
			},
			{
				524288, /* xlnx,ipi-bitmask */
				7 /* xlnx,ipi-buf-index */
			},
			{
				16777216, /* xlnx,ipi-bitmask */
				3 /* xlnx,ipi-buf-index */
			},
			{
				33554432, /* xlnx,ipi-bitmask */
				4 /* xlnx,ipi-buf-index */
			},
		}, /* child,required */
	},
	{
		"xlnx,zynqmp-ipi-mailbox", /* compatible */
		0xff340000, /* reg */
		0x1000000, /* xlnx,ipi-bitmask */
		0x3, /* xlnx,ipi-buf-index */
		0x401d, /* interrupts */
		0xf9010000, /* interrupt-parent */
		0x9, /* xlnx,ipi-target-count */
		{
			{
				1, /* xlnx,ipi-bitmask */
				2 /* xlnx,ipi-buf-index */
			},
			{
				256, /* xlnx,ipi-bitmask */
				0 /* xlnx,ipi-buf-index */
			},
			{
				512, /* xlnx,ipi-bitmask */
				1 /* xlnx,ipi-buf-index */
			},
			{
				65536, /* xlnx,ipi-bitmask */
				7 /* xlnx,ipi-buf-index */
			},
			{
				131072, /* xlnx,ipi-bitmask */
				7 /* xlnx,ipi-buf-index */
			},
			{
				262144, /* xlnx,ipi-bitmask */
				7 /* xlnx,ipi-buf-index */
			},
			{
				524288, /* xlnx,ipi-bitmask */
				7 /* xlnx,ipi-buf-index */
			},
			{
				16777216, /* xlnx,ipi-bitmask */
				3 /* xlnx,ipi-buf-index */
			},
			{
				33554432, /* xlnx,ipi-bitmask */
				4 /* xlnx,ipi-buf-index */
			},
		}, /* child,required */
	},
	{
		"xlnx,zynqmp-ipi-mailbox", /* compatible */
		0xff350000, /* reg */
		0x2000000, /* xlnx,ipi-bitmask */
		0x4, /* xlnx,ipi-buf-index */
		0x401e, /* interrupts */
		0xf9010000, /* interrupt-parent */
		0x9, /* xlnx,ipi-target-count */
		{
			{
				1, /* xlnx,ipi-bitmask */
				2 /* xlnx,ipi-buf-index */
			},
			{
				256, /* xlnx,ipi-bitmask */
				0 /* xlnx,ipi-buf-index */
			},
			{
				512, /* xlnx,ipi-bitmask */
				1 /* xlnx,ipi-buf-index */
			},
			{
				65536, /* xlnx,ipi-bitmask */
				7 /* xlnx,ipi-buf-index */
			},
			{
				131072, /* xlnx,ipi-bitmask */
				7 /* xlnx,ipi-buf-index */
			},
			{
				262144, /* xlnx,ipi-bitmask */
				7 /* xlnx,ipi-buf-index */
			},
			{
				524288, /* xlnx,ipi-bitmask */
				7 /* xlnx,ipi-buf-index */
			},
			{
				16777216, /* xlnx,ipi-bitmask */
				3 /* xlnx,ipi-buf-index */
			},
			{
				33554432, /* xlnx,ipi-bitmask */
				4 /* xlnx,ipi-buf-index */
			},
		}, /* child,required */
	},
	 {
		 NULL
	}
};