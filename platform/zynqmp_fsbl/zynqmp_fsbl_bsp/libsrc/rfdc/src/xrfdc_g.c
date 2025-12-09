#include "xrfdc.h"

XRFdc_Config XRFdc_ConfigTable[] __attribute__ ((section (".drvcfg_sec"))) = {

	{
		"xlnx,usp-rf-data-converter-2.6", /* compatible */
		0xb4b00000, /* reg */
		0x0, /* xlnx,high-speed-adc */
		0x0, /* xlnx,sysref-master */
		0x0, /* xlnx,sysref-master */
		0x1, /* xlnx,sysref-source */
		0x1, /* xlnx,sysref-source */
		0x2, /* xlnx,ip-type */
		0x1, /* xlnx,silicon-revision */
		{
			{
				1, /* xlnx,enable */
				1, /* xlnx,pll-enable */
				7.86432, /* xlnx,sampling-rate */
				245.760, /* xlnx,refclk-freq */
				245.760, /* xlnx,fabric-freq */
				32, /* xlnx,fbdiv */
				1, /* xlnx,outdiv */
				1, /* xlnx,refclk-div */
				0, /* xlnx,band */
				10.000, /* xlnx,fs-max */
				4, /* xlnx,slices */
				0, /* xlnx,link-coupling */
		{
			{
				1, /* xlnx,slice-enable */
				0, /* xlnx,invsinc-ctrl */
				0, /* xlnx,mixer-mode */
				0, /* xlnx,decoder-mode */
			},
			{
				1, /* xlnx,slice-enable */
				0, /* xlnx,invsinc-ctrl */
				0, /* xlnx,mixer-mode */
				0, /* xlnx,decoder-mode */
			},
			{
				1, /* xlnx,slice-enable */
				0, /* xlnx,invsinc-ctrl */
				0, /* xlnx,mixer-mode */
				0, /* xlnx,decoder-mode */
			},
			{
				1, /* xlnx,slice-enable */
				0, /* xlnx,invsinc-ctrl */
				0, /* xlnx,mixer-mode */
				0, /* xlnx,decoder-mode */
			},
},
{
			{
				0, /* xlnx,data-type */
				16, /* xlnx,data-width */
				2, /* xlnx,interpolation-mode */
				0, /* xlnx,fifo-enable */
				0, /* xlnx,adder-enable */
				2, /* xlnx,mixer-type */
				0.0 /* xlnx,nco-freq */
			},
			{
				0, /* xlnx,data-type */
				16, /* xlnx,data-width */
				2, /* xlnx,interpolation-mode */
				0, /* xlnx,fifo-enable */
				0, /* xlnx,adder-enable */
				2, /* xlnx,mixer-type */
				0.0 /* xlnx,nco-freq */
			},
			{
				0, /* xlnx,data-type */
				16, /* xlnx,data-width */
				2, /* xlnx,interpolation-mode */
				0, /* xlnx,fifo-enable */
				0, /* xlnx,adder-enable */
				2, /* xlnx,mixer-type */
				0.0 /* xlnx,nco-freq */
			},
			{
				0, /* xlnx,data-type */
				16, /* xlnx,data-width */
				2, /* xlnx,interpolation-mode */
				0, /* xlnx,fifo-enable */
				0, /* xlnx,adder-enable */
				2, /* xlnx,mixer-type */
				0.0 /* xlnx,nco-freq */
			},
		},
			},
			{
				1, /* xlnx,enable */
				1, /* xlnx,pll-enable */
				7.86432, /* xlnx,sampling-rate */
				245.760, /* xlnx,refclk-freq */
				245.760, /* xlnx,fabric-freq */
				32, /* xlnx,fbdiv */
				1, /* xlnx,outdiv */
				1, /* xlnx,refclk-div */
				0, /* xlnx,band */
				10.000, /* xlnx,fs-max */
				4, /* xlnx,slices */
				0, /* xlnx,link-coupling */
		{
			{
				1, /* xlnx,slice-enable */
				0, /* xlnx,invsinc-ctrl */
				0, /* xlnx,mixer-mode */
				0, /* xlnx,decoder-mode */
			},
			{
				1, /* xlnx,slice-enable */
				0, /* xlnx,invsinc-ctrl */
				0, /* xlnx,mixer-mode */
				0, /* xlnx,decoder-mode */
			},
			{
				1, /* xlnx,slice-enable */
				0, /* xlnx,invsinc-ctrl */
				0, /* xlnx,mixer-mode */
				0, /* xlnx,decoder-mode */
			},
			{
				1, /* xlnx,slice-enable */
				0, /* xlnx,invsinc-ctrl */
				0, /* xlnx,mixer-mode */
				0, /* xlnx,decoder-mode */
			},
},
{
			{
				0, /* xlnx,data-type */
				16, /* xlnx,data-width */
				2, /* xlnx,interpolation-mode */
				0, /* xlnx,fifo-enable */
				0, /* xlnx,adder-enable */
				2, /* xlnx,mixer-type */
				0.0 /* xlnx,nco-freq */
			},
			{
				0, /* xlnx,data-type */
				16, /* xlnx,data-width */
				2, /* xlnx,interpolation-mode */
				0, /* xlnx,fifo-enable */
				0, /* xlnx,adder-enable */
				2, /* xlnx,mixer-type */
				0.0 /* xlnx,nco-freq */
			},
			{
				0, /* xlnx,data-type */
				16, /* xlnx,data-width */
				2, /* xlnx,interpolation-mode */
				0, /* xlnx,fifo-enable */
				0, /* xlnx,adder-enable */
				2, /* xlnx,mixer-type */
				0.0 /* xlnx,nco-freq */
			},
			{
				0, /* xlnx,data-type */
				16, /* xlnx,data-width */
				2, /* xlnx,interpolation-mode */
				0, /* xlnx,fifo-enable */
				0, /* xlnx,adder-enable */
				2, /* xlnx,mixer-type */
				0.0 /* xlnx,nco-freq */
			},
		},
			},
			{
				1, /* xlnx,enable */
				1, /* xlnx,pll-enable */
				7.86432, /* xlnx,sampling-rate */
				245.760, /* xlnx,refclk-freq */
				245.760, /* xlnx,fabric-freq */
				32, /* xlnx,fbdiv */
				1, /* xlnx,outdiv */
				1, /* xlnx,refclk-div */
				0, /* xlnx,band */
				10.000, /* xlnx,fs-max */
				4, /* xlnx,slices */
				0, /* xlnx,link-coupling */
		{
			{
				1, /* xlnx,slice-enable */
				0, /* xlnx,invsinc-ctrl */
				0, /* xlnx,mixer-mode */
				0, /* xlnx,decoder-mode */
			},
			{
				1, /* xlnx,slice-enable */
				0, /* xlnx,invsinc-ctrl */
				0, /* xlnx,mixer-mode */
				0, /* xlnx,decoder-mode */
			},
			{
				1, /* xlnx,slice-enable */
				0, /* xlnx,invsinc-ctrl */
				0, /* xlnx,mixer-mode */
				0, /* xlnx,decoder-mode */
			},
			{
				1, /* xlnx,slice-enable */
				0, /* xlnx,invsinc-ctrl */
				0, /* xlnx,mixer-mode */
				0, /* xlnx,decoder-mode */
			},
},
{
			{
				0, /* xlnx,data-type */
				16, /* xlnx,data-width */
				2, /* xlnx,interpolation-mode */
				0, /* xlnx,fifo-enable */
				0, /* xlnx,adder-enable */
				2, /* xlnx,mixer-type */
				0.0 /* xlnx,nco-freq */
			},
			{
				0, /* xlnx,data-type */
				16, /* xlnx,data-width */
				2, /* xlnx,interpolation-mode */
				0, /* xlnx,fifo-enable */
				0, /* xlnx,adder-enable */
				2, /* xlnx,mixer-type */
				0.0 /* xlnx,nco-freq */
			},
			{
				0, /* xlnx,data-type */
				16, /* xlnx,data-width */
				2, /* xlnx,interpolation-mode */
				0, /* xlnx,fifo-enable */
				0, /* xlnx,adder-enable */
				2, /* xlnx,mixer-type */
				0.0 /* xlnx,nco-freq */
			},
			{
				0, /* xlnx,data-type */
				16, /* xlnx,data-width */
				2, /* xlnx,interpolation-mode */
				0, /* xlnx,fifo-enable */
				0, /* xlnx,adder-enable */
				2, /* xlnx,mixer-type */
				0.0 /* xlnx,nco-freq */
			},
		},
			},
			{
				1, /* xlnx,enable */
				1, /* xlnx,pll-enable */
				7.86432, /* xlnx,sampling-rate */
				245.760, /* xlnx,refclk-freq */
				245.760, /* xlnx,fabric-freq */
				32, /* xlnx,fbdiv */
				1, /* xlnx,outdiv */
				1, /* xlnx,refclk-div */
				0, /* xlnx,band */
				10.000, /* xlnx,fs-max */
				4, /* xlnx,slices */
				0, /* xlnx,link-coupling */
		{
			{
				1, /* xlnx,slice-enable */
				0, /* xlnx,invsinc-ctrl */
				0, /* xlnx,mixer-mode */
				0, /* xlnx,decoder-mode */
			},
			{
				1, /* xlnx,slice-enable */
				0, /* xlnx,invsinc-ctrl */
				0, /* xlnx,mixer-mode */
				0, /* xlnx,decoder-mode */
			},
			{
				1, /* xlnx,slice-enable */
				0, /* xlnx,invsinc-ctrl */
				0, /* xlnx,mixer-mode */
				0, /* xlnx,decoder-mode */
			},
			{
				1, /* xlnx,slice-enable */
				0, /* xlnx,invsinc-ctrl */
				0, /* xlnx,mixer-mode */
				0, /* xlnx,decoder-mode */
			},
},
{
			{
				0, /* xlnx,data-type */
				16, /* xlnx,data-width */
				2, /* xlnx,interpolation-mode */
				0, /* xlnx,fifo-enable */
				0, /* xlnx,adder-enable */
				2, /* xlnx,mixer-type */
				0.0 /* xlnx,nco-freq */
			},
			{
				0, /* xlnx,data-type */
				16, /* xlnx,data-width */
				2, /* xlnx,interpolation-mode */
				0, /* xlnx,fifo-enable */
				0, /* xlnx,adder-enable */
				2, /* xlnx,mixer-type */
				0.0 /* xlnx,nco-freq */
			},
			{
				0, /* xlnx,data-type */
				16, /* xlnx,data-width */
				2, /* xlnx,interpolation-mode */
				0, /* xlnx,fifo-enable */
				0, /* xlnx,adder-enable */
				2, /* xlnx,mixer-type */
				0.0 /* xlnx,nco-freq */
			},
			{
				0, /* xlnx,data-type */
				16, /* xlnx,data-width */
				2, /* xlnx,interpolation-mode */
				0, /* xlnx,fifo-enable */
				0, /* xlnx,adder-enable */
				2, /* xlnx,mixer-type */
				0.0 /* xlnx,nco-freq */
			},
		},
			},
},
{
			{
				1, /* xlnx,enable */
				1, /* xlnx,pll-enable */
				2.21184, /* xlnx,sampling-rate */
				245.760, /* xlnx,refclk-freq */
				368.640, /* xlnx,fabric-freq */
				36, /* xlnx,fbdiv */
				4, /* xlnx,outdiv */
				1, /* xlnx,refclk-div */
				0, /* xlnx,band */
				2.500, /* xlnx,fs-max */
				4, /* xlnx,slices */
		{
			{
				1, /* xlnx,slice-enable */
				0, /* xlnx,mixer-mode */
			},
			{
				1, /* xlnx,slice-enable */
				0, /* xlnx,mixer-mode */
			},
			{
				1, /* xlnx,slice-enable */
				0, /* xlnx,mixer-mode */
			},
			{
				1, /* xlnx,slice-enable */
				0, /* xlnx,mixer-mode */
			},
},
{
			{
				1, /* xlnx,data-type */
				12, /* xlnx,data-width */
				1, /* xlnx,decimation-mode */
				0, /* xlnx,fifo-enable */
				2, /* xlnx,mixer-type */
				0.0 /* xlnx,nco-freq */
			},
			{
				1, /* xlnx,data-type */
				12, /* xlnx,data-width */
				1, /* xlnx,decimation-mode */
				0, /* xlnx,fifo-enable */
				2, /* xlnx,mixer-type */
				0.0 /* xlnx,nco-freq */
			},
			{
				1, /* xlnx,data-type */
				12, /* xlnx,data-width */
				1, /* xlnx,decimation-mode */
				0, /* xlnx,fifo-enable */
				2, /* xlnx,mixer-type */
				0.0 /* xlnx,nco-freq */
			},
			{
				1, /* xlnx,data-type */
				12, /* xlnx,data-width */
				1, /* xlnx,decimation-mode */
				0, /* xlnx,fifo-enable */
				2, /* xlnx,mixer-type */
				0.0 /* xlnx,nco-freq */
			},
		},
			},
			{
				1, /* xlnx,enable */
				1, /* xlnx,pll-enable */
				2.21184, /* xlnx,sampling-rate */
				245.760, /* xlnx,refclk-freq */
				368.640, /* xlnx,fabric-freq */
				36, /* xlnx,fbdiv */
				4, /* xlnx,outdiv */
				1, /* xlnx,refclk-div */
				0, /* xlnx,band */
				2.500, /* xlnx,fs-max */
				4, /* xlnx,slices */
		{
			{
				1, /* xlnx,slice-enable */
				0, /* xlnx,mixer-mode */
			},
			{
				1, /* xlnx,slice-enable */
				0, /* xlnx,mixer-mode */
			},
			{
				1, /* xlnx,slice-enable */
				0, /* xlnx,mixer-mode */
			},
			{
				1, /* xlnx,slice-enable */
				0, /* xlnx,mixer-mode */
			},
},
{
			{
				1, /* xlnx,data-type */
				12, /* xlnx,data-width */
				1, /* xlnx,decimation-mode */
				0, /* xlnx,fifo-enable */
				2, /* xlnx,mixer-type */
				0.0 /* xlnx,nco-freq */
			},
			{
				1, /* xlnx,data-type */
				12, /* xlnx,data-width */
				1, /* xlnx,decimation-mode */
				0, /* xlnx,fifo-enable */
				2, /* xlnx,mixer-type */
				0.0 /* xlnx,nco-freq */
			},
			{
				1, /* xlnx,data-type */
				12, /* xlnx,data-width */
				1, /* xlnx,decimation-mode */
				0, /* xlnx,fifo-enable */
				2, /* xlnx,mixer-type */
				0.0 /* xlnx,nco-freq */
			},
			{
				1, /* xlnx,data-type */
				12, /* xlnx,data-width */
				1, /* xlnx,decimation-mode */
				0, /* xlnx,fifo-enable */
				2, /* xlnx,mixer-type */
				0.0 /* xlnx,nco-freq */
			},
		},
			},
			{
				1, /* xlnx,enable */
				1, /* xlnx,pll-enable */
				2.21184, /* xlnx,sampling-rate */
				245.760, /* xlnx,refclk-freq */
				368.640, /* xlnx,fabric-freq */
				36, /* xlnx,fbdiv */
				4, /* xlnx,outdiv */
				1, /* xlnx,refclk-div */
				0, /* xlnx,band */
				2.500, /* xlnx,fs-max */
				4, /* xlnx,slices */
		{
			{
				1, /* xlnx,slice-enable */
				0, /* xlnx,mixer-mode */
			},
			{
				1, /* xlnx,slice-enable */
				0, /* xlnx,mixer-mode */
			},
			{
				1, /* xlnx,slice-enable */
				0, /* xlnx,mixer-mode */
			},
			{
				1, /* xlnx,slice-enable */
				0, /* xlnx,mixer-mode */
			},
},
{
			{
				1, /* xlnx,data-type */
				12, /* xlnx,data-width */
				1, /* xlnx,decimation-mode */
				0, /* xlnx,fifo-enable */
				2, /* xlnx,mixer-type */
				0.0 /* xlnx,nco-freq */
			},
			{
				1, /* xlnx,data-type */
				12, /* xlnx,data-width */
				1, /* xlnx,decimation-mode */
				0, /* xlnx,fifo-enable */
				2, /* xlnx,mixer-type */
				0.0 /* xlnx,nco-freq */
			},
			{
				1, /* xlnx,data-type */
				12, /* xlnx,data-width */
				1, /* xlnx,decimation-mode */
				0, /* xlnx,fifo-enable */
				2, /* xlnx,mixer-type */
				0.0 /* xlnx,nco-freq */
			},
			{
				1, /* xlnx,data-type */
				12, /* xlnx,data-width */
				1, /* xlnx,decimation-mode */
				0, /* xlnx,fifo-enable */
				2, /* xlnx,mixer-type */
				0.0 /* xlnx,nco-freq */
			},
		},
			},
			{
				1, /* xlnx,enable */
				1, /* xlnx,pll-enable */
				2.21184, /* xlnx,sampling-rate */
				245.760, /* xlnx,refclk-freq */
				368.640, /* xlnx,fabric-freq */
				36, /* xlnx,fbdiv */
				4, /* xlnx,outdiv */
				1, /* xlnx,refclk-div */
				0, /* xlnx,band */
				2.500, /* xlnx,fs-max */
				4, /* xlnx,slices */
		{
			{
				1, /* xlnx,slice-enable */
				0, /* xlnx,mixer-mode */
			},
			{
				1, /* xlnx,slice-enable */
				0, /* xlnx,mixer-mode */
			},
			{
				1, /* xlnx,slice-enable */
				0, /* xlnx,mixer-mode */
			},
			{
				1, /* xlnx,slice-enable */
				0, /* xlnx,mixer-mode */
			},
},
{
			{
				1, /* xlnx,data-type */
				12, /* xlnx,data-width */
				1, /* xlnx,decimation-mode */
				0, /* xlnx,fifo-enable */
				2, /* xlnx,mixer-type */
				0.0 /* xlnx,nco-freq */
			},
			{
				1, /* xlnx,data-type */
				12, /* xlnx,data-width */
				1, /* xlnx,decimation-mode */
				0, /* xlnx,fifo-enable */
				2, /* xlnx,mixer-type */
				0.0 /* xlnx,nco-freq */
			},
			{
				1, /* xlnx,data-type */
				12, /* xlnx,data-width */
				1, /* xlnx,decimation-mode */
				0, /* xlnx,fifo-enable */
				2, /* xlnx,mixer-type */
				0.0 /* xlnx,nco-freq */
			},
			{
				1, /* xlnx,data-type */
				12, /* xlnx,data-width */
				1, /* xlnx,decimation-mode */
				0, /* xlnx,fifo-enable */
				2, /* xlnx,mixer-type */
				0.0 /* xlnx,nco-freq */
			},
		},
			},
		}, /* child,required */
	},
	 {
		 NULL
	}
};