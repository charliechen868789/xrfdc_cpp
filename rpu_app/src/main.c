/******************************************************************************
*
* Copyright (C) 2017 Xilinx, Inc.  All rights reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* Use of the Software is limited solely to applications:
* (a) running on a Xilinx device, or
* (b) that interact with a Xilinx device through a bus or interconnect.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* XILINX CONSORTIUM BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
* OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*
* Except as contained in this notice, the name of the Xilinx shall not be used
* in advertising or otherwise to promote the sale, use or other dealings in
* this Software without prior written authorization from Xilinx.
*
*
* CONTENT
* Use Case : Deep Sleep
* 1) The RPU waits for APU to up.
* 2) The RPU request suspend to APU and wait for APU to suspend.
* 3) The RPU configures RTC to generate interrupt
* 4) The RPU initiates self suspend. Before calling pm_self_suspend RPU
*    has saved its context in TCM memory. Suspending of the RPU is
*    followed by TCM retention.
* 5) RTC interrupt triggers wake up of the RPU and restoring its
*    state from TCM memory.
******************************************************************************/

#include <xil_exception.h>
#include <xrtcpsu.h>		/* RTCPSU device driver */
#include "pm_api_sys.h"
#include "pm_client.h"
#include <libeswpm/test_aux.h>
#include <xttcps.h>
#include "xparameters.h"
#include "xgpiops.h"
#include "xstatus.h"
#include "xplatform_info.h"
#include "sleep.h"

extern void __attribute__((weak)) *_vector_table;
static XScuGic GicInst;
static XIpiPsu IpiInst;
static XRtcPsu RtcPsu;          /* Instance of the RTC Device */

#define RTC_DEVICE_ID           XPAR_XRTCPSU_0_BASEADDR //XPAR_XRTCPSU_0_DEVICE_ID
#define INTC_DEVICE_ID          XPAR_SCUGIC_SINGLE_DEVICE_ID
#define RTC_ALARM_INT_IRQ_ID    XPAR_XRTCPSU_ALARM_INTR
#define GPIO_DEVICE_ID          XPAR_XGPIOPS_0_BASEADDR //XPAR_XGPIOPS_0_DEVICE_ID
#define IPI_PMU_0_TRIG_APU_MASK (0x1)
#define GPIO_PIN                (23)
#define ALARM_PERIOD            (20U)            /* in seconds */
#define TIMEOUT	                (2000000)       /* in microseconds  */

XGpioPs Gpio;                   /* The driver instance for GPIO Device. */

/* Do some work */
void do_some_work()
{
	/* Turn on the LED */
	//XGpioPs_WritePin(&Gpio, GPIO_PIN, 0x1);
	/* Wait for some time */
	//usleep(TIMEOUT);
	/* Turn off the LED */
	//XGpioPs_WritePin(&Gpio, GPIO_PIN, 0x0);
    pm_print("RPU do_some_work\n");
}

static XStatus TimerConfigure()
{
	XStatus status = XST_SUCCESS;
	XRtcPsu_Config *Config;

	/*
	 * Initialize the RTC driver so that it's ready to use
	 * Look up the configuration in the config table, then initialize it.
	 */
	Config = XRtcPsu_LookupConfig(XPAR_XRTCPSU_0_BASEADDR);
	if (NULL == Config) {
		pm_print("Invalid config\n");
		return XST_FAILURE;
	}

	status = XRtcPsu_CfgInitialize(&RtcPsu, Config, Config->BaseAddr);
	if (status != XST_SUCCESS) {
		pm_print("Cfg initialization failed\n");
		return XST_FAILURE;
	}

	/* Enable the interrupt for the RTC */
	XScuGic_Enable(&GicInst, RTC_ALARM_INT_IRQ_ID);

	return status;
}

static void enableTimer()
{
	u32 CurrentTime, Alarm;

	/* Enable the interrupt of the RTC device so interrupts will occur. */
	XRtcPsu_SetInterruptMask(&RtcPsu, XRTC_INT_EN_ALRM_MASK );

	CurrentTime = XRtcPsu_GetCurrentTime(&RtcPsu);
	Alarm = CurrentTime + ALARM_PERIOD;
	XRtcPsu_SetAlarm(&RtcPsu,Alarm,0);
}

static void suspendAPU()
{
	XStatus status = XST_SUCCESS;

	/* Call request suspend */
	pm_print("Call pm_req_suspend for APU\n");
	status = XPm_RequestSuspend(NODE_APU, REQUEST_ACK_NON_BLOCKING, MAX_LATENCY, 0);
    //status = XPm_RequestSuspend(NODE_APU, REQUEST_ACK_NO, MAX_LATENCY, 0);
	HALT_IF_ERR(status, "in requesting suspend for APU\n");

	/* Wait for acknowledge */
	IpiWaitForAck();
	HALT_IF_ERR(pm_ack.status, "failure ack to request suspend for APU\n");
}

static void configureGpio()
{
	XStatus status = XST_SUCCESS;
	XGpioPs_Config *ConfigPtr;

	/* Initialize the GPIO driver. */
	ConfigPtr = XGpioPs_LookupConfig(GPIO_DEVICE_ID);

	status = XGpioPs_CfgInitialize(&Gpio, ConfigPtr, ConfigPtr->BaseAddr);
	HALT_IF_ERR(status, "setting up gpio");

	/* Set the direction for the pin to be output. */
	XGpioPs_SetDirectionPin(&Gpio, GPIO_PIN, 1);
	/* Enable the Output enable for the LED Pin. */
	XGpioPs_SetOutputEnablePin(&Gpio, GPIO_PIN, 1);
	/* Set the GPIO output to be low. */
	XGpioPs_WritePin(&Gpio, GPIO_PIN, 0x0);
}

static void init(void)
{
	XStatus status = XST_SUCCESS;
	enum XPmBootStatus boot;

	boot = XPm_GetBootStatus();
	if (PM_INITIAL_BOOT == boot) {
		pm_print("RPU INITIAL BOOT\n");

		/* Initialize GIC, IPIs, and Xilpm */
		PmTestInit(&GicInst, &IpiInst);
        //debug
        //IpiInst.Config.BitMask = 0x00000001;
        //IpiInst.Config.TargetList[0].BufferIndex = 2; 
        //IpiInst.Config.TargetList[0].Mask = 0x00000001;
		/* Block until other masters in the system become ready */
        pm_print("My BitMask = 0x%08x\n", primary_master->ipi->Config.BitMask);
        pm_print("GEN_STORAGE5 before ready = 0x%08x\n", pm_read(0xFFD80064));
		SyncWaitForReady(IPI_PMU_0_TRIG_APU_MASK);
        pm_print("GEN_STORAGE5 after ready  = 0x%08x\n", pm_read(0xFFD80064));
        //XIpiPsu_InterruptEnable(&IpiInst, 0x00010000U);
		/* Suspend APU  */
		suspendAPU();

		/* Configure timer, if configuration fails abort */
		status = TimerConfigure();
		HALT_IF_ERR(status, "setting up timer");

		/* Configure GPIO */
		configureGpio();

	} else if (PM_RESUME == boot) {
		/* Disable the interrupt of the RTC device so interrupts will not occur. */
		XRtcPsu_ClearInterruptMask(&RtcPsu,XRTC_INT_DIS_ALRM_MASK);

		/* Timer is already counting, just enable interrupts */
		Xil_ExceptionEnable();

	} else {
		status = XST_FAILURE;
		HALT_IF_ERR(status, "cannot identify boot reason");
	}
}

/**
 * prepare_suspend() - request suspend
 */
static void prepare_suspend(void)
{
	XStatus status;

	status = XPm_SetWakeUpSource(NODE_RPU_0, NODE_RTC, 1);
	HALT_IF_ERR(status, "in enable wake-up source NODE_RTC");

    /* Tell PMU: Wake up APU Core 0 when RTC alarm fires */
    status = XPm_SetWakeUpSource(NODE_APU_0, NODE_RTC, 1);
    HALT_IF_ERR(status, "Failed to set RTC wakeup for APU");

	status = XPm_SelfSuspend(NODE_RPU_0, MAX_LATENCY, 0, (u64)&_vector_table);
	HALT_IF_ERR(status, "in self suspend call for RPU_0");

	status = XPm_SetRequirement(NODE_TCM_0_A, PM_CAP_CONTEXT, 0, REQUEST_ACK_NO);
	HALT_IF_ERR(status, "in set requirement for NODE_TCM_0_A");

	status = XPm_SetRequirement(NODE_TCM_0_B, PM_CAP_CONTEXT, 0, REQUEST_ACK_NO);
	HALT_IF_ERR(status, "in set requirement for NODE_TCM_0_B");

	status = XPm_SetRequirement(NODE_TCM_1_A, PM_CAP_CONTEXT, 0, REQUEST_ACK_NO);
	HALT_IF_ERR(status, "in set requirement for NODE_TCM_1_A");

	status = XPm_SetRequirement(NODE_TCM_1_B, PM_CAP_CONTEXT, 0, REQUEST_ACK_NO);
	HALT_IF_ERR(status, "in set requirement for NODE_TCM_1_B");
}

int main(void)
{
    
	/* Initialization  */
	init();

	/* Toggle GPIO LED */
	do_some_work();

	/* Enable RTC timer */
	enableTimer();

	/* Go to self-suspend */
	prepare_suspend();
	XPm_SuspendFinalize();

	/*
	 * Can execute code below only if interrupt is generated between calling
	 * the prepare_suspend and executing wfi. Shouldn't happen.
	 */
	XIL_FAIL(1);
	return XST_FAILURE;
}
