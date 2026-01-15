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
#include <xstatus.h>
#include <pm_api_sys.h>
#include <libeswpm/test_aux.h>

extern void __attribute__((weak)) *_vector_table;
static XScuGic GicInst;
static XIpiPsu IpiInst;

#define PU_NODE		NODE_APU_0
#define SRAM_0		NODE_OCM_BANK_0
#define SRAM_1		NODE_OCM_BANK_1
#define SRAM_2		NODE_OCM_BANK_2
#define SRAM_3		NODE_OCM_BANK_3

/**
 * prepare_suspend() - save sontext and request suspend
 */
static void prepare_suspend(void)
{
	XStatus status;

	status = XPm_SelfSuspend(PU_NODE, MAX_LATENCY, 0, (u64)&_vector_table);
	HALT_IF_ERR(status, "in self suspend call for %s",
		    SHOW_DEFINE(PU_NODE));

	status = XPm_SetRequirement(NODE_L2, 0, 0, REQUEST_ACK_NO);
	HALT_IF_ERR(status, "in set requirement for NODE_L2");

	status = XPm_SetRequirement(SRAM_0, PM_CAP_CONTEXT, 0, REQUEST_ACK_NO);
	HALT_IF_ERR(status, "in set requirement for %s", SHOW_DEFINE(SRAM_0));

	status = XPm_SetRequirement(SRAM_1, PM_CAP_CONTEXT, 0, REQUEST_ACK_NO);
	HALT_IF_ERR(status, "in set requirement for %s", SHOW_DEFINE(SRAM_1));

	status = XPm_SetRequirement(SRAM_2, PM_CAP_CONTEXT, 0, REQUEST_ACK_NO);
	HALT_IF_ERR(status, "in set requirement for %s", SHOW_DEFINE(SRAM_2));

	status = XPm_SetRequirement(SRAM_3, PM_CAP_CONTEXT, 0, REQUEST_ACK_NO);
	HALT_IF_ERR(status, "in set requirement for %s", SHOW_DEFINE(SRAM_3));

	status = XPm_SetRequirement(NODE_DDR, PM_CAP_CONTEXT, 0, REQUEST_ACK_NO);
	HALT_IF_ERR(status, "in set requirement for NODE_DDR");

	status = XPm_ReleaseNode(NODE_PL);
	HALT_IF_ERR(status, "in release node for NODE_PL");

	status = XPm_ReleaseNode(NODE_I2C_0);
	HALT_IF_ERR(status, "in release node for NODE_I2C_0");

	status = XPm_ReleaseNode(NODE_I2C_1);
	HALT_IF_ERR(status, "in release node for NODE_I2C_1");

	status = XPm_ReleaseNode(NODE_SD_1);
	HALT_IF_ERR(status, "in release node for NODE_SD_1");

	status = XPm_ReleaseNode(NODE_QSPI);
	HALT_IF_ERR(status, "in release node for NODE_QSPI");

	GicSuspend(&GicInst);
    /* --- ADD THIS LINE --- */
    //XPm_ClientSuspendFinalize();
}

int main(void)
{
	XStatus status;

	pm_print("INITIAL BOOT\r\n");

	/* Initialize GIC, IPIs, and Xilpm */
	PmTestInit(&GicInst, &IpiInst);

	/* Let other masters in the system know I'm ready */
    pm_print("My BitMask = 0x%08x\r\n", primary_master->ipi->Config.BitMask);
    pm_print("GEN_STORAGE5 before ready = 0x%08x\r\n", pm_read(0xFFD80064));
	SyncDeclareReady();
    pm_print("GEN_STORAGE5 after ready  = 0x%08x\r\n", pm_read(0xFFD80064));

	/* Wait to receive init suspend request from PMU */
	IpiWaitForInitSuspend();
	if (pm_susp.reason != SUSPEND_REASON_PU_REQ) {
		pm_print("Unexpected suspend reason %u\n", pm_susp.reason);
		goto fail;
	}

	//status = XPm_RequestWakeUp(NODE_RPU_1, 1, 0, REQUEST_ACK_NO);
	//HALT_IF_ERR(status, "in request wakeup for NODE_RPU_1");

	//status = XPm_ForcePowerDown(NODE_RPU_1, REQUEST_ACK_NO);
	//HALT_IF_ERR(status, "in force power down for NODE_RPU_1");

	/* Initiate suspend */
	prepare_suspend();
	XPm_SuspendFinalize();

fail:
	/* Should never execute code below */
	return XST_FAILURE;
}

