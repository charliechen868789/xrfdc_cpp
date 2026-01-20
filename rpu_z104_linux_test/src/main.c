#include "pm_init.h"
#include "pm_api_sys.h"
#include "gic_setup.h"
#include "ipi.h"
#include "rtc.h"
#include "xilfpga.h"
#include "psgpio.h"
#include "xparameters.h"
extern u32 CountsPerSec;

extern void __attribute__((weak)) *_vector_table;
#define RESUME_ADDR			((u32)&_vector_table)
#define SUSPEND_TARGET			NODE_APU
#define WAKEUP_TARGET			NODE_APU_0
#define SELF_DEV_ID			NODE_RPU_0
#define LATENCY_VAL			MAX_LATENCY
#define NODE_IN_FPD			NODE_SATA
#define RTC_DEVICE			NODE_RTC
#define SUSPEND_TYPE			0
#define BLOCKING_ACK			REQUEST_ACK_BLOCKING
#define NON_BLOCKING_ACK		REQUEST_ACK_NON_BLOCKING
#define FPD_NODE			NODE_FPD
#define PL_NODE				NODE_PLD

#define DELAY_COUNT(x)			((x) * (u64)XPAR_CPU_CORE_CLOCK_FREQ_HZ / 10)
/* Calculate latency from counter ticks to microseconds */
#define CALCULATE_LATENCY(x)		((x) / (CountsPerSec / 1000000) )

#define SYNC_APU_MASK			(0x000000FFU)
#define SYNC_RPU_MASK			(0x0000FF00U)
#define SYNC_DELAY_VAL_MASK		(0x00FF0000U)
#define SYNC_DELAY_VAL_SHIFT		(16U)
#define SYNC_ITERATION_CNT_MASK		(0xFF000000U)
#define SYNC_ITERATION_CNT_SHIFT	(24U)
#define SYNC_APU_READY			(0x000000A5U)
#define SYNC_APU_FINISH			(0x0000005AU)
#define SYNC_RPU_SIGNAL			(0x0000AA00U)
#define SYNC_RPU_SIGNAL_APU_SUSPEND	(0x0000AB00U)
#define SYNC_RPU_FINISH			(0x00005500U)
#define SYNC_PL_DOWN			(0x00000011U)
#define SYNC_PL_UP			(0x00000012U)

#define PRINT_RPU_ON_APU_ON			xil_printf("RPU: ******************************** RPU ON, APU ON *********************************\r\n")
#define PRINT_RPU_ON_APU_SUSPEND	xil_printf("RPU: *********************** RPU ON, APU suspended with FPD ON ***********************\r\n")
#define PRINT_RPU_IDLE_APU_SUSPEND	xil_printf("RPU: ********************** RPU Idle, APU suspended with FPD ON **********************\r\n")
#define PRINT_RPU_ON_FPD_OFF		xil_printf("RPU: *********************** RPU ON, APU suspended with FPD OFF **********************\r\n")
#define PRINT_RPU_IDLE_FPD_OFF		xil_printf("RPU: ********************** RPU Idle, APU suspended with FPD OFF *********************\r\n")
#define PRINT_RPU_SUSPENDED_FPD_OFF	xil_printf("RPU: ******************* RPU Suspended, APU suspended with FPD OFF *******************\r\n")
#define PRINT_RPU_OUTLINE			xil_printf("RPU: *********************************************************************************\r\n")
#define PRINT_RPU_RESUME			xil_printf("RPU: ********************************     RESUME     *********************************\r\n")

#define BITSTREAM_SIZE 0x1A00000U /* Bin or bit or PDI image size */

static XIpiPsu IpiInst;
static XRtcPsu RtcInstPtr;
static XScuGic GicInst;
u64 tNotify;
u32 DelayVal;
u32 IterationCnt;

///
static XGpioPs PsGpioInst;
static XFpga XFpgaInstance = {0U};
///
#define PGGS3_REG			(0xFFD8005C)


static void Notify_CallBack(XPm_Notifier* const notifier)
{
	tNotify = ReadTime();
}

static XPm_Notifier notifier = {
	.callback = Notify_CallBack,
	.node = FPD_NODE,
	.event = EVENT_STATE_CHANGE,
	.flags = 0,
};

static int InitApp(void)
{
	int Status;

	Status = PmInit(&GicInst, &IpiInst);
	if (Status != XST_SUCCESS) {
		xil_printf("RPU: Error 0x%x in PmInit\r\n", Status);
		goto done;
	}

	Status = PmRtcInit(&GicInst, &RtcInstPtr);
	if (XST_SUCCESS != Status) {
		xil_printf("RPU: Error 0x%x in PmRtcInit\r\n", Status);
		goto done;
	}

	Status = PsGpioInit(&PsGpioInst);
	if (XST_SUCCESS != Status) {
		xil_printf("RPU: Error 0x%x in PsGpioInit\r\n", Status);
		goto done;
	}
	ClrTestPoint(&PsGpioInst, PIN_TP1);
	ClrTestPoint(&PsGpioInst, PIN_TP2);

done:
	return Status;
}

static void Wait(u32 Seconds)
{
	u64 WaitCount;

	//xil_printf("RPU: (%d seconds delay)\r\n", Seconds);
	WaitCount = DELAY_COUNT(Seconds);
	for (; WaitCount > 0; WaitCount--);
}

static int prepare_suspend(void)
{
	XStatus Status;

	Status = XPm_SetWakeUpSource(SELF_DEV_ID, RTC_DEVICE, 1);
	if (XST_SUCCESS != Status) {
		xil_printf("RPU: Error 0x%x in SetWakeUpSource of RTC\r\n", Status);
		goto done;
	}

	Status = XPm_SelfSuspend(SELF_DEV_ID, LATENCY_VAL, SUSPEND_TYPE, RESUME_ADDR);
	if (XST_SUCCESS != Status) {
		xil_printf("RPU: Error 0x%x in SelfSuspend\r\n", Status);
		goto done;
	}

	u32 SramMemList[] = {
		NODE_TCM_0_A,
		NODE_TCM_0_B,
		NODE_TCM_1_A,
		NODE_TCM_1_B,
	};
	u32 OtherDevList[] = {
		NODE_OCM_BANK_0,
		NODE_OCM_BANK_1,
		NODE_OCM_BANK_2,
		NODE_OCM_BANK_3,
		NODE_I2C_0,
		NODE_I2C_1,
		NODE_SD_1,
		NODE_QSPI,
		NODE_ADMA,
		NODE_GPIO,
	};
	u32 Idx;

	for (Idx = 0; Idx < PM_ARRAY_SIZE(SramMemList); Idx++) {
		Status = XPm_SetRequirement(SramMemList[Idx], PM_CAP_CONTEXT, 0, REQUEST_ACK_NO);
		if (XST_SUCCESS != Status) {
			xil_printf("RPU: Error 0x%x in SetRequirement of 0x%x\r\n", Status, SramMemList[Idx]);
			goto done;
		}
	}

	for (Idx = 0; Idx < PM_ARRAY_SIZE(OtherDevList); Idx++) {
		Status = XPm_SetRequirement(OtherDevList[Idx], 0, 0, REQUEST_ACK_NO);
		if (XST_SUCCESS != Status) {
			xil_printf("RPU: Error 0x%x in SetRequirement of 0x%x\r\n", Status, OtherDevList[Idx]);
			goto done;
		}
	}

done:
	return Status;
}

int main()
{
	enum XPmBootStatus BootStatus;
	int Status = XST_FAILURE;
	u64 tStart, tEnd;
	u32 PlLatency;
	u32 WakeupLatency;
	//
	UINTPTR BitAddr;
	UINTPTR KeyAddr = (UINTPTR)NULL;
	//

	BootStatus = XPm_GetBootStatus();
	if (PM_INITIAL_BOOT == BootStatus) 
    {
		/* Add delay to avoid print mix-up */
		Wait(3);
		xil_printf("RPU: INITIAL BOOT\r\n");

		Status = InitApp();
		if (XST_SUCCESS != Status) {
			xil_printf("RPU: Error 0x%x in InitApp\r\n", Status);
			goto done;
		}

		Status = XPm_RequestNode(NODE_IN_FPD, PM_CAP_ACCESS, 0, BLOCKING_ACK);
		if (XST_SUCCESS != Status) {
			xil_printf("RPU: Error 0x%x in RequestNode of 0x%x\r\n", Status, NODE_IN_FPD);
			goto done;
		}

		Status = XPm_RequestNode(NODE_UART_0, PM_CAP_ACCESS, 0, BLOCKING_ACK);
		if (XST_SUCCESS != Status) {
			xil_printf("RPU: Error 0x%x in RequestNode of 0x%x\r\n", Status, NODE_UART_0);
			goto done;
		}

		Status = XPm_RegisterNotifier(&notifier);
		if (XST_SUCCESS != Status) {
			xil_printf("RPU: Error 0x%x in RegisterNotifier\r\n", Status);
			goto done;
		}
	} else if (PM_RESUME == BootStatus) {
		Status = XPm_RequestNode(NODE_UART_0, PM_CAP_ACCESS, 0, BLOCKING_ACK);
		if (XST_SUCCESS != Status) {
			xil_printf("RPU: Error 0x%x in RequestNode of 0x%x\r\n", Status, NODE_UART_0);
			goto done;
		}

		//xil_printf("RPU: RESUMED\r\n");
		PRINT_RPU_OUTLINE;
		PRINT_RPU_RESUME;
		PRINT_RPU_OUTLINE;
		
		/* Timer is already counting, just enable interrupts */
		Status = GicResume(&GicInst);
		if (XST_SUCCESS != Status) {
			xil_printf("RPU: Error 0x%x in GicResume\r\n", Status);
			goto done;
		}

		Status = XPm_RequestNode(NODE_IN_FPD, PM_CAP_ACCESS, 0, BLOCKING_ACK);
		if (XST_SUCCESS != Status) {
			xil_printf("RPU: Error 0x%x in RequestNode of 0x%x\r\n", Status, NODE_IN_FPD);
			goto done;
		}

		/* Waking up APU */
		SetTestPoint(&PsGpioInst, PIN_TP1);
		tStart = ReadTime();
		Status = XPm_RequestWakeUp(WAKEUP_TARGET, 0, 0, BLOCKING_ACK);
		tEnd = ReadTime();

		/* Send SYNC_RPU_FINISH to APU */
		/* When APU0 wakes up and comes back to process "apu_script.sh",
		 * it would check SYNC_RPU_FINISH from RPU and drive the testpoint low
		 * for wakeup time measurement
		 */
		SyncSetMask(SYNC_RPU_MASK, SYNC_RPU_FINISH);
		/* Add delay to avoid print mix-up */
		Wait(1);

		if (XST_SUCCESS != Status) {
			xil_printf("RPU: Error 0x%x in RequestWakeup of 0x%x\r\n", Status, WAKEUP_TARGET);
			goto done;
		}
		WakeupLatency = CALCULATE_LATENCY(tEnd - tStart);
		xil_printf("RPU: [ APU Wakeup Latency in micro seconds: %ld ]\r\n", WakeupLatency);

		/* Waiting SYNC_RPU_FINISH from APU */
		SyncWaitForReady(SYNC_APU_READY);
		SyncClearReady(SYNC_APU_MASK);
		Wait(1);

		/* Power on PL */
		SyncWaitForReady(SYNC_PL_UP);
		SyncClearReady(SYNC_PL_UP);
		xil_printf("RPU: Powering up PL\r\n");
		tStart = ReadTime();
		Status = XPm_RequestNode(NODE_PL, PM_CAP_ACCESS, 0, BLOCKING_ACK);
		tEnd = ReadTime();
		if (XST_SUCCESS != Status) {
			xil_printf("RPU: Error 0x%x in RequestNode of 0x%x\r\n", Status, PL_NODE);
			goto done;
		}
		PlLatency = CALCULATE_LATENCY(tEnd - tStart);
		xil_printf("RPU: [ PL ON Latency in micro seconds: %ld ]\r\n", PlLatency);

		/////////////////////////////////////////////////////////////////////
		// Testing PL Programming
		BitAddr = (UINTPTR)0x60000000U;
		xil_printf("Loading Bitstream # for DDR location :0x%llx\n\r", BitAddr);
		xil_printf("Trying to configure the PL ......\n\r");

		Status = XFpga_Initialize(&XFpgaInstance);
		if (Status != XST_SUCCESS) {
			goto done;
		}

		tStart = ReadTime();
		SetTestPoint(&PsGpioInst, PIN_TP2);
		Status = XFpga_BitStream_Load(&XFpgaInstance, BitAddr, KeyAddr,
					      BITSTREAM_SIZE, XFPGA_FULLBIT_EN);
		ClrTestPoint(&PsGpioInst, PIN_TP2);
		tEnd = ReadTime();
		PlLatency = CALCULATE_LATENCY(tEnd - tStart);
		xil_printf("RPU: [ PL programming duration in micro seconds: %ld ]\r\n", PlLatency);
		if (Status == XFPGA_SUCCESS)
			xil_printf("PL Configuration done successfully\n\r");
		else
			xil_printf("PL configuration failed\n\r");

		//////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// Need to re-initialize interrupt system as
		// XFpga_Initialize() has touched the interrupt controller through underlying call to XMailbox_Initialize()
		//
		Status = PmInit(&GicInst, &IpiInst);
		if (Status != XST_SUCCESS) {
			xil_printf("RPU: Error 0x%x in PmInit\r\n", Status);
			goto done;
		}

		Status = PmRtcInit(&GicInst, &RtcInstPtr);
		if (XST_SUCCESS != Status) {
			xil_printf("RPU: Error 0x%x in PmRtcInit\r\n", Status);
			goto done;
		}
		//
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////

	} else {
		xil_printf("RPU: Invalid Boot Status\r\n");
		Status = XST_FAILURE;
		goto done;
	}

	/*****************
	 * Power down PL *
	 * ***************
	 */
	xil_printf("RPU: Waiting PL_DOWN ........\r\n");

	/* Waiting for PL power down command from APU */
	SyncWaitForReady(SYNC_PL_DOWN);
	SyncClearReady(SYNC_PL_DOWN);

	/* Power down PL */
	xil_printf("RPU: Powering down PL\r\n");
	tStart = ReadTime();
	//SetTestPoint(&PsGpioInst, PIN_TP2);
	Status = XPm_ForcePowerDown(PL_NODE, BLOCKING_ACK);
	//ClrTestPoint(&PsGpioInst, PIN_TP2);
	tEnd = ReadTime();
	if (XST_SUCCESS != Status) {
		xil_printf("RPU: Error 0x%x in ForcePowerDown of 0x%x\r\n", Status, PL_NODE);
		goto done;
	}
	PlLatency = CALCULATE_LATENCY(tEnd - tStart);
	xil_printf("RPU: [ PL OFF Latency in micro seconds: %ld ]\r\n", PlLatency);

	/* Sync APU */
	SyncSetMask(SYNC_RPU_MASK, SYNC_RPU_SIGNAL);

	/**********************
	 * Get APU parameters *
	 * ********************
	 */
	/* Waiting for APU parameters */
	SyncWaitForReady(SYNC_APU_READY);
	SyncClearReady(SYNC_APU_MASK);

	/* Get delay value */
	DelayVal = SyncGetValue(SYNC_DELAY_VAL_MASK) >> SYNC_DELAY_VAL_SHIFT;
	if (DelayVal < 10U) {
		DelayVal = 10U;
	}
	//xil_printf("RPU: DelayVal = %d\r\n", DelayVal);

	/* Get number of iterations for latency measurement */
	IterationCnt = SyncGetValue(SYNC_ITERATION_CNT_MASK) >> SYNC_ITERATION_CNT_SHIFT;
	if (IterationCnt > 5U) {
		IterationCnt = 5U;
	}
	//xil_printf("RPU: IterationCnt = %d\r\n", IterationCnt);

	/* Sync APU */
	SyncSetMask(SYNC_RPU_MASK, SYNC_RPU_SIGNAL);

	/************************************************
	 * Waiting SYNC_APU_FINISH, then suspend system *
	 * **********************************************
	 */
	SyncWaitForReady(SYNC_APU_FINISH);
	SyncClearReady(SYNC_APU_MASK);
    //NODE_APU_0
	//Status = XPm_RequestSuspend(SUSPEND_TARGET, NON_BLOCKING_ACK, LATENCY_VAL, 0);
    Status = XPm_RequestSuspend(NODE_APU_0, NON_BLOCKING_ACK, LATENCY_VAL, 0);
	// TODO: Need further check this call
	IpiWaitForAck();
	if (XST_SUCCESS != Status) {
		xil_printf("RPU: Error 0x%x in RequestSuspend of 0x%x\r\n", Status, SUSPEND_TARGET);
		goto done;
	}

	Status = XPm_ReleaseNode(NODE_IN_FPD);
	if (Status != XST_SUCCESS) {
		xil_printf("RPU: Error 0x%x in ReleaseNode of 0x%x\r\n", Status, NODE_IN_FPD);
		goto done;
	}

	SetRtcAlarm(&RtcInstPtr, 10);
	prepare_suspend();
	GicSuspend(&GicInst);
	PRINT_RPU_OUTLINE;
	PRINT_RPU_SUSPENDED_FPD_OFF;
	PRINT_RPU_OUTLINE;
	xil_printf("RPU:  ----------------\r\n");
	xil_printf("RPU: | Waiting 10sec  |\r\n");
	xil_printf("RPU:  ----------------\r\n");
	Wait(1);

	Status = XPm_ReleaseNode(NODE_UART_0);
	if (Status != XST_SUCCESS) {
		xil_printf("RPU: Error 0x%x in ReleaseNode of NODE_UART_0\r\n", Status);
		goto done;
	}

	XPm_ClientSuspendFinalize();

done:
	return Status;
}
