#include "pm_init.h"
#include "pm_api_sys.h"
#include "gic_setup.h"
#include "ipi.h"
#include "rtc.h"
#include "xilfpga.h"
#include "xparameters.h"
#include "xuartps.h"
#include "xil_mmu.h"

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

/* IPI Message Types */
#define IPI_MSG_RPU_FINISH		0x5A5A5A5AU
#define IPI_MSG_RPU_SIGNAL		0xAAAAAAAAU
#define IPI_MSG_APU_FINISH		0x5A000000U
#define IPI_MSG_PL_DOWN			0x00000011U
#define IPI_MSG_PL_UP			0x00000012U
#define IPI_MSG_PARAMS			0xDEADBEEFU

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

#define RPU0_REQ_BUF_ADDR   0xFF990040U  // Incoming from APU
#define APU_REQ_BUF_ADDR    0xFF990000U  // Outgoing to APU

#define PRINT_RPU_ON_APU_ON			xil_printf("RPU: ******************************** RPU ON, APU ON *********************************\r\n")
#define PRINT_RPU_ON_APU_SUSPEND	xil_printf("RPU: *********************** RPU ON, APU suspended with FPD ON ***********************\r\n")
#define PRINT_RPU_IDLE_APU_SUSPEND	xil_printf("RPU: ********************** RPU Idle, APU suspended with FPD ON **********************\r\n")
#define PRINT_RPU_ON_FPD_OFF		xil_printf("RPU: *********************** RPU ON, APU suspended with FPD OFF **********************\r\n")
#define PRINT_RPU_IDLE_FPD_OFF		xil_printf("RPU: ********************** RPU Idle, APU suspended with FPD OFF *********************\r\n")
#define PRINT_RPU_SUSPENDED_FPD_OFF	xil_printf("RPU: ******************* RPU Suspended, APU suspended with FPD OFF *******************\r\n")
#define PRINT_RPU_OUTLINE			xil_printf("RPU: *********************************************************************************\r\n")
#define PRINT_RPU_RESUME			xil_printf("RPU: ********************************     RESUME     *********************************\r\n")

#define BITSTREAM_SIZE 0x20D78ECU /* Bin or bit or PDI image size */

static XIpiPsu IpiInst;
static XRtcPsu RtcInstPtr;
static XScuGic GicInst;
u64 tNotify;
u32 DelayVal;
u32 IterationCnt;

/* IPI message flags */
static volatile int ipi_msg_received = 0;
static u32 received_msg_type = 0;
static u32 received_param1 = 0;
static u32 received_param2 = 0;

///
static XFpga XFpgaInstance = {0U};
///
#define PGGS3_REG			(0xFFD8005C)
#define IPI_APU_CH_MASK		0x00000001U
#define IPI_RPU0_CH_MASK    0x00000002U  // RPU0 is Self
#define IPI_MSG_ACK         0xAAAAAAAAU  // The code the script is waiting for

static XUartPs UartInst;

/* CRITICAL: Mark IPI buffer region as non-cacheable */
#if 0 
static void ConfigureIpiBufferMemory(void)
{
	// Set IPI buffer region (0xFF990000 - 0xFF99FFFF) as Device memory (non-cacheable)
	// This is CRITICAL for cache coherency
	Xil_SetTlbAttributes(0xFF990000, 0x04de2);  // Device memory attributes
	
	xil_printf("RPU: IPI buffer configured as non-cacheable\r\n");
}
#endif
static int InitUart0WithDriver(void)
{
    XUartPs_Config *Config;
    int Status;
    
    // Get hardware configuration
    Config = XUartPs_LookupConfig(XPAR_XUARTPS_1_BASEADDR);
    if (NULL == Config) {
        return XST_FAILURE;
    }
    
    // Initialize driver
    Status = XUartPs_CfgInitialize(&UartInst, Config, Config->BaseAddress);
    if (Status != XST_SUCCESS) {
        return Status;
    }
    
    // Set baud rate to 115200
    Status = XUartPs_SetBaudRate(&UartInst, 115200);
    if (Status != XST_SUCCESS) {
        return Status;
    }
    
    // Set operating mode to Normal (not loopback, etc.)
    XUartPs_SetOperMode(&UartInst, XUARTPS_OPER_MODE_NORMAL);
    
    return XST_SUCCESS;
}

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

/* Custom IPI Handler - FIXED VERSION */
#if 0
void CustomIpiHandler(XIpiPsu *InstancePtr) {
    volatile u32 *RawBuf = (volatile u32 *)RPU0_REQ_BUF_ADDR;
    u32 msg_val;
    int retries = 100;
    
    // **FIX 1: Invalidate cache BEFORE reading**
    Xil_DCacheInvalidateRange((INTPTR)RawBuf, 16);
    __asm__ __volatile__ ("dsb sy" : : : "memory");
    __asm__ __volatile__ ("isb" : : : "memory");
    
    // **FIX 2: Retry reading until valid data appears**
    while (retries > 0) {
        msg_val = RawBuf[0];
        
        if (msg_val != 0xFFFFFFFF && msg_val != 0x00000000) {
            break;
        }
        
        // Small delay and retry
        for (volatile int i = 0; i < 200; i++);
        
        // Re-invalidate cache before retry
        Xil_DCacheInvalidateRange((INTPTR)RawBuf, 16);
        __asm__ __volatile__ ("dsb sy" : : : "memory");
        
        retries--;
    }
    
    //xil_printf("RPU: IPI Handler - Msg: 0x%08X (retries: %d)\r\n", msg_val, 100 - retries);
    
    if (msg_val != 0xFFFFFFFF && msg_val != 0x00000000) {
        received_msg_type = RawBuf[0];
        received_param1   = RawBuf[1];
        received_param2   = RawBuf[2];
        ipi_msg_received  = 1;
        
        //xil_printf("RPU: Captured: Type=0x%08X, P1=0x%08X, P2=0x%08X\r\n", 
        //           received_msg_type, received_param1, received_param2);
        
        // Clear buffer
        //RawBuf[0] = 0; 
        //RawBuf[1] = 0;
        //RawBuf[2] = 0;
        
        // **FIX 3: Flush cache after clearing**
        //Xil_DCacheFlushRange((INTPTR)RawBuf, 16);
    } else {
        xil_printf("RPU: WARNING - Empty buffer after retries\r\n");
    }
    
    XIpiPsu_ClearInterruptStatus(InstancePtr, IPI_APU_CH_MASK);
}
#endif

void CustomIpiHandler(XIpiPsu *InstancePtr) {
    volatile u32 *RawBuf = (volatile u32 *)RPU0_REQ_BUF_ADDR;
    u32 msg_val;
    int retries = 100;
    
    // Cache invalidation
    Xil_DCacheInvalidateRange((INTPTR)RawBuf, 16);
    __asm__ __volatile__ ("dsb sy" : : : "memory");
    __asm__ __volatile__ ("isb" : : : "memory");
    
    // Retry until data appears
    while (retries > 0) {
        msg_val = RawBuf[0];
        if (msg_val != 0xFFFFFFFF && msg_val != 0x00000000) {
            break;
        }
        for (volatile int i = 0; i < 200; i++);
        Xil_DCacheInvalidateRange((INTPTR)RawBuf, 16);
        __asm__ __volatile__ ("dsb sy" : : : "memory");
        retries--;
    }
    
    if (msg_val != 0xFFFFFFFF && msg_val != 0x00000000) {
        received_msg_type = RawBuf[0];
        received_param1   = RawBuf[1];
        received_param2   = RawBuf[2];
        ipi_msg_received  = 1;
        
        // Clear buffer
        RawBuf[0] = 0; 
        RawBuf[1] = 0;
        RawBuf[2] = 0;
        Xil_DCacheFlushRange((INTPTR)RawBuf, 16);
    }
    
    // **CRITICAL: Clear ISR to acknowledge interrupt AND clear sender's OBS bit**
    XIpiPsu_ClearInterruptStatus(InstancePtr, IPI_APU_CH_MASK);
    
    // **ADDITIONAL: Make sure the clear is visible**
    __asm__ __volatile__ ("dsb sy" : : : "memory");
}

static int InitApp(void)
{
	int Status;
    //ConfigureIpiBufferMemory();
	Status = PmInit(&GicInst, &IpiInst);
	if (Status != XST_SUCCESS) {
		//xil_printf("RPU: Error 0x%x in PmInit\r\n", Status);
		goto done;
	}

	Status = PmRtcInit(&GicInst, &RtcInstPtr);
	if (XST_SUCCESS != Status) {
		//xil_printf("RPU: Error 0x%x in PmRtcInit\r\n", Status);
		goto done;
	}
	// **ADD: Clear all IPI buffers at startup**
	//xil_printf("RPU: Clearing IPI buffers...\r\n");
	
	// Clear incoming buffer (where APU writes)
	volatile u32 *IncomingBuf = (volatile u32 *)RPU0_REQ_BUF_ADDR;
	IncomingBuf[0] = 0;
	IncomingBuf[1] = 0;
	IncomingBuf[2] = 0;
	IncomingBuf[3] = 0;
	
	// Clear outgoing buffer (where RPU writes to APU)
	volatile u32 *OutgoingBuf = (volatile u32 *)APU_REQ_BUF_ADDR;
	OutgoingBuf[0] = 0;
	OutgoingBuf[1] = 0;
	OutgoingBuf[2] = 0;
	OutgoingBuf[3] = 0;
	
	// Flush cache
	Xil_DCacheFlushRange((INTPTR)IncomingBuf, 16);
	Xil_DCacheFlushRange((INTPTR)OutgoingBuf, 16);
	
	// Clear any pending interrupts
	XIpiPsu_ClearInterruptStatus(&IpiInst, IPI_APU_CH_MASK);
	/* Register custom IPI callback for APU channel */
	Status = IpiRegisterCallback(&IpiInst, IPI_APU_CH_MASK, CustomIpiHandler);
	if (Status != XST_SUCCESS) {
		//xil_printf("RPU: IpiRegisterCallback failed: 0x%x\r\n", Status);
		return Status;
	}
	//xil_printf("RPU: Init complete - APU mask: 0x%08X\r\n", IPI_APU_CH_MASK);
	return XST_SUCCESS;
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
		//xil_printf("RPU: Error 0x%x in SetWakeUpSource of RTC\r\n", Status);
		goto done;
	}

	Status = XPm_SelfSuspend(SELF_DEV_ID, LATENCY_VAL, SUSPEND_TYPE, RESUME_ADDR);
	if (XST_SUCCESS != Status) {
		//xil_printf("RPU: Error 0x%x in SelfSuspend\r\n", Status);
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
			//xil_printf("RPU: Error 0x%x in SetRequirement of 0x%x\r\n", Status, SramMemList[Idx]);
			goto done;
		}
	}

	for (Idx = 0; Idx < PM_ARRAY_SIZE(OtherDevList); Idx++) {
		Status = XPm_SetRequirement(OtherDevList[Idx], 0, 0, REQUEST_ACK_NO);
		if (XST_SUCCESS != Status) {
			//xil_printf("RPU: Error 0x%x in SetRequirement of 0x%x\r\n", Status, OtherDevList[Idx]);
			goto done;
		}
	}
    XPm_SetRequirement(NODE_IPI_APU, PM_CAP_CONTEXT, 0, REQUEST_ACK_NO);
done:
	return Status;
}

/* Receive IPI message with timeout */
#if 0
static int ReceiveIpiMessage(u32 expectedMsgType, u32 timeoutMs)
{
	u64 startTime = ReadTime();
	u64 timeoutUs = (u64)timeoutMs * 1000ULL;
	
	ipi_msg_received = 0;
	
	while (1) {
		if (ipi_msg_received) {
			if (received_msg_type == expectedMsgType) {
				DelayVal = received_param1;
				IterationCnt = received_param2;
				ipi_msg_received = 0;
				return XST_SUCCESS;
			} else {
				//xil_printf("RPU: Wrong type: 0x%08X\r\n", received_msg_type);
				ipi_msg_received = 0;
			}
		}
		
		if (CALCULATE_LATENCY(ReadTime() - startTime) >= timeoutUs) {
			return XST_FAILURE;
		}
		
		for (volatile u32 i = 0; i < 1000; i++);
	}
}
#endif
static int ReceiveIpiMessage(u32 expectedMsgType, u32 timeoutMs)
{
	u64 startTime = ReadTime();
	u64 timeoutUs = (u64)timeoutMs * 1000ULL;
	
	ipi_msg_received = 0;
	
	while (1) {
		if (ipi_msg_received) {
			if (received_msg_type == expectedMsgType) {
				DelayVal = received_param1;
				IterationCnt = received_param2;
				ipi_msg_received = 0;
				return XST_SUCCESS;
			} else {
				ipi_msg_received = 0;
			}
		}
		
		// **CHANGE: Only check timeout if timeoutMs > 0**
		if (timeoutMs > 0 && CALCULATE_LATENCY(ReadTime() - startTime) >= timeoutUs) {
			return XST_FAILURE;
		}
		
		for (volatile u32 i = 0; i < 1000; i++);
	}
}

static int SendIpiMessage(u32 msgType, u32 param1, u32 param2)
{
    // Write directly to the APU's Request Buffer (Base 0xFF990000)
    volatile u32 *ApuBuf = (volatile u32 *)0xFF990000U; 
    ApuBuf[0] = msgType;
    ApuBuf[1] = param1;
    ApuBuf[2] = param2;
    ApuBuf[3] = 0;

    // Trigger BOTH bits to ensure the APU hears it
    XIpiPsu_TriggerIpi(&IpiInst, 0x00000001U); // APU Standard
    XIpiPsu_TriggerIpi(&IpiInst, 0x00000100U); // The bit your OBS detected
    
    return XST_SUCCESS;
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
    //XPm_RequestNode(NODE_UART_1, PM_CAP_ACCESS, 0, REQUEST_ACK_NO);
    //xil_printf("RPU: INITIAL BOOT\r\n");
	BootStatus = XPm_GetBootStatus();
	if (PM_INITIAL_BOOT == BootStatus) 
    {
        //Status = PmSetConfig();
        //if (XST_SUCCESS != Status) {
	//		goto done;
    //    }
        //InitUart0WithDriver();
		/* Add delay to avoid print mix-up */
		Wait(3);
		//xil_printf("RPU: INITIAL BOOT\r\n");

		Status = InitApp();
		if (XST_SUCCESS != Status) {
			//xil_printf("RPU: Error 0x%x in InitApp\r\n", Status);
			goto done;
		}

		Status = XPm_RequestNode(NODE_IN_FPD, PM_CAP_ACCESS, 0, BLOCKING_ACK);
		if (XST_SUCCESS != Status) {
			//xil_printf("RPU: Error 0x%x in RequestNode of 0x%x\r\n", Status, NODE_IN_FPD);
			goto done;
		}

		//Status = XPm_RequestNode(NODE_UART_0, PM_CAP_ACCESS, 0, BLOCKING_ACK);
		//if (XST_SUCCESS != Status) {
			//xil_printf("RPU: Error 0x%x in RequestNode of 0x%x\r\n", Status, NODE_UART_0);
		//	goto done;
		//}

		Status = XPm_RegisterNotifier(&notifier);
		if (XST_SUCCESS != Status) {
			//xil_printf("RPU: Error 0x%x in RegisterNotifier\r\n", Status);
			goto done;
		}
	} else if (PM_RESUME == BootStatus) {
		//Status = XPm_RequestNode(NODE_UART_0, PM_CAP_ACCESS, 0, BLOCKING_ACK);
		//if (XST_SUCCESS != Status) {
			//xil_printf("RPU: Error 0x%x in RequestNode of 0x%x\r\n", Status, NODE_UART_0);
		//	goto done;
		//}

		//xil_printf("RPU: RESUMED\r\n");
		//PRINT_RPU_OUTLINE;
		//PRINT_RPU_RESUME;
		//PRINT_RPU_OUTLINE;
		
		/* Timer is already counting, just enable interrupts */
		Status = GicResume(&GicInst);
		if (XST_SUCCESS != Status) {
			//xil_printf("RPU: Error 0x%x in GicResume\r\n", Status);
			goto done;
		}

		Status = XPm_RequestNode(NODE_IN_FPD, PM_CAP_ACCESS, 0, BLOCKING_ACK);
		if (XST_SUCCESS != Status) {
			//xil_printf("RPU: Error 0x%x in RequestNode of 0x%x\r\n", Status, NODE_IN_FPD);
			goto done;
		}

		/* Waking up APU */
		tStart = ReadTime();
		Status = XPm_RequestWakeUp(WAKEUP_TARGET, 0, 0, BLOCKING_ACK);
		tEnd = ReadTime();

		/* Send SYNC_RPU_FINISH to APU */
		/* When APU0 wakes up and comes back to process "apu_script.sh",
		 * it would check SYNC_RPU_FINISH from RPU and drive the testpoint low
		 * for wakeup time measurement
		 */
        Wait(2);
        SendIpiMessage(IPI_MSG_RPU_FINISH, 0, 0);

		//SyncSetMask(SYNC_RPU_MASK, SYNC_RPU_FINISH);
		/* Add delay to avoid print mix-up */
		Wait(1);

		//if (XST_SUCCESS != Status) {
			//xil_printf("RPU: Error 0x%x in RequestWakeup of 0x%x\r\n", Status, WAKEUP_TARGET);
		//	goto done;
		//}
		WakeupLatency = CALCULATE_LATENCY(tEnd - tStart);
		//xil_printf("RPU: [ APU Wakeup Latency in micro seconds: %ld ]\r\n", WakeupLatency);

		/* Waiting SYNC_RPU_FINISH from APU */
		//SyncWaitForReady(SYNC_APU_READY);
		//SyncClearReady(SYNC_APU_MASK);
		//Wait(3);

		/* Power on PL */
		//SyncWaitForReady(SYNC_PL_UP);
		//SyncClearReady(SYNC_PL_UP);
		//xil_printf("RPU: Powering up PL\r\n");
		tStart = ReadTime();
		Status = XPm_RequestNode(NODE_PL, PM_CAP_ACCESS, 0, BLOCKING_ACK);
		tEnd = ReadTime();
		if (XST_SUCCESS != Status) {
			//xil_printf("RPU: Error 0x%x in RequestNode of 0x%x\r\n", Status, PL_NODE);
			goto done;
		}
		PlLatency = CALCULATE_LATENCY(tEnd - tStart);
		//xil_printf("RPU: [ PL ON Latency in micro seconds: %ld ]\r\n", PlLatency);

		/////////////////////////////////////////////////////////////////////
		// Testing PL Programming
		BitAddr = (UINTPTR)0x60000000U;
		//xil_printf("Loading Bitstream # for DDR location :0x%llx\n\r", BitAddr);
		//xil_printf("Trying to configure the PL ......\n\r");

		Status = XFpga_Initialize(&XFpgaInstance);
		if (Status != XST_SUCCESS) {
			goto done;
		}

		tStart = ReadTime();
		Status = XFpga_BitStream_Load(&XFpgaInstance, BitAddr, KeyAddr,
					      BITSTREAM_SIZE, XFPGA_FULLBIT_EN);
		tEnd = ReadTime();
		PlLatency = CALCULATE_LATENCY(tEnd - tStart);

		Status = PmInit(&GicInst, &IpiInst);
		if (Status != XST_SUCCESS) {
			//xil_printf("RPU: Error 0x%x in PmInit\r\n", Status);
			goto done;
		}

		Status = PmRtcInit(&GicInst, &RtcInstPtr);
		if (XST_SUCCESS != Status) {
			//xil_printf("RPU: Error 0x%x in PmRtcInit\r\n", Status);
			goto done;
		}

        // Re-register IPI and clear buffers
		XIpiPsu_InterruptEnable(&IpiInst, IPI_APU_CH_MASK);
		Status = IpiRegisterCallback(&IpiInst, IPI_APU_CH_MASK, CustomIpiHandler);
		if (Status != XST_SUCCESS) {
			//xil_printf("RPU: IPI re-register failed: 0x%x\r\n", Status);
			goto done;
		}
		
		// Re-configure non-cacheable
		//ConfigureIpiBufferMemory();
		
		// Clear buffers
        // **FIX: Only clear INCOMING buffer, not outgoing**
        //xil_printf("RPU: Clearing incoming IPI buffer only...\r\n");
        volatile u32 *InBuf = (volatile u32 *)RPU0_REQ_BUF_ADDR;
        InBuf[0] = InBuf[1] = InBuf[2] = InBuf[3] = 0;
		
		//xil_printf("RPU: Resume complete\r\n");
		
		Xil_DCacheFlushRange((INTPTR)InBuf, 16);
		//Xil_DCacheFlushRange((INTPTR)OutBuf, 16);
		XIpiPsu_ClearInterruptStatus(&IpiInst, IPI_APU_CH_MASK);
		
		//xil_printf("RPU: Resume complete\r\n");
		//
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////

	} else {
		//xil_printf("RPU: Invalid Boot Status\r\n");
		Status = XST_FAILURE;
		goto done;
	}

	/*****************
	 * Power down PL *
	 * ***************
	 */
	//xil_printf("RPU: Waiting PL_DOWN ........\r\n");

	/* Waiting for PL power down command from APU */
	//SyncWaitForReady(SYNC_PL_DOWN);
	//SyncClearReady(SYNC_PL_DOWN);

    Status = ReceiveIpiMessage(IPI_MSG_PL_DOWN, 1800000);
	if (Status != XST_SUCCESS) {
		//xil_printf("RPU: TIMEOUT\r\n");
		goto done;
	}
	//xil_printf("RPU: PL_DOWN received\r\n");


	/* Power down PL */
	//xil_printf("RPU: Powering down PL\r\n");
	tStart = ReadTime();
	Status = XPm_ForcePowerDown(PL_NODE, BLOCKING_ACK);
	tEnd = ReadTime();
	if (XST_SUCCESS != Status) {
		//xil_printf("RPU: Error 0x%x in ForcePowerDown of 0x%x\r\n", Status, PL_NODE);
		goto done;
	}

	PlLatency = CALCULATE_LATENCY(tEnd - tStart);
    Status = SendIpiMessage(IPI_MSG_RPU_SIGNAL, 0, 0);
    if (Status != XST_SUCCESS) 
    {
		//xil_printf("RPU: Failed to send ACK!\r\n");
		goto done;
	}
	//xil_printf("RPU: [ PL OFF Latency in micro seconds: %ld ]\r\n", PlLatency);

	/* Sync APU */
	//SyncSetMask(SYNC_RPU_MASK, SYNC_RPU_SIGNAL);

	/**********************
	 * Get APU parameters *
	 * ********************
	 */
    //xil_printf("RPU: Waiting for PARAMS...\r\n");
	Status = ReceiveIpiMessage(IPI_MSG_PARAMS, 1800000);
	if (Status != XST_SUCCESS) goto done;
	//xil_printf("RPU: PARAMS received - Delay:%d, Iter:%d\r\n", DelayVal, IterationCnt);
	
	if (DelayVal < 10U) DelayVal = 10U;
	if (IterationCnt > 5U) IterationCnt = 5U;
	
	SendIpiMessage(IPI_MSG_RPU_SIGNAL, 0, 0);
	
	//xil_printf("RPU: Waiting for APU_FINISH...\r\n");
	Status = ReceiveIpiMessage(IPI_MSG_APU_FINISH, 1800000);
	if (Status != XST_SUCCESS) goto done;
	//xil_printf("RPU: APU_FINISH received\r\n");

    //NODE_APU_0
	//Status = XPm_RequestSuspend(SUSPEND_TARGET, NON_BLOCKING_ACK, LATENCY_VAL, 0);
    Status = XPm_RequestSuspend(NODE_APU_0, NON_BLOCKING_ACK, LATENCY_VAL, 0); 
    // TODO: Need further check this call
    
	IpiWaitForAck();
	if (XST_SUCCESS != Status) {
		//xil_printf("RPU: Error 0x%x in RequestSuspend of 0x%x\r\n", Status, SUSPEND_TARGET);
		goto done;
	}
    Wait(1);
	Status = XPm_ReleaseNode(NODE_IN_FPD);
	if (Status != XST_SUCCESS) {
		//xil_printf("RPU: Error 0x%x in ReleaseNode of 0x%x\r\n", Status, NODE_IN_FPD);
		goto done;
	}

	SetRtcAlarm(&RtcInstPtr, 10);
	prepare_suspend();
	GicSuspend(&GicInst);
	//PRINT_RPU_OUTLINE;
	//PRINT_RPU_SUSPENDED_FPD_OFF;
	//PRINT_RPU_OUTLINE;
	//xil_printf("RPU:  ----------------\r\n");
	//xil_printf("RPU: | Waiting 10sec  |\r\n");
	//xil_printf("RPU:  ----------------\r\n");
	Wait(1);

    //Status = XPm_ReleaseNode(NODE_UART_0);
	//if (Status != XST_SUCCESS) {
		//xil_printf("RPU: Error 0x%x in ReleaseNode of NODE_UART_0\r\n", Status);
	//	goto done;
	//}
	XPm_ClientSuspendFinalize();

done:
    return Status;
}
