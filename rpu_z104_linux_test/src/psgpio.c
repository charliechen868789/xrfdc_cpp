#include "psgpio.h"

#define PSGPIO_DEVICE_ID           XPAR_XGPIOPS_0_BASEADDR

int InitTestPoints(XGpioPs *PsGpioInstPtr)
{
	XGpioPs_SetDirectionPin(PsGpioInstPtr, PIN_TP1, 1); // output
	XGpioPs_SetOutputEnablePin(PsGpioInstPtr, PIN_TP1, 1);

	XGpioPs_SetDirectionPin(PsGpioInstPtr, PIN_TP2, 1); // output
	XGpioPs_SetOutputEnablePin(PsGpioInstPtr, PIN_TP2, 1);

	return XST_SUCCESS;
}

int PsGpioInit(XGpioPs *PsGpioInst)
{
	int Status = XST_FAILURE;
	XGpioPs_Config *Config;

	Config = XGpioPs_LookupConfig(PSGPIO_DEVICE_ID);
	if (NULL == Config) {
		xil_printf("RPU: %s ERROR in getting CfgPtr\n", __func__);
		goto done;
	}

	Status = XGpioPs_CfgInitialize(PsGpioInst, Config, Config->BaseAddr);
	if (XST_SUCCESS != Status) {
		xil_printf("RPU: %s ERROR in_CfgInitialize\n", __func__);
		goto done;
	}

	InitTestPoints(PsGpioInst);

done:
	return Status;

}

int SetTestPoint(XGpioPs *PsGpioInstPtr, u32 Pin)
{
	XGpioPs_WritePin(PsGpioInstPtr, Pin, 1);
	return XST_SUCCESS;
}

int ClrTestPoint(XGpioPs *PsGpioInstPtr, u32 Pin)
{
	XGpioPs_WritePin(PsGpioInstPtr, Pin, 0);
	return XST_SUCCESS;
}

