#include "xgpiops.h"
#include "xparameters.h"
#define PIN_TP1		24
#define PIN_TP2 	25

int PsGpioInit(XGpioPs *PsGpioInst);
int SetTestPoint(XGpioPs *PsGpioInstPtr, u32 Pin);
int ClrTestPoint(XGpioPs *PsGpioInstPtr, u32 Pin);
