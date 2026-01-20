#ifndef GIC_SETUP_H_
#define GIC_SETUP_H_

#include <xscugic.h>

s32 GicSetupInterruptSystem(XScuGic *GicInst);
s32 GicResume(XScuGic *const GicInst);
void GicSuspend(XScuGic *const GicInst);

#endif
