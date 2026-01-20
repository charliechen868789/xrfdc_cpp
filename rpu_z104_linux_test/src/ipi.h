#ifndef IPI_H_
#define IPI_H_

#include <xstatus.h>
#include <xscugic.h>
#include <xipipsu.h>

typedef void (*IpiCallback)(XIpiPsu *const InstancePtr);

XStatus IpiInit(XScuGic *const GicInst, XIpiPsu *const InstancePtr);
XStatus IpiRegisterCallback(XIpiPsu *const IpiInst, const u32 SrcMask,
			    IpiCallback Callback);
void IpiWaitForAck(void);

#endif
