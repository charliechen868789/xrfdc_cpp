/*********************************************************************
 * 2017 Xilinx, Inc.
 *
 * CONTENT
 * For testing the power management framework, a configuration object
 * needs to be provided to the PFW in order to enable the power management.
 * The configuration object used for the testing is defined in this file.
 *********************************************************************/

#include <libeswpm/config.h>
#include <pm_api_sys.h>
#include <libeswpm/test_aux.h>
#include <xipipsu_hw.h>
#include <pm_cfg_obj.h>

#define CONFIG_IPI_BUFF	(XIPIPSU_MSG_RAM_BASE + 3 * XIPIPSU_BUFFER_OFFSET_GROUP)
#define CONFIG_OBJ_SINGLE	0x1U
#define CONFIG_OBJ_MULTI	0x2U

static const u32 PmConfigObjectMulti[] = {
};

static const u32 PmConfigObjectSingle[] = {
};
/**
 * PmSetConfig() - Set configuration object
 *
 * @return	Status of setting the configuration object (XST_SUCCESS or error
 * 		code)
 */
XStatus PmSetConfig(void)
{
    XStatus status;
	u32 val;

	val = pm_read(CONFIG_IPI_BUFF);

	if (CONFIG_OBJ_SINGLE == val)
		status = XPm_SetConfiguration((uintptr_t)PmConfigObjectSingle);
	else if (CONFIG_OBJ_MULTI == val)
		status = XPm_SetConfiguration((uintptr_t)PmConfigObjectMulti);
	else
		status = XPm_SetConfiguration((uintptr_t)XPm_ConfigObject);

    if (status == XST_SUCCESS) {
        pm_print("PMU Config Object accepted successfully!\n");
    } else {
        // This will print the Hex error code (e.g., 0x2000 for internal error)
        pm_print("PMU Config Object REJECTED! Error: 0x%x\n", status);
    }

    return status;
}
