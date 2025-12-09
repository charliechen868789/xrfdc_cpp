#include <stdio.h>
#include "xrfdc.h"
#include "xrfclk.h"

#define SUCCESS 0
#define FAIL 1
int main()
{
    /**********************************************************************
     * 1. Init CLK104
     **********************************************************************/
    printf("Init CLK104...\n");

    if (XRFClk_Init(541) != SUCCESS) {
        printf("ERROR: CLK104 init failed\n");
        return -1;
    }
  return 0;
}
