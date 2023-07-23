/* clocks.c              (c) 2015 Freescale Semiconductor, Inc.
 * Descriptions: Basic clock functions
 * 28 Sept 2015 Osvaldo Romero et al: Initial version
 */

#include "derivative.h"
#include "clocks.h"

void init_clks_FEI_48MHz (void) {      /* FLL Enabled with Internal clock */
  OSC_CR = 0x00;       /* (default value) */
                       /* OSCEN=0: OSC module disabled */
                       /* OSCSTEN=0: OSC clock disabled in Stop mode */
                       /* OSCOS=0: Ext clk source (don't care here) */
                       /* RANGE=0: Low Freq range of 32 KHz */
                       /* HGO=0: low power High Gan Osc mode (don't care here) */
  ICS_C2 = 0x20;       /* Use defaults until dividers configured (default) */
                       /* BDIV=1:  divided by 2  */
                       /* LP = 0:  FLL Not disabled in bypass mode  */
  ICS_C1 = 0x04;       /* Internal ref clock is FLL source (default)*/
                       /* CLKS=0: Output of FLL is selected to control bus freq */
                       /* RDIV=0: Ref divider = 1 since RANGE = 0 */
                       /* IREFS=0: Int Ref clock is selected */
                       /* IRCLKEN=0: ICSIRCLK is inactive */
                       /* IREFSTEN=0: Int ref clk is disabled in Stop mode */
  while ((ICS_S & ICS_S_LOCK_MASK) == 0); /* Wait for FLL to lock*/
  SIM_CLKDIV = 0x01100000; /* OUTDIV1 = 0; Core/sysclk is ICSOUTCLK div by 1 */
                           /* OUTDIV2 = 1  bus/flash is OUTDIV1/2 */
                           /* OUTDIV3 = 1; FTMs, PWT is ICSOUTCLK div by 2 */
  ICS_C2 = 0x00;           /* BDIV div by 1- increases bus/flash freq */
}

void init_clks_FEE_40MHz(void) {     /* FLL Enabled with External clock */
  OSC_CR = 0x96;       /* High range & gain; select osc */
                       /* OSCEN =1 ; OSC module enabled */
                       /* OSCSTEN = 0; OSC clock disabled in stop mode */
                       /* OSCOS = 1; OSC clcok source is selected */
                       /* RANGE = 1; High freq range of 4-24 MHz */
                       /* HGO = 1; High-gain mode */
  while ((OSC_CR & OSC_CR_OSCINIT_MASK) == 0); /* Wait until oscillator is ready*/
  ICS_C2 = 0x20;      /* BDIV div by 2; use default until dividers configured*/
                      /* LP = 0; FLL is not disabled in bypass mode */
  ICS_C1 = 0x18;      /* 8 Mhz ext ref clk/256 is source to FLL */
                      /* CLKS = 0; Output of FLL is selected (default) */
                      /* RDIV = 3; ref clk prescaled by 256 with RANGE=0 */
                      /* IREFS = 0; ext clk source selected */
                      /* IRCLKEN = 0; ICSIRCLK inactive */
                      /* IREFSTEN = 0; Int ref clk disabled in Stop mode */
  while ((ICS_S & ICS_S_IREFST_MASK) == 1);    /* Wait for external source selected */
  while ((ICS_S & ICS_S_LOCK_MASK) == 0);      /* Wait for FLL to lock */
  SIM_CLKDIV = 0x01100000; /* OUTDIV1 = 0; Core/sysclk is ICSOUTCLK div by 1 */
                           /* OUTDIV2 = 1  bus/flash is OUTDIV1/2 */
                           /* OUTDIV3 = 1; FTMs, PWT is ICSOUTCLK div by 2 */
  ICS_C2 = 0x00;           /* BDIV div by 1- increases bus/flash freq */
}

