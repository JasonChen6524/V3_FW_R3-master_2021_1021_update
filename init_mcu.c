/***************************************************************************//**
 * @file
 * @brief init_mcu.c
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#if defined(HAL_CONFIG)
#include "bsphalconfig.h"
#include "hal-config.h"
#else
#include "bspconfig.h"
#endif

#include "board_features.h"

#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_rtcc.h"

#include "bsp.h"

#include "init_mcu.h"


// Bit [19] in MODULEINFO is the HFXOCALVAL:
// 1=No factory cal, use default XO tunning value in FW
// 0=Factory Cal, use XO tunning value in DI 
#define DEVINFO_MODULEINFO_HFXOCALVAL_MASK  0x00080000UL
// Calibration value for HFXO CTUNE is at DEVINFO Offset 0x08
#define DEVINFO_MODULEINFO_CRYSTALOSCCALVAL  (*((uint16_t *) (uint32_t)(DEVINFO_BASE+0x8UL)))
// [15:9] : (LFXOTUNING) Calibration for LFXO TUNING
// [8:0]  : (HFXOCTUNE) Calibration for HFXO CTUNE
#define DEVINFO_HFXOCTUNE_MASK  0x01FFUL

#define set_HFXO_CTUNE(val)         \
  do {                              \
    hfxoInit.ctuneXoAna = (val);    \
    hfxoInit.ctuneXiAna = (val);    \
  } while (0)

static void initMcu_clocks(void);
static void initHFXO(void);

void initMcu(void)
{
  // Device errata
  CHIP_Init();

  // Set up clocks
  initMcu_clocks();


  RTCC_Init_TypeDef rtccInit = RTCC_INIT_DEFAULT;
  rtccInit.enable                = true;
  rtccInit.debugRun              = false;
  rtccInit.precntWrapOnCCV0      = false;
  rtccInit.cntWrapOnCCV1         = false;
  rtccInit.prescMode             = rtccCntTickPresc;
  rtccInit.presc                 = rtccCntPresc_1;
  RTCC_Init(&rtccInit);

#if defined(EMU_VSCALE_PRESENT)
  // Set up EM0, EM1 energy mode configuration
  EMU_EM01Init_TypeDef em01Init = EMU_EM01INIT_DEFAULT;
  EMU_EM01Init(&em01Init);
#endif // EMU_VSCALE_PRESENT
  // Set up EM2, EM3 energy mode configuration
  EMU_EM23Init_TypeDef em23init = EMU_EM23INIT_DEFAULT;
#if defined(EMU_VSCALE_PRESENT)
  em23init.vScaleEM23Voltage = emuVScaleEM23_LowPower;
#endif // EMU_VSCALE_PRESENT
  EMU_EM23Init(&em23init);


#if defined(RMU_PRESENT)
  // Set reset mode for sysreset back to DEFAULT (extended), this might have
  // been changed by the bootloader to FULL reset.
  RMU->CTRL = (RMU->CTRL & ~_RMU_CTRL_SYSRMODE_MASK) | RMU_CTRL_SYSRMODE_DEFAULT;
#endif

}

static void initMcu_clocks(void)
{
  // Initialize HFXO
  initHFXO();

  // Set system HFXO frequency
  SystemHFXOClockSet(BSP_CLK_HFXO_FREQ);

  CMU_HFRCODPLLBandSet(cmuHFRCODPLLFreq_80M0Hz);
  CMU_ClockSelectSet(cmuClock_SYSCLK, cmuSelect_HFRCODPLL);
//RBG
  //CMU_ClockSelectSet(cmuClock_RTCC , cmuSelect_LFRCO);                                                        

  //#warning "WARNING: Use the CMU_LFXOInit() function only if the LFXO hardware is actually present!"                      // Commented by Jason Chen, 2022.06.22
  // Initialize LFXO
  //CMU_LFXOInit_TypeDef lfxoInit = BSP_CLK_LFXO_INIT;
  //lfxoInit.capTune = BSP_CLK_LFXO_CTUNE;
  //CMU_LFXOInit(&lfxoInit);
  // Set system LFXO frequency
  //SystemLFXOClockSet(BSP_CLK_LFXO_FREQ);


}

static void initHFXO(void)
{
  // Initialize HFXO
  // Use BSP_CLK_HFXO_INIT as last result (4th)
  CMU_HFXOInit_TypeDef hfxoInit = BSP_CLK_HFXO_INIT;
  // if Factory Cal exists in DEVINFO then use it above all (1st)
  if (0 == (DEVINFO->MODULEINFO & DEVINFO_MODULEINFO_HFXOCALVAL_MASK)) {
#if defined(_SILICON_LABS_32B_SERIES_1)
    set_HFXO_CTUNE(DEVINFO_MODULEINFO_CRYSTALOSCCALVAL);
#elif defined(_SILICON_LABS_32B_SERIES_2)
    set_HFXO_CTUNE(DEVINFO->MODXOCAL & _DEVINFO_MODXOCAL_HFXOCTUNEXIANA_MASK);
#endif
  }
  // if User page has CTUNE from studio use that in 2nd place
#if (MFG_CTUNE_EN == 1)
  else if (MFG_CTUNE_VAL != 0xFFFF) {
    set_HFXO_CTUNE(MFG_CTUNE_VAL);
  }
#endif
  // 3rd option, get data from header defined for product/board
#if defined(BSP_CLK_HFXO_CTUNE) && BSP_CLK_HFXO_CTUNE >= 0
  else {
    set_HFXO_CTUNE(BSP_CLK_HFXO_CTUNE);
  }
#endif
  CMU_HFXOInit(&hfxoInit);
  // Set system HFXO frequency
  SystemHFXOClockSet(BSP_CLK_HFXO_FREQ);
}
