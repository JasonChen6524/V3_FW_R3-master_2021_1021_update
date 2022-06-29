/***************************************************************************//**
 * @file app.c
 * @brief SPP-over-BLE example
 *
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

/* Bluetooth stack headers */
#include "bg_types.h"
#include "native_gecko.h"
#include "gatt_db.h"

#include "app.h"
#include "spp_utils.h"

/* Main application */
void appMain(gecko_configuration_t *pconfig)
{
#if DISABLE_SLEEP > 0
  pconfig->sleep.flags = 0;
#endif

  /* Initialize debug prints. Note: debug prints are off by default. See DEBUG_LEVEL in app.h */
  initLog();

  /* Initialize stack */
  gecko_init(pconfig);

#if WATCHDOG_ENABLE                                                                                                            //Added by Jason Chen, 2022.06.29
  initWDOG();                                                                                                                  //Added by Jason Chen, 2022.06.29
#endif                                                                                                                         //Added by Jason Chen, 2022.06.29

  /*  jump to SPP program main loop */
  spp_main();
}

/**************************************************************************//**
 * @brief Watchdog initialization
 *****************************************************************************/
#include "em_wdog.h"
#include "em_cmu.h"
#if 0
void initWDOG(void)
{
  // Enable clock for the WDOG module; has no effect on xG21
  CMU_ClockEnable(cmuClock_WDOG0, true);

  // Watchdog Initialize settings
  WDOG_Init_TypeDef wdogInit = WDOG_INIT_DEFAULT;
  CMU_ClockSelectSet(cmuClock_WDOG0, cmuSelect_ULFRCO); /* ULFRCO as clock source */
  wdogInit.debugRun = true;
  wdogInit.em3Run = true;
  wdogInit.perSel = wdogPeriod_2k; // 2049 clock cycles of a 1kHz clock  ~2 seconds period

  // Initializing watchdog with chosen settings
  WDOG_Init(&wdogInit);
}
#else
void initWDOG(void)
{

  // Enable clock for the WDOG module; has no effect on xG21
  CMU_ClockEnable(cmuClock_WDOG0, true);

  // Watchdog Initialize settings
  WDOG_Init_TypeDef wdogInit = WDOG_INIT_DEFAULT;
  CMU_ClockSelectSet(cmuClock_WDOG0, cmuSelect_ULFRCO); /* ULFRCO 1000Hz as clock source */
  wdogInit.enable   = true;//false;
  wdogInit.debugRun = true;
  wdogInit.em2Run   = true;
//wdogInit.em3Run   = true;
  wdogInit.em4Block = false;
  wdogInit.perSel   = wdogPeriod_8k; //~4.2 seconds // 2049 clock cycles of a 1kHz clock  ~2 seconds period

  // Initializing watchdog with chosen settings
  WDOGn_Init(DEFAULT_WDOG, &wdogInit);                                          // WDOG_Init(&wdogInit);
}
#endif
