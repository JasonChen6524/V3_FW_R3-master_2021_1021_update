/*******************************************************************************
 * Copyright (C) 2018 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *******************************************************************************
 */


/**********************************************************************************
 *
 *  Desc: Example Code to get algorithm estimation results of Blood Pressure from sensor hub and display on Command Console
 *        Example starts in Example Measurement Mode.
 *
 *        Example,
 *
 *        1. In Command Mode
 *                   Executes MAx3010xBPT Command Table commands from user.
 *
 *        2. In example User Mode
 *                   1. Sets Coefficients for calibration med, rest, time, spo2 coefs, sys_bp , dia_bp
 *                   2. Starts calibration
 *                   	        1. initialize algorithm config struct enabled
 *                              2. enable data type to both raw sensor and algorithm data
 *                              3. set fifo threshold for mfio event frequency
 *                              5. enable sensor to acquire ppg data
 *                              6. disables AGC algorithm
 *                              7. enable BPT algorithm in calibration mode
 *
 *                   3. get BPT calibration for USER
 *                   4. Sets Coefficients for estimation med, rest, time, spo2 coefs
 *                   5. Sets calibration data before each estimation measurement
 *                   6. Starts BPT Estimation for USER
 *                   	        1. initialize algorithm config struct enabled
 *                              2. enable data type to both raw sensor and algorithm data
 *                              3. set fifo threshold for mfio event frequency
 *                              5. enable sensor to acquire ppg data
 *                              6. enables/disables AGC algorithm
 *                              7. enable BPT algorithm in estimation mode
 *
 *                  7. Example calls SH_Max3010x_data_report_execute() which
 *				                1. calls SH API's sh_ss_execute_once() function which:
 *                                 writes sensor hub's report fifo content (sensor/algorithm data samples) to a buffer(1).
 *                              2. Parses buffer(1) data to extract numeric sensor and algorithm samples according to enabled algorithms.
 * 			                      look:  bpt_data_rx() , max3010x_data_rx() and sample structs defined within SH_Max3010x_BareMetal.cpp
 *
 *        3. In Booloader Mode
 *                 Executes Bootloader Command Table commands from user. Look at Bootloader Python Script for PC side example.
 *
 *
 *
 *
 ***********************************************************************************/

#include "Global.h"
#include "SHComm.h"
#include "SH_Max3010x_BareMetal.h"

#include <stdbool.h>
//#include "app.h"
#include "V3.h"
#include "sl_sleeptimer.h"
#include "I2C.h"

#define WAIT_SENSORHUB_STABLE_BOOTUP_MS  ((uint32_t)2000)

#define CALIBRATION_TIMEOUT_SEC          ((uint32_t)100)

enum{
	HOSTMODEAPPLICATION = 0,
	HOSTMODEBOOTLOADER  = 1
};

int hostMode = HOSTMODEAPPLICATION;
demo_appstate_t appState  = ST_COMMAND_MODE;
//static demo_appstate_t appPrevState;     Jason

extern uint8_t  bptMesurementProgress;

struct BIO_SENSOR_SETTING biosensor_setting  = {
   //0,
   //0,
	 {20201202, 134908},
   //{20200828, 164834},
   //{20201202, 134934},
   //{120, 122, 125},
   //{80, 81, 82},
     {0, 120, 80},
   //{1, 119, 79},
   //{2, 121, 81},
     {0x00026F60,  0xFFCB1D12,  0x00ABF37B},  // endianess needs to be switched
//   {0x606F0200,  0x121DCBFF,  0x7BF3AB00},
	 {0}
};

//static uint8_t hostOperatingMode = HOSTMODEAPPLICATION;

sl_sleeptimer_timer_handle_t bpt_timer;
static uint16_t bpt_second_count = 0;
void bpt_timer_callback(sl_sleeptimer_timer_handle_t *handle, void *data)
{
  //Code executed when the timer expire.
	bpt_second_count++;
  //printLog("BPT Timer: %d\r\n", bpt_second_count);
}

uint16_t calibrationTimer_read(void)
{
	return bpt_second_count;
}

void calibrationTimer_reset(void)
{
	bpt_second_count = 0;
}

void calibrationTimer_start(void)
{
	sl_status_t status;

	uint32_t timer_timeout = sl_sleeptimer_ms_to_tick(1000);                // 1 second timer
	bpt_second_count = 0;

	status = sl_sleeptimer_start_periodic_timer(&bpt_timer,
	                                            timer_timeout,
	                                            bpt_timer_callback,
	                                            (void *)NULL,
	                                            0,
	                                            0);
	if(status != SL_STATUS_OK) {
	    printLog("Timer not started.\r\n");
	}
}
void calibrationTimer_stop(void)
{
	sl_sleeptimer_stop_timer(&bpt_timer);
}


const char *fw_version;
uint32_t count_tick_last = 0;

void bpt_init(void)
{
	wait_ms(WAIT_SENSORHUB_STABLE_BOOTUP_MS);
	sh_init_hwcomm_interface();
	sh_disable_irq_mfioevent();
	sh_clear_mfio_event_flag();
	sh_enable_irq_mfioevent();

	fw_version = sh_get_hub_fw_version();
	fw_version = sh_get_hub_algo_version();
}

int delay_ms_count = 0;
uint8_t state_flag = 0;
static uint8_t reset_count = 0;
//extern struct v3_status v3status;
void bpt_main(void)
{
	static int measurement_count = 0;
	static int status_count02;
	static int idx02 = 0;
	static uint8_t cal_result[CAL_DATA_SIZE_BYTES + STATUS_OFFSET_BYTE ] = {0};

    //while(1)
    {
		switch(appState)
		{
			case ST_COMMAND_MODE:
			{
		      //appState = ST_EXAMPLEUSER_CALIBRATION_SETTINGS;
				appState = ST_EXAMPLEUSER_BIOSENSOR_RESET;
				printLog("\r\nBioSensor reseting...............                            Line:%d\r\n", __LINE__);
			}
			break;

			case ST_EXAMPLEUSER_BIOSENSOR_RESET:
			{
				ExpSetPins(EN5V_HIGH|RESET_LOW|MFIO_HIGH);
				reset_count = 0;
				appState = ST_EXAMPLEUSER_BIOSENSOR_RESET02;
			}
			break;

			case ST_EXAMPLEUSER_BIOSENSOR_RESET02:
			{
				reset_count++;
				if(reset_count > 2)
				{
					ExpSetPins(EN5V_HIGH|RESET_HIGH|MFIO_HIGH);
					appState = ST_EXAMPLEUSER_BIOSENSOR_RESET03;
				}
			}
			break;

			case ST_EXAMPLEUSER_BIOSENSOR_RESET03:
			{
				reset_count++;
				if(reset_count > 15)
				{
					appState = ST_EXAMPLEUSER_CALIBRATION_SETTINGS;
					printLog("Starting Calibrating...............,                         Line:%d\r\n\r\n", __LINE__);
				}
			}
			break;

			case ST_EXAMPLEUSER_CALIBRATION_SETTINGS:
			{
				int status = 0;

				//wait_ms(1000);  // This delay may be reduced to 0 if the startup time of the host is slower than the startup of the MAX32664
				delay_ms_count++;
				if(delay_ms_count < 11)
					break;
				else
					delay_ms_count = 0;

				printLog("BIO SENSOR IS STARTED , SETTING UP FOR CALIBRATION ........, Line:%d\r\n", __LINE__);

				uint8_t CalCoef[12] = { (uint8_t)((biosensor_setting.general_const_spo2_coeffients[0] >> (3*8)) & 0xFF),
						                (uint8_t)((biosensor_setting.general_const_spo2_coeffients[0] >> (2*8)) & 0xFF),
						                (uint8_t)((biosensor_setting.general_const_spo2_coeffients[0] >> (1*8)) & 0xFF),
										(uint8_t)((biosensor_setting.general_const_spo2_coeffients[0] >> (0*8)) & 0xFF), // A
										(uint8_t)((biosensor_setting.general_const_spo2_coeffients[1] >> (3*8)) & 0xFF),
										(uint8_t)((biosensor_setting.general_const_spo2_coeffients[1] >> (2*8)) & 0xFF),
										(uint8_t)((biosensor_setting.general_const_spo2_coeffients[1] >> (1*8)) & 0xFF),
										(uint8_t)((biosensor_setting.general_const_spo2_coeffients[1] >> (0*8)) & 0xFF), // B
										(uint8_t)((biosensor_setting.general_const_spo2_coeffients[2] >> (3*8)) & 0xFF),
										(uint8_t)((biosensor_setting.general_const_spo2_coeffients[2] >> (2*8)) & 0xFF),
										(uint8_t)((biosensor_setting.general_const_spo2_coeffients[2] >> (1*8)) & 0xFF),
										(uint8_t)((biosensor_setting.general_const_spo2_coeffients[2] >> (0*8)) & 0xFF)};// C

				//set SPo2 coeffcients based on final form factor and hypoxia lab data.
				//status = sh_set_algo_cfg_extendedwait(SH_ALGOIDX_BPT, SS_CFGIDX_BP_SPO2_COEFS, (uint8_t*) &biosensor_setting.general_const_spo2_coeffients[0], sizeof(biosensor_setting.general_const_spo2_coeffients), 5);
				status = sh_set_algo_cfg_extendedwait(SH_ALGOIDX_BPT, SS_CFGIDX_BP_SPO2_COEFS, CalCoef, sizeof(CalCoef), 5);
				//printLog(" \r\n Set algo SPO2 cfg: %d \r\n" , status)5			 	    	//set calibration date and time. Alias of action taken by Command "set_cfg bpt date_time 190308 095829"
				status += sh_set_algo_cfg_extendedwait(SH_ALGOIDX_BPT, SS_CFGIDX_BP_EST_DATE, (uint8_t*) &biosensor_setting.date_time_calib_instant[0], sizeof(biosensor_setting.date_time_calib_instant), 5);
				//set example cal_index, systolic, diastolic values. Alias of action taken by Command "set_cfg bpt sys_dia 0 120 80"
				status  += sh_set_algo_cfg_extendedwait(SH_ALGOIDX_BPT, SS_CFGIDX_BP_SYS_DIA, &biosensor_setting.user_sys_dia_0_initials[0], 3 , 5 );

				//wait_ms(100);
				if( status == 0)
				{
					//start BPT calibration
					SH_Max3010x_stop(0);
					status = SH_Max3010x_default_init(bptExecModeCalibration , ENABLE_AGC_USAGE);
					wait_ms(2);
					if(status == 0)
					{
						appState = ST_EXAMPLEUSER_CALIBRATION_STARTING;//ST_EXAMPLEUSER_CALIBRATION_WAIT_COMPLETE;
						//calibrationTimer_reset();
						//calibrationTimer_start();
						printLog("EXAMPLE CALIBRATION CONGIGURATION/INITIALIZATION.........OK, Line:%d\r\n", __LINE__);
					}else
					{
						appState = ST_EXAMPLEUSER_FAILURE;
						printLog("EXAMPLE CALIBRATION INITIALIZATION FAILURE..., Line:%d\r\n", __LINE__);
					}
				}
				else
				{
					appState = ST_EXAMPLEUSER_FAILURE;
					printLog("EXAMPLE CALIBRATION CONGIGURATION ERROR, Line:%d...,\r\n", __LINE__);
				}

			}
			break;

			case ST_EXAMPLEUSER_CALIBRATION_STARTING:                                        // Added by Jason, 2021.03.02
			{
				int status = 0;
				status = SH_Max3010x_algo_start( bptExecModeCalibration);
				if(status == 0x00)
				{
					calibrationTimer_reset();
					calibrationTimer_start();
					appState = ST_EXAMPLEUSER_CALIBRATION_WAIT_COMPLETE;
					printLog("EXAMPLE CALIBRATION STARTING.............................OK, Line:%d\r\n", __LINE__);
				}
				else
				{
					appState = ST_EXAMPLEUSER_FAILURE;
				}
			}
			break;

			case ST_EXAMPLEUSER_CALIBRATION_WAIT_COMPLETE:
			{
				static uint8_t bpt_status_count;
				static uint8_t bpt_status_count02;

				uint32_t count_tick = calibrationTimer_read();
				if( count_tick <= 60 )
				{
					if( bptMesurementProgress == BPT_CALIBRATION_PROCESSDONE_VAL)
					{
						//Stop calibration measurement
						SH_Max3010x_stop(0);
						calibrationTimer_stop();
				        appState = ST_EXAMPLEUSER_DELAY_COUNT;//ST_EXAMPLEUSER_ESTIMATION_SETTINGS;
					  //appState = ST_EXAMPLEUSER_CALIBRATION_RESULT; status_count02 = 0x10;
						state_flag = 2;
						delay_ms_count = 0;
						//curren_tick1 = sl_sleeptimer_get_tick_count();
						printLog("CALIBRATION Progress:%3d, sensor status:%2X, tick_count:%3d,  Line:%d\r\n\r\n", bptMesurementProgress, bpt_status, (int)count_tick, __LINE__);
						bptMesurementProgress = 0;
						measurement_count     = 0;
						bpt_status_count      = 0;
						bpt_status_count02    = 0;
					}
					else
					{
						static uint32_t last_tick_count = 0;
						if(last_tick_count != count_tick)
						{
							printLog("CALIBRATION Progress:%3d, sensor status:%2X, tick_count:%3d,  Line:%d\r\n", bptMesurementProgress, bpt_status, (int)count_tick, __LINE__);

							if(bpt_status == 0) bpt_status_count++;
							else    			bpt_status_count = 0;
							if(bpt_status_count > 20)
							{
								bpt_status_count = 0;
								appState =  ST_EXAMPLEUSER_FAILURE;                                                                //Added by Jason Chen, 2022.07.22
							  //measurement_count++;                                                                               //Added by Jason Chen, 2022.07.22
								break;
							}

							if(bpt_status == 4) bpt_status_count02++;
							else    			bpt_status_count02 = 0;
							if(bpt_status_count02 > 15)                                                                           //Added by Jason Chen, 2022.07.21
							{                                                                                                      //Added by Jason Chen, 2022.07.21
								bpt_status_count02 = 0;
								appState =  ST_EXAMPLEUSER_FAILURE;                                                                //Added by Jason Chen, 2022.07.21
							  //measurement_count++;                                                                               //Added by Jason Chen, 2022.07.21
								break;
							}                                                                                                      //Added by Jason Chen, 2022.07.21
						}
						last_tick_count = count_tick;
						appState =  ST_EXAMPLEUSER_CALIBRATION_WAIT_COMPLETE;
					}
				}
				else
				{
					if(bptMesurementProgress > bptMesurementProgressLast)                                                      //Added by Jason Chen, 2022.07.21
					{                                                                                                          //Added by Jason Chen, 2022.07.21
						calibrationTimer_reset();							                                                   //Added by Jason Chen, 2022.07.21
					}                                                                                                          //Added by Jason Chen, 2022.07.21
					else
					{
						if((bpt_status == 0x17)||(bpt_status == 0x18))
						{
							bpt_status_count   = 0;
							bpt_status_count02 = 0;
							calibrationTimer_reset();
							break;
						}

						appState =  ST_EXAMPLEUSER_TIMEOUT;
						measurement_count++;
						printLog("\r\nTIMEOUT CONDITION OCCURED in CALIBRATION,         count = %d, Line:%d\r\n", measurement_count, __LINE__);
					}
				}
				bptMesurementProgressLast = bptMesurementProgress;
			}
			break;

			case ST_EXAMPLEUSER_CALIBRATION_RESULT:
			{
				if(status_count02 == 0x10)
				{
					status_count02 = sh_get_algo_cfg_extendedwait(SH_ALGOIDX_BPT, SS_CFGIDX_BP_CAL_DATA, &cal_result[0], 513/*sizeof(cal_result)*/, 30);
					//if(status_count02 != 0)
					//{
					//	appState = ST_COMMAND_MODE;
					//	printLog("Get Calibration Results Error......%d\r\n", cal_result[0]);
					//	break;
					//}
					//else
					{
						status_count02 = 0x11;
						idx02          = 0;
						printLog("Get Calibration Results, index = %d, read_status = %d\r\n", cal_result[0], status_count02);
					}
				}

				if(status_count02 == 0x11)
				{
					if( idx02 < CAL_DATA_SIZE_BYTES)//sizeof(cal_result) - 1 )
					{
						if(cal_result[idx02+1] == 0x00)          printLog("00");
						else if((cal_result[idx02+1]&0xF0) == 0) printLog("0%X", cal_result[idx02+1]);
						else                                     printLog("%2X", cal_result[idx02+1]);

						idx02++;
						if(idx02%32 == 0) printLog("%d\r\n", idx02);
						//wait_ms(10);
					}
					else
					{
						printLog("%d\r\n", idx02);
						appState = ST_COMMAND_MODE;
						idx02 = 0;
						status_count02 = 0;
					}
				}
			}
			break;

			case ST_EXAMPLEUSER_ESTIMATION_SETTINGS:
			{
				//wait_ms(2000);
				int status = 0;
				uint8_t cal_result[CAL_DATA_SIZE_BYTES + STATUS_OFFSET_BYTE ] = {0};

			  //get calibration results. Alias of action taken by Command "get_cfg bpt cal_result" which prints calibration data to Console
				status += sh_get_algo_cfg_extendedwait(SH_ALGOIDX_BPT, SS_CFGIDX_BP_CAL_DATA, &cal_result[0], sizeof(cal_result), 30);
                #if 1
				printLog("Get Calibration Results, index = %d\r\n", cal_result[0]);
				int idx = 0; /*skip status byte*/
				while( idx < sizeof(cal_result) - 1 )
				{
					if(cal_result[idx+1] == 0x00)          printLog("00");
					else if((cal_result[idx+1]&0xF0) == 0) printLog("0%X", cal_result[idx+1]);
					else                                   printLog("%2X", cal_result[idx+1]);
					idx++;
					if(idx%32 == 0) printLog("\r\n");
				}
				printLog("\r\n");
                #else
					printLog("get calibration results.\r\n");
                #endif
			  //Copy as user calibration data
				memcpy( &biosensor_setting.cal_data[0] , &cal_result[STATUS_OFFSET_BYTE] , CAL_DATA_SIZE_BYTES );
				status += sh_set_algo_cfg_extendedwait(SH_ALGOIDX_BPT, SS_CFGIDX_BP_CAL_DATA, &cal_result[STATUS_OFFSET_BYTE], CAL_DATA_SIZE_BYTES, 100 );
			  //status = ssx_set_algo_cfg(SH_ALGOIDX_BPT, SS_CFGIDX_BP_CAL_DATA, &example_user.cal_data[0], CAL_DATA_SIZE_BYTES);

				 uint8_t CalCoef[12] = { (uint8_t)((biosensor_setting.general_const_spo2_coeffients[0] >> (3*8)) & 0xFF),
						                 (uint8_t)((biosensor_setting.general_const_spo2_coeffients[0] >> (2*8)) & 0xFF),
										 (uint8_t)((biosensor_setting.general_const_spo2_coeffients[0] >> (1*8)) & 0xFF),
										 (uint8_t)((biosensor_setting.general_const_spo2_coeffients[0] >> (0*8)) & 0xFF), // A
										 (uint8_t)((biosensor_setting.general_const_spo2_coeffients[1] >> (3*8)) & 0xFF),
										 (uint8_t)((biosensor_setting.general_const_spo2_coeffients[1] >> (2*8)) & 0xFF),
										 (uint8_t)((biosensor_setting.general_const_spo2_coeffients[1] >> (1*8)) & 0xFF),
										 (uint8_t)((biosensor_setting.general_const_spo2_coeffients[1] >> (0*8)) & 0xFF), // B
										 (uint8_t)((biosensor_setting.general_const_spo2_coeffients[2] >> (3*8)) & 0xFF),
										 (uint8_t)((biosensor_setting.general_const_spo2_coeffients[2] >> (2*8)) & 0xFF),
										 (uint8_t)((biosensor_setting.general_const_spo2_coeffients[2] >> (1*8)) & 0xFF),
										 (uint8_t)((biosensor_setting.general_const_spo2_coeffients[2] >> (0*8)) & 0xFF)};// C

			   //status = sh_set_algo_cfg_extendedwait(SH_ALGOIDX_BPT, SS_CFGIDX_BP_SPO2_COEFS, (uint8_t*) &biosensor_setting.general_const_spo2_coeffients[0], sizeof(biosensor_setting.general_const_spo2_coeffients), 5);
				 status = sh_set_algo_cfg_extendedwait(SH_ALGOIDX_BPT, SS_CFGIDX_BP_SPO2_COEFS, CalCoef, sizeof(CalCoef), 5);

				if( status == 0x00 )
				{
					appState =  ST_EXAMPLEUSER_DELAY_COUNT;//ST_EXAMPLEUSER_ESTIMATION_SETTINGS;
					state_flag = 4;
					delay_ms_count = 0;
					//curren_tick1 = sl_sleeptimer_get_tick_count();
				}
				else
					appState = ST_EXAMPLEUSER_FAILURE;
			}
			break;

			case ST_EXAMPLEUSER_ESTIMATION_SETTINGS2:
			{
			  //wait_ms(1000);
			  //if( status == 0x00 )
				{
#if 0
				  //start BPT estimation , AGC CAN BE DISABLED
					status = SH_Max3010x_default_init( bptExecModeEstimation , ENABLE_AGC_USAGE);
					if(status == 0x00)
					{
						calibrationTimer_reset();
						calibrationTimer_start();
						appState = ST_EXAMPLEUSER_ESTIMATION_MEASUREMENT;
						printLog("EXAMPLE ESTIMATION MEASUREMENT STARTED \r\n");
					}
					else
					{
						appState = ST_EXAMPLEUSER_FAILURE;
					}
#else
					appState = ST_EXAMPLEUSER_DELAY_COUNT;  //ST_EXAMPLEUSER_ESTIMATION_STARTING;
					delay_ms_count = 0;
					state_flag = 3;
					//curren_tick1 = sl_sleeptimer_get_tick_count();
#endif

				}
				//else
				//{
				//	appState = ST_EXAMPLEUSER_FAILURE;
				//}
				measurement_count = 0;
			}
			break;

			case ST_EXAMPLEUSER_ESTIMATION_STARTING:
			{
				int status = 0;
				//measurement_count = 0;
				status = SH_Max3010x_default_init( bptExecModeEstimation , ENABLE_AGC_USAGE);
				if(status == 0x00)
				{
#if 0
					calibrationTimer_reset();
					calibrationTimer_start();
					appState = ST_EXAMPLEUSER_ESTIMATION_MEASUREMENT;
					printLog("EXAMPLE ESTIMATION MEASUREMENT STARTED Line:%d\r\n", __LINE__);
#else
			      //appState = ST_EXAMPLEUSER_ESTIMATION_RE_STARTING;
			        appState = ST_EXAMPLEUSER_ESTIMATION_SENSOR_ENABLE;
					printLog("EXAMPLEUSER_ESTIMATION_INITIALIZATION......OK,               Line:%d\r\n", __LINE__);
#endif
				}
				else
				{
					appState = ST_EXAMPLEUSER_FAILURE;
				}
			}
			break;

			case ST_EXAMPLEUSER_ESTIMATION_RE_STARTING:                                           // Added by Jason, 2021.03.02
			{
				int status = 0;
				status = SH_Max3010x_algo_start( bptExecModeEstimation);
				if(status == 0x00)
				{
					calibrationTimer_reset();
					calibrationTimer_stop();
					calibrationTimer_start();
					appState = ST_EXAMPLEUSER_ESTIMATION_MEASUREMENT;
					printLog("EXAMPLE ESTIMATION MEASUREMENT STARTED Line:%d\r\n", __LINE__);
				}
				else
				{
					appState = ST_EXAMPLEUSER_FAILURE;
				}
			}
			break;

			case ST_EXAMPLEUSER_ESTIMATION_SENSOR_ENABLE:                                           // Added by Jason, 2021.03.04
			{
				int status = 0;
				status = SH_Max3010x_sensor_enable();
				if(status == 0x00)
				{
					calibrationTimer_reset();
					calibrationTimer_stop();
					calibrationTimer_start();
					appState = ST_EXAMPLEUSER_ESTIMATION_SENSOR_ENABLE_STATUS;
					printLog("EXAMPLE ESTIMATION MEASUREMENT SENSOR ENABLE,                Line:%d\r\n", __LINE__);
				}
				else
				{
					appState = ST_EXAMPLEUSER_FAILURE;
				}
			}
			break;

			case ST_EXAMPLEUSER_ESTIMATION_SENSOR_ENABLE_STATUS:                                           // Added by Jason, 2021.03.04
			{
				int status = 0;
				status = SH_Max3010x_sensor_enable_status();
				if(status == 0x00)
				{
					calibrationTimer_reset();
					calibrationTimer_stop();
					calibrationTimer_start();
					appState = ST_EXAMPLEUSER_ESTIMATION_RE_STARTING02;
					printLog("EXAMPLE ESTIMATION MEASUREMENT SENSOR ENABLE....OK,          Line:%d\r\n", __LINE__);
				}
				else
				{
					appState = ST_EXAMPLEUSER_FAILURE;
				}
			}
			break;


			case ST_EXAMPLEUSER_ESTIMATION_RE_STARTING02:                                         // Added by Jason, 2021.03.04
			{
				int status = 0;
				status = SH_Max3010x_algo_start_02( bptExecModeEstimation);
				if(status == 0x00)
				{
					//calibrationTimer_reset();
					//calibrationTimer_start();
#if 0
					appState = ST_EXAMPLEUSER_ESTIMATION_RE_STARTING03;
#else
					// more delay
					state_flag = 6;
					appState = ST_EXAMPLEUSER_DELAY_COUNT;
#endif
					printLog("ST_EXAMPLEUSER_ESTIMATION_RE_STARTING02,                     Line:%d\r\n", __LINE__);
				}
				else
				{
					appState = ST_EXAMPLEUSER_FAILURE;
				}
			}
			break;

			case ST_EXAMPLEUSER_ESTIMATION_RE_STARTING03:                                         // Added by Jason, 2021.03.04
			{
				int status = 0;
				status = SH_Max3010x_algo_start_02_status( bptExecModeEstimation);
				if(status == 0x00)
				{
					calibrationTimer_reset();
					calibrationTimer_stop();
					calibrationTimer_start();
					appState = ST_EXAMPLEUSER_ESTIMATION_MEASUREMENT;
					printLog("EXAMPLE ESTIMATION MEASUREMENT RE_STARTED,                   Line:%d\r\n", __LINE__);
				}
				else
				{
					appState = ST_EXAMPLEUSER_FAILURE;
				}
			}
			break;

			case ST_EXAMPLEUSER_ESTIMATION_MEASUREMENT:
			{
				static uint8_t bpt_status_count;
				static uint8_t bpt_status_count02;

				uint32_t count_tick = calibrationTimer_read();
				if( bptMesurementProgress == BPT_CALIBRATION_PROCESSDONE_VAL)
				{
					//curren_tick1 = sl_sleeptimer_get_tick_count();
					measurement_count = 0;
					calibrationTimer_stop();
#if 0
				    SH_Max3010x_stop(0);
					delay_ms_count = 0;
					state_flag = 2;                            //3
#else
					delay_ms_count = 0;
					state_flag = 5;                            //3
#endif
					appState = ST_EXAMPLEUSER_DELAY_COUNT;
					printLog("ESTIMATION Progress:%3d, sensor status:%2X, tick_count:%3d,   Line:%d\r\n\r\n", bptMesurementProgress, bpt_status, (int)count_tick, __LINE__);
					bpt_status_count   = 0;
					bpt_status_count02 = 0;
				}
				else
				{
					//count_tick = calibrationTimer_read();
					if( count_tick > 60 )
					{
						if(bptMesurementProgress > bptMesurementProgressLast)                                                  //Added by Jason Chen, 2022.07.21
						{                                                                                                      //Added by Jason Chen, 2022.07.21
							calibrationTimer_reset();							                                               //Added by Jason Chen, 2022.07.21
						}                                                                                                      //Added by Jason Chen, 2022.07.21
						else                                                                                                   //Added by Jason Chen, 2022.07.21
						{
							appState =  ST_EXAMPLEUSER_ESTIMATION_RE_MEASUREMENT;                                              //ST_EXAMPLEUSER_TIMEOUTaaaa;
							break;
						}
					}
					else
					{
						static uint32_t last_tick = 0;
						if(last_tick != count_tick)
						{
							printLog("ESTIMATION Progress:%3d, sensor status:%2X, tick_count:%3d,   Line:%d\r\n", bptMesurementProgress, bpt_status, (int)count_tick, __LINE__);
							if(bpt_status == 0) bpt_status_count++;
							else    			bpt_status_count = 0;
							if(bpt_status_count > 20)
							{
								bpt_status_count = 0;
								appState =  ST_EXAMPLEUSER_ESTIMATION_RE_MEASUREMENT;                                              //Added by Jason Chen, 2022.07.22
							  //measurement_count = 0;                                                                             //Added by Jason Chen, 2022.07.22
								break;
							}

							if(bpt_status == 4) bpt_status_count02++;
							else    			bpt_status_count02 = 0;
							if(bpt_status_count02 > 15)                                                                            //Added by Jason Chen, 2022.07.21
							{                                                                                                      //Added by Jason Chen, 2022.07.21
								bpt_status_count02 = 0;
								appState =  ST_EXAMPLEUSER_ESTIMATION_RE_MEASUREMENT;                                              //Added by Jason Chen, 2022.07.21
							  //measurement_count = 0;                                                                             //Added by Jason Chen, 2022.07.21
								break;                                                                                             //Added by Jason Chen, 2022.07.21
							}                                                                                                      //Added by Jason Chen, 2022.07.21
						}
						last_tick = count_tick;
					}
					appState = ST_EXAMPLEUSER_ESTIMATION_MEASUREMENT;
				}
				//appState = ST_EXAMPLEUSER_ESTIMATION_MEASUREMENT;
				bptMesurementProgressLast = bptMesurementProgress;                                                             //Added by Jason Chen, 2022.07.21
			}
			break;

			case ST_EXAMPLEUSER_ESTIMATION_RE_MEASUREMENT:                                                         // Jason
			{
				//curren_tick1 = sl_sleeptimer_get_tick_count();
				bptMesurementProgress = 0;
				calibrationTimer_stop();
				SH_Max3010x_stop(0);
				measurement_count++;
				if(measurement_count > 4)
				{
					measurement_count = 0;
					printLog("\r\nFinger take off OCCURED, Re-Calibration.....,             Line:%d \r\n", __LINE__);
					appState = ST_EXAMPLEUSER_DELAY_COUNT;                     //Jason //ST_COMMAND_MODE;
					delay_ms_count = 0;
					state_flag = 1;
					//wait_ms(WAIT_SENSORHUB_STABLE_BOOTUP_MS);
				}
				else
				{
					printLog("\r\nFinger take off OCCURED, Re-Measurement = %d,                 Line:%d \r\n", measurement_count, __LINE__);
					appState = ST_EXAMPLEUSER_DELAY_COUNT;     //ST_EXAMPLEUSER_ESTIMATION_STARTING;//ST_EXAMPLEUSER_ESTIMATION_SETTINGS;
					delay_ms_count = 0;
					state_flag = 0;
					//wait_ms(WAIT_SENSORHUB_STABLE_BOOTUP_MS);
				}
			}
			break;

			case ST_EXAMPLEUSER_TIMEOUT:
			case ST_EXAMPLEUSER_FAILURE:
			{
				//curren_tick1 = sl_sleeptimer_get_tick_count();
				printLog("TIMEOUT/FAULT CONDITION OCCURED FOR EXAMPLE MEASUREMENT,     Line:%d \r\n", __LINE__);
			  //is_example_measurement_active =  false;
				calibrationTimer_stop();
				SH_Max3010x_stop(0);
				appState = ST_EXAMPLEUSER_DELAY_COUNT;                     //Jason //ST_COMMAND_MODE;
				delay_ms_count = 0;
				state_flag = 1;
				//wait_ms(WAIT_SENSORHUB_STABLE_BOOTUP_MS);
			}
			break;
			case ST_EXAMPLEUSER_DELAY_COUNT:
			{
				uint16_t DELAY_COUNT = 20;
				//uint32_t diff = sl_sleeptimer_get_tick_count() - curren_tick1;
				//printLog("DELAY %d ms.......... \r\n", (int)sl_sleeptimer_tick_to_ms(diff));
				delay_ms_count++;
				if(state_flag == 4)
					DELAY_COUNT = 8;
				else if(state_flag == 6)
					DELAY_COUNT = 2;
				else
					DELAY_COUNT = 10;
				if(delay_ms_count > DELAY_COUNT)
				{
					delay_ms_count = 0;
					if(state_flag == 1)
						appState = ST_COMMAND_MODE;
					else if(state_flag == 2)
						appState = ST_EXAMPLEUSER_ESTIMATION_SETTINGS;
					else if(state_flag == 3)
						appState = ST_EXAMPLEUSER_ESTIMATION_STARTING;
					else if(state_flag == 4)
						appState = ST_EXAMPLEUSER_ESTIMATION_SETTINGS2;
					else if(state_flag == 0)
						appState = ST_EXAMPLEUSER_ESTIMATION_STARTING;
					else if(state_flag == 5)
						appState = ST_EXAMPLEUSER_ESTIMATION_RE_STARTING02;
					else if(state_flag == 6)
						appState = ST_EXAMPLEUSER_ESTIMATION_RE_STARTING03;
				}
			}
		    break;
		}

		//wait_ms(2);
		SH_Max3010x_data_report_execute02();
	}
}

void bpt_main_reset(void)
{
	//curren_tick1 = sl_sleeptimer_get_tick_count();
	printLog("BPT_MAIN state machine RESET FOR EXAMPLE MEASUREMENT.....,   Line:%d\r\n", __LINE__);
  //is_example_measurement_active =  false;
	calibrationTimer_stop();
	SH_Max3010x_stop(0);
	appState = ST_EXAMPLEUSER_DELAY_COUNT;                     //Jason //ST_COMMAND_MODE;
	delay_ms_count = 0;
	state_flag = 1;
}


