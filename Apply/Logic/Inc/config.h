#ifndef __CONFIG_H_
#define __CONFIG_H_

#include "drv_hal_conf.h"
#include "ocd_conf.h"
#include "algo_conf.h"
#include "dev_conf.h"

/* 用户句柄声明包含区 */

extern tagTIM_T tTimer2;

extern tagGPIO_T demoGPIO[];

extern tagUART_T Uart1;

extern tagUART_T Uart3;

extern tagPWM_T thruster[]; 

//extern tagPWM_T paw[];

extern tagJY901_T JY901S;

extern tagPID_T PID;


extern PIDInitStruct PID_init;

extern tagPID_T depth_PID;

extern PIDInitStruct depth_PID_init;

extern tagIWDG_T demoIWDG;
#endif
