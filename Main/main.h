#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "cs32l010_hal.h"
#include "DFRobot_I2C_Slave.h"
#include "stdio.h"
#include <stdbool.h>
#include "log.h"
/* USER CODE END Includes */



	/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
void SystemClock_Config(void);
	

/* Private defines -----------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
