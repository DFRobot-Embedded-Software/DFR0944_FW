#ifndef _DFROBOT_I2C_SLAVE_H_
#define _DFROBOT_I2C_SLAVE_H_
#include "cs32l010_hal.h"
#include "main.h"

#define DEVICE_ADDRESS    0x002E
#define I2C_SPEED_RATE                    100 //i2c速率 100k
#define I2C_SLAVE_IRQ_LEVEL				        0x00U
#define UART_SLAVE_IRQ_LEVEL				      0x00U
#define	DATA_LEN_MAX						  0X30U
/**
 * @brief I2C从机初始化
 */
void i2cInitSlave(void);

/**
 *@brief 初始化I2C中断
 */
void i2cIRQConfig(void);

#endif
