#include "DFRobot_I2C_Slave.h"

I2C_HandleTypeDef i2cSlave = {0};

volatile uint8_t receive_size = 0,transmit_size = 0;
uint8_t i2c_RX_receive_data;
uint8_t i2c_TX_Send_data;
uint8_t pData[DATA_LEN_MAX]={0};

extern void copyData(uint8_t* buf,uint16_t lne);
void i2cInitSlave(void)
{
    /**if i2c is  
		GPIO Configuration:    
    PD6     ------> SDA
    PD5     ------> SCL
    */
    GPIO_InitTypeDef  gpioi2c={0};
    gpioi2c.Pin    = GPIO_PIN_6;
    gpioi2c.Mode = GPIO_MODE_AF; 
    gpioi2c.Alternate = GPIO_AF4_I2C_SDA;
    gpioi2c.OpenDrain = GPIO_OPENDRAIN; // 开漏
    gpioi2c.Debounce.Enable = GPIO_DEBOUNCE_DISABLE; 
    gpioi2c.SlewRate = GPIO_SLEW_RATE_HIGH; 
    gpioi2c.DrvStrength = GPIO_DRV_STRENGTH_HIGH; 
    gpioi2c.Pull = GPIO_NOPULL;			
    HAL_GPIO_Init(GPIOD, &gpioi2c);

    gpioi2c.Pin = GPIO_PIN_5;
    gpioi2c.Mode = GPIO_MODE_AF;
    gpioi2c.Alternate = GPIO_AF4_I2C_SCL;
    gpioi2c.OpenDrain = GPIO_OPENDRAIN; 
    gpioi2c.Debounce.Enable = GPIO_DEBOUNCE_DISABLE;
    gpioi2c.SlewRate = GPIO_SLEW_RATE_HIGH;
    gpioi2c.DrvStrength = GPIO_DRV_STRENGTH_HIGH;
    gpioi2c.Pull = GPIO_NOPULL;	
    HAL_GPIO_Init(GPIOD, &gpioi2c);

    i2cSlave.Instance = I2C;
    i2cSlave.Init.master = I2C_MASTER_MODE_DISABLE;
    i2cSlave.Init.slave = I2C_SLAVE_MODE_ENABLE; 
    i2cSlave.Mode = HAL_I2C_MODE_SLAVE; 
    i2cSlave.Init.slaveAddr = DEVICE_ADDRESS; 

    i2cSlave.Init.broadack = I2C_BROAD_ACK_DISABLE; 
    i2cSlave.Init.speedclock = I2C_SPEED_RATE; 
    i2cSlave.State = HAL_I2C_STATE_RESET; 
    HAL_I2C_Init(&i2cSlave);
}

void i2cIRQConfig(void)
{
    HAL_NVIC_SetPriority(I2C_IRQn,I2C_SLAVE_IRQ_LEVEL);
    HAL_NVIC_EnableIRQ(I2C_IRQn);
}
void HAL_I2C_SlaveCallback(I2C_HandleTypeDef *hi2c)
{
    uint32_t i2c_flag = 0XFF;
    HAL_NVIC_DisableIRQ(I2C_IRQn);
    HAL_I2C_Wait_Flag(hi2c, &i2c_flag);
    switch(i2c_flag){
        case I2C_FLAG_SLAVE_RX_SLAW_ACK://I2C通信成功
            receive_size = 0;
            break;
        case I2C_FLAG_SLAVE_RX_SDATA_ACK:
            pData[receive_size] = hi2c->Instance->DATA; // 80H I2C从机接收数据
            i2c_RX_receive_data = 	pData[0];
            receive_size++;
            break;
        case I2C_FLAG_SLAVE_STOP_RESTART://接收到停止位
            if(i2c_RX_receive_data  == 0x55){
                i2c_RX_receive_data = 0;
                copyData(pData,receive_size);
            }
            break;
        case I2C_FLAG_SLAVE_TX_SLAW_ACK: // A8H 第一次获取数据
            break;
        case I2C_FLAG_SLAVE_TX_DATA_ACK: // B8H 数据发送成功接收到ACK
            break;
        default:
            break;
    }
    i2c_flag = 0XFF;
    HAL_I2C_ACK_Config(hi2c, ENABLE);
    HAL_I2C_Clear_Interrupt_Flag(hi2c);
    NVIC_ClearPendingIRQ(I2C_IRQn);
    HAL_NVIC_EnableIRQ(I2C_IRQn);	
}
