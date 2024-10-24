/*
 * i2c_tx.c
 *
 *  Created on: Sep 9, 2024
 *      Author: LENOVO
 */


#include <stdio.h>
#include <string.h>

#include "stm32f1.h"
#include "stm32f1_i2c.h"
#include "stm32f1_gpio.h"

#define MY_ADDR         0x61
#define SLAVE_ADDR      0x68

I2C_Handle_t I2C1Handle;

/*
 *  PB6 -> SCL
 *  PB7 -> SDA
 */

uint8_t data[] = "Hello XuanThuc \n";

void I2C1_GPIOInits(void)
{
    GPIO_Handle_t I2CPins;

    I2CPins.pGPIOx = GPIOB;
    I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_AF_OUTPUT_OD_10MHz;
    I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

    //SCL
    I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
    //SDA
    I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;

    GPIO_Init(&I2CPins);
}

void I2C1_Inits(void)
{
    I2C1Handle.pI2Cx = I2C1;
    I2C1Handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
    I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
    I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
    I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

    I2C_Init(&I2C1Handle);
}

int main(void)
{
    I2C1_GPIOInits();
    I2C1_Inits();

    I2C_PeripheralControl(I2C1, ENABLE);

    while (1)
    {
        //Send some data to slave
        I2C_MasterSendData(&I2C1Handle, strlen((char *)data), 8, SLAVE_ADDR, 0);
        while(1);
    }   
}