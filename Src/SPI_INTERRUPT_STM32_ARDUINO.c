/*
 * SPI_INTERRUPT_STM32_ARDUINO.c
 *
 *  Created on: Aug 14, 2024
 *      Author: LENOVO
 */

/*
 * master_slave.c
 *
 *  Created on: Aug 9, 2024
 *      Author: ADMIN
 */

#include <stm32f1_spi.h>
#include <stm32f1_gpio.h>
#include <string.h>

#define MAX_LEN 500

char RcvBuff[MAX_LEN];
uint8_t ReadByte;

int c = 0;

SPI_Handle_t hSPI2;

/*
    PB15: SPI2_MOSI
    PB14: SPI2_MISO
    PB13: SPI2_SCLK
    PB12: SPI2_NSS


*/

void SPI2_GPIOInit(void){
    GPIO_Handle_t SPIPins;
    SPIPins.pGPIOx = GPIOB;
    SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_AF_OUTPUT_PP_10MHz;

    //MOSI
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
    GPIO_Init(&SPIPins);

    //SCLK
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
    GPIO_Init(&SPIPins);

    //NSS
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    GPIO_Init(&SPIPins);

    //MISO
    SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT_PULL;
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
    SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
    GPIO_Init(&SPIPins);

}

void SPI2_Init(void)
{
    hSPI2.pSPIx = SPI2;
    hSPI2.SPI_config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
    hSPI2.SPI_config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
    hSPI2.SPI_config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV32; //0.5MHz
    hSPI2.SPI_config.SPI_DFF = SPI_DFF_8BITS;
    hSPI2.SPI_config.SPI_CPOL = SPI_CPOL_LOW;
    hSPI2.SPI_config.SPI_CPHA = SPI_CPHA_LOW;
    hSPI2.SPI_config.SPI_SSM = SPI_SSM_DI;  //Hardware slave management enabled for NSS pin

    SPI_Init(&hSPI2);
}

int main()
{
    //This function is used to intialize the GPIO pins to behave as SPI2 pins
    SPI2_GPIOInit();

    //This function is used to intialize the SPI peripheral parameters
    SPI2_Init();
    /*
     *
     *
     * 
     * 
     *
     */
    SPI_SSOEConfig(SPI2, ENABLE);
    SPI_IRQInteruptConfig(IRQ_NO_SPI2, ENABLE);

    while (1)
    {
        //enable the SPI2 peripheral
        SPI_PeripheralControl(SPI2, ENABLE);

        /* fetch the data from the SPI peripheral byte by byte in interrupt mode*/
        while(SPI_ReceiveDataIT(&hSPI2, &ReadByte, 1) == SPI_BUSY_IN_RX);

        //config SPI is not busy
        while (SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG));

        //disable the SPI2 peripheral
        SPI_PeripheralControl(SPI2, DISABLE);

    }
    return 0;
}

/*Runs when a data byte is received from the peripheral over SPI*/

void SPI2_IRQHandler(void)
{
    SPI_IRQHandling(&hSPI2);
}

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{
    static uint32_t i = 0;
    if(AppEv == SPI_EVENT_RX_CMPLT)
    {
    	c = 1;
        RcvBuff[i++] = ReadByte;
        if(ReadByte == '\0' || (i == MAX_LEN))
        {
            RcvBuff[i-1] = '\0';
            i = 0;
        }
    }
}





