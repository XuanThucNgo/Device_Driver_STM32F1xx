/*
 * master_slave.c
 *
 *  Created on: Aug 9, 2024
 *      Author: ADMIN
 */

#include <stm32f1_spi.h>
#include <stm32f1_gpio.h>
#include <string.h>

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

    //MISO
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
    GPIO_Init(&SPIPins);

    //SCLK
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
    GPIO_Init(&SPIPins);

    //NSS
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    GPIO_Init(&SPIPins);

}

void SPI2_Init(void){
    SPI_Handle_t hSPI2;
    hSPI2.pSPIx = SPI2;
    hSPI2.SPI_config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
    hSPI2.SPI_config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
    hSPI2.SPI_config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2; //8MHz
    hSPI2.SPI_config.SPI_DFF = SPI_DFF_8BITS;
    hSPI2.SPI_config.SPI_CPOL = SPI_CPOL_LOW;
    hSPI2.SPI_config.SPI_CPHA = SPI_CPHA_LOW;
    hSPI2.SPI_config.SPI_SSM = SPI_SSM_DI;

    SPI_Init(&hSPI2);
}

int main()
{
    char data[] = "Hello World!";
    SPI2_GPIOInit();
    SPI2_Init();
    
    SPI_SSOEConfig(SPI2, ENABLE);

    SPI_PeripheralControl(SPI2, ENABLE);

    SPI_SendData(SPI2, (uint8_t*)data, strlen(data));

    while (SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG));

    SPI_PeripheralControl(SPI2, DISABLE);
    return 0;
}


