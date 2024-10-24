/*
 * stm32f1_rcc.c
 *
 *  Created on: Aug 16, 2024
 *      Author: LENOVO
 */

#include "stm32f1_rcc.h"

uint16_t AHB_PreScaler[9] = {2, 4, 8, 16, 32, 64, 128, 256, 512};
uint8_t APB1_PreScaler[4] = {2, 4, 8, 16};
//This return APB1 clock value
uint32_t RCC_GetPCLK1Value(void)
{
    uint32_t pclk1, SystemClk;
    uint8_t clkscr, temp, ahbp, apb1p;
    clkscr = (RCC->CFGR >> 2 & 0x03);

    if(clkscr == 0)
    {
        SystemClk = 8000000;        //HSI
    }
    else if(clkscr == 1)
    {
        SystemClk = 8000000;        //HSE
    }
    else if(clkscr == 2)
    {
        SystemClk = RCC_GetPLLOutputClock();
    }

    //AHB
    temp = ((RCC->CFGR >> 4) & 0xF);
    if(temp < 8)
    {
        ahbp = 1;
    }
    else
    {
        ahbp = AHB_PreScaler[temp - 8];
    }
    
    //APB1
    temp = ((RCC->CFGR >> 8) & 0x7);
    if(temp < 4)
    {
        apb1p = 1;
    }
    else
    {
        apb1p = APB1_PreScaler[temp - 4];
    }

    pclk1 = (SystemClk / ahbp) / apb1p;
    return pclk1;
}

uint32_t RCC_GetPLLOutputClock(void)
{
    return 0;
}

//This return APB2 clock value
uint32_t RCC_GetPCLK2Value(void)
{
    uint32_t tmp,pclk2, SystemClock = 0;
    uint8_t clk_scr = (RCC->CFGR >> 2 & 0x03);
    uint8_t ahbp, apb2p;

    if(clk_scr == 0)
    {
        SystemClock = 8000000;        //HSI
    }
    else if(clk_scr == 1)
    {
        SystemClock = 8000000;        //HSE
    }
    else if(clk_scr == 2)
    {
        SystemClock = RCC_GetPLLOutputClock();
    }

    //AHB
    tmp = ((RCC->CFGR >> 4) & 0xF);
    if(tmp < 8)
    {
        ahbp = 1;
    }
    else
    {
        ahbp = AHB_PreScaler[tmp - 8];
    }
    
    //APB1
    tmp = ((RCC->CFGR >> 11) & 0x7);
    if(tmp < 4)
    {
        tmp = 1;
    }
    else
    {
        apb2p = APB1_PreScaler[tmp - 4];
    }

    pclk2 = (SystemClock / ahbp) / apb2p;
    return pclk2;
}
