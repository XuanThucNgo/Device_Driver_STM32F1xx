#ifndef _INC_STM32F1_H
#define _INC_STM32F1_H

#define _vo volatile
#define ENABLE          1
#define DISABLE         0
#define SET             ENABLE
#define RESET           DISABLE
#define GPIO_PIN_SET    SET
#define GPIO_PIN_RESET  RESET
#define FLAG_SET        SET
#define FLAG_RESET      RESET

#include <stdint.h>

//
#define FLASH_BASEADDR  0x08000000U
#define SRAM_BASEADDR   0x20000000U

#define PERIPH_BASEADDR 0x40000000U
#define APB1PERIPH_BASEADDR PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR	0x40010000U
#define AHBPERIPH_BASEADDR	0x40018000U

//AHB
#define SDIO_BASEADDR (AHBPERIPH_BASEADDR + 0x0000)
#define DMA1_BASEADDR (AHBPERIPH_BASEADDR + 0x8000)
#define DMA2_BASEADDR (AHBPERIPH_BASEADDR + 0x8400)
#define RCC_BASEADDR  (AHBPERIPH_BASEADDR + 0x9000)
#define FMI_BASEADDR  (AHBPERIPH_BASEADDR + 0xA000)
#define CRC_BASEADDR  (AHBPERIPH_BASEADDR + 0xB000)
#define FSMC_BASEADDR (AHBPERIPH_BASEADDR + 0x5FFE8000)

//APB1
#define SPI2_BASEADDR   (APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR   (APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR (APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR (APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR  (APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR  (APB1PERIPH_BASEADDR + 0x5000)

#define I2C1_BASEADDR   (APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR   (APB1PERIPH_BASEADDR + 0x5800)

#define CAN1_BASE       (APB1PERIPH_BASEADDR + 0x6400)
#define CAN2_BASE       (APB1PERIPH_BASEADDR + 0x6800)


//APB2
#define AFIO_BASEADDR   (APB2PERIPH_BASEADDR + 0x0000)
#define EXTI_BASEADDR   (APB2PERIPH_BASEADDR + 0x0400)

#define GPIOA_BASEADDR  (APB2PERIPH_BASEADDR + 0x0800)
#define GPIOB_BASEADDR  (APB2PERIPH_BASEADDR + 0x0C00)
#define GPIOC_BASEADDR  (APB2PERIPH_BASEADDR + 0x1000)
#define GPIOD_BASEADDR  (APB2PERIPH_BASEADDR + 0x1400)
#define GPIOE_BASEADDR  (APB2PERIPH_BASEADDR + 0x1800)
#define GPIOF_BASEADDR  (APB2PERIPH_BASEADDR + 0x1C00)
#define GPIOG_BASEADDR  (APB2PERIPH_BASEADDR + 0x2000)

#define SPI1_BASEADDR   (APB2PERIPH_BASEADDR + 0x3000)

#define USART1_BASEADDR (APB2PERIPH_BASEADDR + 0x3800)

//NVIC ISERx core
#define NVIC_ISER0      ((_vo uint32_t *) 0xE000E100)
#define NVIC_ISER1      ((_vo uint32_t *) 0xE000E104)
#define NVIC_ISER2      ((_vo uint32_t *) 0xE000E108)
#define NVIC_ISER3      ((_vo uint32_t *) 0xE000E10C)

//NVIC ICER core
#define NVIC_ICER0      ((_vo uint32_t *) 0xE000E180)
#define NVIC_ICER1      ((_vo uint32_t *) 0xE000E184)
#define NVIC_ICER2      ((_vo uint32_t *) 0xE000E188)
#define NVIC_ICER3      ((_vo uint32_t *) 0xE000E18C)

//NVIC priority core
#define NVIC_PR_BASEADDR ((_vo uint32_t *) 0xE000E400)

//GPIO port
                                                //Address (bit)     Default             Description
//typedef struct
//{
//    union
//    {
//        uint32_t REGS;
//        struct
//        {
//            uint32_t MODE_0 : 2;                //0 -> 1            0                           0: Input (reset state)
//                                                //                                              1: Output, max speed 10 MHz
//                                                //                                              2: Output, max speed 2 MHz
//                                                //                                              3: Output, max speed 50 MHz
//            uint32_t  CNF_0 : 2;                //2 -> 3            1                   In input mode (MODE[1:0]= 00):
//                                                //                                              0: Analog mode
//                                                //                                              1: Floating input (reset state)
//                                                //                                              2: Input with pull-up / pull-down
//                                                //                                      In output mode (MODE[1:0] > 00):
//                                                //                                              0: General purpose output push-pull
//                                                //                                              1: General purpose output Open-drain
//                                                //                                              2: Alternate function output Push-pull
//                                                //                                              3: Alternate function output Open-drain
//            uint32_t MODE_1 : 2;
//            uint32_t  CNF_1 : 2;
//            uint32_t MODE_2 : 2;
//            uint32_t  CNF_2 : 2;
//            uint32_t MODE_3 : 2;
//            uint32_t  CNF_3 : 2;
//            uint32_t MODE_4 : 2;
//            uint32_t  CNF_4 : 2;
//            uint32_t MODE_5 : 2;
//            uint32_t  CNF_5 : 2;
//            uint32_t MODE_6 : 2;
//            uint32_t  CNF_6 : 2;
//            uint32_t MODE_7 : 2;
//            uint32_t  CNF_7 : 2;
//        } BITS;
//    } CRL;
//
//    union
//    {
//        uint32_t REGS;
//        struct
//        {
//            uint32_t MODE_8  : 2;
//            uint32_t  CNF_8  : 2;
//            uint32_t MODE_9  : 2;
//            uint32_t  CNF_9  : 2;
//            uint32_t MODE_10 : 2;
//            uint32_t  CNF_10 : 2;
//            uint32_t MODE_11 : 2;
//            uint32_t  CNF_11 : 2;
//            uint32_t MODE_12 : 2;
//            uint32_t  CNF_12 : 2;
//            uint32_t MODE_13 : 2;
//            uint32_t  CNF_13 : 2;
//            uint32_t MODE_14 : 2;
//            uint32_t  CNF_14 : 2;
//            uint32_t MODE_15 : 2;
//            uint32_t  CNF_15 : 2;
//        } BITS;
//    } CRH;
//
//    union
//    {
//        uint32_t REGS;
//        struct
//        {
//            uint32_t b0   : 1;
//            uint32_t b1   : 1;
//            uint32_t b2   : 1;
//            uint32_t b3   : 1;
//            uint32_t b4   : 1;
//            uint32_t b5   : 1;
//            uint32_t b6   : 1;
//            uint32_t b7   : 1;
//            uint32_t b8   : 1;
//            uint32_t b9   : 1;
//            uint32_t b10  : 1;
//            uint32_t b11  : 1;
//            uint32_t b12  : 1;
//            uint32_t b13  : 1;
//            uint32_t b14  : 1;
//            uint32_t b15  : 1;
//        } BITS;
//    } IDR;
//
//    union
//    {
//        uint32_t REGS;
//        struct
//        {
//            uint32_t b0   : 1;
//            uint32_t b1   : 1;
//            uint32_t b2   : 1;
//            uint32_t b3   : 1;
//            uint32_t b4   : 1;
//            uint32_t b5   : 1;
//            uint32_t b6   : 1;
//            uint32_t b7   : 1;
//            uint32_t b8   : 1;
//            uint32_t b9   : 1;
//            uint32_t b10  : 1;
//            uint32_t b11  : 1;
//            uint32_t b12  : 1;
//            uint32_t b13  : 1;
//            uint32_t b14  : 1;
//            uint32_t b15  : 1;
//        } BITS;
//    } ODR;
//
//    union
//    {
//        uint32_t REGS;
//        union
//        {
//            uint16_t REGS;
//            struct
//            {
//                uint16_t b0   : 1;
//                uint16_t b1   : 1;
//                uint16_t b2   : 1;
//                uint16_t b3   : 1;
//                uint16_t b4   : 1;
//                uint16_t b5   : 1;
//                uint16_t b6   : 1;
//                uint16_t b7   : 1;
//                uint16_t b8   : 1;
//                uint16_t b9   : 1;
//                uint16_t b10  : 1;
//                uint16_t b11  : 1;
//                uint16_t b12  : 1;
//                uint16_t b13  : 1;
//                uint16_t b14  : 1;
//                uint16_t b15  : 1;
//            } BITS;
//        } BSR;
//
//        union
//        {
//            uint16_t REGS;
//            struct
//            {
//                uint16_t b0   : 1;
//                uint16_t b1   : 1;
//                uint16_t b2   : 1;
//                uint16_t b3   : 1;
//                uint16_t b4   : 1;
//                uint16_t b5   : 1;
//                uint16_t b6   : 1;
//                uint16_t b7   : 1;
//                uint16_t b8   : 1;
//                uint16_t b9   : 1;
//                uint16_t b10  : 1;
//                uint16_t b11  : 1;
//                uint16_t b12  : 1;
//                uint16_t b13  : 1;
//                uint16_t b14  : 1;
//                uint16_t b15  : 1;
//            } BITS;
//        } BR;
//
//    } BSRR;
//
//    union
//    {
//        uint32_t REGS;
//        struct
//        {
//            uint32_t b0   : 1;
//            uint32_t b1   : 1;
//            uint32_t b2   : 1;
//            uint32_t b3   : 1;
//            uint32_t b4   : 1;
//            uint32_t b5   : 1;
//            uint32_t b6   : 1;
//            uint32_t b7   : 1;
//            uint32_t b8   : 1;
//            uint32_t b9   : 1;
//            uint32_t b10  : 1;
//            uint32_t b11  : 1;
//            uint32_t b12  : 1;
//            uint32_t b13  : 1;
//            uint32_t b14  : 1;
//            uint32_t b15  : 1;
//        } BITS;
//    } BRR;
//
//    union
//    {
//        uint32_t REGS;
//        struct
//        {
//            uint32_t b0   : 1;
//            uint32_t b1   : 1;
//            uint32_t b2   : 1;
//            uint32_t b3   : 1;
//            uint32_t b4   : 1;
//            uint32_t b5   : 1;
//            uint32_t b6   : 1;
//            uint32_t b7   : 1;
//            uint32_t b8   : 1;
//            uint32_t b9   : 1;
//            uint32_t b10  : 1;
//            uint32_t b11  : 1;
//            uint32_t b12  : 1;
//            uint32_t b13  : 1;
//            uint32_t b14  : 1;
//            uint32_t b15  : 1;
//            uint32_t LCKK : 1;
//        } BITS;
//    } LCKR;
//
//} GPIO_RegDef_t;

//GPIO port
                                                //Address (bit)     Default             Description
typedef struct 
{
    _vo uint32_t   CR[2];           // CRL  CRH
    _vo uint32_t   IDR;
    _vo uint32_t   ODR;
    _vo uint32_t   BSRR;
    _vo uint32_t   BSR;
    _vo uint32_t   LCKR;
}GPIO_RegDef_t;

//AFIO
typedef struct
{
    uint32_t EVCR;
    uint32_t MAPR;
    uint32_t EXTICR[4];
    uint32_t MAPR2;

} AFIO_RegDef_t;

//EXTI


#define GPIOA ((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB ((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC ((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD ((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE ((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOF ((GPIO_RegDef_t*) GPIOF_BASEADDR)
#define GPIOG ((GPIO_RegDef_t*) GPIOG_BASEADDR)

#define AFIO  ((AFIO_RegDef_t*) AFIO_BASEADDR)


//RCC
typedef struct
{
    _vo uint32_t CR;
    _vo uint32_t CFGR;
    _vo uint32_t CIR;
    _vo uint32_t APB2RSTR;
    _vo uint32_t APB1RSTR;
    _vo uint32_t AHBENR;
    _vo uint32_t APB2ENR;
    _vo uint32_t APB1ENR;
    _vo uint32_t BDCR;
    _vo uint32_t CSR;
    _vo uint32_t AHBRSTR;
    _vo uint32_t CFGR2;
} RCC_RegDef_t;

#define RCC ((RCC_RegDef_t*) RCC_BASEADDR)

//Clock Enable Macro For GPIOx Peripherals
#define AFIO_PCLK_EN()  (RCC->APB2ENR |= (1 << 0))
#define GPIOA_PCLK_EN() (RCC->APB2ENR |= (1 << 2))
#define GPIOB_PCLK_EN() (RCC->APB2ENR |= (1 << 3))
#define GPIOC_PCLK_EN() (RCC->APB2ENR |= (1 << 4))
#define GPIOD_PCLK_EN() (RCC->APB2ENR |= (1 << 5))
#define GPIOE_PCLK_EN() (RCC->APB2ENR |= (1 << 6))

//Clock Disable Macro For GPIOx Peripherals
#define AFIO_PCLK_DI()  (RCC->APB2ENR &= ~(1 << 0))
#define GPIOA_PCLK_DI() (RCC->APB2ENR &= ~(1 << 2)) 
#define GPIOB_PCLK_DI() (RCC->APB2ENR &= ~(1 << 3))
#define GPIOC_PCLK_DI() (RCC->APB2ENR &= ~(1 << 4))
#define GPIOD_PCLK_DI() (RCC->APB2ENR &= ~(1 << 5))
#define GPIOE_PCLK_DI() (RCC->APB2ENR &= ~(1 << 6))

//Reset REG
#define AFIO_REG_RESET()            do { (RCC->APB2ENR |= (1 << 0));     (RCC->APB2ENR &= ~(1 << 0)); } while(0)
#define GPIOA_REG_RESET()           do { (RCC->APB2ENR |= (1 << 2));     (RCC->APB2ENR &= ~(1 << 2)); } while(0)
#define GPIOB_REG_RESET()           do { (RCC->APB2ENR |= (1 << 3));     (RCC->APB2ENR &= ~(1 << 3)); } while(0)
#define GPIOC_REG_RESET()           do { (RCC->APB2ENR |= (1 << 4));     (RCC->APB2ENR &= ~(1 << 4)); } while(0)
#define GPIOD_REG_RESET()           do { (RCC->APB2ENR |= (1 << 5));     (RCC->APB2ENR &= ~(1 << 5)); } while(0)
#define GPIOE_REG_RESET()           do { (RCC->APB2ENR |= (1 << 6));     (RCC->APB2ENR &= ~(1 << 6)); } while(0)

#define GPIO_BASEADDR_TO_CODE(x)    ((x == GPIOA) ? 0 : (x == GPIOB) ? 1 : (x == GPIOC) ? 2 : (x == GPIOD) ? 3 : (x == GPIOE) ? 4 : 0)

//EXTI
typedef struct
{
    _vo uint32_t IMR;
    _vo uint32_t EMR;
    _vo uint32_t RTSR;
    _vo uint32_t FTSR;
    _vo uint32_t SWIER;
    _vo uint32_t PR;
}EXTI_RegDef_t;

#define EXTI ((EXTI_RegDef_t*) EXTI_BASEADDR)

//IRQ NVIC
#define IRQ_NO_EXTI0        6
#define IRQ_NO_EXTI1        7
#define IRQ_NO_EXTI2        8
#define IRQ_NO_EXTI3        9
#define IRQ_NO_EXTI4        10
#define IRQ_NO_EXTI9_5      23
#define IRQ_NO_EXTI15_10    40

//IRQ SPI
#define IRQ_NO_SPI1         35
#define IRQ_NO_SPI2         36
#define IRQ_NO_SPI3         51

//IRQ USART
#define IRQ_NO_USART1       37
#define IRQ_NO_USART2       38
#define IRQ_NO_USART3       39

#define IRQ_NO_UART4        52
#define IRQ_NO_UART5        53

//IRQ I2C
#define IRQ_NO_I2C1_EV      31
#define IRQ_NO_I2C1_ER      32

#define IRQ_NO_I2C2_EV      33
#define IRQ_NO_I2C2_ER      34

//SPI protocol
typedef struct
{
    _vo uint32_t CR1;
    _vo uint32_t CR2;
    _vo uint32_t SR;
    _vo uint32_t DR;
    _vo uint32_t CRCPR;
    _vo uint32_t RXCRCR;
    _vo uint32_t TXCRCR;
    _vo uint32_t I2SCFGR;
    _vo uint32_t I2SPR;
}SPI_RegDef_t;

//SPI_CR1
#define SPI_CR1_CPHA            0
#define SPI_CR1_CPOL            1
#define SPI_CR1_MSTR            2
#define SPI_CR1_BR              3
#define SPI_CR1_SPE             6
#define SPI_CR1_LSBFIRST        7
#define SPI_CR1_SSI             8
#define SPI_CR1_SSM             9
#define SPI_CR1_RXONLY          10
#define SPI_CR1_DFF             11
#define SPI_CR1_CRCNEXT         12
#define SPI_CR1_CRCEN           13
#define SPI_CR1_BIDIOE          14
#define SPI_CR1_BIDIMODE        15

//SPI_CR2
#define SPI_CR2_RXDMAEN         0
#define SPI_CR2_TXDMAEN         1
#define SPI_CR2_SSOE            2
#define SPI_CR2_FRF             4
#define SPI_CR2_ERRIE           5
#define SPI_CR2_RXNEIE          6
#define SPI_CR2_TXEIE           7

//SPI_SR
#define SPI_SR_RXNE             0
#define SPI_SR_TXE              1
#define SPI_SR_CHSIDE           2
#define SPI_SR_UDR              3
#define SPI_SR_CRCERR           4
#define SPI_SR_MODF             5
#define SPI_SR_OVR              6
#define SPI_SR_BSY              7
//#define SPI_SR_FRE              8




#define SPI1 ((SPI_RegDef_t*) SPI1_BASEADDR)
#define SPI2 ((SPI_RegDef_t*) SPI2_BASEADDR)
#define SPI3 ((SPI_RegDef_t*) SPI3_BASEADDR)

//Clock Enable Macro For SPIx Peripherals
#define SPI1_PCLK_EN() (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN() (RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN() (RCC->APB1ENR |= (1 << 15))

//Clock Disable Macro For SPIx Peripherals
#define SPI1_PCLK_DI() (RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI() (RCC->APB1ENR &= ~(1 << 15))


//I2C protocol
typedef struct
{
    _vo uint32_t CR1;
    _vo uint32_t CR2;
    _vo uint32_t OAR1;
    _vo uint32_t OAR2;
    _vo uint32_t DR;
    _vo uint32_t SR1;
    _vo uint32_t SR2;
    _vo uint32_t CCR;
    _vo uint32_t TRISE;
} I2C_RegDef_t;

#define I2C1 ((I2C_RegDef_t*) I2C1_BASEADDR)
#define I2C2 ((I2C_RegDef_t*) I2C2_BASEADDR)

//Clock Enable Macro For I2Cx Peripherals
#define I2C1_PCLK_EN() (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN() (RCC->APB1ENR |= (1 << 22))

//Clock Disable Macro For I2Cx Peripherals
#define I2C1_PCLK_DI() (RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 22))

/** 
  * @brief Controller Area Network TxMailBox 
  */

typedef struct
{
  _vo uint32_t TIR;
  _vo uint32_t TDTR;
  _vo uint32_t TDLR;
  _vo uint32_t TDHR;
} CAN_TxMailBox_TypeDef;

/** 
  * @brief Controller Area Network FIFOMailBox 
  */
typedef struct
{
  _vo uint32_t RIR;
  _vo uint32_t RDTR;
  _vo uint32_t RDLR;
  _vo uint32_t RDHR;
} CAN_FIFOMailBox_TypeDef;

/** 
  * @brief Controller Area Network FilterRegister 
  */
  
typedef struct
{
  _vo uint32_t FR1;
  _vo uint32_t FR2;
} CAN_FilterRegister_TypeDef;

//CAN protocol
typedef struct 
{
    _vo uint32_t MCR;
    _vo uint32_t MSR;
    _vo uint32_t TSR;
    _vo uint32_t RF0R;
    _vo uint32_t RF1R;
    _vo uint32_t IER;
    _vo uint32_t ESR;
    _vo uint32_t BTR;

    uint32_t  RESERVED0[88];

    CAN_TxMailBox_TypeDef sTxMailBox[3];
    CAN_FIFOMailBox_TypeDef sFIFOMailBox[2];

    uint32_t  RESERVED1[12];

    _vo uint32_t FMR;
    _vo uint32_t FM1R;
    uint32_t  RESERVED2;
    _vo uint32_t FS1R;
    uint32_t  RESERVED3;
    _vo uint32_t FFA1R;
    uint32_t  RESERVED4;
    _vo uint32_t FA1R;
    uint32_t  RESERVED5[8];
    #ifndef STM32F10X_CL
    CAN_FilterRegister_TypeDef sFilterRegister[14];
    #else
    CAN_FilterRegister_TypeDef sFilterRegister[28];
    #endif /* STM32F10X_CL */
} CAN_RegDef_t;

#define CAN1 ((CAN_RegDef_t *) CAN1_BASE)
#define CAN2 ((CAN_RegDef_t *) CAN1_BASE)

//Clock Enable Macro For CANx Peripherals
#define CAN1_PCLK_EN() (RCC->APB1ENR |= (1 << 25))
#define CAN2_PCLK_EN() (RCC->APB1ENR |= (1 << 26))

//Clock Disable Macro For CANx Peripherals
#define CAN1_PCLK_DI() (RCC->APB1ENR &= ~(1 << 25))
#define CAN2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 26))


//UART protocol
typedef struct
{
    _vo uint32_t SR;
    _vo uint32_t DR;
    _vo uint32_t BRR;
    _vo uint32_t CR1;
    _vo uint32_t CR2;
    _vo uint32_t CR3;
    _vo uint32_t GTPR;
} USART_RegDef_t;

#define USART1 ((USART_RegDef_t*) USART1_BASEADDR)
#define USART2 ((USART_RegDef_t*) USART2_BASEADDR)
#define USART3 ((USART_RegDef_t*) USART3_BASEADDR)
#define UART4  ((USART_RegDef_t*) UART4_BASEADDR )
#define UART5  ((USART_RegDef_t*) UART5_BASEADDR )

//Clock Enable Macro For USARTx Peripherals
#define USART1_PCLK_EN() (RCC->APB2ENR |= (1 << 14))
#define USART2_PCLK_EN() (RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN() (RCC->APB1ENR |= (1 << 18))
#define  UART4_PCLK_EN() (RCC->APB1ENR |= (1 << 19))
#define  UART5_PCLK_EN() (RCC->APB1ENR |= (1 << 20))

//Clock Disable Macro For USARTx Peripherals
#define USART1_PCLK_DI() (RCC->APB2ENR &= ~(1 << 14))
#define USART2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI() (RCC->APB1ENR &= ~(1 << 18))
#define  UART4_PCLK_DI() (RCC->APB1ENR &= ~(1 << 19))
#define  UART5_PCLK_DI() (RCC->APB1ENR &= ~(1 << 20))

/******************************************************************************/
/*                                                                            */
/*         Universal Synchronous Asynchronous Receiver Transmitter            */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for USART_SR register  *******************/
#define  USART_SR_PE                         0            /*!< Parity Error */
#define  USART_SR_FE                         1            /*!< Framing Error */
#define  USART_SR_NE                         2            /*!< Noise Error Flag */
#define  USART_SR_ORE                        3            /*!< OverRun Error */
#define  USART_SR_IDLE                       4            /*!< IDLE line detected */
#define  USART_SR_RXNE                       5            /*!< Read Data Register Not Empty */
#define  USART_SR_TC                         6            /*!< Transmission Complete */
#define  USART_SR_TXE                        7            /*!< Transmit Data Register Empty */
#define  USART_SR_LBD                        8            /*!< LIN Break Detection Flag */
#define  USART_SR_CTS                        9            /*!< CTS Flag */

/******************  Bit definition for USART_CR1 register  *******************/
#define  USART_CR1_SBK                       0            /*!< Send Break */
#define  USART_CR1_RWU                       1            /*!< Receiver wakeup */
#define  USART_CR1_RE                        2            /*!< Receiver Enable */
#define  USART_CR1_TE                        3            /*!< Transmitter Enable */
#define  USART_CR1_IDLEIE                    4            /*!< IDLE Interrupt Enable */
#define  USART_CR1_RXNEIE                    5            /*!< RXNE Interrupt Enable */
#define  USART_CR1_TCIE                      6            /*!< Transmission Complete Interrupt Enable */
#define  USART_CR1_TXEIE                     7            /*!< PE Interrupt Enable */
#define  USART_CR1_PEIE                      8            /*!< PE Interrupt Enable */
#define  USART_CR1_PS                        9            /*!< Parity Selection */
#define  USART_CR1_PCE                       10           /*!< Parity Control Enable */
#define  USART_CR1_WAKE                      11           /*!< Wakeup method */
#define  USART_CR1_M                         12           /*!< Word length */
#define  USART_CR1_UE                        13           /*!< USART Enable */
#define  USART_CR1_OVER8                     15           /*!< USART Oversmapling 8-bits */

/******************  Bit definition for USART_CR2 register  *******************/
#define  USART_CR2_ADD                       0            /*!< Address of the USART node */
#define  USART_CR2_LBDL                      5            /*!< LIN Break Detection Length */
#define  USART_CR2_LBDIE                     6            /*!< LIN Break Detection Interrupt Enable */
#define  USART_CR2_LBCL                      8            /*!< Last Bit Clock pulse */
#define  USART_CR2_CPHA                      9            /*!< Clock Phase */
#define  USART_CR2_CPOL                      10           /*!< Clock Polarity */
#define  USART_CR2_CLKEN                     11           /*!< Clock Enable */

#define  USART_CR2_STOP                      12           /*!< STOP[1:0] bits (STOP bits) */
#define  USART_CR2_LINEN                     14           /*!< LIN mode enable */

/******************  Bit definition for USART_CR3 register  *******************/
#define  USART_CR3_EIE                       0            /*!< Error Interrupt Enable */
#define  USART_CR3_IREN                      1            /*!< IrDA mode Enable */
#define  USART_CR3_IRLP                      2            /*!< IrDA Low-Power */
#define  USART_CR3_HDSEL                     3            /*!< Half-Duplex Selection */
#define  USART_CR3_NACK                      4            /*!< Smartcard NACK enable */
#define  USART_CR3_SCEN                      5            /*!< Smartcard mode enable */
#define  USART_CR3_DMAR                      6            /*!< DMA Enable Receiver */
#define  USART_CR3_DMAT                      7            /*!< DMA Enable Transmitter */
#define  USART_CR3_RTSE                      8            /*!< RTS Enable */
#define  USART_CR3_CTSE                      9            /*!< CTS Enable */
#define  USART_CR3_CTSIE                     10           /*!< CTS Interrupt Enable */
#define  USART_CR3_ONEBIT                    11           /*!< One Bit method */

/******************************************************************************/
/*                                                                            */
/*                      Inter-integrated Circuit Interface                    */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for I2C_CR1 register  ********************/
#define  I2C_CR1_PE                          0            /*!< Peripheral Enable */
#define  I2C_CR1_SMBUS                       1            /*!< SMBus Mode */
#define  I2C_CR1_SMBTYPE                     3            /*!< SMBus Type */
#define  I2C_CR1_ENARP                       4            /*!< ARP Enable */
#define  I2C_CR1_ENPEC                       5            /*!< PEC Enable */
#define  I2C_CR1_ENGC                        6            /*!< General Call Enable */
#define  I2C_CR1_NOSTRETCH                   7            /*!< Clock Stretching Disable (Slave mode) */
#define  I2C_CR1_START                       8            /*!< Start Generation */
#define  I2C_CR1_STOP                        9            /*!< Stop Generation */
#define  I2C_CR1_ACK                         10           /*!< Acknowledge Enable */
#define  I2C_CR1_POS                         11           /*!< Acknowledge/PEC Position (for data reception) */
#define  I2C_CR1_PEC                         12           /*!< Packet Error Checking */
#define  I2C_CR1_ALERT                       13           /*!< SMBus Alert */
#define  I2C_CR1_SWRST                       15           /*!< Software Reset */

/*******************  Bit definition for I2C_CR2 register  ********************/
#define  I2C_CR2_FREQ                        0            /*!< FREQ[5:0] bits (Peripheral Clock Frequency) */

#define  I2C_CR2_ITERREN                     8            /*!< Error Interrupt Enable */
#define  I2C_CR2_ITEVTEN                     9            /*!< Event Interrupt Enable */
#define  I2C_CR2_ITBUFEN                     10           /*!< Buffer Interrupt Enable */
#define  I2C_CR2_DMAEN                       11           /*!< DMA Requests Enable */
#define  I2C_CR2_LAST                        12           /*!< DMA Last Transfer */

/*******************  Bit definition for I2C_OAR1 register  *******************/
#define  I2C_OAR1_ADD1_0                     0
#define  I2C_OAR1_ADD1_71                    1            /*!< Interface Address */
#define  I2C_OAR1_ADD8_98                    8            /*!< Interface Address */


#define  I2C_OAR1_ADDMODE                    15           /*!< Addressing Mode (Slave mode) */

/*******************  Bit definition for I2C_OAR2 register  *******************/
#define  I2C_OAR2_ENDUAL                     0               /*!< Dual addressing mode enable */
#define  I2C_OAR2_ADD2                       1               /*!< Interface address */

/********************  Bit definition for I2C_DR register  ********************/
#define  I2C_DR_DR                           0               /*!< 8-bit Data Register */

/*******************  Bit definition for I2C_SR1 register  ********************/
#define  I2C_SR1_SB                          0            /*!< Start Bit (Master mode) */
#define  I2C_SR1_ADDR                        1            /*!< Address sent (master mode)/matched (slave mode) */
#define  I2C_SR1_BTF                         2            /*!< Byte Transfer Finished */
#define  I2C_SR1_ADD10                       3            /*!< 10-bit header sent (Master mode) */
#define  I2C_SR1_STOPF                       4            /*!< Stop detection (Slave mode) */
#define  I2C_SR1_RXNE                        6            /*!< Data Register not Empty (receivers) */
#define  I2C_SR1_TXE                         7            /*!< Data Register Empty (transmitters) */
#define  I2C_SR1_BERR                        8            /*!< Bus Error */
#define  I2C_SR1_ARLO                        9            /*!< Arbitration Lost (master mode) */
#define  I2C_SR1_AF                          10           /*!< Acknowledge Failure */
#define  I2C_SR1_OVR                         11           /*!< Overrun/Underrun */
#define  I2C_SR1_PECERR                      12           /*!< PEC Error in reception */
#define  I2C_SR1_TIMEOUT                     14           /*!< Timeout or Tlow Error */
#define  I2C_SR1_SMBALERT                    15           /*!< SMBus Alert */

/*******************  Bit definition for I2C_SR2 register  ********************/
#define  I2C_SR2_MSL                         0            /*!< Master/Slave */
#define  I2C_SR2_BUSY                        1            /*!< Bus Busy */
#define  I2C_SR2_TRA                         2            /*!< Transmitter/Receiver */
#define  I2C_SR2_GENCALL                     4            /*!< General Call Address (Slave mode) */
#define  I2C_SR2_SMBDEFAULT                  5           /*!< SMBus Device Default Address (Slave mode) */
#define  I2C_SR2_SMBHOST                     6            /*!< SMBus Host Header (Slave mode) */
#define  I2C_SR2_DUALF                       7            /*!< Dual Flag (Slave mode) */
#define  I2C_SR2_PEC                         8            /*!< Packet Error Checking Register */

/*******************  Bit definition for I2C_CCR register  ********************/
#define  I2C_CCR_CCR                         0            /*!< Clock Control Register in Fast/Standard mode (Master mode) */
#define  I2C_CCR_DUTY                        14           /*!< Fast Mode Duty Cycle */
#define  I2C_CCR_FS                          15           /*!< I2C Master Mode Selection */

/******************************************************************************/
/*                                                                            */
/*                         Controller Area Network                            */
/*                                                                            */
/******************************************************************************/

/*!< CAN control and status registers */
/*******************  Bit definition for CAN_MCR register  ********************/
#define CAN_MCR_INRQ_Pos       (0U)
#define CAN_MCR_INRQ_Msk       (0x1UL << CAN_MCR_INRQ_Pos)                      /*!< 0x00000001 */
#define CAN_MCR_INRQ           CAN_MCR_INRQ_Msk                                /*!<Initialization Request */
#define CAN_MCR_SLEEP_Pos      (1U)
#define CAN_MCR_SLEEP_Msk      (0x1UL << CAN_MCR_SLEEP_Pos)                     /*!< 0x00000002 */
#define CAN_MCR_SLEEP          CAN_MCR_SLEEP_Msk                               /*!<Sleep Mode Request */
#define CAN_MCR_TXFP_Pos       (2U)
#define CAN_MCR_TXFP_Msk       (0x1UL << CAN_MCR_TXFP_Pos)                      /*!< 0x00000004 */
#define CAN_MCR_TXFP           CAN_MCR_TXFP_Msk                                /*!<Transmit FIFO Priority */
#define CAN_MCR_RFLM_Pos       (3U)
#define CAN_MCR_RFLM_Msk       (0x1UL << CAN_MCR_RFLM_Pos)                      /*!< 0x00000008 */
#define CAN_MCR_RFLM           CAN_MCR_RFLM_Msk                                /*!<Receive FIFO Locked Mode */
#define CAN_MCR_NART_Pos       (4U)
#define CAN_MCR_NART_Msk       (0x1UL << CAN_MCR_NART_Pos)                      /*!< 0x00000010 */
#define CAN_MCR_NART           CAN_MCR_NART_Msk                                /*!<No Automatic Retransmission */
#define CAN_MCR_AWUM_Pos       (5U)
#define CAN_MCR_AWUM_Msk       (0x1UL << CAN_MCR_AWUM_Pos)                      /*!< 0x00000020 */
#define CAN_MCR_AWUM           CAN_MCR_AWUM_Msk                                /*!<Automatic Wakeup Mode */
#define CAN_MCR_ABOM_Pos       (6U)
#define CAN_MCR_ABOM_Msk       (0x1UL << CAN_MCR_ABOM_Pos)                      /*!< 0x00000040 */
#define CAN_MCR_ABOM           CAN_MCR_ABOM_Msk                                /*!<Automatic Bus-Off Management */
#define CAN_MCR_TTCM_Pos       (7U)
#define CAN_MCR_TTCM_Msk       (0x1UL << CAN_MCR_TTCM_Pos)                      /*!< 0x00000080 */
#define CAN_MCR_TTCM           CAN_MCR_TTCM_Msk                                /*!<Time Triggered Communication Mode */
#define CAN_MCR_RESET_Pos      (15U)
#define CAN_MCR_RESET_Msk      (0x1UL << CAN_MCR_RESET_Pos)                     /*!< 0x00008000 */
#define CAN_MCR_RESET          CAN_MCR_RESET_Msk                               /*!<bxCAN software master reset */
#define CAN_MCR_DBF_Pos        (16U)
#define CAN_MCR_DBF_Msk        (0x1UL << CAN_MCR_DBF_Pos)                       /*!< 0x00010000 */
#define CAN_MCR_DBF            CAN_MCR_DBF_Msk                                 /*!<bxCAN Debug freeze */

/*******************  Bit definition for CAN_MSR register  ********************/
#define CAN_MSR_INAK_Pos       (0U)
#define CAN_MSR_INAK_Msk       (0x1UL << CAN_MSR_INAK_Pos)                      /*!< 0x00000001 */
#define CAN_MSR_INAK           CAN_MSR_INAK_Msk                                /*!<Initialization Acknowledge */
#define CAN_MSR_SLAK_Pos       (1U)
#define CAN_MSR_SLAK_Msk       (0x1UL << CAN_MSR_SLAK_Pos)                      /*!< 0x00000002 */
#define CAN_MSR_SLAK           CAN_MSR_SLAK_Msk                                /*!<Sleep Acknowledge */
#define CAN_MSR_ERRI_Pos       (2U)
#define CAN_MSR_ERRI_Msk       (0x1UL << CAN_MSR_ERRI_Pos)                      /*!< 0x00000004 */
#define CAN_MSR_ERRI           CAN_MSR_ERRI_Msk                                /*!<Error Interrupt */
#define CAN_MSR_WKUI_Pos       (3U)
#define CAN_MSR_WKUI_Msk       (0x1UL << CAN_MSR_WKUI_Pos)                      /*!< 0x00000008 */
#define CAN_MSR_WKUI           CAN_MSR_WKUI_Msk                                /*!<Wakeup Interrupt */
#define CAN_MSR_SLAKI_Pos      (4U)
#define CAN_MSR_SLAKI_Msk      (0x1UL << CAN_MSR_SLAKI_Pos)                     /*!< 0x00000010 */
#define CAN_MSR_SLAKI          CAN_MSR_SLAKI_Msk                               /*!<Sleep Acknowledge Interrupt */
#define CAN_MSR_TXM_Pos        (8U)
#define CAN_MSR_TXM_Msk        (0x1UL << CAN_MSR_TXM_Pos)                       /*!< 0x00000100 */
#define CAN_MSR_TXM            CAN_MSR_TXM_Msk                                 /*!<Transmit Mode */
#define CAN_MSR_RXM_Pos        (9U)
#define CAN_MSR_RXM_Msk        (0x1UL << CAN_MSR_RXM_Pos)                       /*!< 0x00000200 */
#define CAN_MSR_RXM            CAN_MSR_RXM_Msk                                 /*!<Receive Mode */
#define CAN_MSR_SAMP_Pos       (10U)
#define CAN_MSR_SAMP_Msk       (0x1UL << CAN_MSR_SAMP_Pos)                      /*!< 0x00000400 */
#define CAN_MSR_SAMP           CAN_MSR_SAMP_Msk                                /*!<Last Sample Point */
#define CAN_MSR_RX_Pos         (11U)
#define CAN_MSR_RX_Msk         (0x1UL << CAN_MSR_RX_Pos)                        /*!< 0x00000800 */
#define CAN_MSR_RX             CAN_MSR_RX_Msk                                  /*!<CAN Rx Signal */

/*******************  Bit definition for CAN_TSR register  ********************/
#define CAN_TSR_RQCP0_Pos      (0U)
#define CAN_TSR_RQCP0_Msk      (0x1UL << CAN_TSR_RQCP0_Pos)                     /*!< 0x00000001 */
#define CAN_TSR_RQCP0          CAN_TSR_RQCP0_Msk                               /*!<Request Completed Mailbox0 */
#define CAN_TSR_TXOK0_Pos      (1U)
#define CAN_TSR_TXOK0_Msk      (0x1UL << CAN_TSR_TXOK0_Pos)                     /*!< 0x00000002 */
#define CAN_TSR_TXOK0          CAN_TSR_TXOK0_Msk                               /*!<Transmission OK of Mailbox0 */
#define CAN_TSR_ALST0_Pos      (2U)
#define CAN_TSR_ALST0_Msk      (0x1UL << CAN_TSR_ALST0_Pos)                     /*!< 0x00000004 */
#define CAN_TSR_ALST0          CAN_TSR_ALST0_Msk                               /*!<Arbitration Lost for Mailbox0 */
#define CAN_TSR_TERR0_Pos      (3U)
#define CAN_TSR_TERR0_Msk      (0x1UL << CAN_TSR_TERR0_Pos)                     /*!< 0x00000008 */
#define CAN_TSR_TERR0          CAN_TSR_TERR0_Msk                               /*!<Transmission Error of Mailbox0 */
#define CAN_TSR_ABRQ0_Pos      (7U)
#define CAN_TSR_ABRQ0_Msk      (0x1UL << CAN_TSR_ABRQ0_Pos)                     /*!< 0x00000080 */
#define CAN_TSR_ABRQ0          CAN_TSR_ABRQ0_Msk                               /*!<Abort Request for Mailbox0 */
#define CAN_TSR_RQCP1_Pos      (8U)
#define CAN_TSR_RQCP1_Msk      (0x1UL << CAN_TSR_RQCP1_Pos)                     /*!< 0x00000100 */
#define CAN_TSR_RQCP1          CAN_TSR_RQCP1_Msk                               /*!<Request Completed Mailbox1 */
#define CAN_TSR_TXOK1_Pos      (9U)
#define CAN_TSR_TXOK1_Msk      (0x1UL << CAN_TSR_TXOK1_Pos)                     /*!< 0x00000200 */
#define CAN_TSR_TXOK1          CAN_TSR_TXOK1_Msk                               /*!<Transmission OK of Mailbox1 */
#define CAN_TSR_ALST1_Pos      (10U)
#define CAN_TSR_ALST1_Msk      (0x1UL << CAN_TSR_ALST1_Pos)                     /*!< 0x00000400 */
#define CAN_TSR_ALST1          CAN_TSR_ALST1_Msk                               /*!<Arbitration Lost for Mailbox1 */
#define CAN_TSR_TERR1_Pos      (11U)
#define CAN_TSR_TERR1_Msk      (0x1UL << CAN_TSR_TERR1_Pos)                     /*!< 0x00000800 */
#define CAN_TSR_TERR1          CAN_TSR_TERR1_Msk                               /*!<Transmission Error of Mailbox1 */
#define CAN_TSR_ABRQ1_Pos      (15U)
#define CAN_TSR_ABRQ1_Msk      (0x1UL << CAN_TSR_ABRQ1_Pos)                     /*!< 0x00008000 */
#define CAN_TSR_ABRQ1          CAN_TSR_ABRQ1_Msk                               /*!<Abort Request for Mailbox 1 */
#define CAN_TSR_RQCP2_Pos      (16U)
#define CAN_TSR_RQCP2_Msk      (0x1UL << CAN_TSR_RQCP2_Pos)                     /*!< 0x00010000 */
#define CAN_TSR_RQCP2          CAN_TSR_RQCP2_Msk                               /*!<Request Completed Mailbox2 */
#define CAN_TSR_TXOK2_Pos      (17U)
#define CAN_TSR_TXOK2_Msk      (0x1UL << CAN_TSR_TXOK2_Pos)                     /*!< 0x00020000 */
#define CAN_TSR_TXOK2          CAN_TSR_TXOK2_Msk                               /*!<Transmission OK of Mailbox 2 */
#define CAN_TSR_ALST2_Pos      (18U)
#define CAN_TSR_ALST2_Msk      (0x1UL << CAN_TSR_ALST2_Pos)                     /*!< 0x00040000 */
#define CAN_TSR_ALST2          CAN_TSR_ALST2_Msk                               /*!<Arbitration Lost for mailbox 2 */
#define CAN_TSR_TERR2_Pos      (19U)
#define CAN_TSR_TERR2_Msk      (0x1UL << CAN_TSR_TERR2_Pos)                     /*!< 0x00080000 */
#define CAN_TSR_TERR2          CAN_TSR_TERR2_Msk                               /*!<Transmission Error of Mailbox 2 */
#define CAN_TSR_ABRQ2_Pos      (23U)
#define CAN_TSR_ABRQ2_Msk      (0x1UL << CAN_TSR_ABRQ2_Pos)                     /*!< 0x00800000 */
#define CAN_TSR_ABRQ2          CAN_TSR_ABRQ2_Msk                               /*!<Abort Request for Mailbox 2 */
#define CAN_TSR_CODE_Pos       (24U)
#define CAN_TSR_CODE_Msk       (0x3UL << CAN_TSR_CODE_Pos)                      /*!< 0x03000000 */
#define CAN_TSR_CODE           CAN_TSR_CODE_Msk                                /*!<Mailbox Code */

#define CAN_TSR_TME_Pos        (26U)
#define CAN_TSR_TME_Msk        (0x7UL << CAN_TSR_TME_Pos)                       /*!< 0x1C000000 */
#define CAN_TSR_TME            CAN_TSR_TME_Msk                                 /*!<TME[2:0] bits */
#define CAN_TSR_TME0_Pos       (26U)
#define CAN_TSR_TME0_Msk       (0x1UL << CAN_TSR_TME0_Pos)                      /*!< 0x04000000 */
#define CAN_TSR_TME0           CAN_TSR_TME0_Msk                                /*!<Transmit Mailbox 0 Empty */
#define CAN_TSR_TME1_Pos       (27U)
#define CAN_TSR_TME1_Msk       (0x1UL << CAN_TSR_TME1_Pos)                      /*!< 0x08000000 */
#define CAN_TSR_TME1           CAN_TSR_TME1_Msk                                /*!<Transmit Mailbox 1 Empty */
#define CAN_TSR_TME2_Pos       (28U)
#define CAN_TSR_TME2_Msk       (0x1UL << CAN_TSR_TME2_Pos)                      /*!< 0x10000000 */
#define CAN_TSR_TME2           CAN_TSR_TME2_Msk                                /*!<Transmit Mailbox 2 Empty */

#define CAN_TSR_LOW_Pos        (29U)
#define CAN_TSR_LOW_Msk        (0x7UL << CAN_TSR_LOW_Pos)                       /*!< 0xE0000000 */
#define CAN_TSR_LOW            CAN_TSR_LOW_Msk                                 /*!<LOW[2:0] bits */
#define CAN_TSR_LOW0_Pos       (29U)
#define CAN_TSR_LOW0_Msk       (0x1UL << CAN_TSR_LOW0_Pos)                      /*!< 0x20000000 */
#define CAN_TSR_LOW0           CAN_TSR_LOW0_Msk                                /*!<Lowest Priority Flag for Mailbox 0 */
#define CAN_TSR_LOW1_Pos       (30U)
#define CAN_TSR_LOW1_Msk       (0x1UL << CAN_TSR_LOW1_Pos)                      /*!< 0x40000000 */
#define CAN_TSR_LOW1           CAN_TSR_LOW1_Msk                                /*!<Lowest Priority Flag for Mailbox 1 */
#define CAN_TSR_LOW2_Pos       (31U)
#define CAN_TSR_LOW2_Msk       (0x1UL << CAN_TSR_LOW2_Pos)                      /*!< 0x80000000 */
#define CAN_TSR_LOW2           CAN_TSR_LOW2_Msk                                /*!<Lowest Priority Flag for Mailbox 2 */


/*******************  Bit definition for CAN_RF0R register  *******************/
#define CAN_RF0R_FMP0_Pos      (0U)
#define CAN_RF0R_FMP0_Msk      (0x3UL << CAN_RF0R_FMP0_Pos)                     /*!< 0x00000003 */
#define CAN_RF0R_FMP0          CAN_RF0R_FMP0_Msk                               /*!<FIFO 0 Message Pending */
#define CAN_RF0R_FULL0_Pos     (3U)
#define CAN_RF0R_FULL0_Msk     (0x1UL << CAN_RF0R_FULL0_Pos)                    /*!< 0x00000008 */
#define CAN_RF0R_FULL0         CAN_RF0R_FULL0_Msk                              /*!<FIFO 0 Full */
#define CAN_RF0R_FOVR0_Pos     (4U)
#define CAN_RF0R_FOVR0_Msk     (0x1UL << CAN_RF0R_FOVR0_Pos)                    /*!< 0x00000010 */
#define CAN_RF0R_FOVR0         CAN_RF0R_FOVR0_Msk                              /*!<FIFO 0 Overrun */
#define CAN_RF0R_RFOM0_Pos     (5U)
#define CAN_RF0R_RFOM0_Msk     (0x1UL << CAN_RF0R_RFOM0_Pos)                    /*!< 0x00000020 */
#define CAN_RF0R_RFOM0         CAN_RF0R_RFOM0_Msk                              /*!<Release FIFO 0 Output Mailbox */

/*******************  Bit definition for CAN_RF1R register  *******************/
#define CAN_RF1R_FMP1_Pos      (0U)
#define CAN_RF1R_FMP1_Msk      (0x3UL << CAN_RF1R_FMP1_Pos)                     /*!< 0x00000003 */
#define CAN_RF1R_FMP1          CAN_RF1R_FMP1_Msk                               /*!<FIFO 1 Message Pending */
#define CAN_RF1R_FULL1_Pos     (3U)
#define CAN_RF1R_FULL1_Msk     (0x1UL << CAN_RF1R_FULL1_Pos)                    /*!< 0x00000008 */
#define CAN_RF1R_FULL1         CAN_RF1R_FULL1_Msk                              /*!<FIFO 1 Full */
#define CAN_RF1R_FOVR1_Pos     (4U)
#define CAN_RF1R_FOVR1_Msk     (0x1UL << CAN_RF1R_FOVR1_Pos)                    /*!< 0x00000010 */
#define CAN_RF1R_FOVR1         CAN_RF1R_FOVR1_Msk                              /*!<FIFO 1 Overrun */
#define CAN_RF1R_RFOM1_Pos     (5U)
#define CAN_RF1R_RFOM1_Msk     (0x1UL << CAN_RF1R_RFOM1_Pos)                    /*!< 0x00000020 */
#define CAN_RF1R_RFOM1         CAN_RF1R_RFOM1_Msk                              /*!<Release FIFO 1 Output Mailbox */

/********************  Bit definition for CAN_IER register  *******************/
#define CAN_IER_TMEIE_Pos      (0U)
#define CAN_IER_TMEIE_Msk      (0x1UL << CAN_IER_TMEIE_Pos)                     /*!< 0x00000001 */
#define CAN_IER_TMEIE          CAN_IER_TMEIE_Msk                               /*!<Transmit Mailbox Empty Interrupt Enable */
#define CAN_IER_FMPIE0_Pos     (1U)
#define CAN_IER_FMPIE0_Msk     (0x1UL << CAN_IER_FMPIE0_Pos)                    /*!< 0x00000002 */
#define CAN_IER_FMPIE0         CAN_IER_FMPIE0_Msk                              /*!<FIFO Message Pending Interrupt Enable */
#define CAN_IER_FFIE0_Pos      (2U)
#define CAN_IER_FFIE0_Msk      (0x1UL << CAN_IER_FFIE0_Pos)                     /*!< 0x00000004 */
#define CAN_IER_FFIE0          CAN_IER_FFIE0_Msk                               /*!<FIFO Full Interrupt Enable */
#define CAN_IER_FOVIE0_Pos     (3U)
#define CAN_IER_FOVIE0_Msk     (0x1UL << CAN_IER_FOVIE0_Pos)                    /*!< 0x00000008 */
#define CAN_IER_FOVIE0         CAN_IER_FOVIE0_Msk                              /*!<FIFO Overrun Interrupt Enable */
#define CAN_IER_FMPIE1_Pos     (4U)
#define CAN_IER_FMPIE1_Msk     (0x1UL << CAN_IER_FMPIE1_Pos)                    /*!< 0x00000010 */
#define CAN_IER_FMPIE1         CAN_IER_FMPIE1_Msk                              /*!<FIFO Message Pending Interrupt Enable */
#define CAN_IER_FFIE1_Pos      (5U)
#define CAN_IER_FFIE1_Msk      (0x1UL << CAN_IER_FFIE1_Pos)                     /*!< 0x00000020 */
#define CAN_IER_FFIE1          CAN_IER_FFIE1_Msk                               /*!<FIFO Full Interrupt Enable */
#define CAN_IER_FOVIE1_Pos     (6U)
#define CAN_IER_FOVIE1_Msk     (0x1UL << CAN_IER_FOVIE1_Pos)                    /*!< 0x00000040 */
#define CAN_IER_FOVIE1         CAN_IER_FOVIE1_Msk                              /*!<FIFO Overrun Interrupt Enable */
#define CAN_IER_EWGIE_Pos      (8U)
#define CAN_IER_EWGIE_Msk      (0x1UL << CAN_IER_EWGIE_Pos)                     /*!< 0x00000100 */
#define CAN_IER_EWGIE          CAN_IER_EWGIE_Msk                               /*!<Error Warning Interrupt Enable */
#define CAN_IER_EPVIE_Pos      (9U)
#define CAN_IER_EPVIE_Msk      (0x1UL << CAN_IER_EPVIE_Pos)                     /*!< 0x00000200 */
#define CAN_IER_EPVIE          CAN_IER_EPVIE_Msk                               /*!<Error Passive Interrupt Enable */
#define CAN_IER_BOFIE_Pos      (10U)
#define CAN_IER_BOFIE_Msk      (0x1UL << CAN_IER_BOFIE_Pos)                     /*!< 0x00000400 */
#define CAN_IER_BOFIE          CAN_IER_BOFIE_Msk                               /*!<Bus-Off Interrupt Enable */
#define CAN_IER_LECIE_Pos      (11U)
#define CAN_IER_LECIE_Msk      (0x1UL << CAN_IER_LECIE_Pos)                     /*!< 0x00000800 */
#define CAN_IER_LECIE          CAN_IER_LECIE_Msk                               /*!<Last Error Code Interrupt Enable */
#define CAN_IER_ERRIE_Pos      (15U)
#define CAN_IER_ERRIE_Msk      (0x1UL << CAN_IER_ERRIE_Pos)                     /*!< 0x00008000 */
#define CAN_IER_ERRIE          CAN_IER_ERRIE_Msk                               /*!<Error Interrupt Enable */
#define CAN_IER_WKUIE_Pos      (16U)
#define CAN_IER_WKUIE_Msk      (0x1UL << CAN_IER_WKUIE_Pos)                     /*!< 0x00010000 */
#define CAN_IER_WKUIE          CAN_IER_WKUIE_Msk                               /*!<Wakeup Interrupt Enable */
#define CAN_IER_SLKIE_Pos      (17U)
#define CAN_IER_SLKIE_Msk      (0x1UL << CAN_IER_SLKIE_Pos)                     /*!< 0x00020000 */
#define CAN_IER_SLKIE          CAN_IER_SLKIE_Msk                               /*!<Sleep Interrupt Enable */
#define CAN_IER_EWGIE_Pos      (8U)

/********************  Bit definition for CAN_ESR register  *******************/
#define CAN_ESR_EWGF_Pos       (0U)
#define CAN_ESR_EWGF_Msk       (0x1UL << CAN_ESR_EWGF_Pos)                      /*!< 0x00000001 */
#define CAN_ESR_EWGF           CAN_ESR_EWGF_Msk                                /*!<Error Warning Flag */
#define CAN_ESR_EPVF_Pos       (1U)
#define CAN_ESR_EPVF_Msk       (0x1UL << CAN_ESR_EPVF_Pos)                      /*!< 0x00000002 */
#define CAN_ESR_EPVF           CAN_ESR_EPVF_Msk                                /*!<Error Passive Flag */
#define CAN_ESR_BOFF_Pos       (2U)
#define CAN_ESR_BOFF_Msk       (0x1UL << CAN_ESR_BOFF_Pos)                      /*!< 0x00000004 */
#define CAN_ESR_BOFF           CAN_ESR_BOFF_Msk                                /*!<Bus-Off Flag */

#define CAN_ESR_LEC_Pos        (4U)
#define CAN_ESR_LEC_Msk        (0x7UL << CAN_ESR_LEC_Pos)                       /*!< 0x00000070 */
#define CAN_ESR_LEC            CAN_ESR_LEC_Msk                                 /*!<LEC[2:0] bits (Last Error Code) */
#define CAN_ESR_LEC_0          (0x1UL << CAN_ESR_LEC_Pos)                       /*!< 0x00000010 */
#define CAN_ESR_LEC_1          (0x2UL << CAN_ESR_LEC_Pos)                       /*!< 0x00000020 */
#define CAN_ESR_LEC_2          (0x4UL << CAN_ESR_LEC_Pos)                       /*!< 0x00000040 */

#define CAN_ESR_TEC_Pos        (16U)
#define CAN_ESR_TEC_Msk        (0xFFUL << CAN_ESR_TEC_Pos)                      /*!< 0x00FF0000 */
#define CAN_ESR_TEC            CAN_ESR_TEC_Msk                                 /*!<Least significant byte of the 9-bit Transmit Error Counter */
#define CAN_ESR_REC_Pos        (24U)
#define CAN_ESR_REC_Msk        (0xFFUL << CAN_ESR_REC_Pos)                      /*!< 0xFF000000 */
#define CAN_ESR_REC            CAN_ESR_REC_Msk                                 /*!<Receive Error Counter */

/*******************  Bit definition for CAN_BTR register  ********************/
#define CAN_BTR_BRP_Pos        (0U)
#define CAN_BTR_BRP_Msk        (0x3FFUL << CAN_BTR_BRP_Pos)                     /*!< 0x000003FF */
#define CAN_BTR_BRP            CAN_BTR_BRP_Msk                                 /*!<Baud Rate Prescaler */
#define CAN_BTR_TS1_Pos        (16U)
#define CAN_BTR_TS1_Msk        (0xFUL << CAN_BTR_TS1_Pos)                       /*!< 0x000F0000 */
#define CAN_BTR_TS1            CAN_BTR_TS1_Msk                                 /*!<Time Segment 1 */
#define CAN_BTR_TS1_0          (0x1UL << CAN_BTR_TS1_Pos)                       /*!< 0x00010000 */
#define CAN_BTR_TS1_1          (0x2UL << CAN_BTR_TS1_Pos)                       /*!< 0x00020000 */
#define CAN_BTR_TS1_2          (0x4UL << CAN_BTR_TS1_Pos)                       /*!< 0x00040000 */
#define CAN_BTR_TS1_3          (0x8UL << CAN_BTR_TS1_Pos)                       /*!< 0x00080000 */
#define CAN_BTR_TS2_Pos        (20U)
#define CAN_BTR_TS2_Msk        (0x7UL << CAN_BTR_TS2_Pos)                       /*!< 0x00700000 */
#define CAN_BTR_TS2            CAN_BTR_TS2_Msk                                 /*!<Time Segment 2 */
#define CAN_BTR_TS2_0          (0x1UL << CAN_BTR_TS2_Pos)                       /*!< 0x00100000 */
#define CAN_BTR_TS2_1          (0x2UL << CAN_BTR_TS2_Pos)                       /*!< 0x00200000 */
#define CAN_BTR_TS2_2          (0x4UL << CAN_BTR_TS2_Pos)                       /*!< 0x00400000 */
#define CAN_BTR_SJW_Pos        (24U)
#define CAN_BTR_SJW_Msk        (0x3UL << CAN_BTR_SJW_Pos)                       /*!< 0x03000000 */
#define CAN_BTR_SJW            CAN_BTR_SJW_Msk                                 /*!<Resynchronization Jump Width */
#define CAN_BTR_SJW_0          (0x1UL << CAN_BTR_SJW_Pos)                       /*!< 0x01000000 */
#define CAN_BTR_SJW_1          (0x2UL << CAN_BTR_SJW_Pos)                       /*!< 0x02000000 */
#define CAN_BTR_LBKM_Pos       (30U)
#define CAN_BTR_LBKM_Msk       (0x1UL << CAN_BTR_LBKM_Pos)                      /*!< 0x40000000 */
#define CAN_BTR_LBKM           CAN_BTR_LBKM_Msk                                /*!<Loop Back Mode (Debug) */
#define CAN_BTR_SILM_Pos       (31U)
#define CAN_BTR_SILM_Msk       (0x1UL << CAN_BTR_SILM_Pos)                      /*!< 0x80000000 */
#define CAN_BTR_SILM           CAN_BTR_SILM_Msk                                /*!<Silent Mode */

/*!<Mailbox registers */
/******************  Bit definition for CAN_TI0R register  ********************/
#define CAN_TI0R_TXRQ_Pos      (0U)
#define CAN_TI0R_TXRQ_Msk      (0x1UL << CAN_TI0R_TXRQ_Pos)                     /*!< 0x00000001 */
#define CAN_TI0R_TXRQ          CAN_TI0R_TXRQ_Msk                               /*!<Transmit Mailbox Request */
#define CAN_TI0R_RTR_Pos       (1U)
#define CAN_TI0R_RTR_Msk       (0x1UL << CAN_TI0R_RTR_Pos)                      /*!< 0x00000002 */
#define CAN_TI0R_RTR           CAN_TI0R_RTR_Msk                                /*!<Remote Transmission Request */
#define CAN_TI0R_IDE_Pos       (2U)
#define CAN_TI0R_IDE_Msk       (0x1UL << CAN_TI0R_IDE_Pos)                      /*!< 0x00000004 */
#define CAN_TI0R_IDE           CAN_TI0R_IDE_Msk                                /*!<Identifier Extension */
#define CAN_TI0R_EXID_Pos      (3U)
#define CAN_TI0R_EXID_Msk      (0x3FFFFUL << CAN_TI0R_EXID_Pos)                 /*!< 0x001FFFF8 */
#define CAN_TI0R_EXID          CAN_TI0R_EXID_Msk                               /*!<Extended Identifier */
#define CAN_TI0R_STID_Pos      (21U)
#define CAN_TI0R_STID_Msk      (0x7FFUL << CAN_TI0R_STID_Pos)                   /*!< 0xFFE00000 */
#define CAN_TI0R_STID          CAN_TI0R_STID_Msk                               /*!<Standard Identifier or Extended Identifier */

/******************  Bit definition for CAN_TDT0R register  *******************/
#define CAN_TDT0R_DLC_Pos      (0U)
#define CAN_TDT0R_DLC_Msk      (0xFUL << CAN_TDT0R_DLC_Pos)                     /*!< 0x0000000F */
#define CAN_TDT0R_DLC          CAN_TDT0R_DLC_Msk                               /*!<Data Length Code */
#define CAN_TDT0R_TGT_Pos      (8U)
#define CAN_TDT0R_TGT_Msk      (0x1UL << CAN_TDT0R_TGT_Pos)                     /*!< 0x00000100 */
#define CAN_TDT0R_TGT          CAN_TDT0R_TGT_Msk                               /*!<Transmit Global Time */
#define CAN_TDT0R_TIME_Pos     (16U)
#define CAN_TDT0R_TIME_Msk     (0xFFFFUL << CAN_TDT0R_TIME_Pos)                 /*!< 0xFFFF0000 */
#define CAN_TDT0R_TIME         CAN_TDT0R_TIME_Msk                              /*!<Message Time Stamp */

/******************  Bit definition for CAN_TDL0R register  *******************/
#define CAN_TDL0R_DATA0_Pos    (0U)
#define CAN_TDL0R_DATA0_Msk    (0xFFUL << CAN_TDL0R_DATA0_Pos)                  /*!< 0x000000FF */
#define CAN_TDL0R_DATA0        CAN_TDL0R_DATA0_Msk                             /*!<Data byte 0 */
#define CAN_TDL0R_DATA1_Pos    (8U)
#define CAN_TDL0R_DATA1_Msk    (0xFFUL << CAN_TDL0R_DATA1_Pos)                  /*!< 0x0000FF00 */
#define CAN_TDL0R_DATA1        CAN_TDL0R_DATA1_Msk                             /*!<Data byte 1 */
#define CAN_TDL0R_DATA2_Pos    (16U)
#define CAN_TDL0R_DATA2_Msk    (0xFFUL << CAN_TDL0R_DATA2_Pos)                  /*!< 0x00FF0000 */
#define CAN_TDL0R_DATA2        CAN_TDL0R_DATA2_Msk                             /*!<Data byte 2 */
#define CAN_TDL0R_DATA3_Pos    (24U)
#define CAN_TDL0R_DATA3_Msk    (0xFFUL << CAN_TDL0R_DATA3_Pos)                  /*!< 0xFF000000 */
#define CAN_TDL0R_DATA3        CAN_TDL0R_DATA3_Msk                             /*!<Data byte 3 */

/******************  Bit definition for CAN_TDH0R register  *******************/
#define CAN_TDH0R_DATA4_Pos    (0U)
#define CAN_TDH0R_DATA4_Msk    (0xFFUL << CAN_TDH0R_DATA4_Pos)                  /*!< 0x000000FF */
#define CAN_TDH0R_DATA4        CAN_TDH0R_DATA4_Msk                             /*!<Data byte 4 */
#define CAN_TDH0R_DATA5_Pos    (8U)
#define CAN_TDH0R_DATA5_Msk    (0xFFUL << CAN_TDH0R_DATA5_Pos)                  /*!< 0x0000FF00 */
#define CAN_TDH0R_DATA5        CAN_TDH0R_DATA5_Msk                             /*!<Data byte 5 */
#define CAN_TDH0R_DATA6_Pos    (16U)
#define CAN_TDH0R_DATA6_Msk    (0xFFUL << CAN_TDH0R_DATA6_Pos)                  /*!< 0x00FF0000 */
#define CAN_TDH0R_DATA6        CAN_TDH0R_DATA6_Msk                             /*!<Data byte 6 */
#define CAN_TDH0R_DATA7_Pos    (24U)
#define CAN_TDH0R_DATA7_Msk    (0xFFUL << CAN_TDH0R_DATA7_Pos)                  /*!< 0xFF000000 */
#define CAN_TDH0R_DATA7        CAN_TDH0R_DATA7_Msk                             /*!<Data byte 7 */

/*******************  Bit definition for CAN_TI1R register  *******************/
#define CAN_TI1R_TXRQ_Pos      (0U)
#define CAN_TI1R_TXRQ_Msk      (0x1UL << CAN_TI1R_TXRQ_Pos)                     /*!< 0x00000001 */
#define CAN_TI1R_TXRQ          CAN_TI1R_TXRQ_Msk                               /*!<Transmit Mailbox Request */
#define CAN_TI1R_RTR_Pos       (1U)
#define CAN_TI1R_RTR_Msk       (0x1UL << CAN_TI1R_RTR_Pos)                      /*!< 0x00000002 */
#define CAN_TI1R_RTR           CAN_TI1R_RTR_Msk                                /*!<Remote Transmission Request */
#define CAN_TI1R_IDE_Pos       (2U)
#define CAN_TI1R_IDE_Msk       (0x1UL << CAN_TI1R_IDE_Pos)                      /*!< 0x00000004 */
#define CAN_TI1R_IDE           CAN_TI1R_IDE_Msk                                /*!<Identifier Extension */
#define CAN_TI1R_EXID_Pos      (3U)
#define CAN_TI1R_EXID_Msk      (0x3FFFFUL << CAN_TI1R_EXID_Pos)                 /*!< 0x001FFFF8 */
#define CAN_TI1R_EXID          CAN_TI1R_EXID_Msk                               /*!<Extended Identifier */
#define CAN_TI1R_STID_Pos      (21U)
#define CAN_TI1R_STID_Msk      (0x7FFUL << CAN_TI1R_STID_Pos)                   /*!< 0xFFE00000 */
#define CAN_TI1R_STID          CAN_TI1R_STID_Msk                               /*!<Standard Identifier or Extended Identifier */

/*******************  Bit definition for CAN_TDT1R register  ******************/
#define CAN_TDT1R_DLC_Pos      (0U)
#define CAN_TDT1R_DLC_Msk      (0xFUL << CAN_TDT1R_DLC_Pos)                     /*!< 0x0000000F */
#define CAN_TDT1R_DLC          CAN_TDT1R_DLC_Msk                               /*!<Data Length Code */
#define CAN_TDT1R_TGT_Pos      (8U)
#define CAN_TDT1R_TGT_Msk      (0x1UL << CAN_TDT1R_TGT_Pos)                     /*!< 0x00000100 */
#define CAN_TDT1R_TGT          CAN_TDT1R_TGT_Msk                               /*!<Transmit Global Time */
#define CAN_TDT1R_TIME_Pos     (16U)
#define CAN_TDT1R_TIME_Msk     (0xFFFFUL << CAN_TDT1R_TIME_Pos)                 /*!< 0xFFFF0000 */
#define CAN_TDT1R_TIME         CAN_TDT1R_TIME_Msk                              /*!<Message Time Stamp */

/*******************  Bit definition for CAN_TDL1R register  ******************/
#define CAN_TDL1R_DATA0_Pos    (0U)
#define CAN_TDL1R_DATA0_Msk    (0xFFUL << CAN_TDL1R_DATA0_Pos)                  /*!< 0x000000FF */
#define CAN_TDL1R_DATA0        CAN_TDL1R_DATA0_Msk                             /*!<Data byte 0 */
#define CAN_TDL1R_DATA1_Pos    (8U)
#define CAN_TDL1R_DATA1_Msk    (0xFFUL << CAN_TDL1R_DATA1_Pos)                  /*!< 0x0000FF00 */
#define CAN_TDL1R_DATA1        CAN_TDL1R_DATA1_Msk                             /*!<Data byte 1 */
#define CAN_TDL1R_DATA2_Pos    (16U)
#define CAN_TDL1R_DATA2_Msk    (0xFFUL << CAN_TDL1R_DATA2_Pos)                  /*!< 0x00FF0000 */
#define CAN_TDL1R_DATA2        CAN_TDL1R_DATA2_Msk                             /*!<Data byte 2 */
#define CAN_TDL1R_DATA3_Pos    (24U)
#define CAN_TDL1R_DATA3_Msk    (0xFFUL << CAN_TDL1R_DATA3_Pos)                  /*!< 0xFF000000 */
#define CAN_TDL1R_DATA3        CAN_TDL1R_DATA3_Msk                             /*!<Data byte 3 */

/*******************  Bit definition for CAN_TDH1R register  ******************/
#define CAN_TDH1R_DATA4_Pos    (0U)
#define CAN_TDH1R_DATA4_Msk    (0xFFUL << CAN_TDH1R_DATA4_Pos)                  /*!< 0x000000FF */
#define CAN_TDH1R_DATA4        CAN_TDH1R_DATA4_Msk                             /*!<Data byte 4 */
#define CAN_TDH1R_DATA5_Pos    (8U)
#define CAN_TDH1R_DATA5_Msk    (0xFFUL << CAN_TDH1R_DATA5_Pos)                  /*!< 0x0000FF00 */
#define CAN_TDH1R_DATA5        CAN_TDH1R_DATA5_Msk                             /*!<Data byte 5 */
#define CAN_TDH1R_DATA6_Pos    (16U)
#define CAN_TDH1R_DATA6_Msk    (0xFFUL << CAN_TDH1R_DATA6_Pos)                  /*!< 0x00FF0000 */
#define CAN_TDH1R_DATA6        CAN_TDH1R_DATA6_Msk                             /*!<Data byte 6 */
#define CAN_TDH1R_DATA7_Pos    (24U)
#define CAN_TDH1R_DATA7_Msk    (0xFFUL << CAN_TDH1R_DATA7_Pos)                  /*!< 0xFF000000 */
#define CAN_TDH1R_DATA7        CAN_TDH1R_DATA7_Msk                             /*!<Data byte 7 */

/*******************  Bit definition for CAN_TI2R register  *******************/
#define CAN_TI2R_TXRQ_Pos      (0U)
#define CAN_TI2R_TXRQ_Msk      (0x1UL << CAN_TI2R_TXRQ_Pos)                     /*!< 0x00000001 */
#define CAN_TI2R_TXRQ          CAN_TI2R_TXRQ_Msk                               /*!<Transmit Mailbox Request */
#define CAN_TI2R_RTR_Pos       (1U)
#define CAN_TI2R_RTR_Msk       (0x1UL << CAN_TI2R_RTR_Pos)                      /*!< 0x00000002 */
#define CAN_TI2R_RTR           CAN_TI2R_RTR_Msk                                /*!<Remote Transmission Request */
#define CAN_TI2R_IDE_Pos       (2U)
#define CAN_TI2R_IDE_Msk       (0x1UL << CAN_TI2R_IDE_Pos)                      /*!< 0x00000004 */
#define CAN_TI2R_IDE           CAN_TI2R_IDE_Msk                                /*!<Identifier Extension */
#define CAN_TI2R_EXID_Pos      (3U)
#define CAN_TI2R_EXID_Msk      (0x3FFFFUL << CAN_TI2R_EXID_Pos)                 /*!< 0x001FFFF8 */
#define CAN_TI2R_EXID          CAN_TI2R_EXID_Msk                               /*!<Extended identifier */
#define CAN_TI2R_STID_Pos      (21U)
#define CAN_TI2R_STID_Msk      (0x7FFUL << CAN_TI2R_STID_Pos)                   /*!< 0xFFE00000 */
#define CAN_TI2R_STID          CAN_TI2R_STID_Msk                               /*!<Standard Identifier or Extended Identifier */

/*******************  Bit definition for CAN_TDT2R register  ******************/
#define CAN_TDT2R_DLC_Pos      (0U)
#define CAN_TDT2R_DLC_Msk      (0xFUL << CAN_TDT2R_DLC_Pos)                     /*!< 0x0000000F */
#define CAN_TDT2R_DLC          CAN_TDT2R_DLC_Msk                               /*!<Data Length Code */
#define CAN_TDT2R_TGT_Pos      (8U)
#define CAN_TDT2R_TGT_Msk      (0x1UL << CAN_TDT2R_TGT_Pos)                     /*!< 0x00000100 */
#define CAN_TDT2R_TGT          CAN_TDT2R_TGT_Msk                               /*!<Transmit Global Time */
#define CAN_TDT2R_TIME_Pos     (16U)
#define CAN_TDT2R_TIME_Msk     (0xFFFFUL << CAN_TDT2R_TIME_Pos)                 /*!< 0xFFFF0000 */
#define CAN_TDT2R_TIME         CAN_TDT2R_TIME_Msk                              /*!<Message Time Stamp */

/*******************  Bit definition for CAN_TDL2R register  ******************/
#define CAN_TDL2R_DATA0_Pos    (0U)
#define CAN_TDL2R_DATA0_Msk    (0xFFUL << CAN_TDL2R_DATA0_Pos)                  /*!< 0x000000FF */
#define CAN_TDL2R_DATA0        CAN_TDL2R_DATA0_Msk                             /*!<Data byte 0 */
#define CAN_TDL2R_DATA1_Pos    (8U)
#define CAN_TDL2R_DATA1_Msk    (0xFFUL << CAN_TDL2R_DATA1_Pos)                  /*!< 0x0000FF00 */
#define CAN_TDL2R_DATA1        CAN_TDL2R_DATA1_Msk                             /*!<Data byte 1 */
#define CAN_TDL2R_DATA2_Pos    (16U)
#define CAN_TDL2R_DATA2_Msk    (0xFFUL << CAN_TDL2R_DATA2_Pos)                  /*!< 0x00FF0000 */
#define CAN_TDL2R_DATA2        CAN_TDL2R_DATA2_Msk                             /*!<Data byte 2 */
#define CAN_TDL2R_DATA3_Pos    (24U)
#define CAN_TDL2R_DATA3_Msk    (0xFFUL << CAN_TDL2R_DATA3_Pos)                  /*!< 0xFF000000 */
#define CAN_TDL2R_DATA3        CAN_TDL2R_DATA3_Msk                             /*!<Data byte 3 */

/*******************  Bit definition for CAN_TDH2R register  ******************/
#define CAN_TDH2R_DATA4_Pos    (0U)
#define CAN_TDH2R_DATA4_Msk    (0xFFUL << CAN_TDH2R_DATA4_Pos)                  /*!< 0x000000FF */
#define CAN_TDH2R_DATA4        CAN_TDH2R_DATA4_Msk                             /*!<Data byte 4 */
#define CAN_TDH2R_DATA5_Pos    (8U)
#define CAN_TDH2R_DATA5_Msk    (0xFFUL << CAN_TDH2R_DATA5_Pos)                  /*!< 0x0000FF00 */
#define CAN_TDH2R_DATA5        CAN_TDH2R_DATA5_Msk                             /*!<Data byte 5 */
#define CAN_TDH2R_DATA6_Pos    (16U)
#define CAN_TDH2R_DATA6_Msk    (0xFFUL << CAN_TDH2R_DATA6_Pos)                  /*!< 0x00FF0000 */
#define CAN_TDH2R_DATA6        CAN_TDH2R_DATA6_Msk                             /*!<Data byte 6 */
#define CAN_TDH2R_DATA7_Pos    (24U)
#define CAN_TDH2R_DATA7_Msk    (0xFFUL << CAN_TDH2R_DATA7_Pos)                  /*!< 0xFF000000 */
#define CAN_TDH2R_DATA7        CAN_TDH2R_DATA7_Msk                             /*!<Data byte 7 */

/*******************  Bit definition for CAN_RI0R register  *******************/
#define CAN_RI0R_RTR_Pos       (1U)
#define CAN_RI0R_RTR_Msk       (0x1UL << CAN_RI0R_RTR_Pos)                      /*!< 0x00000002 */
#define CAN_RI0R_RTR           CAN_RI0R_RTR_Msk                                /*!<Remote Transmission Request */
#define CAN_RI0R_IDE_Pos       (2U)
#define CAN_RI0R_IDE_Msk       (0x1UL << CAN_RI0R_IDE_Pos)                      /*!< 0x00000004 */
#define CAN_RI0R_IDE           CAN_RI0R_IDE_Msk                                /*!<Identifier Extension */
#define CAN_RI0R_EXID_Pos      (3U)
#define CAN_RI0R_EXID_Msk      (0x3FFFFUL << CAN_RI0R_EXID_Pos)                 /*!< 0x001FFFF8 */
#define CAN_RI0R_EXID          CAN_RI0R_EXID_Msk                               /*!<Extended Identifier */
#define CAN_RI0R_STID_Pos      (21U)
#define CAN_RI0R_STID_Msk      (0x7FFUL << CAN_RI0R_STID_Pos)                   /*!< 0xFFE00000 */
#define CAN_RI0R_STID          CAN_RI0R_STID_Msk                               /*!<Standard Identifier or Extended Identifier */

/*******************  Bit definition for CAN_RDT0R register  ******************/
#define CAN_RDT0R_DLC_Pos      (0U)
#define CAN_RDT0R_DLC_Msk      (0xFUL << CAN_RDT0R_DLC_Pos)                     /*!< 0x0000000F */
#define CAN_RDT0R_DLC          CAN_RDT0R_DLC_Msk                               /*!<Data Length Code */
#define CAN_RDT0R_FMI_Pos      (8U)
#define CAN_RDT0R_FMI_Msk      (0xFFUL << CAN_RDT0R_FMI_Pos)                    /*!< 0x0000FF00 */
#define CAN_RDT0R_FMI          CAN_RDT0R_FMI_Msk                               /*!<Filter Match Index */
#define CAN_RDT0R_TIME_Pos     (16U)
#define CAN_RDT0R_TIME_Msk     (0xFFFFUL << CAN_RDT0R_TIME_Pos)                 /*!< 0xFFFF0000 */
#define CAN_RDT0R_TIME         CAN_RDT0R_TIME_Msk                              /*!<Message Time Stamp */

/*******************  Bit definition for CAN_RDL0R register  ******************/
#define CAN_RDL0R_DATA0_Pos    (0U)
#define CAN_RDL0R_DATA0_Msk    (0xFFUL << CAN_RDL0R_DATA0_Pos)                  /*!< 0x000000FF */
#define CAN_RDL0R_DATA0        CAN_RDL0R_DATA0_Msk                             /*!<Data byte 0 */
#define CAN_RDL0R_DATA1_Pos    (8U)
#define CAN_RDL0R_DATA1_Msk    (0xFFUL << CAN_RDL0R_DATA1_Pos)                  /*!< 0x0000FF00 */
#define CAN_RDL0R_DATA1        CAN_RDL0R_DATA1_Msk                             /*!<Data byte 1 */
#define CAN_RDL0R_DATA2_Pos    (16U)
#define CAN_RDL0R_DATA2_Msk    (0xFFUL << CAN_RDL0R_DATA2_Pos)                  /*!< 0x00FF0000 */
#define CAN_RDL0R_DATA2        CAN_RDL0R_DATA2_Msk                             /*!<Data byte 2 */
#define CAN_RDL0R_DATA3_Pos    (24U)
#define CAN_RDL0R_DATA3_Msk    (0xFFUL << CAN_RDL0R_DATA3_Pos)                  /*!< 0xFF000000 */
#define CAN_RDL0R_DATA3        CAN_RDL0R_DATA3_Msk                             /*!<Data byte 3 */

/*******************  Bit definition for CAN_RDH0R register  ******************/
#define CAN_RDH0R_DATA4_Pos    (0U)
#define CAN_RDH0R_DATA4_Msk    (0xFFUL << CAN_RDH0R_DATA4_Pos)                  /*!< 0x000000FF */
#define CAN_RDH0R_DATA4        CAN_RDH0R_DATA4_Msk                             /*!<Data byte 4 */
#define CAN_RDH0R_DATA5_Pos    (8U)
#define CAN_RDH0R_DATA5_Msk    (0xFFUL << CAN_RDH0R_DATA5_Pos)                  /*!< 0x0000FF00 */
#define CAN_RDH0R_DATA5        CAN_RDH0R_DATA5_Msk                             /*!<Data byte 5 */
#define CAN_RDH0R_DATA6_Pos    (16U)
#define CAN_RDH0R_DATA6_Msk    (0xFFUL << CAN_RDH0R_DATA6_Pos)                  /*!< 0x00FF0000 */
#define CAN_RDH0R_DATA6        CAN_RDH0R_DATA6_Msk                             /*!<Data byte 6 */
#define CAN_RDH0R_DATA7_Pos    (24U)
#define CAN_RDH0R_DATA7_Msk    (0xFFUL << CAN_RDH0R_DATA7_Pos)                  /*!< 0xFF000000 */
#define CAN_RDH0R_DATA7        CAN_RDH0R_DATA7_Msk                             /*!<Data byte 7 */

/*******************  Bit definition for CAN_RI1R register  *******************/
#define CAN_RI1R_RTR_Pos       (1U)
#define CAN_RI1R_RTR_Msk       (0x1UL << CAN_RI1R_RTR_Pos)                      /*!< 0x00000002 */
#define CAN_RI1R_RTR           CAN_RI1R_RTR_Msk                                /*!<Remote Transmission Request */
#define CAN_RI1R_IDE_Pos       (2U)
#define CAN_RI1R_IDE_Msk       (0x1UL << CAN_RI1R_IDE_Pos)                      /*!< 0x00000004 */
#define CAN_RI1R_IDE           CAN_RI1R_IDE_Msk                                /*!<Identifier Extension */
#define CAN_RI1R_EXID_Pos      (3U)
#define CAN_RI1R_EXID_Msk      (0x3FFFFUL << CAN_RI1R_EXID_Pos)                 /*!< 0x001FFFF8 */
#define CAN_RI1R_EXID          CAN_RI1R_EXID_Msk                               /*!<Extended identifier */
#define CAN_RI1R_STID_Pos      (21U)
#define CAN_RI1R_STID_Msk      (0x7FFUL << CAN_RI1R_STID_Pos)                   /*!< 0xFFE00000 */
#define CAN_RI1R_STID          CAN_RI1R_STID_Msk                               /*!<Standard Identifier or Extended Identifier */

/*******************  Bit definition for CAN_RDT1R register  ******************/
#define CAN_RDT1R_DLC_Pos      (0U)
#define CAN_RDT1R_DLC_Msk      (0xFUL << CAN_RDT1R_DLC_Pos)                     /*!< 0x0000000F */
#define CAN_RDT1R_DLC          CAN_RDT1R_DLC_Msk                               /*!<Data Length Code */
#define CAN_RDT1R_FMI_Pos      (8U)
#define CAN_RDT1R_FMI_Msk      (0xFFUL << CAN_RDT1R_FMI_Pos)                    /*!< 0x0000FF00 */
#define CAN_RDT1R_FMI          CAN_RDT1R_FMI_Msk                               /*!<Filter Match Index */
#define CAN_RDT1R_TIME_Pos     (16U)
#define CAN_RDT1R_TIME_Msk     (0xFFFFUL << CAN_RDT1R_TIME_Pos)                 /*!< 0xFFFF0000 */
#define CAN_RDT1R_TIME         CAN_RDT1R_TIME_Msk                              /*!<Message Time Stamp */

/*******************  Bit definition for CAN_RDL1R register  ******************/
#define CAN_RDL1R_DATA0_Pos    (0U)
#define CAN_RDL1R_DATA0_Msk    (0xFFUL << CAN_RDL1R_DATA0_Pos)                  /*!< 0x000000FF */
#define CAN_RDL1R_DATA0        CAN_RDL1R_DATA0_Msk                             /*!<Data byte 0 */
#define CAN_RDL1R_DATA1_Pos    (8U)
#define CAN_RDL1R_DATA1_Msk    (0xFFUL << CAN_RDL1R_DATA1_Pos)                  /*!< 0x0000FF00 */
#define CAN_RDL1R_DATA1        CAN_RDL1R_DATA1_Msk                             /*!<Data byte 1 */
#define CAN_RDL1R_DATA2_Pos    (16U)
#define CAN_RDL1R_DATA2_Msk    (0xFFUL << CAN_RDL1R_DATA2_Pos)                  /*!< 0x00FF0000 */
#define CAN_RDL1R_DATA2        CAN_RDL1R_DATA2_Msk                             /*!<Data byte 2 */
#define CAN_RDL1R_DATA3_Pos    (24U)
#define CAN_RDL1R_DATA3_Msk    (0xFFUL << CAN_RDL1R_DATA3_Pos)                  /*!< 0xFF000000 */
#define CAN_RDL1R_DATA3        CAN_RDL1R_DATA3_Msk                             /*!<Data byte 3 */

/*******************  Bit definition for CAN_RDH1R register  ******************/
#define CAN_RDH1R_DATA4_Pos    (0U)
#define CAN_RDH1R_DATA4_Msk    (0xFFUL << CAN_RDH1R_DATA4_Pos)                  /*!< 0x000000FF */
#define CAN_RDH1R_DATA4        CAN_RDH1R_DATA4_Msk                             /*!<Data byte 4 */
#define CAN_RDH1R_DATA5_Pos    (8U)
#define CAN_RDH1R_DATA5_Msk    (0xFFUL << CAN_RDH1R_DATA5_Pos)                  /*!< 0x0000FF00 */
#define CAN_RDH1R_DATA5        CAN_RDH1R_DATA5_Msk                             /*!<Data byte 5 */
#define CAN_RDH1R_DATA6_Pos    (16U)
#define CAN_RDH1R_DATA6_Msk    (0xFFUL << CAN_RDH1R_DATA6_Pos)                  /*!< 0x00FF0000 */
#define CAN_RDH1R_DATA6        CAN_RDH1R_DATA6_Msk                             /*!<Data byte 6 */
#define CAN_RDH1R_DATA7_Pos    (24U)
#define CAN_RDH1R_DATA7_Msk    (0xFFUL << CAN_RDH1R_DATA7_Pos)                  /*!< 0xFF000000 */
#define CAN_RDH1R_DATA7        CAN_RDH1R_DATA7_Msk                             /*!<Data byte 7 */

/*!<CAN filter registers */
/*******************  Bit definition for CAN_FMR register  ********************/
#define  CAN_FMR_FINIT_Pos                  (0U)
#define  CAN_FMR_FINIT_Msk                  (0x1UL << CAN_FMR_FINIT_Pos)                     /*!< 0x00000001 */
#define  CAN_FMR_FINIT                      CAN_FMR_FINIT_Msk                               /*!<Filter Init Mode */
#define  CAN_FMR_CAN2SB_Pos                 (8U)
#define  CAN_FMR_CAN2SB_Msk                 (0x3FUL << CAN_FMR_CAN2SB_Pos)                   /*!< 0x00003F00 */
#define  CAN_FMR_CAN2SB                     CAN_FMR_CAN2SB_Msk                              /*!<CAN2 start bank */

#endif
