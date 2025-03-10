/*
 * stm32f407xx.h
 *
 *  Created on: Mar 13, 2021
 *  Author: Raveendran Arun
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>

#define __vo volatile

/***********************************START: Processor Specific Details****************************/

/*
 * ARM Cortex Mx Processor NVIC ISERx register addresses
 */

#define NVIC_ISER0                          ((__vo uint32_t* )0xE000E100)
#define NVIC_ISER1                          ((__vo uint32_t* )0xE000E104)
#define NVIC_ISER2                          ((__vo uint32_t* )0xE000E108)
#define NVIC_ISER3                          ((__vo uint32_t* )0xE000E10C)

/*
 * ARM Cortex Mx Processor NVIC ICERx register addresses
 */

#define NVIC_ICER0                          ((__vo uint32_t* )0xE000E180)
#define NVIC_ICER1                          ((__vo uint32_t* )0xE000E184)
#define NVIC_ICER2                          ((__vo uint32_t* )0xE000E188)
#define NVIC_ICER3                          ((__vo uint32_t* )0xE000E18C)

/*
 * ARM Cortex Mx Processor Priority Register Address Calculation
 */

#define NVIC_PR_BASEADDR                    ((__vo uint32_t* )0xE000E400)

/*
 * ARM Cortex Mx Processor number of priority bits implemented in Priority Register
 */

#define NUM_PR_BITS_IMPLEMENTED             4

/*
 * Base addresses of Flash and SRAM memories
 */

#define FLASH_BASEADDR                      0x08000000U         					/*!<Base address of Flash */
#define SRAM1_BASEADDR                      0x20000000U         					/*!<Base address of SRAM 1 (112KB) */
#define SRAM2_BASEADDR                      0x2001C000U         					/*!<Base address of SRAM 2 (16KB)*/
#define ROM_BASEADDR                        0x1FFF0000U         					/*!<Base address of system memory (ROM) */
#define SRAM                                SRAM1_BASEADDR      					/*!<Base address of SRAM */

/*
 * AHBx and APBx Bus Peripheral base addresses
 */

#define PERIPH_BASEADDR                      0x40000000U         					/*!<Peripheral base address */
#define APB1PERIPH_BASEADDR                  PERIPH_BASEADDR     					/*!<Bus APB1 base address */
#define APB2PERIPH_BASEADDR                  0x40010000U         					/*!<Bus APB2 base address */
#define AHB1PERIPH_BASEADDR                  0x40020000U         					/*!<Bus AHB1 base address */
#define AHB2PERIPH_BASEADDR                  0x50000000U         					/*!<Bus AHB2 base address */

/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 */

#define GPIOA_BASEADDR                       (AHB1PERIPH_BASEADDR + 0x0000)         /*!<Base address of GPIOA */
#define GPIOB_BASEADDR                       (AHB1PERIPH_BASEADDR + 0x0400)			/*!<Base address of GPIOB */
#define GPIOC_BASEADDR                       (AHB1PERIPH_BASEADDR + 0x0800)			/*!<Base address of GPIOC */
#define GPIOD_BASEADDR                       (AHB1PERIPH_BASEADDR + 0x0C00)			/*!<Base address of GPIOD */
#define GPIOE_BASEADDR                       (AHB1PERIPH_BASEADDR + 0x1000)			/*!<Base address of GPIOE */
#define GPIOF_BASEADDR                       (AHB1PERIPH_BASEADDR + 0x1400)			/*!<Base address of GPIOF */
#define GPIOG_BASEADDR                       (AHB1PERIPH_BASEADDR + 0x1800)			/*!<Base address of GPIOG */
#define GPIOH_BASEADDR                       (AHB1PERIPH_BASEADDR + 0x1C00)			/*!<Base address of GPIOH */
#define GPIOI_BASEADDR                       (AHB1PERIPH_BASEADDR + 0x2000)			/*!<Base address of GPIOI */

#define RCC_BASEADDR                         (AHB1PERIPH_BASEADDR + 0x3800)			/*!<Base address of RCC */

/*
 * Base addresses of peripherals which are hanging on APB1 bus
 */

#define I2C1_BASEADDR                         (APB1PERIPH_BASEADDR + 0x5400)		/*!<Base address of I2C1 */
#define I2C2_BASEADDR                         (APB1PERIPH_BASEADDR + 0x5800)		/*!<Base address of I2C2 */
#define I2C3_BASEADDR                         (APB1PERIPH_BASEADDR + 0x5C00)		/*!<Base address of I2C3 */

#define SPI2_BASEADDR                         (APB1PERIPH_BASEADDR + 0x3800)		/*!<Base address of SPI2 */
#define SPI3_BASEADDR                         (APB1PERIPH_BASEADDR + 0x3C00)		/*!<Base address of SPI3 */

#define USART2_BASEADDR                       (APB1PERIPH_BASEADDR + 0x4400)		/*!<Base address of USART2 */
#define USART3_BASEADDR                       (APB1PERIPH_BASEADDR + 0x4800)		/*!<Base address of USART3 */

#define UART4_BASEADDR                        (APB1PERIPH_BASEADDR + 0x4C00)		/*!<Base address of UART4 */
#define UART5_BASEADDR                        (APB1PERIPH_BASEADDR + 0x5000)		/*!<Base address of UART5 */


/*
 * Base addresses of peripherals which are hanging on APB2 bus
 */

#define SPI1_BASEADDR                         (APB2PERIPH_BASEADDR + 0x3000)		 /*!<Base address of SPI1 */
#define SYSCFG_BASEADDR                       (APB2PERIPH_BASEADDR + 0x3800)		 /*!<Base address of SYSCFG */
#define EXTI_BASEADDR                         (APB2PERIPH_BASEADDR + 0x3C00)         /*!<Base address of EXTI */
#define USART1_BASEADDR                       (APB2PERIPH_BASEADDR + 0x1000)		 /*!<Base address of USART1 */
#define USART6_BASEADDR                       (APB2PERIPH_BASEADDR + 0x1400)		 /*!<Base address of USART6 */


/**********************************Peripheral register definition structures******************************/
/*
 * Registers of a peripheral are specific to MCU
 * eg: Number of registers of SPI peripheral of STM32F4x family of MCUs may be different(more or less)
 * compared to number of registers of SPI peripheral of STM32Lx or STM32F0x family od MCus
 *
 */

/*
 * Peripheral register definition structure for GPIO
 */

typedef struct _GPIO_RegDef_t
{
	__vo uint32_t MODER;                /*!<GPIO port mode register                        Address offset: 0x00 */
	__vo uint32_t OTYPER;				/*!<GPIO port output type register                 Address offset: 0x04 */
	__vo uint32_t OSPEEDR;				/*!<GPIO port output speed register                Address offset: 0x08 */
	__vo uint32_t PUPDR;				/*!<GPIO port pull-up/pull-down register           Address offset: 0x0C */
	__vo uint32_t IDR;					/*!<GPIO port input data register                  Address offset: 0x10 */
	__vo uint32_t ODR;					/*!<GPIO port output data register                 Address offset: 0x14 */
	__vo uint32_t BSRR;					/*!<GPIO port bit set/reset register               Address offset: 0x18 */
	__vo uint32_t LCKR;					/*!<GPIO port configuration lock register          Address offset: 0x1C */
	__vo uint32_t AFR[2];				/*!<AF[0]: GPIO alternate function low register, AF[1]: GPIO alternate function low register   Address offset: 0x20 */
}GPIO_RegDef_t;

/*
 * Peripheral register definition structure for RCC
 */

typedef struct _RCC_RegDef_t
{
	__vo uint32_t CR;					/* RCC clock configuration register                  	Address offset: 0x00 */
	__vo uint32_t PLLCFGR;				/* RCC PLL configuration register                		Address offset: 0x04 */
	__vo uint32_t CFGR;					/* RCC clock configuration register                  	Address offset: 0x08 */
	__vo uint32_t CIR;					/* RCC clock interrupt register                  		Address offset: 0x0C */
	__vo uint32_t AHB1RSTR;				/* RCC AHB1 peripheral reset register                  	Address offset: 0x10 */
	__vo uint32_t AHB2RSTR;				/* RCC AHB2 peripheral reset register                  	Address offset: 0x14 */
	__vo uint32_t AHB3RSTR;          	/* RCC AHB3 peripheral reset register                  	Address offset: 0x18 */
	__vo uint32_t RESERVED0;
	__vo uint32_t APB1RSTR;				/* RCC APB1 peripheral reset register                  	Address offset: 0x20 */
	__vo uint32_t APB2RSTR;          	/* RCC APB2 peripheral reset register                  	Address offset: 0x24 */
	uint32_t      RESERVED1[2];
	__vo uint32_t AHB1ENR;           	/* RCC AHB1 peripheral clock register                  	Address offset: 0x30 */
	__vo uint32_t AHB2ENR;           	/* RCC AHB2 peripheral clock enable register           	Address offset: 0x34 */
	__vo uint32_t AHB3ENR;           	/* RCC AHB3 peripheral clock enable register           	Address offset: 0x38 */
	uint32_t      RESERVED3;
	__vo uint32_t APB1ENR;           	/* RCC APB1 peripheral clock enable register           	Address offset: 0x40 */
	__vo uint32_t APB2ENR;           	/* RCC APB2 peripheral clock enable register           	Address offset: 0x44 */
	uint32_t      RESERVED4[2];
	__vo uint32_t AHB1LPENR;           	/* RCC AHB1 peripheral clock enable in low power mode  	Address offset: 0x50 */
	__vo uint32_t AHB2LPENR;            /* RCC AHB2 peripheral clock enable in low power mode  	Address offset: 0x54 */
	__vo uint32_t AHB3LPENR;            /* RCC AHB3 peripheral clock enable in low power mode  	Address offset: 0x58 */
	uint32_t      RESERVED5;
	__vo uint32_t APB1LPENR;            /* RCC APB1 peripheral clock enable in low power mode      Address offset: 0x60 */
	__vo uint32_t APB2LPENR;            /* RCC APB2 peripheral clock enabled in low power mode     Address offset: 0x64 */
	uint32_t      RESERVED6[2];
	__vo uint32_t BDCR;                 /* RCC Backup domain control register                      Address offset: 0x70 */
	__vo uint32_t CSR;                  /* RCC clock control & status register                     Address offset: 0x74 */
	uint32_t      RESERVED7[2];
	__vo uint32_t SSCGR;                /* RCC spread spectrum clock generation register           Address offset: 0x80*/
	__vo uint32_t PLLI2SCFGR;           /* RCC PLLI2S configuration register                       Address offset: 0x84 */
}RCC_RegDef_t;

/*
 * Peripheral register definition structure for EXTI
 */
typedef struct _EXTI_RegDef_t
{
	__vo uint32_t IMR;                  /* Interrupt mask register                      Address offset: 0x00 */
	__vo uint32_t EMR;                  /* Event mask register                          Address offset: 0x04 */
	__vo uint32_t RTSR;					/* Rising trigger selection register            Address offset: 0x08 */
	__vo uint32_t FTSR;					/* Falling trigger selection register           Address offset: 0x0C */
	__vo uint32_t SWIER;				/* Software interrupt event register            Address offset: 0x10 */
	__vo uint32_t PR;					/* Pending register                             Address offset: 0x14 */
}EXTI_RegDef_t;

/*
 * Peripheral register definition structure for EXTI
 */
typedef struct _SYSCFG_RegDef_t
{
	__vo uint32_t MEMRMP;				/* SYSCFG memory remap register                             Address offset: 0x00 */
	__vo uint32_t PMC;					/* SYSCFG peripheral mode configuration register            Address offset: 0x04 */
	__vo uint32_t EXTICR[4];			/* SYSCFG external interrupt configuration register 1 - 4   Address offset: 0x08- 0x14 */
	uint32_t      RESERVED[2];          /* Reserved                                                 Address offset: 0x18- 0x1C  */
	__vo uint32_t CMPCR;				/* Compensation cell control register                       Address offset: 0x20 */
}SYSCFG_RegDef_t;

/*
 * Peripheral definitions (Peripheral base addresses type casted to xxx_RegDef_t )
 */

#define GPIOA         	((GPIO_RegDef_t* )GPIOA_BASEADDR)			/*!<Base address of GPIOA */
#define GPIOB         	((GPIO_RegDef_t* )GPIOB_BASEADDR)			/*!<Base address of GPIOB */
#define GPIOC         	((GPIO_RegDef_t* )GPIOC_BASEADDR)			/*!<Base address of GPIOC */
#define GPIOD         	((GPIO_RegDef_t* )GPIOD_BASEADDR)			/*!<Base address of GPIOD */
#define GPIOE         	((GPIO_RegDef_t* )GPIOE_BASEADDR)			/*!<Base address of GPIOE */
#define GPIOF         	((GPIO_RegDef_t* )GPIOF_BASEADDR)			/*!<Base address of GPIOF */
#define GPIOG         	((GPIO_RegDef_t* )GPIOG_BASEADDR)			/*!<Base address of GPIOG */
#define GPIOH         	((GPIO_RegDef_t* )GPIOH_BASEADDR)			/*!<Base address of GPIOH */
#define GPIOI         	((GPIO_RegDef_t* )GPIOI_BASEADDR)			/*!<Base address of GPIOI */

#define RCC           	((RCC_RegDef_t* )RCC_BASEADDR)				/*!<Base address of RCC */

#define EXTI			((EXTI_RegDef_t* )EXTI_BASEADDR)			/*!<Base address of EXTI */

#define SYSCFG          ((SYSCFG_RegDef_t* )SYSCFG_BASEADDR)		/*!<Base address of SYSCFG */

/*
 * Clock enable macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()                    ( RCC->AHB1ENR |= (1 << 0) )
#define GPIOB_PCLK_EN()                    ( RCC->AHB1ENR |= (1 << 1) )
#define GPIOC_PCLK_EN()                    ( RCC->AHB1ENR |= (1 << 2) )
#define GPIOD_PCLK_EN()                    ( RCC->AHB1ENR |= (1 << 3) )
#define GPIOE_PCLK_EN()                    ( RCC->AHB1ENR |= (1 << 4) )
#define GPIOF_PCLK_EN()                    ( RCC->AHB1ENR |= (1 << 5) )
#define GPIOG_PCLK_EN()                    ( RCC->AHB1ENR |= (1 << 6) )
#define GPIOH_PCLK_EN()                    ( RCC->AHB1ENR |= (1 << 7) )
#define GPIOI_PCLK_EN()                    ( RCC->AHB1ENR |= (1 << 8) )

/*
 * Clock enable macros for I2Cx peripherals
 */

#define I2C1_PCLK_EN()                     ( RCC->APB1ENR |= (1 << 21) )
#define I2C2_PCLK_EN()                     ( RCC->APB1ENR |= (1 << 22) )
#define I2C3_PCLK_EN()                     ( RCC->APB1ENR |= (1 << 23) )

/*
 * Clock enable macros for SPIx peripherals
 */

#define SPI1_PCLK_EN()                     ( RCC->APB2ENR |= (1 << 12) )
#define SPI2_PCLK_EN()                     ( RCC->APB1ENR |= (1 << 14) )
#define SPI3_PCLK_EN()                     ( RCC->APB1ENR |= (1 << 15) )

/*
 * Clock enable macros for USARTx/UARTx peripherals
 */

#define USART1_PCLK_EN()                   ( RCC->APB2ENR |= (1 << 4) )
#define USART2_PCLK_EN()                   ( RCC->APB1ENR |= (1 << 17) )
#define USART3_PCLK_EN()                   ( RCC->APB1ENR |= (1 << 18) )
#define UART4_PCLK_EN()                    ( RCC->APB1ENR |= (1 << 19) )
#define UART5_PCLK_EN()                    ( RCC->APB1ENR |= (1 << 20) )
#define USART6_PCLK_EN()                   ( RCC->APB2ENR |= (1 << 5) )

/*
 * Clock enable macros for SYSCFG peripherals
 */

#define SYSCFG_PCLK_EN()                   ( RCC->APB2ENR |= (1 << 14) )


/*
 * Clock disable macros for GPIOx peripherals
 */
#define GPIOA_PCLK_DI()                    ( RCC->AHB1ENR &= ~(1 << 1) )
#define GPIOB_PCLK_DI()                    ( RCC->AHB1ENR &= ~(1 << 1) )
#define GPIOC_PCLK_DI()                    ( RCC->AHB1ENR &= ~(1 << 2) )
#define GPIOD_PCLK_DI()                    ( RCC->AHB1ENR &= ~(1 << 3) )
#define GPIOE_PCLK_DI()                    ( RCC->AHB1ENR &= ~(1 << 4) )
#define GPIOF_PCLK_DI()                    ( RCC->AHB1ENR &= ~(1 << 5) )
#define GPIOG_PCLK_DI()                    ( RCC->AHB1ENR &= ~(1 << 6) )
#define GPIOH_PCLK_DI()                    ( RCC->AHB1ENR &= ~(1 << 7) )
#define GPIOI_PCLK_DI()                    ( RCC->AHB1ENR &= ~(1 << 8) )

/*
 * Clock disable macros for I2Cx peripherals
 */

/*
 * Clock disable macros for SPIx peripherals
 */


/*
 * Clock disable macros for USARTx/UARTx peripherals
 */

/*
 * Macros to reset GPIOx peripheral
 */

#define GPIOA_REG_RESET()           do{ ( RCC->AHB1RSTR |= (1 << 0) ); ( RCC->AHB1RSTR &= ~(1 << 0) ); }while(0)
#define GPIOB_REG_RESET()           do{ ( RCC->AHB1RSTR |= (1 << 1) ); ( RCC->AHB1RSTR &= ~(1 << 1) ); }while(0)
#define GPIOC_REG_RESET()           do{ ( RCC->AHB1RSTR |= (1 << 2) ); ( RCC->AHB1RSTR &= ~(1 << 2) ); }while(0)
#define GPIOD_REG_RESET()           do{ ( RCC->AHB1RSTR |= (1 << 3) ); ( RCC->AHB1RSTR &= ~(1 << 3) ); }while(0)
#define GPIOE_REG_RESET()           do{ ( RCC->AHB1RSTR |= (1 << 4) ); ( RCC->AHB1RSTR &= ~(1 << 4) ); }while(0)
#define GPIOF_REG_RESET()           do{ ( RCC->AHB1RSTR |= (1 << 5) ); ( RCC->AHB1RSTR &= ~(1 << 5) ); }while(0)
#define GPIOG_REG_RESET()           do{ ( RCC->AHB1RSTR |= (1 << 6) ); ( RCC->AHB1RSTR &= ~(1 << 6) ); }while(0)
#define GPIOH_REG_RESET()           do{ ( RCC->AHB1RSTR |= (1 << 7) ); ( RCC->AHB1RSTR &= ~(1 << 7) ); }while(0)
#define GPIOI_REG_RESET()           do{ ( RCC->AHB1RSTR |= (1 << 8) ); ( RCC->AHB1RSTR &= ~(1 << 8) ); }while(0)

/*
 * Returns port code for GPIOx base address
 */

#define GPIO_BASEADDR_TO_CODE(x)	( (x == GPIOA) ? 0 : \
									  (x == GPIOB) ? 1 : \
									  (x == GPIOC) ? 2 : \
									  (x == GPIOD) ? 3 : \
									  (x == GPIOF) ? 4 : \
									  (x == GPIOF) ? 5 : \
									  (x == GPIOG) ? 6 : \
								 	  (x == GPIOH) ? 7 : \
									  (x == GPIOI) ? 8 : 0 )


/*
 * IRQ(Interrupt Request) numbers of STM32F407xx MCU
 */

#define IRQ_NO_EXTI0                        6
#define IRQ_NO_EXTI1                        7
#define IRQ_NO_EXTI2                        8
#define IRQ_NO_EXTI3                        9
#define IRQ_NO_EXTI4                        10
#define IRQ_NO_EXTI9_5                      23
#define IRQ_NO_EXTI15_10                    40

/*
 *  Macros for all the possible priority levels
 */

#define NVIC_IRQ_PRIO0                      0
#define NVIC_IRQ_PRIO1                      1
#define NVIC_IRQ_PRIO2                      2
#define NVIC_IRQ_PRIO3                      3
#define NVIC_IRQ_PRIO4                      4
#define NVIC_IRQ_PRIO5                      5
#define NVIC_IRQ_PRIO6                      6
#define NVIC_IRQ_PRIO7                      7
#define NVIC_IRQ_PRIO8                      8
#define NVIC_IRQ_PRIO9                      9
#define NVIC_IRQ_PRIO10                     10
#define NVIC_IRQ_PRIO11                     11
#define NVIC_IRQ_PRIO12                     12
#define NVIC_IRQ_PRIO13                     13
#define NVIC_IRQ_PRIO14                     14
#define NVIC_IRQ_PRIO15                     15

/*
 * Some generic macros
 */

#define ENABLE             1
#define DISABLE            0
#define SET                ENABLE
#define RESET              DISABLE
#define GPIO_PIN_SET       SET
#define GPIO_PIN_RESET     RESET

#endif /* INC_STM32F407XX_H_ */
