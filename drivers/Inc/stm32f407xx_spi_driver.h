/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: 06-Mar-2023
 *      Author: eapple
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"


typedef struct {
   uint8_t SPI_DeviceMode;  /* Mode selection master or salve */
   uint8_t SPI_BusConfig;   /*  Full or half Duplex or simplex  */
   uint8_t SPI_SclkSpeed;   /*  Data frame format 8 bit or 16bit  */
   uint8_t SPI_DFF;         /*  Clock Phase  */
   uint8_t SPI_CPOL;        /*  Clock Polarity  */
   uint8_t SPI_CPHA;        /*  software or hardware slave management  */
   uint8_t SPI_SSM;         /*  SPI speed  */
}SPI_Config_t;


typedef struct {
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPIConfig;
	uint8_t      *pTxBuffer;
	uint8_t      *pRxBuffer;
	uint8_t      TxLen;
	uint8_t      RxLen;
	uint8_t      TxState;
	uint8_t      RxState;
}SPI_Handle_t;


#define SPI_DEVICE_MODE_MASTER    1
#define SPI_DEVICE_MODE_SLAVE     0


#define SPI_BUS_CONFIG_FD               1
#define SPI_BUS_CONFIG_HD               2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY   3



#define SPI_SCLK_SPEED_DIV2             0
#define SPI_SCLK_SPEED_DIV4             1
#define SPI_SCLK_SPEED_DIV8             2
#define SPI_SCLK_SPEED_DIV16            3
#define SPI_SCLK_SPEED_DIV32            4
#define SPI_SCLK_SPEED_DIV64            5
#define SPI_SCLK_SPEED_DIV128           6
#define SPI_SCLK_SPEED_DIV256           7




#define SPI_DFF_8BITS        0
#define SPI_DFF_16BITS       1


#define SPI_CPOL_HIGH        1
#define SPI_CPOL_LOW         0

#define SPI_CPHA_HIGH        1
#define SPI_CPHA_LOW         0

#define SPI_SSM_EN           1
#define SPI_SSM_DI           0


#define SPI_READY            0
#define SPI_BUSY_IN_RX       1
#define SPI_BUSY_IN_TX       2


/* *********************************** API Supported  *****************************/

/*
 * Peripheral clock setup
 */

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx,uint8_t EnOrDi);


/*
 * Init and De-init
 */

void SPI_Init(SPI_Handle_t *pHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);


/*
 * Data send and receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer,uint32_t len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer,uint32_t len);

uint8_t SPI_SendData_IT(SPI_Handle_t *pHandle,uint8_t *pTxBuffer,uint32_t len);
uint8_t SPI_ReceiveData_IT(SPI_Handle_t *pHandle,uint8_t *pRxBuffer,uint32_t len);

// Others API calls

void SPI_PeripheralControl(SPI_RegDef_t * pSPIx,uint8_t EnOrDi);
void SPI_SSIConfig(SPI_RegDef_t * pSPIx,uint8_t EnOrDi);


/*
 *  IRQ Configuration and ISR Handling
 */

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);



#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
