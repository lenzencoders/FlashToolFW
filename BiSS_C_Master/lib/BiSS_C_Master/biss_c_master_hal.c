/*!
 * @file biss_c_master_hal.c
 * @author Kirill Rostovskiy (kmrost@lenzencoders.com)
 * @brief BiSS C Master STM32G4x Hardware abstraction layer driver
 * @version 0.1
 * @copyright Lenz Encoders (c) 2024
 * @section disclaimer  
 * Disclaimer! This is tamplate driver for debug/review BiSS C master library on STM32Gx (and similar) microcontrollers.
 * It supports only LENZ IRS/SAB/SIB encoders only with 3 additional Ack bits and SCD length - 32 bits(24 bits position data, 
 * 2 bits nError/nWaring and 6 bits CRC). You should implement and test this part on your own.
 */

#include "hw_cfg.h"
#include "stm32g4xx.h"
#include "biss_c_master.h"
#include "biss_c_master_hal.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_spi.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_usart.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_rcc.h"
#include "uart.h"

#define SPI_CR1_BISS_CDM    SPI_CR1_SSI | SPI_CR1_SPE | SPI_CR1_MSTR | SPI_CR1_SSM | ((0x5U << SPI_CR1_BR_Pos) & SPI_CR1_BR_Msk)
#define SPI_CR1_BISS_nCDM   SPI_CR1_CPOL | SPI_CR1_CPHA | SPI_CR1_BISS_CDM

#define SPI_CR2_BISS_CFG		SPI_CR2_RXDMAEN |SPI_CR2_TXDMAEN | SPI_CR2_FRXTH | (0x7U << SPI_CR2_DS_Pos)
static volatile enum {RS485_ADR1, RS485_ADR2} RS485_ADR = RS485_ADR2;
static volatile uint8_t adr_reset_cou = 0;
typedef enum{
	RS485_CDM_ADR1_REQ = 0x81U,
	RS485_CDM_ADR2_REQ = 0x22U,
	RS485_CDM_ADR3_REQ = 0x43U,
	RS485_CDM_ADR4_REQ = 0x04U,
	RS485_CDM_ADR5_REQ = 0x65U,
	RS485_CDM_ADR6_REQ = 0xC6U,
	RS485_CDM_ADR7_REQ = 0xA7U,
	RS485_nCDM_ADR1_REQ = 0xB1U,
	RS485_nCDM_ADR2_REQ = 0x12U,
	RS485_nCDM_ADR3_REQ = 0x73U,
	RS485_nCDM_ADR4_REQ = 0x34U,
	RS485_nCDM_ADR5_REQ = 0x55U,
	RS485_nCDM_ADR6_REQ = 0xF6U,
	RS485_nCDM_ADR7_REQ = 0x97U,
}RS485_Req_t;

typedef union{
	volatile uint8_t buf[8];
	struct{
		uint32_t reserv1:24;
		volatile uint32_t CDS:1;
		uint32_t reserv2:7;
		volatile uint32_t revSCD;
	};
}SPI_rx_t;

typedef union{
	volatile uint32_t u32;
	struct{
		uint32_t adr:3;
		uint32_t frame_format:1;
		uint32_t CDS:1;
		uint32_t nW:1;
		uint32_t nE:1;
		uint32_t Pos:19;
		uint32_t _CRC6:6;
	};	
	struct{
		uint32_t Data4CRC:26;
		uint32_t CRC6:6;
	};
}USART_rx_t;

typedef enum{
	CRC6_OK,CRC6_FAULT
}CRC_State_t;

// Test Renishaw Data
//volatile uint32_t renishaw_angle = 0;
//volatile AngleDataRenishaw_t AngleDataRenishaw;

SPI_rx_t BiSS1_SPI_rx;
SPI_rx_t BiSS2_SPI_rx;
USART_rx_t USART_rx;
volatile BiSS_SPI_Ch_t BiSS_SPI_Ch = BISS_SPI_CH_1;
volatile CDS_t USART_CDS_last = CDS;
volatile uint32_t BISS1_SCD;
volatile uint32_t BISS2_SCD;
volatile AngleData_t AngleData1;
volatile AngleData_t AngleData2;
volatile CRC_State_t CRC6_State1 = CRC6_FAULT;
volatile CRC_State_t CRC6_State2 = CRC6_FAULT;
volatile CDM_t last_CDM = CDM;

static const uint8_t CRC6_LUT[256U] = {
	0x00U, 0x0CU, 0x18U, 0x14U, 0x30U, 0x3CU, 0x28U, 0x24U, 0x60U, 0x6CU, 0x78U, 0x74U, 0x50U, 0x5CU, 0x48U, 0x44U, 
	0xC0U, 0xCCU, 0xD8U, 0xD4U, 0xF0U, 0xFCU, 0xE8U, 0xE4U, 0xA0U, 0xACU, 0xB8U, 0xB4U, 0x90U, 0x9CU, 0x88U, 0x84U, 
	0x8CU, 0x80U, 0x94U, 0x98U, 0xBCU, 0xB0U, 0xA4U, 0xA8U, 0xECU, 0xE0U, 0xF4U, 0xF8U, 0xDCU, 0xD0U, 0xC4U, 0xC8U, 
	0x4CU, 0x40U, 0x54U, 0x58U, 0x7CU, 0x70U, 0x64U, 0x68U, 0x2CU, 0x20U, 0x34U, 0x38U, 0x1CU, 0x10U, 0x04U, 0x08U, 
	0x14U, 0x18U, 0x0CU, 0x00U, 0x24U, 0x28U, 0x3CU, 0x30U, 0x74U, 0x78U, 0x6CU, 0x60U, 0x44U, 0x48U, 0x5CU, 0x50U, 
	0xD4U, 0xD8U, 0xCCU, 0xC0U, 0xE4U, 0xE8U, 0xFCU, 0xF0U, 0xB4U, 0xB8U, 0xACU, 0xA0U, 0x84U, 0x88U, 0x9CU, 0x90U, 
	0x98U, 0x94U, 0x80U, 0x8CU, 0xA8U, 0xA4U, 0xB0U, 0xBCU, 0xF8U, 0xF4U, 0xE0U, 0xECU, 0xC8U, 0xC4U, 0xD0U, 0xDCU, 
	0x58U, 0x54U, 0x40U, 0x4CU, 0x68U, 0x64U, 0x70U, 0x7CU, 0x38U, 0x34U, 0x20U, 0x2CU, 0x08U, 0x04U, 0x10U, 0x1CU, 
	0x28U, 0x24U, 0x30U, 0x3CU, 0x18U, 0x14U, 0x00U, 0x0CU, 0x48U, 0x44U, 0x50U, 0x5CU, 0x78U, 0x74U, 0x60U, 0x6CU, 
	0xE8U, 0xE4U, 0xF0U, 0xFCU, 0xD8U, 0xD4U, 0xC0U, 0xCCU, 0x88U, 0x84U, 0x90U, 0x9CU, 0xB8U, 0xB4U, 0xA0U, 0xACU, 
	0xA4U, 0xA8U, 0xBCU, 0xB0U, 0x94U, 0x98U, 0x8CU, 0x80U, 0xC4U, 0xC8U, 0xDCU, 0xD0U, 0xF4U, 0xF8U, 0xECU, 0xE0U, 
	0x64U, 0x68U, 0x7CU, 0x70U, 0x54U, 0x58U, 0x4CU, 0x40U, 0x04U, 0x08U, 0x1CU, 0x10U, 0x34U, 0x38U, 0x2CU, 0x20U, 
	0x3CU, 0x30U, 0x24U, 0x28U, 0x0CU, 0x00U, 0x14U, 0x18U, 0x5CU, 0x50U, 0x44U, 0x48U, 0x6CU, 0x60U, 0x74U, 0x78U, 
	0xFCU, 0xF0U, 0xE4U, 0xE8U, 0xCCU, 0xC0U, 0xD4U, 0xD8U, 0x9CU, 0x90U, 0x84U, 0x88U, 0xACU, 0xA0U, 0xB4U, 0xB8U, 
	0xB0U, 0xBCU, 0xA8U, 0xA4U, 0x80U, 0x8CU, 0x98U, 0x94U, 0xD0U, 0xDCU, 0xC8U, 0xC4U, 0xE0U, 0xECU, 0xF8U, 0xF4U, 
	0x70U, 0x7CU, 0x68U, 0x64U, 0x40U, 0x4CU, 0x58U, 0x54U, 0x10U, 0x1CU, 0x08U, 0x04U, 0x20U, 0x2CU, 0x38U, 0x34U
};

/*!
 * @brief CRC6 calculation function
 * @param data including position data, error and warning bits
 * @return uint8_t CRC6, poly 0x43, inverted
 */
__STATIC_INLINE uint8_t BISS_CRC6_Calc(uint32_t data){	
	uint8_t crc = CRC6_LUT[(data >> 24U) & 0x3U];
	crc = CRC6_LUT[((data >> 16U) & 0xFFU) ^ crc];
	crc = CRC6_LUT[((data >> 8U) & 0xFFU) ^ crc];
	crc = CRC6_LUT[(data & 0xFFU) ^ crc];
	crc = ((crc ^ 0xFFU) >> 2U) & 0x3FU;
	return(crc);
}

static void BISS1_SPI_Init(void);
static void BISS1_SPI_DeInit(void);
static void BISS2_SPI_Init(void);
static void BISS2_SPI_DeInit(void);

static void BiSS1_SPI_nCDM_Req(void){
	LL_DMA_DisableChannel(DMA_BISS1_RX);
	LL_DMA_DisableChannel(DMA_BISS1_TX);
	LL_SPI_DeInit(BISS1_SPI);
	BISS1_SPI->CR1 = SPI_CR1_BISS_nCDM;
	BISS1_SPI->CR2 = SPI_CR2_BISS_CFG;
	LL_DMA_SetDataLength(DMA_BISS1_TX, 5U); // TODO try 1U via define
	LL_DMA_SetDataLength(DMA_BISS1_RX, 5U); // TODO try 1U via define
	LL_DMA_EnableChannel(DMA_BISS1_TX);	 
	LL_DMA_EnableChannel(DMA_BISS1_RX);		
}

static void BiSS2_SPI_nCDM_Req(void){
	LL_DMA_DisableChannel(DMA_BISS2_RX);
	LL_DMA_DisableChannel(DMA_BISS2_TX);
	LL_SPI_DeInit(BISS2_SPI);
	BISS2_SPI->CR1 = SPI_CR1_BISS_nCDM;
	BISS2_SPI->CR2 = SPI_CR2_BISS_CFG;
	LL_DMA_SetDataLength(DMA_BISS2_TX, 5U); // TODO try 1U via define
	LL_DMA_SetDataLength(DMA_BISS2_RX, 5U); // TODO try 1U via define
	LL_DMA_EnableChannel(DMA_BISS2_TX);	 
	LL_DMA_EnableChannel(DMA_BISS2_RX);		
}

static void BiSS1_SPI_CDM_Req(void){			
	LL_GPIO_SetPinMode(MA1_PIN, LL_GPIO_MODE_OUTPUT);
	LL_DMA_DisableChannel(DMA_BISS1_RX);
	LL_DMA_DisableChannel(DMA_BISS1_TX);
	LL_SPI_DeInit(BISS1_SPI);
	BISS1_SPI->CR1 = SPI_CR1_BISS_CDM;
	BISS1_SPI->CR2 = SPI_CR2_BISS_CFG;
	LL_DMA_SetDataLength(DMA_BISS1_TX, 5U); // TODO try 1U via define
	LL_DMA_SetDataLength(DMA_BISS1_RX, 5U); // TODO try 1U via define
	LL_DMA_EnableChannel(DMA_BISS1_TX);	    
	LL_GPIO_SetPinMode(MA1_PIN, LL_GPIO_MODE_ALTERNATE);
	LL_DMA_EnableChannel(DMA_BISS1_RX);	
}

static void BiSS2_SPI_CDM_Req(void){			
	LL_GPIO_SetPinMode(MA2_PIN, LL_GPIO_MODE_OUTPUT);
	LL_DMA_DisableChannel(DMA_BISS2_RX);
	LL_DMA_DisableChannel(DMA_BISS2_TX);
	LL_SPI_DeInit(BISS2_SPI);
	BISS2_SPI->CR1 = SPI_CR1_BISS_CDM;
	BISS2_SPI->CR2 = SPI_CR2_BISS_CFG;
	LL_DMA_SetDataLength(DMA_BISS2_TX, 5U); // TODO try 1U via define
	LL_DMA_SetDataLength(DMA_BISS2_RX, 5U); // TODO try 1U via define
	LL_DMA_EnableChannel(DMA_BISS2_TX);	    
	LL_GPIO_SetPinMode(MA2_PIN, LL_GPIO_MODE_ALTERNATE);
	LL_DMA_EnableChannel(DMA_BISS2_RX);	
}



void BissRequest_CDM(void){
	switch(BISS_MODE){
		case BISS_MODE_SPI:		
			BiSS1_SPI_CDM_Req();
			break;	
		case BISS_MODE_UART:
			LL_DMA_DisableChannel(DMA_BISS2_UART_RX);
			LL_DMA_SetDataLength(DMA_BISS2_UART_RX, 4U);
			LL_DMA_EnableChannel(DMA_BISS2_UART_RX);	 
			switch(RS485_ADR){
				case RS485_ADR1:
					LL_USART_TransmitData8(BISS2_UART, RS485_CDM_ADR1_REQ);
					break;
				case RS485_ADR2:
					LL_USART_TransmitData8(BISS2_UART, RS485_CDM_ADR2_REQ);
					break;
			}
			break;		
	}		
}

void BissRequest_nCDM(void){
	switch(BISS_MODE){
		case BISS_MODE_SPI:		
			BiSS1_SPI_nCDM_Req();
			BiSS2_SPI_nCDM_Req();
			break;	
		case BISS_MODE_UART:
			LL_DMA_DisableChannel(DMA_BISS2_UART_RX);
			LL_DMA_SetDataLength(DMA_BISS2_UART_RX, 4U);
			LL_DMA_EnableChannel(DMA_BISS2_UART_RX);	 
			switch(RS485_ADR){
				case RS485_ADR1:
					LL_USART_TransmitData8(BISS2_UART, RS485_nCDM_ADR1_REQ);
					break;
				case RS485_ADR2:
					LL_USART_TransmitData8(BISS2_UART, RS485_nCDM_ADR2_REQ);
					break;
			}
			break;		
	}		
}

void BISS_Task_IRQHandler(void) {
	LL_TIM_ClearFlag_UPDATE(BISS_Task_TIM);
	switch(BISS_MODE){
		case BISS_MODE_SPI:		
			BISS1_SCD = __REV(BiSS1_SPI_rx.revSCD);
			if(BISS_CRC6_Calc(BISS1_SCD >> 6) == (BISS1_SCD & 0x3FU)){
				CRC6_State1 = CRC6_OK;
				AngleData1.angle_data = BISS1_SCD >> 8;
				AngleData1.time_of_life_counter++;
			}
			else{
				CRC6_State1 = CRC6_FAULT;
			}			
			BISS2_SCD = __REV(BiSS2_SPI_rx.revSCD);
			if(BISS_CRC6_Calc(BISS2_SCD >> 6) == (BISS2_SCD & 0x3FU)){
				CRC6_State2 = CRC6_OK;
				AngleData2.angle_data = BISS2_SCD >> 8;
				AngleData2.time_of_life_counter++;
			}
			else{
				CRC6_State2 = CRC6_FAULT;
			}					
			if (BiSS_C_Master_StateMachine(BiSS1_SPI_rx.CDS) == CDM) {
				switch(BiSS_SPI_Ch){
					case BISS_SPI_CH_1:
						BiSS1_SPI_CDM_Req();
						BiSS2_SPI_nCDM_Req();
						break;
					case BISS_SPI_CH_2:
						BiSS1_SPI_nCDM_Req();
						BiSS2_SPI_CDM_Req();
						break;
					default:
						BiSS1_SPI_nCDM_Req();
						BiSS2_SPI_nCDM_Req();
						break;
				}
				BissRequest_CDM();
			}
			else {
				BiSS1_SPI_nCDM_Req();
				BiSS2_SPI_nCDM_Req();
			}	
			break;
		case BISS_MODE_UART:		
			if(LL_DMA_GetDataLength(DMA_BISS2_UART_RX) == 0){
				if(BISS_CRC6_Calc(USART_rx.Data4CRC) == USART_rx.CRC6){
					CRC6_State1 = CRC6_OK;
					AngleData1.angle_data = USART_rx.Pos;
					AngleData1.time_of_life_counter++;
				}
				else{
					CRC6_State1 = CRC6_FAULT;
				}	
				if (BiSS_C_Master_StateMachine(USART_CDS_last) == CDM) {
					BissRequest_CDM();
					last_CDM = CDM;
				}
				else {
					BissRequest_nCDM();
					last_CDM = nCDM;
				}	
				USART_CDS_last = (CDS_t)USART_rx.CDS;
				adr_reset_cou = 0;
			}
			else{
				adr_reset_cou++;
				if(adr_reset_cou == 255){
					RS485_ADR = RS485_ADR == RS485_ADR1? RS485_ADR2 : RS485_ADR1;
				}
				if(last_CDM == CDM){
					BissRequest_CDM();
				}
				else {
					BissRequest_nCDM();
				}						
			}
			break;
	}
	// DEBUG BEGIN
	//Test Renishaw angle data
//	AngleDataRenishaw.angle_data= LL_TIM_GetCounter(TIM_RENISHAW);
	// DEBUG END
	// UART STATEMachine
	UART_StateMachine();
}

void BiSS_C_Master_HAL_Init(void){
	switch(BISS_MODE){
		case BISS_MODE_SPI:						
			BISS1_SPI_Init();
			BISS2_SPI_Init();
			break;
		case BISS_MODE_UART:
			LL_GPIO_SetOutputPin(PWR2_EN_PIN);
			LL_GPIO_SetOutputPin(LED1_RED); // Set LED1 to high --> Green light
			LL_USART_Disable(BISS2_UART);		
			LL_GPIO_SetPinMode(MA2_PIN, LL_GPIO_MODE_ANALOG);
			LL_GPIO_SetPinMode(BISS_MA_UART_PIN, LL_GPIO_MODE_ALTERNATE);
			LL_DMA_SetPeriphAddress(DMA_BISS2_UART_RX, (uint32_t) &BISS2_UART->RDR);
			LL_DMA_SetMemoryAddress(DMA_BISS2_UART_RX, (uint32_t) &USART_rx.u32);
			LL_DMA_SetDataLength(DMA_BISS2_UART_RX, 4);	
			LL_USART_EnableDMAReq_RX(BISS2_UART);
			LL_USART_SetDEAssertionTime(BISS2_UART, 16U);
			LL_USART_SetDEDeassertionTime(BISS2_UART, 16U);
			LL_USART_Enable(BISS2_UART);		
			LL_DMA_EnableChannel(DMA_BISS2_UART_RX);
			break;		
	}
}


static void BISS1_SPI_Init(void)
{	
	
  /* Peripheral clock enable */
	
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMAMUX1);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);	
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
	BISS1_SPI_GRP_EN();
		
	/* Disable UartPin*/
	
	LL_GPIO_SetPinMode(BISS_MA_UART_PIN, LL_GPIO_MODE_ANALOG);
	
  /**SPI1 GPIO Configuration
  MA1_PIN   ------> SPI_SCK
  SLO1_PIN   ------> SPI_MISO
  */	
	
	LL_GPIO_SetOutputPin(PWR1_EN_PIN);
	LL_GPIO_SetPinMode(PWR1_EN_PIN, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinOutputType(DE1_PIN, LL_GPIO_OUTPUT_PUSHPULL);

	LL_GPIO_SetOutputPin(DE1_PIN);
	LL_GPIO_SetPinMode(DE1_PIN, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinOutputType(DE1_PIN, LL_GPIO_OUTPUT_PUSHPULL);
	
	LL_GPIO_SetPinMode(MA1_PIN, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetOutputPin(MA1_PIN);
	LL_GPIO_SetPinOutputType(MA1_PIN, LL_GPIO_OUTPUT_PUSHPULL);
	
	LL_GPIO_SetPinMode(SLO1_PIN, LL_GPIO_MODE_ALTERNATE);
	
	BISS1_GPIO_SET_AF();

  /* SPI1 DMA Init */

  /* SPI1_RX Init */
	LL_DMA_DisableChannel(DMA_BISS1_RX);
  LL_DMA_SetPeriphRequest(DMA_BISS1_RX, DMA_BISS1_RX_Req);
  LL_DMA_SetDataTransferDirection(DMA_BISS1_RX, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);	
  LL_DMA_SetChannelPriorityLevel(DMA_BISS1_RX, LL_DMA_PRIORITY_LOW);
  LL_DMA_SetMode(DMA_BISS1_RX, LL_DMA_MODE_NORMAL);
  LL_DMA_SetPeriphIncMode(DMA_BISS1_RX, LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(DMA_BISS1_RX, LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphSize(DMA_BISS1_RX, LL_DMA_PDATAALIGN_BYTE);
  LL_DMA_SetMemorySize(DMA_BISS1_RX, LL_DMA_MDATAALIGN_BYTE);

  /* SPI1_TX Init */
	LL_DMA_DisableChannel(DMA_BISS1_TX);
  LL_DMA_SetPeriphRequest(DMA_BISS1_TX, DMA_BISS1_TX_Req);
  LL_DMA_SetDataTransferDirection(DMA_BISS1_TX, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
  LL_DMA_SetChannelPriorityLevel(DMA_BISS1_TX, LL_DMA_PRIORITY_LOW);
  LL_DMA_SetMode(DMA_BISS1_TX, LL_DMA_MODE_NORMAL);
  LL_DMA_SetPeriphIncMode(DMA_BISS1_TX, LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(DMA_BISS1_TX, LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphSize(DMA_BISS1_TX, LL_DMA_PDATAALIGN_BYTE);
  LL_DMA_SetMemorySize(DMA_BISS1_TX, LL_DMA_MDATAALIGN_BYTE);
	
/* Init setup DMA/SPI */	
	LL_DMA_SetPeriphAddress(DMA_BISS1_RX, (uint32_t) &BISS1_SPI->DR);
	LL_DMA_SetMemoryAddress(DMA_BISS1_RX, (uint32_t) &BiSS1_SPI_rx.buf[3]);
	LL_DMA_SetDataLength(DMA_BISS1_RX, 5);	
	LL_DMA_SetPeriphAddress(DMA_BISS1_TX, (uint32_t) &BISS1_SPI->DR);
	LL_DMA_SetDataLength(DMA_BISS1_TX, 5);	
	LL_SPI_EnableDMAReq_TX(BISS1_SPI);
	LL_SPI_EnableDMAReq_RX(BISS1_SPI);
	LL_SPI_Enable(BISS1_SPI);
	LL_DMA_EnableChannel(DMA_BISS1_RX);
}

static void BISS2_SPI_Init(void)
{	
	
  /* Peripheral clock enable */
	
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMAMUX1);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);	
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
	BISS2_SPI_GRP_EN();
		
	/* Disable UartPin*/
	
	LL_GPIO_SetPinMode(BISS_MA_UART_PIN, LL_GPIO_MODE_ANALOG);
	
  /**SPI1 GPIO Configuration
  MA2_PIN   ------> SPI_SCK
  SLO2_PIN   ------> SPI_MISO
  */	
	
	LL_GPIO_SetOutputPin(PWR2_EN_PIN);
	LL_GPIO_SetPinMode(PWR2_EN_PIN, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinOutputType(PWR2_EN_PIN, LL_GPIO_OUTPUT_PUSHPULL);

	LL_GPIO_SetOutputPin(DE2_PIN);
	LL_GPIO_SetPinMode(DE2_PIN, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinOutputType(DE2_PIN, LL_GPIO_OUTPUT_PUSHPULL);
	
	LL_GPIO_SetPinMode(MA2_PIN, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetOutputPin(MA2_PIN);
	LL_GPIO_SetPinOutputType(MA2_PIN, LL_GPIO_OUTPUT_PUSHPULL);
	
	LL_GPIO_SetPinMode(SLO2_PIN, LL_GPIO_MODE_ALTERNATE);
	
	BISS2_GPIO_SET_AF();

  /* SPI1 DMA Init */

  /* SPI1_RX Init */
	LL_DMA_DisableChannel(DMA_BISS2_RX);
  LL_DMA_SetPeriphRequest(DMA_BISS2_RX, DMA_BISS2_RX_Req);
  LL_DMA_SetDataTransferDirection(DMA_BISS2_RX, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);	
  LL_DMA_SetChannelPriorityLevel(DMA_BISS2_RX, LL_DMA_PRIORITY_LOW);
  LL_DMA_SetMode(DMA_BISS2_RX, LL_DMA_MODE_NORMAL);
  LL_DMA_SetPeriphIncMode(DMA_BISS2_RX, LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(DMA_BISS2_RX, LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphSize(DMA_BISS2_RX, LL_DMA_PDATAALIGN_BYTE);
  LL_DMA_SetMemorySize(DMA_BISS2_RX, LL_DMA_MDATAALIGN_BYTE);

  /* SPI1_TX Init */
	LL_DMA_DisableChannel(DMA_BISS2_TX);
  LL_DMA_SetPeriphRequest(DMA_BISS2_TX, DMA_BISS2_TX_Req);
  LL_DMA_SetDataTransferDirection(DMA_BISS2_TX, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
  LL_DMA_SetChannelPriorityLevel(DMA_BISS2_TX, LL_DMA_PRIORITY_LOW);
  LL_DMA_SetMode(DMA_BISS2_TX, LL_DMA_MODE_NORMAL);
  LL_DMA_SetPeriphIncMode(DMA_BISS2_TX, LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(DMA_BISS2_TX, LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphSize(DMA_BISS2_TX, LL_DMA_PDATAALIGN_BYTE);
  LL_DMA_SetMemorySize(DMA_BISS2_TX, LL_DMA_MDATAALIGN_BYTE);
	
/* Init setup DMA/SPI */	
	LL_DMA_SetPeriphAddress(DMA_BISS2_RX, (uint32_t) &BISS2_SPI->DR);
	LL_DMA_SetMemoryAddress(DMA_BISS2_RX, (uint32_t) &BiSS2_SPI_rx.buf[3]);
	LL_DMA_SetDataLength(DMA_BISS2_RX, 5);	
	LL_DMA_SetPeriphAddress(DMA_BISS2_TX, (uint32_t) &BISS2_SPI->DR);
	LL_DMA_SetDataLength(DMA_BISS2_TX, 5);	
	LL_SPI_EnableDMAReq_TX(BISS2_SPI);
	LL_SPI_EnableDMAReq_RX(BISS2_SPI);
	LL_SPI_Enable(BISS2_SPI);
	LL_DMA_EnableChannel(DMA_BISS2_RX);
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_RCC_SetUSARTClockSource(LL_RCC_USART2_CLKSOURCE_PCLK1);

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  /**USART2 GPIO Configuration
  PA1   ------> USART2_DE
  PA15   ------> USART2_RX
  PB3   ------> USART2_TX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USART2 DMA Init */

  /* USART2_RX Init */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_5, LL_DMAMUX_REQ_USART2_RX);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_5, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_5, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_5, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_5, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_5, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_5, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_5, LL_DMA_MDATAALIGN_BYTE);

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
  USART_InitStruct.BaudRate = 3000000;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART2, &USART_InitStruct);
  LL_USART_SetTXFIFOThreshold(USART2, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_SetRXFIFOThreshold(USART2, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_EnableDEMode(USART2);
  LL_USART_SetDESignalPolarity(USART2, LL_USART_DE_POLARITY_HIGH);
  LL_USART_SetDEAssertionTime(USART2, 0);
  LL_USART_SetDEDeassertionTime(USART2, 0);
  LL_USART_DisableFIFO(USART2);
  LL_USART_ConfigAsyncMode(USART2);

  /* USER CODE BEGIN WKUPType USART2 */

  /* USER CODE END WKUPType USART2 */

  LL_USART_Enable(USART2);

  /* Polling USART2 initialisation */
  while((!(LL_USART_IsActiveFlag_TEACK(USART2))) || (!(LL_USART_IsActiveFlag_REACK(USART2))))
  {
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

static void BISS1_SPI_DeInit(void){
	LL_GPIO_SetPinMode(MA1_PIN, LL_GPIO_MODE_ANALOG);
	LL_GPIO_SetPinMode(MA1_PIN, LL_GPIO_MODE_ANALOG);
}

void SetBiSS_SPI_Ch(BiSS_SPI_Ch_t ch_to_set){
}
