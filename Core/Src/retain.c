#include "retain.h"
#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"

#include "stm32f10x_i2c.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_misc.h"

#include <string.h>
#include <stdio.h>

extern I2C_HandleTypeDef hi2c1;

#define EE_I2C						I2C1

#define EE_I2C_DMA					DMA1
#define EE_I2C_DMA_CHANNEL_TX		DMA1_Channel6
#define EE_I2C_DMA_CHANNEL_RX		DMA1_Channel7
#define EE_I2C_DR_Address			((uint32_t)0x40005410)

#define EE_I2C_DMA_TX_IRQn			DMA1_Channel6_IRQn
#define EE_I2C_DMA_RX_IRQn			DMA1_Channel7_IRQn
#define EE_I2C_DMA_TX_IRQHandler	DMA1_Channel6_IRQHandler
#define EE_I2C_DMA_RX_IRQHandler	DMA1_Channel7_IRQHandler
#define EE_I2C_DMA_PREPRIO			0
#define EE_I2C_DMA_SUBPRIO			0

#define EE_TX_DMA_FLAG_GL			DMA1_FLAG_GL6
#define EE_TX_DMA_FLAG_TE			DMA1_FLAG_TE6
#define EE_TX_DMA_FLAG_HT			DMA1_FLAG_HT6
#define EE_TX_DMA_FLAG_TC			DMA1_FLAG_TC6

#define EE_RX_DMA_FLAG_GL			DMA1_FLAG_GL7
#define EE_RX_DMA_FLAG_TE			DMA1_FLAG_TE7
#define EE_RX_DMA_FLAG_HT			DMA1_FLAG_HT7
#define EE_RX_DMA_FLAG_TC			DMA1_FLAG_TC7

#define EE_I2C_PORT					GPIOB
#define EE_I2C_SCL_PIN				GPIO_PIN_6
#define EE_I2C_SDA_PIN				GPIO_PIN_7

#define EE_WP_PORT					GPIOD
#define EE_WP_PIN					GPIO_PIN_10

#define EE_I2C_PORT_CLOCK_CMD(en) 	{ if (en) __HAL_RCC_GPIOB_CLK_ENABLE(); }
#define EE_WP_PORT_CLOCK_CMD(en) 	{ if (en) __HAL_RCC_GPIOD_CLK_ENABLE(); }
#define EE_DMA_CLOCK_CMD(en) 		{ if (en) __HAL_RCC_DMA1_CLK_ENABLE(); }
#define EE_I2C_CLOCK_CMD(en)		{ if (en) __HAL_RCC_I2C1_CLK_ENABLE(); else __HAL_RCC_I2C1_CLK_DISABLE(); }

#define EE_FLAG_TIMEOUT		 		((uint32_t)0x1000)
#define EE_LONG_TIMEOUT		 		((uint32_t)(30 * EE_FLAG_TIMEOUT))
#define EE_ACK_WAIT_REPEAT_MAX300

#define EE_I2C_SPEED				100000//340000	 //kHz// at24c04c sup 1 MHz
#define EE_I2C_SLAVE_ADDRESS7		(0xA0 | 0x1 << 1)

static const uint32_t TX_FLAGS = EE_TX_DMA_FLAG_GL | EE_TX_DMA_FLAG_TE | EE_TX_DMA_FLAG_HT | EE_TX_DMA_FLAG_TC;
static const uint32_t RX_FLAGS = EE_RX_DMA_FLAG_GL | EE_RX_DMA_FLAG_TE | EE_RX_DMA_FLAG_HT | EE_RX_DMA_FLAG_TC;


#define MEMORY_VERSION			DATABASE_VERSION
#define EEPROM_OFFSET			
#define EEPROM_TOTAL_SIZE_BYTE	RETAIN_TOTAL_SIZE_BYTE
#define EEPROM_BYTES_ON_WR_MAX	(2-1) // 3...32 does not work!

#define EEPROM_PAGE0_ADDR		0
#define EEPROM_PAGE1_ADDR		(EEPROM_TOTAL_SIZE_BYTE/2)

#define EEPROM_CNT0_ADDR		0
#define EEPROM_CNT1_ADDR		(EEPROM_TOTAL_SIZE_BYTE/2)
#define EEPROM_CNTX_SIZE		1

#define EEPROM_CRC0_ADDR		((EEPROM_TOTAL_SIZE_BYTE/2)-2)
#define EEPROM_CRC1_ADDR		(EEPROM_TOTAL_SIZE_BYTE-2)
#define EEPROM_CRCX_SIZE		2

#define EEPROM_MEMVER0_ADDR		((EEPROM_TOTAL_SIZE_BYTE/2)-3)
#define EEPROM_MEMVER1_ADDR		(EEPROM_TOTAL_SIZE_BYTE-3)
#define EEPROM_MEMVERX_SIZE		1

#define EEPROM_DATA0_START_ADDR	1
#define EEPROM_DATA1_START_ADDR	((EEPROM_TOTAL_SIZE_BYTE/2)+1)
#define EEPROM_DATAX_SIZE		(sizeof(RetainDataBase_t))
#define EEPROM_DATAX_MAX		((EEPROM_TOTAL_SIZE_BYTE/2)-4)

#define EEPROM_CRCX_DATA_SIZE	((EEPROM_TOTAL_SIZE_BYTE/2)-2)

#define GENERATE_STOP(I2C) {I2C_AcknowledgeConfig(I2C, DISABLE); I2C_GenerateSTOP(I2C, ENABLE);}


struct EepromDrv_s {
	DMA_InitTypeDef2 DMA_InitStructure;
	uint8_t txBuf[34];
	struct {
		uint8_t state;
		uint16_t addr;
		uint8_t *buf;
		uint8_t bufSize;
	} wr;
};

typedef enum {
	EEDir_RX,
	EEDir_TX,
} EE_Direction;

typedef enum {
	eewar_Ok = -2,
	eewar_Error = -1,
	eewar_WaitForIrq = 0,
	eewar_CallAgain = 1,
	eewar_WaitForTimeout = 2,
} EE_WriteAutomatResult;


static void EE_init(EepromDrv_t *eedrv)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStruct;
	I2C_InitTypeDef2 I2C_InitStructure;

	EE_WP_PORT_CLOCK_CMD(ENABLE);
	EE_I2C_PORT_CLOCK_CMD(ENABLE);
	EE_I2C_CLOCK_CMD(ENABLE);
	EE_DMA_CLOCK_CMD(ENABLE);

	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL; //GPIO_PULLUP; //;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Pin = EE_I2C_SCL_PIN | EE_I2C_SDA_PIN;
	HAL_GPIO_Init(EE_I2C_PORT, &GPIO_InitStruct);
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Pin = EE_WP_PIN;
	HAL_GPIO_Init(EE_WP_PORT, &GPIO_InitStruct);

	NVIC_InitStructure.NVIC_IRQChannel = EE_I2C_DMA_TX_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = EE_I2C_DMA_PREPRIO;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = EE_I2C_DMA_SUBPRIO;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	//
	NVIC_InitStructure.NVIC_IRQChannel = EE_I2C_DMA_RX_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = EE_I2C_DMA_PREPRIO;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = EE_I2C_DMA_SUBPRIO;
	NVIC_Init(&NVIC_InitStructure);

	DMA_Cmd(EE_I2C_DMA_CHANNEL_TX, DISABLE);
	DMA_Cmd(EE_I2C_DMA_CHANNEL_RX, DISABLE);
	DMA_DeInit(EE_I2C_DMA_CHANNEL_TX);
	DMA_DeInit(EE_I2C_DMA_CHANNEL_RX);
	DMA_ClearFlag(TX_FLAGS | RX_FLAGS);
	eedrv->DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)EE_I2C_DR_Address;
	eedrv->DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	eedrv->DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	eedrv->DMA_InitStructure.DMA_PeripheralDataSize = DMA_MemoryDataSize_Byte;
	eedrv->DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	eedrv->DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	eedrv->DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	eedrv->DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

	eedrv->DMA_InitStructure.DMA_MemoryBaseAddr = 0;			/* This parameter will be configured durig communication */
	eedrv->DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;	/* This parameter will be configured durig communication */
	eedrv->DMA_InitStructure.DMA_BufferSize = 0;				/* This parameter will be configured durig communication */

	DMA_Init(EE_I2C_DMA_CHANNEL_TX, &eedrv->DMA_InitStructure);
	DMA_Init(EE_I2C_DMA_CHANNEL_RX, &eedrv->DMA_InitStructure);

	DMA_ITConfig(EE_I2C_DMA_CHANNEL_TX, DMA_IT_TC | DMA_IT_TE, ENABLE);
	DMA_ITConfig(EE_I2C_DMA_CHANNEL_RX, DMA_IT_TC | DMA_IT_TE, ENABLE);

	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0xA0;
	I2C_InitStructure.I2C_Ack = 0;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = EE_I2C_SPEED;

	I2C_Init(EE_I2C, &I2C_InitStructure, HAL_RCC_GetPCLK1Freq());
	I2C_Cmd(EE_I2C, ENABLE);

	(void)EE_I2C->SR1;
	(void)EE_I2C->SR2;
}


static void EE_deinit()
{
	HAL_GPIO_DeInit(EE_WP_PORT, EE_WP_PIN);
	HAL_GPIO_DeInit(EE_I2C_PORT, EE_I2C_SCL_PIN | EE_I2C_SDA_PIN);
	DMA_Cmd(EE_I2C_DMA_CHANNEL_TX, DISABLE);
	DMA_Cmd(EE_I2C_DMA_CHANNEL_RX, DISABLE);
	DMA_DeInit(EE_I2C_DMA_CHANNEL_TX);
	DMA_DeInit(EE_I2C_DMA_CHANNEL_RX);
	I2C_Cmd(EE_I2C, DISABLE);
	EE_I2C_CLOCK_CMD(DISABLE);
}


static void EE_dmaConfig(EepromDrv_t *eedrv, uint8_t *buf, uint32_t bufSize, EE_Direction direction)
{
	if (direction == EEDir_TX) {
		DMA_Cmd(EE_I2C_DMA_CHANNEL_TX, DISABLE);
		eedrv->DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)buf;
		eedrv->DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
		eedrv->DMA_InitStructure.DMA_BufferSize = bufSize;
		DMA_Init(EE_I2C_DMA_CHANNEL_TX, &eedrv->DMA_InitStructure);
	} else {
		DMA_Cmd(EE_I2C_DMA_CHANNEL_RX, DISABLE);
		eedrv->DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)buf;
		eedrv->DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
		eedrv->DMA_InitStructure.DMA_BufferSize = bufSize;
		DMA_Init(EE_I2C_DMA_CHANNEL_RX, &eedrv->DMA_InitStructure);
	}
}


int EE_read(EepromDrv_t *eedrv, uint16_t addr, uint8_t* buf, uint16_t bufSize)
{
	if (!buf || !bufSize)
		return 1;

	const uint8_t EEAddress = EE_I2C_SLAVE_ADDRESS7;
	uint32_t timeout;

	I2C_GenerateSTART(EE_I2C, ENABLE);

	// ожидание выдачи START
	timeout = EE_FLAG_TIMEOUT;
	while ( (EE_I2C->SR1 & I2C_SR1_SB) == RESET ) {
		if ((timeout--) == 0)
			return 1;
	}

	// DEV ADDRESS
	I2C_Send7bitAddress(EE_I2C, EEAddress, I2C_Direction_Transmitter);

	timeout = EE_FLAG_TIMEOUT;
	while ( (EE_I2C->SR1 & I2C_SR1_ADDR) == RESET ) {
		if ((timeout--) == 0)
			return 1;
	}

	(void)EE_I2C->SR1;
	(void)EE_I2C->SR2;

	// WORD1 ADDRESS
	I2C_SendData(EE_I2C, (uint8_t)((addr >> 8) & 0x1F));

	// ожидание передачи адреса слова
	timeout = EE_FLAG_TIMEOUT;
	while ( (EE_I2C->SR1 & I2C_SR1_TXE) == RESET ) {
		if ((timeout--) == 0)
			return 1;
	}

	// WORD2 ADDRESS
	I2C_SendData(EE_I2C, (uint8_t)((addr >> 0) & 0xFF));

	// ожидание передачи адреса слова
	timeout = EE_FLAG_TIMEOUT;
	while ( (EE_I2C->SR1 & I2C_SR1_TXE) == RESET ) {
		if ((timeout--) == 0)
			return 1;
	}

	I2C_GenerateSTART(EE_I2C, ENABLE);

	// ожидание выдачи START
	timeout = EE_FLAG_TIMEOUT;
	while ( (EE_I2C->SR1 & I2C_SR1_SB) == RESET ) {
		if ((timeout--) == 0)
			return 1;
	}

	// DEV ADDRESS
	I2C_Send7bitAddress(EE_I2C, EEAddress, I2C_Direction_Receiver);

	timeout = EE_FLAG_TIMEOUT;
	while ( (EE_I2C->SR1 & I2C_SR1_ADDR) == RESET ) {
		if ((timeout--) == 0)
			return 1;
	}

	(void)EE_I2C->SR1;
	(void)EE_I2C->SR2;

	I2C_AcknowledgeConfig(EE_I2C, ENABLE);

	int i = 0;
	while (i != bufSize) {
		(void)EE_I2C->SR2;

		timeout = EE_FLAG_TIMEOUT;
		while ( (EE_I2C->SR1 & I2C_SR1_RXNE) == RESET ) {
			if ((timeout--) == 0)
				return 1;
		}

		if (i == bufSize-1) {
			I2C_AcknowledgeConfig(EE_I2C, DISABLE);
		}

		buf[i++] = (uint8_t)(EE_I2C->DR);
	}

	// STOP
	GENERATE_STOP(EE_I2C);

	timeout = EE_FLAG_TIMEOUT;
	while (EE_I2C->CR1 & I2C_CR1_STOP) {
		if ((timeout--) == 0)
			return 1;
	}

	return 0;
}


void EE_txIrqHandler()
{
	// COMPLETE
	if ( DMA_GetFlagStatus(EE_TX_DMA_FLAG_TC) == SET ) {
		DMA_Cmd(EE_I2C_DMA_CHANNEL_TX, DISABLE);
		DMA_ClearFlag(EE_TX_DMA_FLAG_TC);
	}
	// ERROR
	if ( DMA_GetFlagStatus(EE_TX_DMA_FLAG_TE) == SET ) {
		GENERATE_STOP(EE_I2C);
	}
}


static inline int EE_readOffs(EepromDrv_t *eedrv, uint16_t addr, uint8_t *buf, uint16_t bufSize)
{
	return EE_read(eedrv, addr+EEPROM_OFFSET, buf, bufSize);
}


static inline void EE_prepForWriting(EepromDrv_t *eedrv, uint16_t addr, uint8_t *buf, uint16_t bufSize)
{
	eedrv->wr.addr = addr;
	eedrv->wr.buf = buf;
	eedrv->wr.bufSize = bufSize;
	eedrv->wr.state = 0;
}


static EE_WriteAutomatResult EE_writePageOffs(EepromDrv_t *eedrv)
{
	EE_WriteAutomatResult ret = eewar_Error;
	volatile uint32_t timeout;

	switch (eedrv->wr.state) {
		case 0: { // setup device for writing
			const uint8_t EEAddress = EE_I2C_SLAVE_ADDRESS7;

			I2C_AcknowledgeConfig(EE_I2C, DISABLE);

			eedrv->wr.addr += EEPROM_OFFSET;

			/*!< While the bus is busy */
			timeout = EE_LONG_TIMEOUT;
			while ( I2C_GetFlagStatus(EE_I2C, I2C_FLAG_BUSY) ) {
				if ((timeout--) == 0)
					return eewar_Error;
			}

			// disable write protect
			HAL_GPIO_WritePin(EE_WP_PORT, EE_WP_PIN, GPIO_PIN_RESET);

			(void)EE_I2C->SR1;
			(void)EE_I2C->SR2;

			// START
			I2C_GenerateSTART(EE_I2C, ENABLE);

			/*!< Test on EV5 and clear it */
			timeout = EE_FLAG_TIMEOUT;
			while( I2C_CheckEvent(EE_I2C, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS ) {
				if ((timeout--) == 0)
					return eewar_Error;
			}

			// DEV ADDRESS
			I2C_Send7bitAddress(EE_I2C, EEAddress, I2C_Direction_Transmitter);

			ret = eewar_WaitForTimeout;
			eedrv->wr.state = 1;
		} break;
		case 1: { // setup data for writing
			timeout = EE_FLAG_TIMEOUT;
			while ( (EE_I2C->SR1 & I2C_SR1_ADDR) == RESET ) {
				if ((timeout--) == 0)
					return eewar_Error;
			}

			(void)EE_I2C->SR1;
			(void)EE_I2C->SR2;

			uint16_t addr = eedrv->wr.addr;
			uint8_t* buf = eedrv->wr.buf;
			uint8_t bufSize = eedrv->wr.bufSize;

			uint8_t *p = eedrv->txBuf;
			p[0] = (uint8_t)((addr >> 8) & 0x1F); // WORD1 ADDRESS
			p[1] = (uint8_t)((addr >> 0) & 0xFF); // WORD2 ADDRESS
			memcpy(p+2, buf, bufSize);
			bufSize += 2;

			/* Configure the DMA Tx Channel with the buffer address and the buffer size */
			EE_dmaConfig(eedrv, p, bufSize, EEDir_TX);

			DMA_ClearFlag(TX_FLAGS);

			/* Enable the EE_I2C peripheral DMA requests */
			I2C_DMACmd(EE_I2C, ENABLE);

			/* Enable the DMA Tx Stream */
			DMA_Cmd(EE_I2C_DMA_CHANNEL_TX, ENABLE);

			eedrv->wr.state = 2;
			ret = eewar_WaitForIrq;
		} break;
		case 2: { // i2c-dma irq tx end
			EE_txIrqHandler();

			eedrv->wr.state = 3;
			ret = eewar_CallAgain;
		} break;
		case 3: { // prep for waiting wr end
			while ( I2C_GetFlagStatus(EE_I2C, I2C_FLAG_BTF) != SET )
				;

			// STOP
			GENERATE_STOP(EE_I2C);

			eedrv->wr.state = 4;
			ret = eewar_WaitForTimeout + 3;
		} break;
		case 4: { // waiting for writing end
			const uint8_t EEAddress = EE_I2C_SLAVE_ADDRESS7;

			if (EE_I2C->SR2 & I2C_SR2_MSL) {
				return eewar_Error;
			}

			timeout = EE_LONG_TIMEOUT;
			while ( I2C_GetFlagStatus(EE_I2C, I2C_FLAG_BUSY) == SET ) {
				if ((timeout--) == 0)
					return eewar_Error;
			}

			// START
			I2C_GenerateSTART(EE_I2C, ENABLE);

			/*!< Test on EV5 and clear it */
			timeout = EE_FLAG_TIMEOUT;
			while( I2C_CheckEvent(EE_I2C, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS ) {
				if ((timeout--) == 0)
					return eewar_Error;
			}

			// DEV ADDRESS
			I2C_Send7bitAddress(EE_I2C, EEAddress, I2C_Direction_Transmitter);

			ret = eewar_WaitForTimeout;
			eedrv->wr.state = 5;
		} break;
		case 5: { // check device: still wait or exit
			timeout = EE_FLAG_TIMEOUT;
			while ( (EE_I2C->SR1 & I2C_SR1_ADDR) == RESET ) {
				if ((timeout--) == 0)
					return eewar_Error;
			}

			(void)EE_I2C->SR1;
			(void)EE_I2C->SR2;

			if (EE_I2C->SR1 & I2C_SR1_AF) {
				EE_I2C->SR1 &= ~I2C_SR1_AF;
				eedrv->wr.state = 4;
				ret = eewar_CallAgain;
			} else {
				// STOP
				GENERATE_STOP(EE_I2C);
				eedrv->wr.state = 6;
				ret = eewar_CallAgain;
			}
		} break;
		case 6: { // set wp
			while (EE_I2C->SR2 & I2C_SR2_MSL)
				;
			HAL_GPIO_WritePin(EE_WP_PORT, EE_WP_PIN, GPIO_PIN_SET);

			return eewar_Ok;
		} break;
	}

	return ret;
}


static EE_WriteAutomatResult retain_writeAutomat(Retain_t *drv)
{
	EepromDrv_t *eedrv = drv->eedrv;
	int page = drv->wr.nextPage;
	uint8_t *peeprom = drv->eeprom;
	uint16_t cntAddr, crcAddr, verAddr;

	// prep page data
	{
		if (drv->wr.state == 0) { // begin
			if (page == 1) {
				drv->wr.nextPage = 0;
				drv->wr.waddr = EEPROM_DATA1_START_ADDR;
				cntAddr = EEPROM_CNT1_ADDR;
				crcAddr = EEPROM_CRC1_ADDR;
				verAddr = EEPROM_MEMVER1_ADDR;
				drv->wr.caddr = EEPROM_DATA0_START_ADDR;
			} else {
				drv->wr.nextPage = 1;
				drv->wr.waddr = EEPROM_DATA0_START_ADDR;
				cntAddr = EEPROM_CNT0_ADDR;
				crcAddr = EEPROM_CRC0_ADDR;
				verAddr = EEPROM_MEMVER0_ADDR;
				drv->wr.caddr = EEPROM_DATA1_START_ADDR;
			}
		} else { // process
			if (page == 0) {
				cntAddr = EEPROM_CNT1_ADDR;
				crcAddr = EEPROM_CRC1_ADDR;
				verAddr = EEPROM_MEMVER1_ADDR;
			} else {
				cntAddr = EEPROM_CNT0_ADDR;
				crcAddr = EEPROM_CRC0_ADDR;
				verAddr = EEPROM_MEMVER0_ADDR;
			}
		}
	}

	switch (drv->wr.state) {
		case 0: { // copy data
			__disable_irq();
			memcpy(drv->dcopy, &drv->db, EEPROM_DATAX_SIZE);
			__enable_irq();
			drv->wr.idx = 0;
			drv->wr.chgd = 0; // hope that the data were not change

			if (peeprom[verAddr] != MEMORY_VERSION)
				drv->wr.chgd = 1;

			drv->wr.state = 1;
			return eewar_CallAgain;
		} break;
		//
		case 1: { // prep for database writing
			uint16_t cnt;
			uint16_t wraddr;
			for (; drv->wr.idx < EEPROM_DATAX_SIZE; ++(drv->wr.idx), ++(drv->wr.waddr), ++(drv->wr.caddr)) {
				if (
					(peeprom[drv->wr.caddr] != drv->dcopy[drv->wr.idx])
					||
					(peeprom[drv->wr.waddr] != drv->dcopy[drv->wr.idx])
				) {
					drv->wr.chgd = 1; // rewrite page metadata and data
					cnt = 1;

//					for (; cnt < EEPROM_BYTES_ON_WR_MAX && (drv->wr.idx+cnt) < EEPROM_DATAX_SIZE; ++cnt) {
//						if (
//							(peeprom[drv->wr.caddr+cnt] == pcopy[drv->wr.idx+cnt])
//							&&
//							(peeprom[drv->wr.waddr+cnt] == pcopy[drv->wr.idx+cnt])
//						) {
//							break;
//						}
//					}

					cnt = EEPROM_DATAX_SIZE - (drv->wr.idx);
					if (cnt > EEPROM_BYTES_ON_WR_MAX) cnt = EEPROM_BYTES_ON_WR_MAX;

					wraddr = drv->wr.waddr;

					memcpy(peeprom+wraddr, drv->dcopy+drv->wr.idx, cnt);
					drv->wr.idx += cnt;
					drv->wr.waddr += cnt;
					drv->wr.caddr += cnt;
					//
					EE_prepForWriting(eedrv, wraddr, peeprom+wraddr, cnt);
					drv->wr.state = 2;
					return eewar_CallAgain;
				}
			}
			if (drv->wr.idx >= EEPROM_DATAX_SIZE) { // all checked
				if (drv->wr.chgd) {
					drv->wr.state = 3;
					return eewar_CallAgain;
				} else {
					return eewar_Ok; // all done
				}
			}
		} break;
		case 2: { // database writing
			EE_WriteAutomatResult res = EE_writePageOffs(eedrv);
			switch (res) {
				case eewar_Ok: {
					drv->wr.state = 1;
					return eewar_CallAgain;
				} break;
				default: {
					return res;
				} break;
			}
		} break;
		case 3: { // prep for VER writing
			drv->wr.state = 4;
			if (peeprom[verAddr] != MEMORY_VERSION) {
				peeprom[verAddr] = MEMORY_VERSION;
				//
				EE_prepForWriting(eedrv, verAddr, peeprom+verAddr, 1);
				drv->wr.state = 4;
			} else {
				drv->wr.state = 5;
			}
			return eewar_CallAgain;
		} break;
		case 4: { // VER writing
			EE_WriteAutomatResult res = EE_writePageOffs(eedrv);
			switch (res) {
				case eewar_Ok: {
					drv->wr.state = 5;
					return eewar_CallAgain;
				} break;
				default: {
					return res;
				} break;
			}
		} break;
		case 5: { // prep for CNT writing
			uint8_t v = ++(drv->wr.CNT);
			peeprom[cntAddr] = v;
			//
			EE_prepForWriting(eedrv, cntAddr, peeprom+cntAddr, 1);
			drv->wr.state = 6;
			return eewar_CallAgain;
		} break;
		case 6: { // CNT writing
			EE_WriteAutomatResult res = EE_writePageOffs(eedrv);
			switch (res) {
				case eewar_Ok: {
					drv->wr.state = 7;
					return eewar_CallAgain;
				} break;
				default: {
					return res;
				} break;
			}
		} break;
		case 7: { // prep for CRC writing
			typedef union {
				uint16_t d;
				uint8_t b[2];
			} uint16_str;
			uint16_str crc;
			crc.d = fast_crc16(0, &(peeprom[cntAddr]), EEPROM_CRCX_DATA_SIZE);
			memcpy(&(peeprom[crcAddr]), crc.b, 2);
			//
			EE_prepForWriting(eedrv, crcAddr, peeprom+crcAddr, 2);
			drv->wr.state = 8;
			return eewar_CallAgain;
		} break;
		case 8: { // CRC writing
			return EE_writePageOffs(eedrv);
		} break;
	}

	return eewar_CallAgain;
}


#define DRV_RESOURCES (\
		RESMAN_SUP_3_3V \
		| RESMAN_DMA1 \
		| RESMAN_GPIOB \
		| RESMAN_GPIOD)


void retain_cycle(Retain_t *drv, void *caller)
{
	switch (drv->base.state) {
		case 0: {
		} break;
		case 1: {
			if (drv->trgUpd) {
				drv->trgUpd = 0;
				EE_init(drv->eedrv);
				drv->base.resman.allbits = DRV_RESOURCES;
				drv->wr.state = 0;
				drv->base.state = 2;
				drv->base.callAfter(dToB(drv), 0);
			}
		} break;
		case 2: {
			EE_WriteAutomatResult res = retain_writeAutomat(drv);
			switch (res) {
				case eewar_Ok: {
					if (drv->trgUpd) {
						drv->trgUpd = 0;
						drv->wr.state = 0;
						drv->base.callAfter(dToB(drv), 0);
					} else {
						drv->base.resman.allbits = 0;
						EE_deinit();
						drv->base.state = 1;
					}
				} break;
				case eewar_Error: {
					drv->base.resman.allbits = 0;
					EE_deinit();
					drv->base.state = 1;
				} break;
				case eewar_CallAgain: {
					drv->base.callAfter(dToB(drv), 0);
				} break;
				case eewar_WaitForIrq: {
				} break;
				default: {
					drv->base.callAfter(dToB(drv), (uint32_t)res);
				} break;
			}
		} break;
		default: break;
	}
}


void retain_triggerUpdate(Retain_t *drv)
{
	drv->trgUpd = 1;
	if (drv->base.state == 1) { // cmd waiting
		drv->base.callAfter(dToB(drv), 0);
	}
}


static void retain_setDefaults(Retain_t *drv)
{
	
	
}


static void retain_resetTs(Retain_t *drv)
{
	memset(&drv->db.adc.ts, 0, sizeof(AdcDrvTimeStamp_t));
	memset(&drv->db.din.ts, 0, sizeof(DinDrvTimeStamp_t));
	memset(&drv->db.st1w.ts, 0, sizeof(SensT1wDrvTimeStamp_t));
	memset(&drv->db.sht.ts, 0, sizeof(ShtDrvTimeStamp_t));
	//memset(&drv->db.wt.ts, 0, sizeof(WtDrvTimeStamp_t));
	retain_triggerUpdate(drv);
}


void retain_resetDefault(Retain_t *drv)
{
	retain_setDefaults(drv);
	retain_triggerUpdate(drv);
}


static inline void retain_setDefaultOnReadError(Retain_t *drv)
{
	retain_setDefaults(drv);
	memset(&drv->db.din.ts, 0, sizeof(DinDrvTimeStamp_t));
	memset(&drv->db.adc.ts, 0, sizeof(AdcDrvTimeStamp_t));
	memset(&drv->db.st1w.ts, 0, sizeof(SensT1wDrvTimeStamp_t));
	memset(&drv->db.sht.ts, 0, sizeof(ShtDrvTimeStamp_t));
	memset(&drv->db.wt.ts, 0, sizeof(WtDrvTimeStamp_t));
}


void retain_init(Retain_t *drv, void *unused)
{
	basedriver_init(&drv->base);
	drv->base.cycle = (basedrv_cycle)retain_cycle;
	drv->base.resetTs = (basedrv_resetTs)retain_resetTs;

	if (EEPROM_DATAX_SIZE > EEPROM_DATAX_MAX)
		while(1);

	drv->eedrv = (EepromDrv_t *)calloc(1, sizeof(EepromDrv_t));

	EE_init(drv->eedrv);
	EE_readOffs(drv->eedrv, 0, drv->eeprom, EEPROM_TOTAL_SIZE_BYTE);

	uint16_t crc0 = fast_crc16(0, drv->eeprom+EEPROM_CNT0_ADDR, EEPROM_CRCX_DATA_SIZE);
	uint16_t crc1 = fast_crc16(0, drv->eeprom+EEPROM_CNT1_ADDR, EEPROM_CRCX_DATA_SIZE);
	uint16_t rx_crc0 = *((uint16_t*)(&(drv->eeprom[EEPROM_CRC0_ADDR])));
	uint16_t rx_crc1 = *((uint16_t*)(&(drv->eeprom[EEPROM_CRC1_ADDR])));
	int cnt0 = -1;
	int cnt1 = -1;
	if (crc0 == rx_crc0 && rx_crc0 != 0) {
		cnt0 = drv->eeprom[EEPROM_CNT0_ADDR];
	}
	if (crc1 == rx_crc1 && rx_crc1 != 0) {
		cnt1 = drv->eeprom[EEPROM_CNT1_ADDR];
	}
	if (cnt0 == -1 && cnt1 == -1) {
		retain_setDefaultOnReadError(drv);
		goto init_exit;
	}

	int start_addr = 0;
	if ( (cnt1 > cnt0) || (cnt0 == 255 && cnt1 == 0) ) {
		start_addr = EEPROM_DATA1_START_ADDR;
		drv->wr.nextPage = 0;
		drv->wr.CNT = cnt1;
	} else if (cnt0 != -1) {
		start_addr = EEPROM_DATA0_START_ADDR;
		drv->wr.nextPage = 1;
		drv->wr.CNT = cnt0;
	} else {
		start_addr = EEPROM_DATA1_START_ADDR;
		drv->wr.nextPage = 0;
		drv->wr.CNT = cnt1;
	}

	// проверка памяти на разницу версий
	if (
		((cnt0 > cnt1 && drv->eeprom[EEPROM_MEMVER0_ADDR] != MEMORY_VERSION) ||
		(cnt1 > cnt0 && drv->eeprom[EEPROM_MEMVER1_ADDR] != MEMORY_VERSION))
			||
		((cnt1 == cnt0) &&
		(drv->eeprom[EEPROM_MEMVER0_ADDR] != MEMORY_VERSION || drv->eeprom[EEPROM_MEMVER1_ADDR] != MEMORY_VERSION))
	) {
		retain_setDefaultOnReadError(drv);
		goto init_exit;
	}

	memcpy(&drv->db, &(drv->eeprom[start_addr]), EEPROM_DATAX_SIZE);

init_exit:
	drv->base.state = 1;
	EE_deinit();

	(void)unused;
}


Retain_t *retain_create()
{
	Retain_t *ret = (Retain_t *)calloc(1, sizeof(Retain_t));
	return ret;
}


void retain_free(Retain_t *drv)
{
	free(drv->eedrv);
	free(drv);
}

