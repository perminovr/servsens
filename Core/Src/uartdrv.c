#include "uartdrv.h"


extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;


#define UARTDRV_UART			USART1
#define UARTDRV_UART_IRQ		USART1_IRQn
#define UARTDRV_DMA				DMA1
#define UARTDRV_PORT			GPIOA
#define UARTDRV_PINS_TX			GPIO_PIN_9
#define UARTDRV_PINS_RX			GPIO_PIN_10
#define UARTDRV_UART_CLOCK_CMD(en)	{ if (en) __HAL_RCC_USART1_CLK_ENABLE(); else __HAL_RCC_USART1_CLK_DISABLE(); }
#define UARTDRV_DMA_CLOCK_CMD(en)	{ if (en) __HAL_RCC_DMA1_CLK_ENABLE(); }
#define UARTDRV_PORT_CLOCK_CMD(en)	{ if (en) __HAL_RCC_GPIOA_CLK_ENABLE(); }

#define CONTROLBYTE_DLE 0xAA
#define CONTROLBYTE_STX 0xBB
#define CONTROLBYTE_ETX 0xEE


static void uartdrv_initPeriph(UartDrv_t *drv)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	UARTDRV_UART_CLOCK_CMD(ENABLE);
	UARTDRV_DMA_CLOCK_CMD(ENABLE);
	UARTDRV_PORT_CLOCK_CMD(ENABLE);

	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

	GPIO_InitStruct.Pin = UARTDRV_PINS_TX;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	HAL_GPIO_Init(UARTDRV_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = UARTDRV_PINS_RX;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(UARTDRV_PORT, &GPIO_InitStruct);

	hdma_usart1_tx.Instance = DMA1_Channel4;
	hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
	hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
	hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_usart1_tx.Init.Mode = DMA_NORMAL;
	hdma_usart1_tx.Init.Priority = DMA_PRIORITY_LOW;
	HAL_DMA_Init(&hdma_usart1_tx);

	__HAL_LINKDMA(&huart1, hdmatx, hdma_usart1_tx);

	huart1.Instance = UARTDRV_UART;
	huart1.Init.BaudRate = 9600;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	HAL_UART_Init(&huart1);

	HAL_NVIC_EnableIRQ(UARTDRV_UART_IRQ);
}


static void uartdrv_deinitPeriph(UartDrv_t *drv)
{
	HAL_NVIC_DisableIRQ(UARTDRV_UART_IRQ);
	HAL_UART_DeInit(&huart1);
	HAL_DMA_DeInit(huart1.hdmatx);
	HAL_GPIO_DeInit(UARTDRV_PORT, UARTDRV_PINS_TX|UARTDRV_PINS_RX);
	UARTDRV_UART_CLOCK_CMD(DISABLE);
}


static inline int cycleBuffLen(int startIdx, int endIdx, int maxSize)
{
	int ret = (endIdx >= startIdx)?
		endIdx - startIdx : maxSize - startIdx + endIdx;
	return ret + 1;
}


static uint8_t uartdrv_getRcvdByIdx(UartDrv_t *drv, int i)
{
	int idx = drv->startPos + i;
	if (idx >= UARTDRV_BUFFER_SIZE_MAX) {
		idx -= UARTDRV_BUFFER_SIZE_MAX;
	}
	return drv->rxb[idx];
}
#define byIdx(i) uartdrv_getRcvdByIdx(drv,i)


static int uartdrv_handleReceived(UartDrv_t *drv)
{
	// HAL_UART_Transmit_DMA(&huart1, drv->txb, drv->tit);
	// HAL_UART_Receive_IT(&huart1, drv->rxb+drv->rit, 1);
	int ret = 0;
	uint8_t rcvd = *(drv->rxb + drv->rit);

handleReceived_fromStart:
	// find start
	switch (drv->startDetected) {
		case 0: {
			if (rcvd == CONTROLBYTE_DLE) {
				drv->startDetected = 1;
			}
			goto handleReceived_exit;
		} break;
		case 1: {
			if (rcvd == CONTROLBYTE_STX) {
				drv->startDetected = 2;
				drv->startPos = drv->rit-1;
			} else {
				drv->startDetected = 0;
			}
			goto handleReceived_exit;
		} break;
		// prevent two starting sequence
		case 2: {
			if (rcvd == CONTROLBYTE_DLE) {
				drv->startDetected = 3;
			}
		} break;
		case 3: {
			if (rcvd == CONTROLBYTE_DLE) {
				drv->startDetected = 2;
				break;
			} else if (rcvd == CONTROLBYTE_STX) {
				drv->startDetected = 2;
				drv->stopDetected = 0;
				drv->startPos = drv->rit-1;
				goto handleReceived_exit;
			}
		} break;
		default: break;
	}

	// check roll over
	if (drv->rit == drv->startPos) {
		drv->startDetected = 0;
		drv->stopDetected = 0;
		goto handleReceived_fromStart;
	}

	// find end
	switch (drv->stopDetected) {
		case 0: {
			if (rcvd == CONTROLBYTE_DLE) {
				drv->stopDetected = 1;
			}
		} break;
		case 1: {
			if (rcvd == CONTROLBYTE_ETX) {
				// found
				drv->startDetected = 0;
				drv->stopDetected = 0;
				//
				uint8_t oi = 0;
				int size = cycleBuffLen(drv->startPos, drv->rit, UARTDRV_BUFFER_SIZE_MAX);
				for (int i = 2; i < (size-2); ++i) {
					if (byIdx(i) == CONTROLBYTE_DLE) {
						i++;
					}
					drv->data.message[oi++] = byIdx(i);
				}
				uint16_t crc1 = calc_crc16(drv->data.message, oi-2);
				uint16_t crc2 = mU16(&(drv->data.message[oi-2]));
				if (crc1 == crc2) {
					drv->data.size = oi-2;
					drv->base.dataIsReady = 1;
					drv->base.eventIsReady = 1;
					ret = 1;
					goto handleReceived_exit;
				}
			}
			drv->stopDetected = 0;
		} break;
		default: break;
	}

handleReceived_exit:
	drv->rit++;
	if (drv->rit >= UARTDRV_BUFFER_SIZE_MAX) {
		drv->rit = 0;
	}
	__HAL_UART_CLEAR_OREFLAG(&huart1);
	HAL_UART_Receive_IT(&huart1, drv->rxb+drv->rit, 1);

	return ret;
}


void uartdrv_cycle(UartDrv_t *drv, void *caller)
{
	if (caller == UARTDRV_UART) {
		if ( uartdrv_handleReceived(drv) ) {
			drv->base.callAfter(dToB(drv), 0);
		}
	}
}


#define DRV_RESOURCES (RESMAN_DMA1 | RESMAN_GPIOA)


static inline void uartdrv_resetState(UartDrv_t *drv)
{
	drv->startDetected = 0;
	drv->stopDetected = 0;
	drv->startPos = 0;
	drv->rit = 0;
	if (drv->prms.enabled) {
		drv->base.resman.allbits = DRV_RESOURCES;
		uartdrv_initPeriph(drv);
		HAL_UART_Receive_IT(&huart1, drv->rxb + drv->rit, 1);
	} else {
		drv->base.resman.allbits = 0;
		uartdrv_deinitPeriph(drv);
	}
}


void uartdrv_setParams(UartDrv_t *drv, UartDrvParams_t *params)
{
	__disable_irq();
	memcpy(&drv->prms, params, sizeof(UartDrvParams_t));
	uartdrv_resetState(drv);
	__enable_irq();
}


static int encode(const uint8_t *input, int isize, uint8_t *output, int osize)
{
	int i = 0;
	output[i++] = CONTROLBYTE_DLE;
	output[i++] = CONTROLBYTE_STX;
	for (int j = 0; j < isize && i < osize-4; ++j) {
		uint8_t v = input[j];
		if (v == CONTROLBYTE_DLE) {
			output[i++] = CONTROLBYTE_DLE;
		}
		output[i++] = v;
	}
	output[i++] = CONTROLBYTE_DLE;
	output[i++] = CONTROLBYTE_ETX;
	return i;
}


int uartdrv_setData(UartDrv_t *drv, const UartDrvData_t *msg)
{
	int ret = -1;
	__disable_irq();
	if (HAL_DMA_GetState(&hdma_usart1_tx) == HAL_DMA_STATE_READY) {
		int sz = encode(msg->message, msg->size, drv->txb, UARTDRV_BUFFER_SIZE_MAX);
		HAL_UART_Transmit_DMA(&huart1, drv->txb, sz);
		ret = 0;
	}
	__enable_irq();
	return ret;
}


void uartdrv_getData(UartDrv_t *drv, UartDrvData_t *copyto, uint8_t forceCopy)
{
	if (forceCopy || drv->base.dataIsReady) {
		__disable_irq();
		memcpy(copyto, &drv->data, sizeof(UartDrvData_t));
		drv->base.dataIsReady = 0;
		drv->base.eventIsReady = 0;
		__enable_irq();
	}
}


void uartdrv_init(UartDrv_t *drv, void *unused)
{
	basedriver_init(&drv->base);
	drv->base.cycle = (basedrv_cycle)uartdrv_cycle;
	drv->base.resetTs = (basedrv_resetTs)0;

	(void)unused;
}


UartDrv_t *uartdrv_create()
{
	UartDrv_t *ret = (UartDrv_t *)calloc(1, sizeof(UartDrv_t));
	return ret;
}


void uartdrv_free(UartDrv_t *drv)
{
	free(drv);
}
