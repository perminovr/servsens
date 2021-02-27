#include "shtdrv.h"
#include "utils.h"

#include "stm32f10x_i2c.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_misc.h"

#define TH_I2C						I2C2

#define TH_I2C_DMA					DMA1
#define TH_I2C_DMA_CHANNEL_RX		DMA1_Channel5
#define TH_I2C_DR_Address			((uint32_t)0x40005810)

#define TH_I2C_DMA_RX_IRQn			DMA1_Channel5_IRQn
#define TH_I2C_DMA_RX_IRQHandler	DMA1_Channel5_IRQHandler
#define TH_I2C_DMA_PREPRIO			0
#define TH_I2C_DMA_SUBPRIO			0

#define TH_RX_DMA_FLAG_GL			DMA1_FLAG_GL5		//global
#define TH_RX_DMA_FLAG_TE			DMA1_FLAG_TE5		//transfer error
#define TH_RX_DMA_FLAG_HT			DMA1_FLAG_HT5		//Half transfer
#define TH_RX_DMA_FLAG_TC			DMA1_FLAG_TC5		//Transfer complete

#define TH_I2C_PORT					GPIOB
#define TH_I2C_SCL_PIN				GPIO_PIN_10			//clock
#define TH_I2C_SDA_PIN				GPIO_PIN_11			//data

#define TH_I2C_PORT_CLOCK_CMD(en) 	{ if (en) __HAL_RCC_GPIOB_CLK_ENABLE(); }
#define TH_DMA_CLOCK_CMD(en) 		{ if (en) __HAL_RCC_DMA1_CLK_ENABLE(); }
#define TH_I2C_CLOCK_CMD(en)		{ if (en) __HAL_RCC_I2C2_CLK_ENABLE(); else __HAL_RCC_I2C2_CLK_DISABLE(); }

#define TH_FLAG_TIMEOUT		 		((uint32_t)0x10000)
#define Th_LONG_TIMEOUT		 		((uint32_t)(30 * TH_FLAG_TIMEOUT))
#define TH_ACK_WAIT_REPEAT_MAX300

#define TH_I2C_SPEED				100000	 //kHz
#define TH_I2C_SLAVE_ADDRESS7		(0x07 | 0x1 << 1)

static const uint32_t RX_FLAGS = TH_RX_DMA_FLAG_GL | TH_RX_DMA_FLAG_TE | TH_RX_DMA_FLAG_HT | TH_RX_DMA_FLAG_TC;

static const uint8_t SENSOR_CRC_ARR[256] = {0,49, 98, 83, 196, 245, 166, 151, 185, 136, 219, 234, 125, 76, 31, 46,	//0-15
									   67, 114, 33, 16, 135, 182, 229, 212, 250, 203, 152, 169, 62, 15, 92, 109,	//16-31
									   134, 183, 228, 213, 66, 115, 32, 17, 63, 14, 93, 108, 251, 202, 153, 168,	//32-47
									   197, 244, 167, 150, 1, 48, 99, 82, 124, 77, 30, 47, 184, 137, 218, 235,		//48-63
									   61, 12, 95, 110, 249, 200, 155, 170, 132, 181, 230, 215, 64, 113, 34, 19,	//64-79
									   126, 79, 28, 45, 186, 139, 216, 233, 199, 246, 165, 148, 3, 50, 97, 80,		//80-95
									   187, 138, 217, 232, 127, 78, 29, 44, 2, 51, 96, 81, 198, 247, 164, 149,		//96-111
									   248, 201, 154, 171, 60, 13, 94, 111, 65, 112, 35, 18, 133, 180, 231, 214,	//112-127
									   122, 75, 24, 41, 190, 143, 220, 237, 195, 242, 161, 144, 7, 54, 101, 84,		//128-143
									   57, 8, 91, 106, 253, 204, 159, 174, 128, 177, 226, 211, 68, 117, 38, 23,		//144-159
									   252, 205, 158, 175, 56, 9, 90, 107, 69, 116, 39, 22, 129, 176, 227, 210,		//160-175
									   191, 142, 221, 236, 123, 74, 25, 40, 6, 55, 100, 85, 194, 243, 160, 145,		//176-191
									   71, 118, 37, 20, 131, 178, 225, 208, 254, 207, 156, 173, 58, 11, 88, 105,	//192-207
									   4, 53, 102, 87, 192, 241, 162, 147, 189, 140, 223, 238, 121, 72, 27, 42,		//208-223
									   193, 240, 163, 146, 5, 52, 103, 86, 120, 73, 26, 43, 188, 141, 222, 239,		//224-239
									   130, 179, 224, 209, 70, 119, 36, 21, 59, 10, 89, 104, 255, 206, 157, 172		//240-255
};


typedef enum {
	sht_TempCmd = 0x03,
	sht_HumidCmd = 0x05,
	sht_StatusRegCmd = 0x07
} ShtCmd_e;


static int reverse_bits(uint8_t v)
{
	uint8_t r = v; 					// r will be reversed bits of v; first get LSB of v
	uint8_t s = sizeof(v) * 8 - 1; 	// extra shift needed at end
	for (v >>= 1; v; v >>= 1) {
		r <<= 1;
		r |= v & 1;
		s--;
	}
	return r <<= s; // shift when v's highest bits are zero
}


static int crc_check(uint8_t *buf, ShtCmd_e cmd)
{
	uint8_t temp = 0;

	/* Для случая, если Status Register = 0x00. Если нет, то нужно изменить начальное значение
	 * temp = reverse_bit(Status Register);
	 * */
	temp ^= (uint8_t)cmd;
	temp = SENSOR_CRC_ARR[temp]^buf[0];

	switch (cmd) {
		case sht_StatusRegCmd:
			temp = reverse_bits(SENSOR_CRC_ARR[temp]);
			if (temp == buf[1]) return 0;
			//else return 1;
			break;
		case sht_TempCmd: case sht_HumidCmd:
			temp = SENSOR_CRC_ARR[temp]^buf[1];
			temp = reverse_bits(SENSOR_CRC_ARR[temp]);
			if (temp == buf[2]) return 0;
			break;
		default: return 1;
	}
	return 1;
}


static void shtdrv_rxIrqHandler(void)
{
	if ( TH_I2C_DMA->ISR & (TH_RX_DMA_FLAG_TC | TH_RX_DMA_FLAG_TE) ) {
		// STOP
		I2C_DMACmd(TH_I2C, DISABLE);
		I2C_GenerateSTOP(TH_I2C, ENABLE);
		DMA_Cmd(TH_I2C_DMA_CHANNEL_RX, DISABLE);
		DMA_ClearFlag(TH_RX_DMA_FLAG_TC);
	}
}


static inline void shtdrv_startTrx(void)
{
//	I2C_GenerateSTART(TH_I2C, ENABLE);
//	prg_delay_us(1000);
//	I2C_GenerateSTOP(TH_I2C, ENABLE);
//	prg_delay_us(1000);

	GPIO_InitTypeDef GPIO_InitStruct;

	HAL_GPIO_WritePin(TH_I2C_PORT, TH_I2C_SDA_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(TH_I2C_PORT, TH_I2C_SCL_PIN, GPIO_PIN_SET);
	GPIO_InitStruct.Pin = TH_I2C_SCL_PIN | TH_I2C_SDA_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(TH_I2C_PORT, &GPIO_InitStruct);

	prg_delay_us(100);
	HAL_GPIO_WritePin(TH_I2C_PORT, TH_I2C_SDA_PIN, GPIO_PIN_RESET);
	prg_delay_us(5);
	HAL_GPIO_WritePin(TH_I2C_PORT, TH_I2C_SCL_PIN, GPIO_PIN_RESET);
	prg_delay_us(20);
	HAL_GPIO_WritePin(TH_I2C_PORT, TH_I2C_SCL_PIN, GPIO_PIN_SET);
	prg_delay_us(5);
	HAL_GPIO_WritePin(TH_I2C_PORT, TH_I2C_SDA_PIN, GPIO_PIN_SET);
	prg_delay_us(100);

	GPIO_InitStruct.Pin = TH_I2C_SCL_PIN | TH_I2C_SDA_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(TH_I2C_PORT, &GPIO_InitStruct);
}


static void shtdrv_rxConfig(uint8_t *buf, uint32_t bufSize)
{
	DMA_InitTypeDef2 DMA_InitStructure;

	DMA_Cmd(TH_I2C_DMA_CHANNEL_RX, DISABLE);
	DMA_ClearFlag(RX_FLAGS);
	DMA_ITConfig(TH_I2C_DMA_CHANNEL_RX, DMA_IT_TC | DMA_IT_TE, ENABLE);

	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)TH_I2C_DR_Address;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)buf;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = bufSize;

	DMA_Init(TH_I2C_DMA_CHANNEL_RX, &DMA_InitStructure);

	I2C_DMACmd(TH_I2C, ENABLE);
	DMA_Cmd(TH_I2C_DMA_CHANNEL_RX, ENABLE);
}


static inline void shtdrv_initPeriph(ShtDrv_t *drv)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStruct;
	I2C_InitTypeDef2 I2C_InitStructure;

	TH_I2C_PORT_CLOCK_CMD(ENABLE);
	TH_I2C_CLOCK_CMD(ENABLE);
	TH_DMA_CLOCK_CMD(ENABLE);

	GPIO_InitStruct.Pin = TH_I2C_SCL_PIN | TH_I2C_SDA_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(TH_I2C_PORT, &GPIO_InitStruct);

	NVIC_InitStructure.NVIC_IRQChannel = TH_I2C_DMA_RX_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TH_I2C_DMA_PREPRIO;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = TH_I2C_DMA_SUBPRIO;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Disable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = TH_I2C_SPEED;
	I2C_Init(TH_I2C, &I2C_InitStructure, HAL_RCC_GetPCLK1Freq());

	(void)TH_I2C->SR1;
	(void)TH_I2C->SR2;
}


static inline void shtdrv_deinitPeriph(ShtDrv_t *drv)
{
	HAL_GPIO_DeInit(TH_I2C_PORT, TH_I2C_SCL_PIN | TH_I2C_SDA_PIN);
	DMA_Cmd(TH_I2C_DMA_CHANNEL_RX, DISABLE);
	DMA_DeInit(TH_I2C_DMA_CHANNEL_RX);
	I2C_Cmd(TH_I2C, DISABLE);
	I2C_DeInit(TH_I2C);
	TH_I2C_CLOCK_CMD(DISABLE);
}


static inline int shtdrv_readAutomat(ShtDrv_t *drv)
{
	uint32_t timeout = 0;
	ShtCmd_e cmd = (drv->currentSens == 0)? sht_TempCmd : sht_HumidCmd;

	switch (drv->rd.state) {
		case 0: {
			shtdrv_startTrx();

			I2C_AcknowledgeConfig(TH_I2C, DISABLE);
			I2C_GenerateSTART(TH_I2C, ENABLE); // i2c does not start without it

			timeout = 100;
			drv->rd.state = 1;
		} break;
		case 1: {
			if ( I2C_CheckEvent(TH_I2C, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS )
				return -1;

			I2C_Send7bitAddress(TH_I2C, cmd, I2C_Direction_Receiver);

			//timeout = (cmd == sht_TempCmd)? 320 : 80;
			timeout = 350;
			drv->rd.state = 2;
		} break;
		case 2: {
			if ( !(TH_I2C->SR1 & I2C_SR1_ADDR) ) {
				I2C_GenerateSTOP(TH_I2C, ENABLE);
				return -1;
			}

			I2C_AcknowledgeConfig(TH_I2C, ENABLE);

			(void)TH_I2C->SR2;

			shtdrv_rxConfig(drv->rd.buf, 3);

			timeout = 0xFFFFFFFF; // do not call from cycle, only from dma
			drv->rd.state = 3;
		} break;
		case 3: { // from dma
			shtdrv_rxIrqHandler();
			timeout = 0;
			drv->rd.state = 4;
		} break;
		case 4: {
			uint8_t *pd;
			int16_t *ps;
			uint32_t *pts, *ptse;
			int idx = (cmd == sht_TempCmd)? SHT_TEMP_EVENT : SHT_HUMID_EVENT;
			pd = &drv->data.s[idx].value.diag;
			if (crc_check(drv->rd.buf, cmd) == 0) {
				uint16_t value =
						(((uint16_t)drv->rd.buf[0]) << 8) +
						(((uint16_t)drv->rd.buf[1]) << 0);
				uint32_t prgts = prg_gettimestamp();
				//
				ps = &drv->data.s[idx].value.self;
				pts = &drv->data.s[idx].ts;
				ptse = &drv->data.ts.self[idx];
				//
				switch (cmd) {
					case sht_TempCmd: {
						value &= 0x3FFF; // 14 bit
						uint16_t dvalue = value/100; // d2
						if (value - (dvalue*100) > 15) { // correction
							dvalue++;
						}
						int16_t t = ((int16_t)dvalue) - 39; // d1
						__disable_irq();
						*pd = 0;
						*ps = t;
						*pts = prgts;
						if ((t > drv->prms.s[idx].thresh) && *ptse == 0) {
							*ptse = prgts;
							drv->base.eventIsReady = 1;
						}
						__enable_irq();
					} break;
					case sht_HumidCmd: {
						value &= 0x0FFF;

						float a, b, c, t1, t2;
						c = -2.0468;
						b = 0.0367;
						a = -1.5955E-6;
						t1 = 0.01;
						t2 = 0.00008;
						float fvalue = (float)value;
						fvalue = c + fvalue * b + fvalue * fvalue * a;
						if (fvalue < 0.0) fvalue = 0.4;
						if (drv->data.s[SHT_TEMP_EVENT].value.diag == 0) {
							float t = drv->data.s[SHT_TEMP_EVENT].value.self;
							fvalue = (t - 25.0) * (t1 + t2 * value) + fvalue;
						}
						int16_t ivalue = (int16_t)fvalue;

						__disable_irq();
						*pd = 0;
						*ps = ivalue;
						*pts = prgts;
						if ((ivalue > drv->prms.s[idx].thresh) && *ptse == 0) {
							*ptse = prgts;
							drv->base.eventIsReady = 1;
						}
						__enable_irq();
					} break;
					default: break;
				}

				return 1;
			} else {
				*pd = 1;
				return -1;
			}
		} break;
		default: break;
	}

	if (timeout != 0xFFFFFFFF)
		drv->base.callAfter(&drv->base, timeout);
	return 0;
}


#define DRV_RESOURCES (\
		RESMAN_SUP_3_3V \
		| RESMAN_DMA1 \
		| RESMAN_GPIOB)


void shtdrv_cycle(ShtDrv_t *drv, void *caller)
{
	switch (drv->base.state) {
		case 0: { // on startup
			drv->base.state = 1;
			drv->base.callAfter(&drv->base, 500);
		} break;
		case 1: {
			if (caller != DMA1) {
				if (drv->currentSens+1 > SHTDRV_CNT) {
					drv->currentSens = 0;
					shtdrv_deinitPeriph(drv);
					drv->base.resman.allbits = 0;
					drv->base.dataIsReady = 1;
					drv->base.state = 1;
					basedriver_setSupBusy(&drv->base, 0);
					drv->base.callAfter(&drv->base, drv->prms.period);
					break;
				}
				if (drv->prms.s[drv->currentSens].mode.status == SIGBITSET) {
					drv->rd.state = 0;
					if (drv->currentSens == 0) {
						if ( !basedriver_setSupBusy(&drv->base, 1) ) {
							drv->base.callAfter(&drv->base, (drv->prms.period >> 2));
							return;
						}
						drv->base.resman.allbits = DRV_RESOURCES;
						prg_enablesupply(RESMAN_SUP_3_3V);
						shtdrv_initPeriph(drv);
					}
					drv->base.state = 2;
					drv->base.callAfter(&drv->base, 500);
				} else {
					drv->currentSens++;
					drv->base.callAfter(&drv->base, 0);
				}
			}
		} break;
		case 2: {
			if ( shtdrv_readAutomat(drv) != 0 ) {
				drv->currentSens++;
				drv->base.state = 1;
				drv->base.callAfter(&drv->base, 0);
			}
		} break;
		default: break;
	}
}


void shtdrv_setParams(ShtDrv_t *drv, ShtDrvParams_t *params)
{
	__disable_irq();
	memcpy(&drv->prms, params, sizeof(ShtDrvParams_t));
	__enable_irq();
}


void shtdrv_getData(ShtDrv_t *drv, ShtDrvData_t *copyto, uint8_t forceCopy)
{
	if (forceCopy || drv->base.dataIsReady) {
		__disable_irq();
		memcpy(copyto, &drv->data, sizeof(ShtDrvData_t));
		drv->base.dataIsReady = 0;
		drv->base.eventIsReady = 0;
		__enable_irq();
	}
}


static void shtdrv_resetTs(ShtDrv_t *drv)
{
	__disable_irq();
	memset(&drv->data.ts, 0, sizeof(ShtDrvTimeStamp_t));
	__enable_irq();
}


void shtdrv_init(ShtDrv_t *drv, ShtDrvTimeStamp_t *bckp)
{
	basedriver_init(&drv->base);
	drv->base.cycle = (basedrv_cycle)shtdrv_cycle;
	drv->base.resetTs = (basedrv_resetTs)shtdrv_resetTs;

	memcpy(&drv->data.ts, bckp, sizeof(ShtDrvTimeStamp_t));
}


ShtDrv_t *shtdrv_create()
{
	ShtDrv_t *ret = (ShtDrv_t *)calloc(1, sizeof(ShtDrv_t));
	return ret;
}


void shtdrv_free(ShtDrv_t *drv)
{
	free(drv);
}
