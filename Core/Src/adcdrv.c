#include "adcdrv.h"


extern TIM_HandleTypeDef htim1;
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern TIM_HandleTypeDef htim1;


#define ADCDRV_ADC					ADC1
#define ADCDRV_DMA					DMA1
#define ADCDRV_TIM					TIM1
#define ADCDRV_SAMPLING_TIME		ADC_SAMPLETIME_71CYCLES_5 //ADC_SAMPLETIME_7CYCLES_5 // todo
#define ADCDRV_PORT					GPIOA
#define ADCDRV_PINS					(GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5)
#define ADCDRV_ADC_CLOCK_CMD(en)	{ if (en) __HAL_RCC_ADC1_CLK_ENABLE(); else __HAL_RCC_ADC1_CLK_DISABLE(); }
#define ADCDRV_DMA_CLOCK_CMD(en)	{ if (en) __HAL_RCC_DMA1_CLK_ENABLE(); }
#define ADCDRV_PORT_CLOCK_CMD(en)	{ if (en) __HAL_RCC_GPIOB_CLK_ENABLE(); }
#define ADCDRV_TIM_CLOCK_CMD(en)	{ if (en) __HAL_RCC_TIM1_CLK_ENABLE(); else __HAL_RCC_TIM1_CLK_DISABLE(); }
#define ADCDRV_TIM_CMD(en)			{ if (en) HAL_TIM_Base_Start_IT(&htim1); else HAL_TIM_Base_Stop_IT(&htim1); }


static void adcdrv_initPeriph(AdcDrv_t *drv)
{
	uint32_t conv;
	ADC_ChannelConfTypeDef sConfig = {0};
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	ADCDRV_ADC_CLOCK_CMD(ENABLE);
	ADCDRV_DMA_CLOCK_CMD(ENABLE);
	ADCDRV_PORT_CLOCK_CMD(ENABLE);

	GPIO_InitStruct.Pin = ADCDRV_PINS;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	HAL_GPIO_Init(ADCDRV_PORT, &GPIO_InitStruct);

	conv = 0;
	if (drv->prms.s[ADCDRV_BAT_EVENT].mode.status) {
		conv++;
	}
	if (drv->prms.s[ADCDRV_SHOCK_EVENT].mode.status) {
		conv += 3;
	}
	if (drv->prms.s[ADCDRV_5V_EVENT].mode.status) {
		conv++;
	}

	hadc1.Instance = ADCDRV_ADC;
	hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = conv;
	HAL_ADC_Init(&hadc1);

	sConfig.SamplingTime = ADCDRV_SAMPLING_TIME;

	uint32_t ch[] = {
		ADC_CHANNEL_1, ADC_CHANNEL_2, ADC_CHANNEL_3,
		ADC_CHANNEL_4, ADC_CHANNEL_5
	};
	conv = 0;
	if (drv->prms.s[ADCDRV_BAT_EVENT].mode.status) {
		sConfig.Channel = ch[conv];
		sConfig.Rank = conv+1;
		HAL_ADC_ConfigChannel(&hadc1, &sConfig);
		conv++;
	}
	if (drv->prms.s[ADCDRV_SHOCK_EVENT].mode.status) {
		sConfig.Channel = ch[conv];
		sConfig.Rank = conv+1;
		HAL_ADC_ConfigChannel(&hadc1, &sConfig);
		conv++;
		sConfig.Channel = ch[conv];
		sConfig.Rank = conv+1;
		HAL_ADC_ConfigChannel(&hadc1, &sConfig);
		conv++;
		sConfig.Channel = ch[conv];
		sConfig.Rank = conv+1;
		HAL_ADC_ConfigChannel(&hadc1, &sConfig);
		conv++;
	}
	if (drv->prms.s[ADCDRV_5V_EVENT].mode.status) {
		sConfig.Channel = ch[conv];
		sConfig.Rank = conv+1;
		HAL_ADC_ConfigChannel(&hadc1, &sConfig);
		conv++;
	}

	hdma_adc1.Instance = DMA1_Channel1;
	hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
	hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
	hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
	hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
	hdma_adc1.Init.Mode = DMA_NORMAL;
	hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
	HAL_DMA_Init(&hdma_adc1);

	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&drv->adc, conv);
}


static void adcdrv_deinitPeriph(AdcDrv_t *drv)
{
	HAL_DMA_DeInit(&hdma_adc1);
	HAL_ADC_DeInit(&hadc1);
	HAL_GPIO_DeInit(ADCDRV_PORT, ADCDRV_PINS);
	ADCDRV_ADC_CLOCK_CMD(DISABLE);
}


static uint8_t adcdrv_enabled(AdcDrv_t *drv)
{
	uint8_t ret = 0;
	for (int i = 0; i < ADCDRV_EVENTS_CNT; ++i) {
		ret |= drv->prms.s[i].mode.status;
	}
	return (ret)? ENABLE : DISABLE;
}


static inline void adcdrv_timCtl(uint8_t en, uint16_t period)
{
	if (en) {
		ADCDRV_TIM_CLOCK_CMD(en);
		ADCDRV_TIM->CR1 &= ~(TIM_CR1_CEN);
		ADCDRV_TIM->ARR = period;
		ADCDRV_TIM->CR1 |= TIM_CR1_CEN;
		ADCDRV_TIM_CMD(en);
	}
	else { ADCDRV_TIM_CMD(en); ADCDRV_TIM_CLOCK_CMD(en); }
}


static inline uint32_t getLastBitPos(uint32_t Y)
{
	uint32_t res=0;
	Y = Y>>1;
	while (Y > 0) {
		Y = Y>>1;
		res++;
	}
	return res;
}


static inline uint32_t sqrtint(uint32_t Y)
{
	uint32_t N = getLastBitPos(Y);
	uint32_t X = 1 << (N >> 1);
	uint32_t bit;
	if (N == 0) return Y;
	for (bit = X >> 1; bit > 0; bit >>= 1) {
		X |= bit;
		if (X * X > Y) X &= ~bit;
	}
	return X;
}


static uint32_t getVecLen(uint16_t X, uint16_t Y, uint16_t Z)
{
	int sx, sy, sz;
	sx = (int)X-(1<<11);
	sy = (int)Y-(1<<11);
	sz = (int)Z-(1<<11);
	uint32_t sum = sx*sx + sy*sy + sz*sz;
	const float norm = 1.709; // 2,5v
	return (uint32_t)(((float)sqrtint(sum))*norm);
}


void adcdrv_cycle(AdcDrv_t *drv, void *caller)
{
	switch (drv->base.state) {
		case 0: { // on startup
			adcdrv_timCtl(ENABLE, drv->prms.period);
			drv->base.state = 1;
		} break;
		case 1: { // from timer interruption
			if (caller == ADCDRV_TIM) {
				drv->base.state = 2;
				drv->base.callAfter(dToB(drv), 0);
			}
		} break;
		case 2: {
			if (caller != ADCDRV_TIM) {
				if (adcdrv_enabled(drv) == ENABLE) {
					prg_enablesupply(RESMAN_SUP_5V);
					adcdrv_initPeriph(drv);
					// drv->base.busy = 1;
					drv->base.state = 3;
				} else {
					adcdrv_timCtl(DISABLE, 0);
					drv->base.state = 1;
				}
			} else {
				drv->base.callAfter(dToB(drv), 0);
			}
		} break;
		case 3: { // from adc dma interruption
			if (caller == ADCDRV_ADC) {
				drv->base.state = 4;
				drv->base.callAfter(dToB(drv), 0);
			}
		} break;
		case 4: {
			if (caller != ADCDRV_TIM) {
				uint32_t prgts = prg_gettimestamp();
				uint16_t *r = drv->adc;
				uint16_t *ps;
				uint32_t *pts, *ptse;
				uint32_t conv = 0;
				uint16_t vecLen = 1000;
				if (drv->prms.s[ADCDRV_SHOCK_EVENT].mode.status) {
					uint32_t cconv = conv;
					if (drv->prms.s[ADCDRV_BAT_EVENT].mode.status) {
						cconv++;
					}
					vecLen = getVecLen(r[cconv+0], r[cconv+1], r[cconv+2]);
				}
				__disable_irq();
				if (drv->prms.s[ADCDRV_BAT_EVENT].mode.status) {
					ps = &drv->data.s[ADCDRV_BAT_EVENT].val;
					pts = &drv->data.s[ADCDRV_BAT_EVENT].ts;
					ptse = &drv->data.ts.self[ADCDRV_BAT_EVENT];
					*ps = r[conv];
					*pts = prgts;
					if (r[conv] < drv->thresh[ADCDRV_BAT_EVENT]) {
						drv->data.sup3_3 = 0;
						if (*ptse == 0) {
							*ptse = prgts;
							drv->base.eventIsReady = 1;
						}
					} else {
						drv->data.sup3_3 = 1;
					}
					conv++;
				}
				if (drv->prms.s[ADCDRV_SHOCK_EVENT].mode.status) {
					ps = &drv->data.s[ADCDRV_SHOCK_EVENT].val;
					pts = &drv->data.s[ADCDRV_SHOCK_EVENT].ts;
					ptse = &drv->data.ts.self[ADCDRV_SHOCK_EVENT];
					memcpy(drv->data.accel_raw, &r[conv], 6);
					*ps = vecLen;
					*pts = prgts;
					if (vecLen > drv->thresh[ADCDRV_SHOCK_EVENT]) {
						if (*ptse == 0) {
							*ptse = prgts;
							drv->base.eventIsReady = 1;
						}
					}
					conv += 3;
				}
				if (drv->prms.s[ADCDRV_5V_EVENT].mode.status) {
					ps = &drv->data.s[ADCDRV_5V_EVENT].val;
					pts = &drv->data.s[ADCDRV_5V_EVENT].ts;
					ptse = &drv->data.ts.self[ADCDRV_5V_EVENT];
					*ps = r[conv];
					*pts = prgts;
					if (r[conv] < drv->thresh[ADCDRV_5V_EVENT]) {
						drv->data.sup5 = 0;
						if (*ptse == 0) {
							*ptse = prgts;
							drv->base.eventIsReady = 1;
						}
					} else {
						drv->data.sup5 = 1;
					}
					conv++;
				}
				drv->base.dataIsReady = 1;
				__enable_irq();
				adcdrv_deinitPeriph(drv);
				// drv->base.busy = 0;
				drv->base.state = 1;
			} else {
				drv->base.callAfter(dToB(drv), 0);
			}
		} break;
		default: break;
	}
}


#define multk(v,k) (uint16_t)(((float)(v))*(k))
#define mk(v,max) ((float)(v))/((float)(max))


void adcdrv_setParams(AdcDrv_t *drv, AdcDrvParams_t *params)
{
	__disable_irq();
	memcpy(&drv->prms, params, sizeof(AdcDrvParams_t));
	drv->thresh[ADCDRV_BAT_EVENT] = multk(3953, mk(params->s[ADCDRV_BAT_EVENT].thresh, 3300));
	drv->thresh[ADCDRV_SHOCK_EVENT] = params->s[ADCDRV_SHOCK_EVENT].thresh;
	drv->thresh[ADCDRV_5V_EVENT] = multk(4095, mk(params->s[ADCDRV_5V_EVENT].thresh, 5000));
	adcdrv_timCtl( adcdrv_enabled(drv), params->period );
	__enable_irq();
}


void adcdrv_getData(AdcDrv_t *drv, AdcDrvData_t *copyto, uint8_t forceCopy)
{
	if (forceCopy || drv->base.dataIsReady) {
		__disable_irq();
		memcpy(copyto, &drv->data, sizeof(AdcDrvData_t));
		copyto->s[ADCDRV_BAT_EVENT].val = multk(3300, mk(copyto->s[ADCDRV_BAT_EVENT].val, 3953));
		copyto->s[ADCDRV_5V_EVENT].val = multk(5000, mk(copyto->s[ADCDRV_5V_EVENT].val, 4095));
		drv->base.dataIsReady = 0;
		drv->base.eventIsReady = 0;
		__enable_irq();
	}
}


static void adcdrv_resetTs(AdcDrv_t *drv)
{
	__disable_irq();
	memset(&drv->data.ts, 0, sizeof(AdcDrvTimeStamp_t));
	__enable_irq();
}


void adcdrv_init(AdcDrv_t *drv, AdcDrvTimeStamp_t *bckp)
{
	basedriver_init(&drv->base);
	drv->base.cycle = (basedrv_cycle)adcdrv_cycle;
	drv->base.resetTs = (basedrv_resetTs)adcdrv_resetTs;

	memcpy(&drv->data.ts, bckp, sizeof(AdcDrvTimeStamp_t));
}


AdcDrv_t *adcdrv_create()
{
	AdcDrv_t *ret = (AdcDrv_t *)calloc(1, sizeof(AdcDrv_t));
	return ret;
}


void adcdrv_free(AdcDrv_t *drv)
{
	free(drv);
}
