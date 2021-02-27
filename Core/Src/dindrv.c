#include "dindrv.h"


#define DINDRV_CTL_PORT		GPIOE
#define DINDRV_CTL_D1_PIN	GPIO_PIN_0
#define DINDRV_CTL_D2_PIN	GPIO_PIN_1
#define DINDRV_CTL_D3_PIN	GPIO_PIN_2
#define DINDRV_CTL_D4_PIN	GPIO_PIN_3
#define DINDRV_CTL_D5_PIN	GPIO_PIN_4
#define DINDRV_CTL_D6_PIN	GPIO_PIN_5
#define DINDRV_CTL_D7_PIN	GPIO_PIN_6
#define DINDRV_CTL_CLOCK_CMD(en)	if (en) __HAL_RCC_GPIOE_CLK_ENABLE();

#define DINDRV_IN_PORT		GPIOC
#define DINDRV_IN_D1_PIN	GPIO_PIN_0
#define DINDRV_IN_D2_PIN	GPIO_PIN_1
#define DINDRV_IN_D3_PIN	GPIO_PIN_2
#define DINDRV_IN_D4_PIN	GPIO_PIN_3
#define DINDRV_IN_D5_PIN	GPIO_PIN_4
#define DINDRV_IN_D6_PIN	GPIO_PIN_5
#define DINDRV_IN_D7_PIN	GPIO_PIN_6
#define DINDRV_IN_CLOCK_CMD(en)		if (en) __HAL_RCC_GPIOC_CLK_ENABLE();


static inline void dindrv_initPeriph(DinDrv_t *drv)
{
	GPIO_InitTypeDef init_ctl = {0};
	GPIO_InitTypeDef init_in = {0};

	init_ctl.Mode = GPIO_MODE_OUTPUT_PP;
	init_ctl.Pull = GPIO_PULLUP;
	init_ctl.Speed = GPIO_SPEED_FREQ_LOW;
	init_ctl.Pin = drv->pins_ctl[ drv->currentPin ];

	init_in.Mode = GPIO_MODE_INPUT;
	init_in.Pull = GPIO_NOPULL;
	init_in.Speed = GPIO_SPEED_FREQ_LOW;
	init_in.Pin = drv->pins_in[ drv->currentPin ];

	DINDRV_CTL_CLOCK_CMD(ENABLE);
	DINDRV_IN_CLOCK_CMD(ENABLE);

	HAL_GPIO_Init(DINDRV_CTL_PORT, &init_ctl);
	HAL_GPIO_Init(DINDRV_IN_PORT, &init_in);

	HAL_GPIO_WritePin(DINDRV_CTL_PORT, init_ctl.Pin, GPIO_PIN_RESET); // enable
}


static inline void dindrv_readData(DinDrv_t *drv)
{
	uint8_t *ps = &drv->data.s[ drv->currentPin ].state;
	uint32_t *ptse = &drv->data.ts.self[ drv->currentPin ];
	uint8_t state = (uint8_t)HAL_GPIO_ReadPin(DINDRV_IN_PORT, drv->pins_in[ drv->currentPin ]);
	uint32_t prgts = prg_gettimestamp();
	__disable_irq();
	*ps = state;
	if (state == DINDRV_EVENT_LVL) {
		if (*ptse == 0) {
			*ptse = prgts;
			drv->evFlag++;
		}
	}
	__enable_irq();
}


static inline void dindrv_deinitPeriph(DinDrv_t *drv)
{
	HAL_GPIO_WritePin(DINDRV_CTL_PORT, drv->pins_ctl[ drv->currentPin ], GPIO_PIN_SET);
	HAL_GPIO_DeInit(DINDRV_CTL_PORT, drv->pins_ctl[ drv->currentPin ]);
	HAL_GPIO_DeInit(DINDRV_IN_PORT, drv->pins_in[ drv->currentPin ]);
}


#define DRV_RESOURCES (\
		RESMAN_SUP_5V \
		| RESMAN_GPIOC \
		| RESMAN_GPIOE)


void dindrv_cycle(DinDrv_t *drv, void *caller)
{
	switch (drv->base.state) {
		case 0: { // on startup
			drv->base.state = 1;
			drv->base.callAfter(&drv->base, 100);
		} break;
		case 1: {
			if ( !basedriver_setSupBusy(&drv->base, 1) ) {
				drv->base.callAfter(&drv->base, (drv->prms.period >> 1));
				return;
			}
			drv->evFlag = 0;
			drv->currentPin = 0;
			prg_enablesupply(RESMAN_SUP_5V);
			drv->base.resman.allbits = DRV_RESOURCES;
			drv->base.state = 2;
			drv->base.callAfter(&drv->base, 0);
		} break;
		case 2: {
			if (drv->currentPin+1 > DINDRV_EVENTS_CNT) {
				if (drv->evFlag) {
					drv->base.eventIsReady = 1;
				}
				drv->evFlag = 0;
				drv->currentPin = 0;
				drv->base.resman.allbits = 0;
				drv->base.dataIsReady = 1;
				drv->base.state = 1;
				basedriver_setSupBusy(&drv->base, 0);
				drv->base.callAfter(&drv->base, drv->prms.period);
				break;
			}
			if (drv->prms.s[drv->currentPin].mode.status == SIGBITSET) {
				dindrv_initPeriph(drv);
				drv->base.state = 3;
				drv->base.callAfter(&drv->base, 10);
			} else {
				drv->currentPin++;
				drv->base.callAfter(&drv->base, 0);
			}
		} break;
		case 3: {
			dindrv_readData(drv);
			dindrv_deinitPeriph(drv);
			drv->currentPin++;
			drv->base.state = 2;
			drv->base.callAfter(&drv->base, 0);
		} break;
		default: break;
	}
}


void dindrv_setParams(DinDrv_t *drv, DinDrvParams_t *params)
{
	__disable_irq();
	memcpy(&drv->prms, params, sizeof(DinDrvParams_t));
	__enable_irq();
}


void dindrv_getData(DinDrv_t *drv, DinDrvData_t *copyto, uint8_t forceCopy)
{
	if (forceCopy || drv->base.dataIsReady) {
		__disable_irq();
		memcpy(copyto, &drv->data, sizeof(DinDrvData_t));
		drv->base.dataIsReady = 0;
		drv->base.eventIsReady = 0;
		__enable_irq();
	}
}


static void dindrv_resetTs(DinDrv_t *drv)
{
	__disable_irq();
	memset(&drv->data.ts, 0, sizeof(DinDrvTimeStamp_t));
	__enable_irq();
}


void dindrv_init(DinDrv_t *drv, DinDrvTimeStamp_t *bckp)
{
	basedriver_init(&drv->base);
	drv->base.cycle = (basedrv_cycle)dindrv_cycle;
	drv->base.resetTs = (basedrv_resetTs)dindrv_resetTs;

	for (int i = 0; i < DINDRV_EVENTS_CNT; ++i) {
		uint16_t pin_in, pin_ctl;
		switch (i) {
			case 0: pin_ctl = DINDRV_CTL_D1_PIN; pin_in = DINDRV_IN_D1_PIN; break;
			case 1: pin_ctl = DINDRV_CTL_D2_PIN; pin_in = DINDRV_IN_D2_PIN; break;
			case 2: pin_ctl = DINDRV_CTL_D3_PIN; pin_in = DINDRV_IN_D3_PIN; break;
			case 3: pin_ctl = DINDRV_CTL_D4_PIN; pin_in = DINDRV_IN_D4_PIN; break;
			case 4: pin_ctl = DINDRV_CTL_D5_PIN; pin_in = DINDRV_IN_D5_PIN; break;
			case 5: pin_ctl = DINDRV_CTL_D6_PIN; pin_in = DINDRV_IN_D6_PIN; break;
			case 6: pin_ctl = DINDRV_CTL_D7_PIN; pin_in = DINDRV_IN_D7_PIN; break;
			default: break;
		}
		drv->pins_ctl[i] = pin_ctl;
		drv->pins_in[i] = pin_in;
	}

	memcpy(&drv->data.ts, bckp, sizeof(DinDrvTimeStamp_t));
}


DinDrv_t *dindrv_create()
{
	DinDrv_t *ret = (DinDrv_t *)calloc(1, sizeof(DinDrv_t));
	return ret;
}


void dindrv_free(DinDrv_t *drv)
{
	free(drv);
}
