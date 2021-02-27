#include "senst1wdrv.h"
#include "utils.h"

#define DSCMD_READROM	0x33
#define DSCMD_SKIPROM	0xcc
#define DSCMD_CONVT		0x44
#define DSCMD_READDATA	0xbe
#define DSCMD_POWERBIT	0xb4

/* Период сброса */
#define DSWTIMERES		600
/* Период обнаружения импульса */
#define DSWTIMEIMP		600
/* Время до чтения импульса */
#define DSWRESTIME_T1	80
/* Период тайм слота */
#define DSWTIMESLOT		80
/* Время удержания линии в 0, если запись 1 */
#define DSWWRTIME_T1	2
/* Время удержания линии в 1, завершение слота записи */
#define DSWWRTIME_T2	5
/* Время удержания линии в 0, начало слота чтения */
#define DSWRDTIME_T1	2
/* Время до чтения линии */
#define DSWRDTIME_T2	10
/* Время удержания линии в 1, завершение слота чтения */
#define DSWRDTIME_T3	5
/* Время конверсии измерительного канала */
#define DSWTIMEMEAS		750

#define ONEWIRE_D1_PORT		GPIOE
#define ONEWIRE_D1_PIN_H	GPIO_PIN_7
#define ONEWIRE_D1_PIN_R	GPIO_PIN_8
#define ONEWIRE_D1_PIN_L	GPIO_PIN_9
#define ONEWIRE_D1_CLOCK_CMD(en) { if (en) __HAL_RCC_GPIOE_CLK_ENABLE(); }

#define ONEWIRE_D2_PORT		GPIOE
#define ONEWIRE_D2_PIN_H	GPIO_PIN_10
#define ONEWIRE_D2_PIN_R	GPIO_PIN_11
#define ONEWIRE_D2_PIN_L	GPIO_PIN_12
#define ONEWIRE_D2_CLOCK_CMD(en) { if (en) __HAL_RCC_GPIOE_CLK_ENABLE(); }

/// Функция для установки линии передачи в 1
//\ param num - номер канала
static inline void senst1wdrv_setline(uint8_t num)
{
	switch (num) {
	case 0: ONEWIRE_D1_PORT->ODR &= ~ONEWIRE_D1_PIN_L; break;
	case 1: ONEWIRE_D2_PORT->ODR &= ~ONEWIRE_D2_PIN_L; break;
	}
}

/// Функция для установки линии передачи в 0
//\ param num - номер канала
static inline void senst1wdrv_clrline(uint8_t num)
{
	switch (num) {
	case 0: ONEWIRE_D1_PORT->ODR |= ONEWIRE_D1_PIN_L; break;
	case 1: ONEWIRE_D2_PORT->ODR |= ONEWIRE_D2_PIN_L; break;
	}
}

/// Функция для чтения линии приема
//\ param num - номер канала
static inline uint8_t senst1wdrv_getline(uint8_t num)
{
	uint32_t idr = 0;
	switch (num) {
	case 0: idr = (ONEWIRE_D1_PORT->IDR & ONEWIRE_D1_PIN_R)? 1:0; break;
	case 1: idr = (ONEWIRE_D2_PORT->IDR & ONEWIRE_D2_PIN_R)? 1:0; break;
	}
	return idr;
}


static inline void senst1wdrv_initPeriph(SensT1wDrv_t *drv)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	switch (drv->currentSens) {
	case 0:
		ONEWIRE_D1_CLOCK_CMD(ENABLE);
		GPIO_InitStruct.Pin = ONEWIRE_D1_PIN_L | ONEWIRE_D1_PIN_H;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(ONEWIRE_D1_PORT, &GPIO_InitStruct);
		GPIO_InitStruct.Pin = ONEWIRE_D1_PIN_R;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(ONEWIRE_D1_PORT, &GPIO_InitStruct);
		break;
	case 1:
		ONEWIRE_D2_CLOCK_CMD(ENABLE);
		GPIO_InitStruct.Pin = ONEWIRE_D2_PIN_L | ONEWIRE_D2_PIN_H;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(ONEWIRE_D2_PORT, &GPIO_InitStruct);
		GPIO_InitStruct.Pin = ONEWIRE_D2_PIN_R;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(ONEWIRE_D2_PORT, &GPIO_InitStruct);
		break;
	}
}


static inline void senst1wdrv_deinitPeriph(SensT1wDrv_t *drv)
{
	switch (drv->currentSens) {
	case 0:
		HAL_GPIO_DeInit(ONEWIRE_D1_PORT, ONEWIRE_D1_PIN_L | ONEWIRE_D1_PIN_H);
		HAL_GPIO_DeInit(ONEWIRE_D1_PORT, ONEWIRE_D1_PIN_R);
		break;
	case 1:
		HAL_GPIO_DeInit(ONEWIRE_D2_PORT, ONEWIRE_D2_PIN_L | ONEWIRE_D2_PIN_H);
		HAL_GPIO_DeInit(ONEWIRE_D2_PORT, ONEWIRE_D2_PIN_R);
		break;
	}
}

/// Функция сброса
static u8 senst1wdrv_reset(u8 num)
{
	u8 res;
	__disable_fault_irq();
	senst1wdrv_clrline(num);
	prg_delay_locked_us(DSWTIMERES);
	senst1wdrv_setline(num);
	prg_delay_locked_us(DSWRESTIME_T1);
	res=senst1wdrv_getline(num);
	prg_delay_locked_us(DSWTIMEIMP-DSWRESTIME_T1);
	__enable_fault_irq();
	return res;
}

/// Функция передачи бита
//\ param num - номер линии
//\ param bit - состояние бита
static void senst1wdrv_send_bit(u8 num, u8 bit)
{
	__disable_fault_irq();
	senst1wdrv_clrline(num);
	if (bit) prg_delay_locked_us(DSWWRTIME_T1);
	else prg_delay_locked_us(DSWTIMESLOT);
	senst1wdrv_setline(num);
	if (bit) prg_delay_locked_us(DSWTIMESLOT-DSWWRTIME_T1);
	__enable_fault_irq();
	prg_delay_locked_us(DSWWRTIME_T2);
}

/// Функция передачи команды
//\ param num - номер линии
//\ param cmd - команда
static void senst1wdrv_send(u8 num, u8 cmd)
{
	for (u8 i=0; i<8; i++) {
		senst1wdrv_send_bit(num, (mChkBit(cmd,i)? 1:0));
	}
}

/// Функция приема бита
//\ param num - номер линии
static u8 senst1wdrv_read_bit(u8 num)
{
	u8 rbit;
	__disable_fault_irq();
	senst1wdrv_clrline(num);
	prg_delay_locked_us(DSWRDTIME_T1);
	senst1wdrv_setline(num);
	prg_delay_locked_us(DSWRDTIME_T2);
	rbit=senst1wdrv_getline(num);
	prg_delay_locked_us(DSWTIMESLOT-(DSWRDTIME_T1+DSWRDTIME_T2));
	__enable_fault_irq();
	prg_delay_locked_us(DSWRDTIME_T3);
	return rbit;
}

/// Функция приема байта
//\ param num - номер линии
static u8 senst1wdrv_read(u8 num)
{
	u8 res;
	u8 rbit;
	for (u8 i=0; i<8; i++) {
		rbit=senst1wdrv_read_bit(num);
		if (rbit) mSetBit(res,i); else mClrBit(res,i);
	}
	return res;
}


static inline int senst1wdrv_readAutomat(SensT1wDrv_t *drv)
{
	uint8_t sens = drv->currentSens;
	uint8_t data[16];
	uint8_t crc = 0;
	uint8_t tst = 0;
	uint32_t timeout = 0;

	switch (drv->rd.state) {
		case 0: {
			// reset chip
			if ( senst1wdrv_reset(sens) != 0 ) {
				drv->data.s[sens].value.diag = 1;
				return -1;
			}

			// read chip info
			senst1wdrv_send(sens, DSCMD_READROM);
			for (uint8_t i = 0; i < 8; ++i) {
				data[i] = senst1wdrv_read(sens);
				crc = onewire_crc(crc, data[i]);
				if (data[i]) tst = 1;
			}

			if (crc || !tst) {
				drv->data.s[sens].value.diag = 1;
				return -1;
			}

			drv->data.s[sens].info.family = data[0];
			memcpy(&(drv->data.s[sens].info.serial), data+1, 6);

			drv->rd.state = 1;
		} break;
		case 1: {
			// senst1wdrv_send(sens, DSCMD_SKIPROM);
			senst1wdrv_send(sens, DSCMD_CONVT);

			// check start conv condition
			if ( senst1wdrv_read_bit(sens) != 0 ) {
				drv->data.s[sens].value.diag = 1;
				return -1;
			}

			timeout = DSWTIMEMEAS*2;
			drv->rd.state = 2;
		} break;
		case 2: {
			// reset chip
			if ( senst1wdrv_reset(sens) != 0 ) {
				drv->data.s[sens].value.diag = 1;
				return -1;
			}

			// read temperature
			senst1wdrv_send(sens, DSCMD_SKIPROM);
			senst1wdrv_send(sens, DSCMD_READDATA);
			for (uint8_t i = 0; i < 9; ++i) {
				data[i] = senst1wdrv_read(sens);
				crc = onewire_crc(crc, data[i]);
				if (data[i]) tst = 1;
			}

			if (crc || !tst) {
				drv->data.s[sens].value.diag = 1;
				return -1;
			}

			uint16_t d = mU16(data);
			if (d != 0x550) { // default value (+85C)
				uint8_t *pd;
				int16_t *ps;
				uint32_t *pts, *ptse;
				uint32_t prgts = prg_gettimestamp();
				union {
					uint16_t u; int16_t s;
				} val;
				val.u = d;
				if (val.s < 0) {
					val.u = ((uint16_t)(~val.u)) >> 4;
					val.u = ((uint16_t)(~val.u));
				} else {
					val.u = ((uint16_t)(val.u)) >> 4;
				}
				pd = &drv->data.s[sens].value.diag;
				ps = &drv->data.s[sens].value.self;
				pts = &drv->data.s[sens].ts;
				ptse = &drv->data.ts.self[sens];
				__disable_irq();
				*pd = 0;
				*ps = val.s;
				*pts = prgts;
				if (val.s > drv->prms.s[sens].thresh && *ptse == 0) {
					*ptse = prgts;
					drv->base.eventIsReady = 1;
				}
				__enable_irq();
			} else {
				drv->data.s[sens].value.diag = 1;
			}

			return 1;
		} break;
		default: break;
	}

	drv->base.callAfter(&drv->base, timeout);
	return 0;
}


#define DRV_RESOURCES (\
		RESMAN_SUP_5V \
		| RESMAN_SUP_5V_P \
		| RESMAN_GPIOE)


void senst1wdrv_cycle(SensT1wDrv_t *drv, void *caller)
{
	switch (drv->base.state) {
		case 0: { // on startup
			drv->base.state = 1;
			drv->base.callAfter(&drv->base, 500);
		} break;
		case 1: {
			if (drv->currentSens+1 > SENST1WDRV_CNT) {
				drv->currentSens = 0;
				drv->base.resman.allbits = 0;
				drv->base.dataIsReady = 1;
				basedriver_setSupBusy(&drv->base, 0);
				drv->base.callAfter(&drv->base, drv->prms.period);
				break;
			}
			if (drv->prms.s[drv->currentSens].mode.status == SIGBITSET) {
				if (drv->currentSens == 0) {
					if ( !basedriver_setSupBusy(&drv->base, 1) ) {
						drv->base.callAfter(&drv->base, (drv->prms.period >> 2));
						return;
					}
				}
				drv->rd.state = 0;
				drv->base.resman.allbits = DRV_RESOURCES;
				prg_enablesupply(RESMAN_SUP_5V | RESMAN_SUP_5V_P);
				senst1wdrv_initPeriph(drv);
				drv->base.state = 2;
				drv->base.callAfter(&drv->base, 100);
			} else {
				drv->currentSens++;
				drv->base.callAfter(&drv->base, 0);
			}
		} break;
		case 2: {
			if (senst1wdrv_readAutomat(drv) != 0 ) {
				drv->base.state = 3;
				drv->base.callAfter(&drv->base, 0);
			}
		} break;
		case 3: {
			senst1wdrv_deinitPeriph(drv);
			drv->currentSens++;
			drv->base.state = 1;
			drv->base.callAfter(&drv->base, 0);
		} break;
		default: break;
	}
}


void senst1wdrv_setParams(SensT1wDrv_t *drv, SensT1wDrvParams_t *params)
{
	__disable_irq();
	memcpy(&drv->prms, params, sizeof(SensT1wDrvParams_t));
	__enable_irq();
}


void senst1wdrv_getData(SensT1wDrv_t *drv, SensT1wDrvData_t *copyto, uint8_t forceCopy)
{
	if (forceCopy || drv->base.dataIsReady) {
		__disable_irq();
		memcpy(copyto, &drv->data, sizeof(SensT1wDrvData_t));
		drv->base.dataIsReady = 0;
		drv->base.eventIsReady = 0;
		__enable_irq();
	}
}


static void senst1wdrv_resetTs(SensT1wDrv_t *drv)
{
	__disable_irq();
	memset(&drv->data.ts, 0, sizeof(SensT1wDrvTimeStamp_t));
	__enable_irq();
}


void senst1wdrv_init(SensT1wDrv_t *drv, SensT1wDrvTimeStamp_t *bckp)
{
	basedriver_init(&drv->base);
	drv->base.cycle = (basedrv_cycle)senst1wdrv_cycle;
	drv->base.resetTs = (basedrv_resetTs)senst1wdrv_resetTs;

	memcpy(&drv->data.ts, bckp, sizeof(SensT1wDrvTimeStamp_t));
}


SensT1wDrv_t *senst1wdrv_create()
{
	SensT1wDrv_t *ret = (SensT1wDrv_t *)calloc(1, sizeof(SensT1wDrv_t));
	return ret;
}


void senst1wdrv_free(SensT1wDrv_t *drv)
{
	free(drv);
}
