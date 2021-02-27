#ifndef ADCDRV_H
#define ADCDRV_H

#if defined(__cplusplus)
#else
#endif

#include "basedriver.h"

#define ADCDRV_EVENTS_CNT 3
#define ADCDRV_MES_CNT 5

enum {
	ADCDRV_BAT_EVENT,
	ADCDRV_SHOCK_EVENT,
	ADCDRV_5V_EVENT,
};

#pragma pack (push,1)

typedef struct AdcDrvParams_s AdcDrvParams_t;
struct AdcDrvParams_s {
	uint16_t period;
	struct {
		SignalMode_t mode;
		uint16_t thresh;
	} s[ADCDRV_EVENTS_CNT];
};

typedef struct AdcDrvTimeStamp_s AdcDrvTimeStamp_t;
struct AdcDrvTimeStamp_s {
	uint32_t self[ADCDRV_EVENTS_CNT];
};

#pragma pack (pop)

typedef struct AdcDrvData_s AdcDrvData_t;
struct AdcDrvData_s {
	AdcDrvTimeStamp_t ts;
	struct {
		uint16_t val;
		uint32_t ts;
	} s[ADCDRV_EVENTS_CNT];
	uint8_t sup3_3 : 1;
	uint8_t sup5 : 1;
	uint16_t accel_raw[3];
};

typedef struct AdcDrv_s AdcDrv_t;
struct AdcDrv_s {
	BaseDriver_t base;
	AdcDrvParams_t prms;
	AdcDrvData_t data;

	uint16_t adc[ADCDRV_MES_CNT];
	uint16_t thresh[ADCDRV_EVENTS_CNT];
};

EXTERN_C void adcdrv_setParams(AdcDrv_t *drv, AdcDrvParams_t *params);
EXTERN_C void adcdrv_getData(AdcDrv_t *drv, AdcDrvData_t *copyto, uint8_t forceCopy);

EXTERN_C void adcdrv_init(AdcDrv_t *drv, AdcDrvTimeStamp_t *bckp);
EXTERN_C void adcdrv_cycle(AdcDrv_t *drv, void *caller);

EXTERN_C AdcDrv_t *adcdrv_create();
EXTERN_C void adcdrv_free(AdcDrv_t *drv);

#endif // ADCDRV_H
