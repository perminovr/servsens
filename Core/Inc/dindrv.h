#ifndef DINDRV_H
#define DINDRV_H

#if defined(__cplusplus)
#else
#endif

#include "basedriver.h"

#define DINDRV_EVENT_LVL				1
#define DINDRV_EVENTS_CNT				7
#define DINDRV_DELAY_BEFORE_READ_US		10000

#pragma pack (push,1)

typedef struct DinDrvParams_s DinDrvParams_t;
struct DinDrvParams_s {
	uint16_t period;
	struct {
		SignalMode_t mode;
	} s[DINDRV_EVENTS_CNT];
};

typedef struct DinDrvTimeStamp_s DinDrvTimeStamp_t;
struct DinDrvTimeStamp_s {
	uint32_t self[DINDRV_EVENTS_CNT];
};

#pragma pack (pop)

typedef struct DinDrvData_s DinDrvData_t;
struct DinDrvData_s {
	DinDrvTimeStamp_t ts;
	struct {
		uint8_t state;
	} s[DINDRV_EVENTS_CNT];
};

typedef struct DinDrv_s DinDrv_t;
struct DinDrv_s {
	BaseDriver_t base;
	DinDrvParams_t prms;
	DinDrvData_t data;

	uint8_t currentPin;
	uint8_t evFlag;
	uint16_t pins_ctl[DINDRV_EVENTS_CNT];
	uint16_t pins_in[DINDRV_EVENTS_CNT];
};

EXTERN_C void dindrv_setParams(DinDrv_t *drv, DinDrvParams_t *params);
EXTERN_C void dindrv_getData(DinDrv_t *drv, DinDrvData_t *copyto, uint8_t forceCopy);

EXTERN_C void dindrv_init(DinDrv_t *drv, DinDrvTimeStamp_t *bckp);
EXTERN_C void dindrv_cycle(DinDrv_t *drv, void *caller);

EXTERN_C DinDrv_t *dindrv_create();
EXTERN_C void dindrv_free(DinDrv_t *drv);

#endif // DINDRV_H
