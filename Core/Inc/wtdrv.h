#ifndef WTDRV_H
#define WTDRV_H

#if defined(__cplusplus)
#else
#endif

#include "basedriver.h"

#pragma pack (push,1)

typedef struct WtDrvParams_s WtDrvParams_t;
struct WtDrvParams_s {
	uint16_t period;
};

typedef struct WtDrvTimeStamp_s WtDrvTimeStamp_t;
struct WtDrvTimeStamp_s {
	uint32_t self;
};

#pragma pack (pop)

typedef struct WtDrvData_s WtDrvData_t;
struct WtDrvData_s {
	WtDrvTimeStamp_t ts;
};

typedef struct WtDrv_s WtDrv_t;
struct WtDrv_s {
	BaseDriver_t base;
	WtDrvParams_t prms;
	WtDrvData_t data;

	uint16_t counter;
};

EXTERN_C void wtdrv_setParams(WtDrv_t *drv, WtDrvParams_t *params);
EXTERN_C void wtdrv_getData(WtDrv_t *drv, WtDrvData_t *copyto, uint8_t forceCopy);

EXTERN_C void wtdrv_init(WtDrv_t *drv, WtDrvTimeStamp_t *bckp);
EXTERN_C void wtdrv_cycle(WtDrv_t *drv, void *caller);

EXTERN_C WtDrv_t *wtdrv_create();
EXTERN_C void wtdrv_free(WtDrv_t *drv);

#endif // WTDRV_H