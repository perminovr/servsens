#ifndef SENST1WDRV_H
#define SENST1WDRV_H

#if defined(__cplusplus)
#else
#endif

#include "basedriver.h"

#define SENST1WDRV_CNT	2

#pragma pack (push,1)

typedef struct SensT1wDrvParams_s SensT1wDrvParams_t;
struct SensT1wDrvParams_s {
	uint16_t period;
	struct {
		SignalMode_t mode;
		uint8_t thresh;
	} s[SENST1WDRV_CNT];
};

typedef struct SensT1wDrvTimeStamp_s SensT1wDrvTimeStamp_t;
struct SensT1wDrvTimeStamp_s {
	uint32_t self[SENST1WDRV_CNT];
};

#pragma pack (pop)

typedef struct SensT1wDrvData_s SensT1wDrvData_t;
struct SensT1wDrvData_s {
	SensT1wDrvTimeStamp_t ts;
	struct {
		uint32_t ts;
		struct {
			int16_t self;
			uint8_t diag;
		} value;
		struct {
			uint8_t family;
			uint64_t serial;
		} info;
	} s[SENST1WDRV_CNT];
};

typedef struct SensT1wDrv_s SensT1wDrv_t;
struct SensT1wDrv_s {
	BaseDriver_t base;
	SensT1wDrvParams_t prms;
	SensT1wDrvData_t data;

	uint8_t currentSens;
	struct {
		uint8_t state;
	} rd;
};

EXTERN_C void senst1wdrv_setParams(SensT1wDrv_t *drv, SensT1wDrvParams_t *params);
EXTERN_C void senst1wdrv_getData(SensT1wDrv_t *drv, SensT1wDrvData_t *copyto, uint8_t forceCopy);

EXTERN_C void senst1wdrv_init(SensT1wDrv_t *drv, SensT1wDrvTimeStamp_t *bckp);
EXTERN_C void senst1wdrv_cycle(SensT1wDrv_t *drv, void *caller);

EXTERN_C SensT1wDrv_t *senst1wdrv_create();
EXTERN_C void senst1wdrv_free(SensT1wDrv_t *drv);

#endif // SENST1WDRV_H