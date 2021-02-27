#ifndef SHTDRV_H
#define SHTDRV_H

#if defined(__cplusplus)
#else
#endif

#include "basedriver.h"

#define SHTDRV_CNT 2

enum {
	SHT_TEMP_EVENT,
	SHT_HUMID_EVENT,
};

#pragma pack (push,1)

typedef struct ShtDrvParams_s ShtDrvParams_t;
struct ShtDrvParams_s {
	uint16_t period;
	struct {
		SignalMode_t mode;
		uint8_t thresh;
	} s[SHTDRV_CNT];
};

typedef struct ShtDrvTimeStamp_s ShtDrvTimeStamp_t;
struct ShtDrvTimeStamp_s {
	uint32_t self[SHTDRV_CNT];
};

#pragma pack (pop)

typedef struct ShtDrvData_s ShtDrvData_t;
struct ShtDrvData_s {
	ShtDrvTimeStamp_t ts;
	struct {
		uint32_t ts;
		struct {
			int16_t self;
			uint8_t diag;
		} value;
	} s[SHTDRV_CNT];
};

struct DMA_InitTypeDef2_s;

typedef struct ShtDrv_s ShtDrv_t;
struct ShtDrv_s {
	BaseDriver_t base;
	ShtDrvParams_t prms;
	ShtDrvData_t data;

	uint8_t currentSens;
	struct {
		uint8_t state;
		uint8_t buf[16];
	} rd;
};

EXTERN_C void shtdrv_setParams(ShtDrv_t *drv, ShtDrvParams_t *params);
EXTERN_C void shtdrv_getData(ShtDrv_t *drv, ShtDrvData_t *copyto, uint8_t forceCopy);

EXTERN_C void shtdrv_init(ShtDrv_t *drv, ShtDrvTimeStamp_t *bckp);
EXTERN_C void shtdrv_cycle(ShtDrv_t *drv, void *caller);

EXTERN_C ShtDrv_t *shtdrv_create();
EXTERN_C void shtdrv_free(ShtDrv_t *drv);

#endif // SHTDRV_H
