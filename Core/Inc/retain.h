#ifndef RETAIN_H
#define RETAIN_H


#include "dindrv.h"
#include "adcdrv.h"
#include "senst1wdrv.h"
#include "shtdrv.h"
#include "wtdrv.h"
#include "utils.h"

#define RETAIN_TOTAL_SIZE_BYTE	256

#pragma pack (push,1)

#define DATABASE_VERSION	1

typedef struct {
	

} RetainDataBase_t;

#pragma pack (pop)

typedef struct EepromDrv_s EepromDrv_t;
struct EepromDrv_s;

typedef struct {
	BaseDriver_t base;

	RetainDataBase_t db;
    uint8_t eeprom[RETAIN_TOTAL_SIZE_BYTE];
    uint8_t dcopy[sizeof(RetainDataBase_t)];

	uint8_t trgUpd : 1;
	struct {
    	uint8_t CNT;
    	uint8_t nextPage;
		uint8_t state;
		uint8_t chgd;
		uint16_t idx;
		uint16_t waddr, caddr;
	} wr;
	struct EepromDrv_s *eedrv;
} Retain_t;

EXTERN_C void retain_triggerUpdate(Retain_t *drv);
EXTERN_C void retain_resetDefault(Retain_t *drv);

EXTERN_C void retain_init(Retain_t *drv, void *unused);
EXTERN_C void retain_cycle(Retain_t *drv, void *caller);

EXTERN_C Retain_t *retain_create();
EXTERN_C void retain_free(Retain_t *drv);

#endif // RETAIN_H
