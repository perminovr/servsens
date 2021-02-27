#ifndef UARTDRV_T
#define UARTDRV_T

#if defined(__cplusplus)
#else
#endif

#include "basedriver.h"

#define UARTDRV_BUFFER_SIZE_MAX	256

#pragma pack (push,1)

typedef struct UartDrvParams_s UartDrvParams_t;
struct UartDrvParams_s {
	uint8_t enabled : 1;
};

#pragma pack (pop)

typedef struct UartDrvData_s UartDrvData_t;
struct UartDrvData_s {
	uint8_t message[UARTDRV_BUFFER_SIZE_MAX-4];
	uint8_t size;
};

typedef struct UartDrv_s UartDrv_t;
struct UartDrv_s {
	BaseDriver_t base;
	UartDrvParams_t prms;
	UartDrvData_t data;

	uint8_t startDetected : 3;
	uint8_t stopDetected : 3;
	uint8_t startPos;

	uint8_t txb[UARTDRV_BUFFER_SIZE_MAX];
	uint8_t rxb[UARTDRV_BUFFER_SIZE_MAX];
	uint8_t rit;
};

EXTERN_C int uartdrv_setData(UartDrv_t *drv, const UartDrvData_t *msg);

EXTERN_C void uartdrv_setParams(UartDrv_t *drv, UartDrvParams_t *params);
EXTERN_C void uartdrv_getData(UartDrv_t *drv, UartDrvData_t *copyto, uint8_t forceCopy);

EXTERN_C void uartdrv_init(UartDrv_t *drv, void *unused);
EXTERN_C void uartdrv_cycle(UartDrv_t *drv, void *caller);

EXTERN_C UartDrv_t *uartdrv_create();
EXTERN_C void uartdrv_free(UartDrv_t *drv);

#endif