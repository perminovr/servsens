#include "wtdrv.h"


void wtdrv_cycle(WtDrv_t *drv, void *caller)
{
	if (caller == RTC) {
		drv->counter++;
		if (drv->counter >= drv->prms.period) {
			drv->counter = 0;
			drv->data.ts.self++;
			drv->base.dataIsReady = 1;
			drv->base.eventIsReady = 1;
		}
	}
}


void wtdrv_setParams(WtDrv_t *drv, WtDrvParams_t *params)
{
	__disable_irq();
	memcpy(&drv->prms, params, sizeof(WtDrvParams_t));
	__enable_irq();
}


void wtdrv_getData(WtDrv_t *drv, WtDrvData_t *copyto, uint8_t forceCopy)
{
	if (forceCopy || drv->base.dataIsReady) {
		__disable_irq();
		memcpy(copyto, &drv->data, sizeof(WtDrvData_t));
		drv->base.dataIsReady = 0;
		drv->base.eventIsReady = 0;
		__enable_irq();
	}
}


static void wtdrv_resetTs(WtDrv_t *drv)
{
//	__disable_irq();
//	memset(&drv->data.ts, 0, sizeof(WtDrvTimeStamp_t));
//	__enable_irq();
}


void wtdrv_init(WtDrv_t *drv, WtDrvTimeStamp_t *bckp)
{
	basedriver_init(&drv->base);
	drv->base.cycle = (basedrv_cycle)wtdrv_cycle;
	drv->base.resetTs = (basedrv_resetTs)wtdrv_resetTs;

	memcpy(&drv->data.ts, bckp, sizeof(WtDrvTimeStamp_t));
}


WtDrv_t *wtdrv_create()
{
	WtDrv_t *ret = (WtDrv_t *)calloc(1, sizeof(WtDrv_t));
	return ret;
}


void wtdrv_free(WtDrv_t *drv)
{
	free(drv);
}
