#include "basedriver.h"


void basedriver_registerQue(BaseDriver_t *base, to_que_t *toque, int id)
{
	if (base->mytoque) {
		to_delete(base->mytoque, &base->toitem);
	}
	to_inititem(&base->toitem, base, id);
	base->mytoque = toque;
}


static void basedriver_callAfter(BaseDriver_t *base, uint32_t timeout)
{
	__disable_fault_irq();
	to_reappend(base->mytoque, &base->toitem, timeout*1000);
	__enable_fault_irq();
}

static int basedriver_getId(BaseDriver_t *base)
{
	return base->toitem.id;
}


void basedriver_init(BaseDriver_t *base)
{
	base->callAfter = basedriver_callAfter;
	base->getId = basedriver_getId;
	to_inititem(&base->toitem, base, 0);
}


uint8_t basedriver_setSupBusy(BaseDriver_t *base, uint8_t state)
{
	int ret = 0;
	if (base->supBusy) {
		__disable_fault_irq();
		if (state) {
			if ( *(base->supBusy) == 0 ) {
				*(base->supBusy) = 1;
				ret = 1;
			}
		} else {
			*(base->supBusy) = 0;
			ret = 1;
		}
		__enable_fault_irq();
	}
	return ret;
}
